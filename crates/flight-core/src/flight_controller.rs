//! Top-level composition of estimators, controllers, safety, and allocation.

use crate::{
    altitude_controller::{
        AltitudeController, AltitudeControllerError, AltitudeControllerInput,
        AltitudeControllerOutput,
    },
    attitude_controller::{AttitudeController, AttitudeControllerError, AttitudeControllerInput},
    attitude_estimator::{AttitudeEstimator, AttitudeEstimatorError},
    collective::{
        CollectivePolicy, CollectivePolicyError, CollectivePolicyInput, CollectivePolicyOutput,
    },
    flight_state::{FlightState, FlightStateInput, FlightStateOutput},
    flow_measurement::{
        FlowMeasurementError, FlowMeasurementInput, FlowMeasurementProcessor, RawOpticalFlowSample,
    },
    horizontal_controller::{
        HorizontalController, HorizontalControllerError, HorizontalControllerInput,
        HorizontalControllerOutput,
    },
    horizontal_estimator::{
        HorizontalEstimate, HorizontalEstimator, HorizontalEstimatorError, HorizontalEstimatorInput,
    },
    mixer::{MixerError, MotorMixer},
    safety::SafetyCutoff,
    types::{
        AttitudeEstimate, BodyAccelerationG, BodyAngularRateDps, CutoffReason, MotorInput,
        MotorMix, PidOutput,
    },
    vertical_acceleration::{
        EarthVerticalAccelerationError, earth_vertical_acceleration_mm_per_second_squared,
    },
    vertical_estimator::{
        TofMeasurement, VerticalEstimate, VerticalEstimator, VerticalEstimatorError,
    },
};

pub struct FlightController {
    flow_measurement_processor: FlowMeasurementProcessor,
    attitude_estimator: AttitudeEstimator,
    vertical_estimator: VerticalEstimator,
    horizontal_estimator: HorizontalEstimator,
    attitude_controller: AttitudeController,
    altitude_controller: AltitudeController,
    horizontal_controller: HorizontalController,
    safety_cutoff: SafetyCutoff,
    flight_state: FlightState,
    collective_policy: CollectivePolicy,
    previous_base_thrust_ramp_complete: bool,
    previous_altitude_integral_learning_enabled: bool,
    attitude_accel_correction_weight: f32,
}

pub struct FlightControllerInput {
    pub flow_update: Option<FlowUpdate>,
    pub body_acceleration_g: BodyAccelerationG,
    pub measured_body_rate_dps: BodyAngularRateDps,
    pub tof_update: Option<TofUpdate>,
    pub target_pitch_deg: f32,
    pub target_roll_deg: f32,
    pub target_yaw_heading_deg: f32,
    pub target_height_mm: f32,
    pub motor_output_authorized: bool,
    pub horizontal_control_enabled: bool,
    pub dt_seconds: f32,
}

pub struct FlightControllerOutput {
    pub attitude_estimate: AttitudeEstimate,
    pub vertical_estimate: VerticalEstimate,
    pub horizontal_estimate: HorizontalEstimate,
    pub attitude_controller_output: PidOutput,
    pub altitude_controller_output: AltitudeControllerOutput,
    pub horizontal_controller_output: HorizontalControllerOutput,
    pub flight_state_output: FlightStateOutput,
    pub collective_output: CollectivePolicyOutput,
    pub cutoff_reason: CutoffReason,
    pub motor_mix: MotorMix,
    pub effective_target_pitch_deg: f32,
    pub effective_target_roll_deg: f32,
    pub attitude_accel_correction_weight: f32,
}

#[derive(Debug)]
pub enum FlightControllerError {
    FlowMeasurement(FlowMeasurementError),
    AttitudeController(AttitudeControllerError),
    AltitudeController(AltitudeControllerError),
    HorizontalController(HorizontalControllerError),
    AttitudeEstimator(AttitudeEstimatorError),
    HorizontalEstimator(HorizontalEstimatorError),
    VerticalEstimator(VerticalEstimatorError),
    EarthVerticalAcceleration(EarthVerticalAccelerationError),
    CollectivePolicy(CollectivePolicyError),
    MotorMix(MixerError),
}

pub struct TofUpdate {
    pub measurement: TofMeasurement,
    pub sample_dt_seconds: f32,
}

pub struct FlowUpdate {
    pub raw_optical_flow_sample: RawOpticalFlowSample,
    pub sample_dt_seconds: f32,
}

impl FlightController {
    pub fn new(battery_voltage_v: f32) -> Result<Self, FlightControllerError> {
        let attitude_controller = match AttitudeController::new() {
            Ok(attitude_controller) => attitude_controller,
            Err(err) => return Err(FlightControllerError::AttitudeController(err)),
        };
        Ok(Self {
            flow_measurement_processor: FlowMeasurementProcessor::new(),
            attitude_estimator: AttitudeEstimator::new(),
            vertical_estimator: VerticalEstimator::new(),
            horizontal_estimator: HorizontalEstimator::new(),
            attitude_controller,
            altitude_controller: AltitudeController::new(),
            horizontal_controller: HorizontalController::new(),
            safety_cutoff: SafetyCutoff::new(),
            flight_state: FlightState::new(),
            collective_policy: CollectivePolicy::new(battery_voltage_v),
            previous_base_thrust_ramp_complete: false,
            previous_altitude_integral_learning_enabled: false,
            attitude_accel_correction_weight: 1.0,
        })
    }

    pub fn update(
        &mut self,
        input: FlightControllerInput,
    ) -> Result<FlightControllerOutput, FlightControllerError> {
        let attitude_accel_correction_weight = self.attitude_accel_correction_weight;
        let attitude_estimate = match self.attitude_estimator.update(
            input.body_acceleration_g,
            input.measured_body_rate_dps,
            input.dt_seconds,
            attitude_accel_correction_weight,
        ) {
            Ok(attitude_estimate) => attitude_estimate,
            Err(err) => return Err(FlightControllerError::AttitudeEstimator(err)),
        };
        let vertical_acceleration_mm_per_second_squared =
            match earth_vertical_acceleration_mm_per_second_squared(
                attitude_estimate.pitch_deg,
                attitude_estimate.roll_deg,
                input.body_acceleration_g,
            ) {
                Ok(earth_vertical_acceleration) => earth_vertical_acceleration,
                Err(err) => return Err(FlightControllerError::EarthVerticalAcceleration(err)),
            };
        let mut vertical_estimate = match self.vertical_estimator.predict(
            vertical_acceleration_mm_per_second_squared,
            input.dt_seconds,
        ) {
            Ok(vertical_estimate) => vertical_estimate,
            Err(err) => return Err(FlightControllerError::VerticalEstimator(err)),
        };
        if let Some(tof_update) = input.tof_update {
            vertical_estimate = match self
                .vertical_estimator
                .correct_tof(tof_update.measurement, tof_update.sample_dt_seconds)
            {
                Ok(corrected_vertical_estimate) => corrected_vertical_estimate,
                Err(err) => return Err(FlightControllerError::VerticalEstimator(err)),
            }
        };

        match self
            .flow_measurement_processor
            .accumulate_body_rotation(input.measured_body_rate_dps, input.dt_seconds)
        {
            Ok(_) => (),
            Err(err) => return Err(FlightControllerError::FlowMeasurement(err)),
        }

        let mut next_attitude_accel_correction_weight = attitude_accel_correction_weight;
        let horizontal_velocity_measurement = if let Some(flow_update) = input.flow_update {
            let output = match self
                .flow_measurement_processor
                .process(FlowMeasurementInput {
                    sample: flow_update.raw_optical_flow_sample,
                    attitude_estimate,
                    measured_body_rate_dps: input.measured_body_rate_dps,
                    estimated_height_mm: vertical_estimate.height_mm,
                    tof_age_seconds: vertical_estimate.tof_age_seconds,
                    flow_sample_dt_seconds: flow_update.sample_dt_seconds,
                }) {
                Ok(output) => output,
                Err(err) => return Err(FlightControllerError::FlowMeasurement(err)),
            };
            next_attitude_accel_correction_weight = output.attitude_accel_correction_weight;
            match output.measurement {
                Ok(measurement) => Some(measurement),
                Err(_) => None,
            }
        } else {
            None
        };

        let horizontal_estimate = match self.horizontal_estimator.update(HorizontalEstimatorInput {
            dt_seconds: input.dt_seconds,
            measurement: horizontal_velocity_measurement,
        }) {
            Ok(estimate) => estimate,
            Err(err) => return Err(FlightControllerError::HorizontalEstimator(err)),
        };

        let horizontal_controller_output =
            match self
                .horizontal_controller
                .update(HorizontalControllerInput {
                    horizontal_estimate,
                    yaw_heading_deg: attitude_estimate.yaw_heading_deg,
                }) {
                Ok(output) => output,
                Err(err) => return Err(FlightControllerError::HorizontalController(err)),
            };

        let cutoff_reason = self.safety_cutoff.update(
            attitude_estimate.pitch_deg,
            attitude_estimate.roll_deg,
            input.measured_body_rate_dps,
        );
        let altitude_controller_output =
            match self.altitude_controller.update(AltitudeControllerInput {
                target_height_mm: input.target_height_mm,
                vertical_estimate,
                dt_seconds: input.dt_seconds,
                integral_learning_enabled: self.previous_altitude_integral_learning_enabled,
            }) {
                Ok(altitude_controller_output) => altitude_controller_output,
                Err(err) => return Err(FlightControllerError::AltitudeController(err)),
            };
        let flight_state_output = self.flight_state.update(FlightStateInput {
            motor_output_authorized: input.motor_output_authorized,
            cutoff_reason,
            altitude_controller_active: altitude_controller_output.active,
            horizontal_control_enabled: input.horizontal_control_enabled,
            base_thrust_ramp_complete: self.previous_base_thrust_ramp_complete,
            attitude_estimate,
            measured_body_rate_dps: input.measured_body_rate_dps,
            horizontal_controller_active: horizontal_controller_output.active,
        });
        let (applied_horizontal_pitch_offset_deg, applied_horizontal_roll_offset_deg) =
            if flight_state_output.horizontal_control_authorized {
                (
                    horizontal_controller_output.target_pitch_deg,
                    horizontal_controller_output.target_roll_deg,
                )
            } else {
                (0.0, 0.0)
            };
        let effective_target_pitch_deg =
            input.target_pitch_deg + applied_horizontal_pitch_offset_deg;
        let effective_target_roll_deg = input.target_roll_deg + applied_horizontal_roll_offset_deg;
        let attitude_controller_output =
            match self.attitude_controller.update(AttitudeControllerInput {
                target_pitch_deg: effective_target_pitch_deg,
                target_roll_deg: effective_target_roll_deg,
                target_yaw_heading_deg: input.target_yaw_heading_deg,
                measured_pitch_deg: attitude_estimate.pitch_deg,
                measured_roll_deg: attitude_estimate.roll_deg,
                measured_yaw_heading_deg: attitude_estimate.yaw_heading_deg,
                measured_pitch_rate_dps: input.measured_body_rate_dps.pitch_dps,
                measured_roll_rate_dps: input.measured_body_rate_dps.roll_dps,
                measured_yaw_rate_dps: input.measured_body_rate_dps.yaw_dps,
                dt_seconds: input.dt_seconds,
            }) {
                Ok(attitude_controller_output) => attitude_controller_output,
                Err(err) => return Err(FlightControllerError::AttitudeController(err)),
            };
        let collective_output = match self.collective_policy.update(CollectivePolicyInput {
            desired_collective_correction_duty: altitude_controller_output
                .desired_collective_correction_duty,
            mode: flight_state_output.collective_mode,
            dt_seconds: input.dt_seconds,
            vertical_velocity_mm_per_second: vertical_estimate.vertical_velocity_mm_per_second,
            attitude_qualified: flight_state_output.attitude_qualified,
        }) {
            Ok(collective_output) => collective_output,
            Err(err) => return Err(FlightControllerError::CollectivePolicy(err)),
        };
        let mut motor_mix = match MotorMixer::mix(MotorInput {
            thrust: collective_output.collective_duty,
            pitch: attitude_controller_output.pitch_correction,
            roll: attitude_controller_output.roll_correction,
            yaw: attitude_controller_output.yaw_correction,
        }) {
            Ok(motor_mix) => motor_mix,
            Err(err) => return Err(FlightControllerError::MotorMix(err)),
        };

        if !input.motor_output_authorized || cutoff_reason != CutoffReason::None {
            motor_mix.front_left = 0.0;
            motor_mix.front_right = 0.0;
            motor_mix.rear_left = 0.0;
            motor_mix.rear_right = 0.0;
        }

        self.previous_base_thrust_ramp_complete = collective_output.base_thrust_ramp_complete;
        self.previous_altitude_integral_learning_enabled =
            flight_state_output.altitude_integral_learning_enabled;
        self.attitude_accel_correction_weight = next_attitude_accel_correction_weight;

        Ok(FlightControllerOutput {
            attitude_estimate,
            vertical_estimate,
            attitude_controller_output,
            altitude_controller_output,
            flight_state_output,
            collective_output,
            cutoff_reason,
            motor_mix,
            horizontal_estimate,
            horizontal_controller_output,
            effective_target_pitch_deg,
            effective_target_roll_deg,
            attitude_accel_correction_weight,
        })
    }
}

#[cfg(test)]
mod tests {
    use crate::types::CollectiveMode;

    use super::*;

    const BATTERY_VOLTAGE_V: f32 = 3.8;
    const DT_SECONDS: f32 = 0.01;
    const FLOW_SAMPLE_DT_SECONDS: f32 = 0.02;
    const TOF_SAMPLE_DT_SECONDS: f32 = 0.05;

    #[test]
    fn unauthorized_motor_output_always_produces_zero_motor_duties() {
        let input = FlightControllerInput {
            body_acceleration_g: BodyAccelerationG {
                x_g: 0.0,
                y_g: 0.0,
                z_g: 1.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            tof_update: Some(TofUpdate {
                measurement: TofMeasurement {
                    range_mm: 120,
                    status: 0,
                    objects: 1,
                },
                sample_dt_seconds: TOF_SAMPLE_DT_SECONDS,
            }),
            target_pitch_deg: 10.0,
            target_roll_deg: 10.0,
            target_yaw_heading_deg: 10.0,
            target_height_mm: 700.0,
            motor_output_authorized: false,
            horizontal_control_enabled: false,
            dt_seconds: DT_SECONDS,
            flow_update: None,
        };
        let mut flight_controller =
            FlightController::new(BATTERY_VOLTAGE_V).expect("should be valid");
        let result = flight_controller.update(input).expect("should be valid");

        assert_eq!(result.cutoff_reason, CutoffReason::None);
        assert_eq!(
            result.flight_state_output.collective_mode,
            CollectiveMode::Disabled
        );
        assert_eq!(result.collective_output.collective_duty, 0.0);
        assert_eq!(result.motor_mix.front_left, 0.0);
        assert_eq!(result.motor_mix.front_right, 0.0);
        assert_eq!(result.motor_mix.rear_left, 0.0);
        assert_eq!(result.motor_mix.rear_right, 0.0);
    }

    #[test]
    fn authorized_motor_output_but_with_safety_cutoff_produces_zero_motor_duties() {
        let above_boundary_yaw_rate_dps: f32 = 351.0;
        let input = FlightControllerInput {
            body_acceleration_g: BodyAccelerationG {
                x_g: 0.0,
                y_g: 0.0,
                z_g: 1.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: above_boundary_yaw_rate_dps,
            },
            tof_update: Some(TofUpdate {
                measurement: TofMeasurement {
                    range_mm: 120,
                    status: 0,
                    objects: 1,
                },
                sample_dt_seconds: TOF_SAMPLE_DT_SECONDS,
            }),
            target_pitch_deg: 10.0,
            target_roll_deg: 10.0,
            target_yaw_heading_deg: 10.0,
            target_height_mm: 700.0,
            motor_output_authorized: true,
            horizontal_control_enabled: false,
            dt_seconds: DT_SECONDS,
            flow_update: None,
        };

        let mut flight_controller =
            FlightController::new(BATTERY_VOLTAGE_V).expect("should be valid");
        for _ in 0..7 {
            let input = FlightControllerInput {
                body_acceleration_g: BodyAccelerationG {
                    x_g: 0.0,
                    y_g: 0.0,
                    z_g: 1.0,
                },
                measured_body_rate_dps: BodyAngularRateDps {
                    pitch_dps: 0.0,
                    roll_dps: 0.0,
                    yaw_dps: above_boundary_yaw_rate_dps,
                },
                tof_update: Some(TofUpdate {
                    measurement: TofMeasurement {
                        range_mm: 120,
                        status: 0,
                        objects: 1,
                    },
                    sample_dt_seconds: TOF_SAMPLE_DT_SECONDS,
                }),
                target_pitch_deg: 10.0,
                target_roll_deg: 10.0,
                target_yaw_heading_deg: 10.0,
                target_height_mm: 700.0,
                motor_output_authorized: true,
                horizontal_control_enabled: false,
                dt_seconds: DT_SECONDS,
                flow_update: None,
            };
            let result = flight_controller.update(input).expect("should be valid");

            assert_eq!(result.cutoff_reason, CutoffReason::None);
            assert_eq!(
                result.flight_state_output.collective_mode,
                CollectiveMode::BaseThrust
            );
        }

        let result = flight_controller.update(input).expect("should be valid");

        assert_eq!(result.cutoff_reason, CutoffReason::YawRate);
        assert_eq!(
            result.flight_state_output.collective_mode,
            CollectiveMode::Disabled
        );
        assert_eq!(result.collective_output.collective_duty, 0.0);
        assert_eq!(result.motor_mix.front_left, 0.0);
        assert_eq!(result.motor_mix.front_right, 0.0);
        assert_eq!(result.motor_mix.rear_left, 0.0);
        assert_eq!(result.motor_mix.rear_right, 0.0);
    }

    #[test]
    fn ticks_progress_from_base_thrust_to_altitude_hold() {
        let input = FlightControllerInput {
            body_acceleration_g: BodyAccelerationG {
                x_g: 0.0,
                y_g: 0.0,
                z_g: 1.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            tof_update: Some(TofUpdate {
                measurement: TofMeasurement {
                    range_mm: 120,
                    status: 0,
                    objects: 1,
                },
                sample_dt_seconds: TOF_SAMPLE_DT_SECONDS,
            }),
            target_pitch_deg: 0.0,
            target_roll_deg: 0.0,
            target_yaw_heading_deg: 0.0,
            target_height_mm: 700.0,
            motor_output_authorized: true,
            horizontal_control_enabled: false,
            dt_seconds: DT_SECONDS,
            flow_update: None,
        };

        let mut flight_controller =
            FlightController::new(BATTERY_VOLTAGE_V).expect("should be valid");
        for update_number in 1..=99 {
            let tof_update = if update_number % 5 == 0 {
                Some(TofUpdate {
                    measurement: TofMeasurement {
                        range_mm: 120,
                        status: 0,
                        objects: 1,
                    },
                    sample_dt_seconds: TOF_SAMPLE_DT_SECONDS,
                })
            } else {
                None
            };
            let input = FlightControllerInput {
                body_acceleration_g: BodyAccelerationG {
                    x_g: 0.0,
                    y_g: 0.0,
                    z_g: 1.0,
                },
                measured_body_rate_dps: BodyAngularRateDps {
                    pitch_dps: 0.0,
                    roll_dps: 0.0,
                    yaw_dps: 0.0,
                },
                tof_update,
                target_pitch_deg: 0.0,
                target_roll_deg: 0.0,
                target_yaw_heading_deg: 0.0,
                target_height_mm: 700.0,
                motor_output_authorized: true,
                horizontal_control_enabled: false,
                dt_seconds: DT_SECONDS,
                flow_update: None,
            };
            let result = flight_controller.update(input).expect("should be valid");

            assert_eq!(result.cutoff_reason, CutoffReason::None);
            assert_eq!(
                result.flight_state_output.collective_mode,
                CollectiveMode::BaseThrust
            );
        }

        let result = flight_controller.update(input).expect("should be valid");

        assert_eq!(result.cutoff_reason, CutoffReason::None);
        assert_eq!(
            result.flight_state_output.collective_mode,
            CollectiveMode::AltitudeHold
        );
        assert!(
            result.collective_output.collective_duty > 175.0,
            "expected altitude hold to increase collective above base duty,
      actual={}",
            result.collective_output.collective_duty
        );
        assert_eq!(result.collective_output.base_thrust_ramp_complete, true);
        assert!(
            result.motor_mix.front_left > 0.0,
            "expected motor to be nonzero,
        actual={}",
            result.motor_mix.front_left
        );
        assert!(
            result.motor_mix.front_right > 0.0,
            "expected motor to be nonzero,
        actual={}",
            result.motor_mix.front_right
        );
        assert!(
            result.motor_mix.rear_left > 0.0,
            "expected motor to be nonzero,
        actual={}",
            result.motor_mix.rear_left
        );
        assert!(
            result.motor_mix.rear_right > 0.0,
            "expected motor to be nonzero,
        actual={}",
            result.motor_mix.rear_right
        );
    }

    #[test]
    fn error_propagation_example() {
        let input = FlightControllerInput {
            body_acceleration_g: BodyAccelerationG {
                x_g: 0.0,
                y_g: 0.0,
                z_g: 1.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            tof_update: Some(TofUpdate {
                measurement: TofMeasurement {
                    range_mm: 120,
                    status: 0,
                    objects: 1,
                },
                sample_dt_seconds: TOF_SAMPLE_DT_SECONDS,
            }),
            target_pitch_deg: 0.0,
            target_roll_deg: 0.0,
            target_yaw_heading_deg: 0.0,
            target_height_mm: 700.0,
            motor_output_authorized: true,
            horizontal_control_enabled: false,
            dt_seconds: 0.0,
            flow_update: None,
        };

        let mut flight_controller =
            FlightController::new(BATTERY_VOLTAGE_V).expect("should be valid");

        let result = flight_controller.update(input);
        assert!(matches!(
            result,
            Err(FlightControllerError::AttitudeEstimator(
                AttitudeEstimatorError::InvalidDt
            ))
        ))
    }

    #[test]
    fn flow_measurement() {
        let input = FlightControllerInput {
            body_acceleration_g: BodyAccelerationG {
                x_g: 0.0,
                y_g: 0.0,
                z_g: 1.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            tof_update: Some(TofUpdate {
                measurement: TofMeasurement {
                    range_mm: 120,
                    status: 0,
                    objects: 1,
                },
                sample_dt_seconds: TOF_SAMPLE_DT_SECONDS,
            }),
            target_pitch_deg: 0.0,
            target_roll_deg: 0.0,
            target_yaw_heading_deg: 0.0,
            target_height_mm: 700.0,
            motor_output_authorized: true,
            horizontal_control_enabled: false,
            dt_seconds: DT_SECONDS,
            flow_update: Some(FlowUpdate {
                raw_optical_flow_sample: RawOpticalFlowSample {
                    motion_detected: true,
                    delta_x_counts: 0,
                    delta_y_counts: 10,
                    surface_quality: 255,
                },
                sample_dt_seconds: FLOW_SAMPLE_DT_SECONDS,
            }),
        };

        let mut flight_controller =
            FlightController::new(BATTERY_VOLTAGE_V).expect("should be valid");
        let result = flight_controller.update(input).expect("should be valid");

        assert_eq!(
            result.horizontal_estimate.measurement_age_seconds,
            Some(0.0)
        );
        assert!(
            result
                .horizontal_estimate
                .world_forward_velocity_counts_per_second
                > 0.0
        );
        assert!(result.horizontal_controller_output.active);
        assert!(result.horizontal_controller_output.target_pitch_deg < 0.0);
    }
}
