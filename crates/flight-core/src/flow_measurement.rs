//! Optical-flow qualification, derotation, scaling, and frame conversion.

use crate::types::{AttitudeEstimate, BodyAngularRateDps};
pub struct FlowMeasurementProcessor {
    pitch_rotation_since_flow_sample_deg: f32,
    roll_rotation_since_flow_sample_deg: f32,
}

pub struct RawOpticalFlowSample {
    pub motion_detected: bool,
    pub delta_x_counts: i16,
    pub delta_y_counts: i16,
    pub surface_quality: u8,
}

pub struct FlowMeasurementInput {
    pub sample: RawOpticalFlowSample,
    pub attitude_estimate: AttitudeEstimate,
    pub measured_body_rate_dps: BodyAngularRateDps,
    pub estimated_height_mm: f32,
    pub tof_age_seconds: Option<f32>,
    pub flow_sample_dt_seconds: f32,
}

pub struct HorizontalVelocityMeasurement {
    pub velocity_variance_counts_squared_per_second_squared: f32,
    pub world_forward_velocity_counts_per_second: f32,
    pub world_left_velocity_counts_per_second: f32,
}

pub struct FlowMeasurementOutput {
    pub measurement: Result<HorizontalVelocityMeasurement, FlowMeasurementRejection>,
    pub body_forward_rotation_counts: f32,
    pub body_left_rotation_counts: f32,
    pub body_forward_translation_counts: f32,
    pub body_left_translation_counts: f32,
    pub body_forward_counts_were_limited: bool,
    pub body_left_counts_were_limited: bool,
    pub attitude_accel_correction_weight: f32,
}

#[derive(Debug, PartialEq, Eq)]
pub enum FlowMeasurementRejection {
    LowSurfaceQuality,
    AttitudeOutsideLimit,
    BodyRateOutsideLimit,
    MissingTof,
    StaleTof,
    HeightOutsideRange,
    MotionNotDetected,
}

#[derive(Debug, PartialEq, Eq)]
pub enum FlowMeasurementError {
    NonFiniteInput,
    RotationDtOutOfRange,
    FlowSampleDtOutOfRange,
    NonFiniteOutput,
}

const MIN_ROTATION_DT_SECONDS: f32 = 0.001;
const MAX_ROTATION_DT_SECONDS: f32 = 0.01;

const MIN_FLOW_DT_SECONDS: f32 = 0.001;
const MAX_FLOW_DT_SECONDS: f32 = 0.1;
const MIN_FLOW_HEIGHT_MM: f32 = 25.0;
const MAX_FLOW_HEIGHT_MM: f32 = 600.0;
const MAX_FLOW_MEASUREMENT_ATTITUDE_DEG: f32 = 20.0;
const MAX_FLOW_MEASUREMENT_PITCH_ROLL_RATE_DPS: f32 = 175.0;
const MAX_FLOW_MEASUREMENT_YAW_RATE_DPS: f32 = 90.0;
const FLOW_DEADZONE_COUNTS: f32 = 3.0;
const FLOW_REFERENCE_HEIGHT_MM: f32 = 180.0;
const FLOW_VELOCITY_MEASUREMENT_STD_COUNTS_PER_SECOND: f32 = 150.0;
const FLOW_QUALITY_UNCERTAINTY_SCALE: f32 = 3.0;

const MAX_VELOCITY_COUNTS_PER_SECOND: f32 = 3000.0;

const ACCEL_CORRECTION_FULL_FLOW_COUNTS: f32 = 5.0;
const ACCEL_CORRECTION_ZERO_FLOW_COUNTS: f32 = 15.0;

const TOF_FRESHNESS_LIMIT_SECONDS: f32 = 0.15;

const PMW_COUNTS_PER_RADIAN: f32 = 385.0;

impl FlowMeasurementProcessor {
    pub fn new() -> Self {
        Self {
            pitch_rotation_since_flow_sample_deg: 0.0,
            roll_rotation_since_flow_sample_deg: 0.0,
        }
    }

    pub fn accumulate_body_rotation(
        &mut self,
        measured_body_rate_dps: BodyAngularRateDps,
        imu_dt_seconds: f32,
    ) -> Result<(), FlowMeasurementError> {
        if !measured_body_rate_dps.pitch_dps.is_finite()
            || !measured_body_rate_dps.roll_dps.is_finite()
            || !measured_body_rate_dps.yaw_dps.is_finite()
            || !imu_dt_seconds.is_finite()
        {
            return Err(FlowMeasurementError::NonFiniteInput);
        }

        if !(MIN_ROTATION_DT_SECONDS..=MAX_ROTATION_DT_SECONDS).contains(&imu_dt_seconds) {
            return Err(FlowMeasurementError::RotationDtOutOfRange);
        }

        let pitch_rotation_candidate_deg = self.pitch_rotation_since_flow_sample_deg
            + measured_body_rate_dps.pitch_dps * imu_dt_seconds;
        let roll_rotation_candidate_deg = self.roll_rotation_since_flow_sample_deg
            + measured_body_rate_dps.roll_dps * imu_dt_seconds;

        if !pitch_rotation_candidate_deg.is_finite() || !roll_rotation_candidate_deg.is_finite() {
            return Err(FlowMeasurementError::NonFiniteOutput);
        }

        self.pitch_rotation_since_flow_sample_deg = pitch_rotation_candidate_deg;
        self.roll_rotation_since_flow_sample_deg = roll_rotation_candidate_deg;
        Ok(())
    }

    pub fn process(
        &mut self,
        input: FlowMeasurementInput,
    ) -> Result<FlowMeasurementOutput, FlowMeasurementError> {
        Self::validate_input(&input)?;

        let sample_rejection = Self::sample_rejection(&input);

        let body_forward_rotation_counts =
            -self.pitch_rotation_since_flow_sample_deg.to_radians() * PMW_COUNTS_PER_RADIAN;
        let body_left_rotation_counts =
            self.roll_rotation_since_flow_sample_deg.to_radians() * PMW_COUNTS_PER_RADIAN;

        let max_counts_per_sample = MAX_VELOCITY_COUNTS_PER_SECOND * input.flow_sample_dt_seconds;
        let body_forward_translation_counts =
            f32::from(input.sample.delta_y_counts) - body_forward_rotation_counts;
        let body_left_translation_counts =
            f32::from(input.sample.delta_x_counts) - body_left_rotation_counts;

        let bounded_body_forward_translation_counts =
            Self::bounded_counts(body_forward_translation_counts, max_counts_per_sample);
        let bounded_body_left_translation_counts =
            Self::bounded_counts(body_left_translation_counts, max_counts_per_sample);

        let height_scale = input.estimated_height_mm / FLOW_REFERENCE_HEIGHT_MM;
        let height_scaled_body_forward_translation_counts =
            bounded_body_forward_translation_counts * height_scale;
        let height_scaled_body_left_translation_counts =
            bounded_body_left_translation_counts * height_scale;

        let yaw_heading_rad = input.attitude_estimate.yaw_heading_deg.to_radians();
        let world_forward_counts = height_scaled_body_forward_translation_counts
            * yaw_heading_rad.cos()
            - height_scaled_body_left_translation_counts * yaw_heading_rad.sin();
        let world_left_counts = height_scaled_body_forward_translation_counts
            * yaw_heading_rad.sin()
            + height_scaled_body_left_translation_counts * yaw_heading_rad.cos();

        let world_forward_velocity_counts_per_second =
            world_forward_counts / input.flow_sample_dt_seconds;
        let world_left_velocity_counts_per_second =
            world_left_counts / input.flow_sample_dt_seconds;

        let quality = f32::from(input.sample.surface_quality) / f32::from(u8::MAX);
        let quality_scale = 1.0 + FLOW_QUALITY_UNCERTAINTY_SCALE * (1.0 - quality).powi(2);
        let velocity_std = FLOW_VELOCITY_MEASUREMENT_STD_COUNTS_PER_SECOND * quality_scale;

        let magnitude = body_forward_translation_counts.hypot(body_left_translation_counts);

        self.reset_state();

        Ok(FlowMeasurementOutput {
            measurement: match sample_rejection {
                Some(rejection) => Err(rejection),
                None => Ok(HorizontalVelocityMeasurement {
                    world_forward_velocity_counts_per_second,
                    world_left_velocity_counts_per_second,
                    velocity_variance_counts_squared_per_second_squared: velocity_std.powi(2),
                }),
            },
            body_forward_rotation_counts,
            body_left_rotation_counts,
            body_forward_translation_counts,
            body_left_translation_counts,
            body_forward_counts_were_limited: body_forward_translation_counts.abs()
                > max_counts_per_sample,
            body_left_counts_were_limited: body_left_translation_counts.abs()
                > max_counts_per_sample,
            attitude_accel_correction_weight: (1.0
                - (magnitude - ACCEL_CORRECTION_FULL_FLOW_COUNTS)
                    / (ACCEL_CORRECTION_ZERO_FLOW_COUNTS - ACCEL_CORRECTION_FULL_FLOW_COUNTS))
                .clamp(0.0, 1.0),
        })
    }

    fn reset_state(&mut self) {
        self.pitch_rotation_since_flow_sample_deg = 0.0;
        self.roll_rotation_since_flow_sample_deg = 0.0;
    }

    fn sample_rejection(input: &FlowMeasurementInput) -> Option<FlowMeasurementRejection> {
        if !input.sample.motion_detected {
            return Some(FlowMeasurementRejection::MotionNotDetected);
        }
        if input.sample.surface_quality < 40 {
            return Some(FlowMeasurementRejection::LowSurfaceQuality);
        }
        let Some(tof_age_seconds) = input.tof_age_seconds else {
            return Some(FlowMeasurementRejection::MissingTof);
        };

        if !(MIN_FLOW_HEIGHT_MM..=MAX_FLOW_HEIGHT_MM).contains(&input.estimated_height_mm) {
            return Some(FlowMeasurementRejection::HeightOutsideRange);
        }

        if input.attitude_estimate.pitch_deg.abs() > MAX_FLOW_MEASUREMENT_ATTITUDE_DEG
            || input.attitude_estimate.roll_deg.abs() > MAX_FLOW_MEASUREMENT_ATTITUDE_DEG
        {
            return Some(FlowMeasurementRejection::AttitudeOutsideLimit);
        }

        if input.measured_body_rate_dps.pitch_dps.abs() > MAX_FLOW_MEASUREMENT_PITCH_ROLL_RATE_DPS
            || input.measured_body_rate_dps.roll_dps.abs()
                > MAX_FLOW_MEASUREMENT_PITCH_ROLL_RATE_DPS
            || input.measured_body_rate_dps.yaw_dps.abs() > MAX_FLOW_MEASUREMENT_YAW_RATE_DPS
        {
            return Some(FlowMeasurementRejection::BodyRateOutsideLimit);
        }

        if tof_age_seconds > TOF_FRESHNESS_LIMIT_SECONDS {
            return Some(FlowMeasurementRejection::StaleTof);
        }
        return None;
    }

    fn validate_input(input: &FlowMeasurementInput) -> Result<(), FlowMeasurementError> {
        if !input.measured_body_rate_dps.pitch_dps.is_finite()
            || !input.measured_body_rate_dps.roll_dps.is_finite()
            || !input.measured_body_rate_dps.yaw_dps.is_finite()
            || !input.flow_sample_dt_seconds.is_finite()
            || !input.attitude_estimate.pitch_deg.is_finite()
            || !input.attitude_estimate.roll_deg.is_finite()
            || !input.attitude_estimate.yaw_heading_deg.is_finite()
            || !input.estimated_height_mm.is_finite()
        {
            return Err(FlowMeasurementError::NonFiniteInput);
        }
        if !(MIN_FLOW_DT_SECONDS..=MAX_FLOW_DT_SECONDS).contains(&input.flow_sample_dt_seconds) {
            return Err(FlowMeasurementError::FlowSampleDtOutOfRange);
        }

        Ok(())
    }

    fn bounded_counts(counts: f32, max_counts_per_sample: f32) -> f32 {
        if counts.abs() <= FLOW_DEADZONE_COUNTS {
            0.0
        } else {
            counts.clamp(-max_counts_per_sample, max_counts_per_sample)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FLOW_SAMPLE_DT_SECONDS: f32 = 0.02;
    const TOLERANCE: f32 = 0.0001;

    fn assert_approximately_equal(actual: f32, expected: f32, tolerance: f32) {
        let difference = (actual - expected).abs();
        assert!(
            difference <= tolerance,
            "mismatch: actual={actual}, expected={expected}, difference={difference}, tolerance={tolerance}"
        );
    }

    #[test]
    fn reject_flow_measurement_during_excessive_body_rotation() {
        let input = FlowMeasurementInput {
            sample: RawOpticalFlowSample {
                motion_detected: true,
                delta_x_counts: 0,
                delta_y_counts: 10,
                surface_quality: 255,
            },
            attitude_estimate: AttitudeEstimate {
                pitch_deg: 0.0,
                roll_deg: 0.0,
                yaw_heading_deg: 0.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 200.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            flow_sample_dt_seconds: FLOW_SAMPLE_DT_SECONDS,
            estimated_height_mm: 180.0,
            tof_age_seconds: Some(0.0),
        };
        let mut processor = FlowMeasurementProcessor::new();
        let output = processor.process(input).expect("input should be valid");

        assert!(matches!(
            output.measurement,
            Err(FlowMeasurementRejection::BodyRateOutsideLimit)
        ));
    }

    #[test]
    fn reject_flow_measurement_when_drone_is_tilted() {
        let input = FlowMeasurementInput {
            sample: RawOpticalFlowSample {
                motion_detected: true,
                delta_x_counts: 0,
                delta_y_counts: 10,
                surface_quality: 255,
            },
            attitude_estimate: AttitudeEstimate {
                pitch_deg: 20.1,
                roll_deg: 0.0,
                yaw_heading_deg: 0.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            flow_sample_dt_seconds: FLOW_SAMPLE_DT_SECONDS,
            estimated_height_mm: 180.0,
            tof_age_seconds: Some(0.0),
        };
        let mut processor = FlowMeasurementProcessor::new();
        let output = processor.process(input).expect("input should be valid");

        assert!(matches!(
            output.measurement,
            Err(FlowMeasurementRejection::AttitudeOutsideLimit)
        ));
    }

    #[test]
    fn reject_flow_measurement_outside_height_trusted_range() {
        let input = FlowMeasurementInput {
            sample: RawOpticalFlowSample {
                motion_detected: true,
                delta_x_counts: 0,
                delta_y_counts: 10,
                surface_quality: 255,
            },
            attitude_estimate: AttitudeEstimate {
                pitch_deg: 0.0,
                roll_deg: 0.0,
                yaw_heading_deg: 0.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            flow_sample_dt_seconds: FLOW_SAMPLE_DT_SECONDS,
            estimated_height_mm: 700.0,
            tof_age_seconds: Some(0.0),
        };
        let mut processor = FlowMeasurementProcessor::new();
        let output = processor.process(input).expect("input should be valid");

        assert!(matches!(
            output.measurement,
            Err(FlowMeasurementRejection::HeightOutsideRange)
        ));
    }

    #[test]
    fn attitude_accel_correction_weight() {
        let input = FlowMeasurementInput {
            sample: RawOpticalFlowSample {
                motion_detected: true,
                delta_x_counts: 0,
                delta_y_counts: 5,
                surface_quality: 255,
            },
            attitude_estimate: AttitudeEstimate {
                pitch_deg: 0.0,
                roll_deg: 0.0,
                yaw_heading_deg: 0.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            flow_sample_dt_seconds: FLOW_SAMPLE_DT_SECONDS,
            estimated_height_mm: 180.0,
            tof_age_seconds: Some(0.0),
        };
        let mut processor = FlowMeasurementProcessor::new();
        let output = processor.process(input).expect("input should be valid");

        assert_eq!(output.attitude_accel_correction_weight, 1.0);

        let input = FlowMeasurementInput {
            sample: RawOpticalFlowSample {
                motion_detected: true,
                delta_x_counts: 0,
                delta_y_counts: 15,
                surface_quality: 255,
            },
            attitude_estimate: AttitudeEstimate {
                pitch_deg: 0.0,
                roll_deg: 0.0,
                yaw_heading_deg: 0.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            flow_sample_dt_seconds: FLOW_SAMPLE_DT_SECONDS,
            estimated_height_mm: 180.0,
            tof_age_seconds: Some(0.0),
        };

        let output = processor.process(input).expect("input should be valid");

        assert_eq!(output.attitude_accel_correction_weight, 0.0);
    }

    #[test]
    fn positive_delta_x_at_zero_yaw_produces_positive_left_velocity() {
        let input = FlowMeasurementInput {
            sample: RawOpticalFlowSample {
                motion_detected: true,
                delta_x_counts: 10,
                delta_y_counts: 0,
                surface_quality: 255,
            },
            attitude_estimate: AttitudeEstimate {
                pitch_deg: 0.0,
                roll_deg: 0.0,
                yaw_heading_deg: 0.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            flow_sample_dt_seconds: FLOW_SAMPLE_DT_SECONDS,
            estimated_height_mm: 180.0,
            tof_age_seconds: Some(0.0),
        };
        let mut processor = FlowMeasurementProcessor::new();
        let output = processor.process(input).expect("input should be valid");
        let measurement = output.measurement.expect("sample should be accepted");

        assert_eq!(output.body_forward_translation_counts, 0.0);
        assert_eq!(output.body_left_translation_counts, 10.0);
        assert_approximately_equal(
            measurement.world_forward_velocity_counts_per_second,
            0.0,
            TOLERANCE,
        );
        assert_approximately_equal(
            measurement.world_left_velocity_counts_per_second,
            500.0,
            TOLERANCE,
        );
    }

    #[test]
    fn positive_delta_y_at_zero_rotation_produces_positive_forward_velocity() {
        let input = FlowMeasurementInput {
            sample: RawOpticalFlowSample {
                motion_detected: true,
                delta_x_counts: 0,
                delta_y_counts: 10,
                surface_quality: 255,
            },
            attitude_estimate: AttitudeEstimate {
                pitch_deg: 0.0,
                roll_deg: 0.0,
                yaw_heading_deg: 0.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            flow_sample_dt_seconds: FLOW_SAMPLE_DT_SECONDS,
            estimated_height_mm: 180.0,
            tof_age_seconds: Some(0.0),
        };
        let mut processor = FlowMeasurementProcessor::new();
        let output = processor.process(input).expect("input should be valid");
        let measurement = output.measurement.expect("sample should be accepted");

        assert_eq!(output.body_forward_translation_counts, 10.0);
        assert_eq!(output.body_left_translation_counts, 0.0);
        assert_approximately_equal(
            measurement.world_forward_velocity_counts_per_second,
            500.0,
            TOLERANCE,
        );
        assert_approximately_equal(
            measurement.world_left_velocity_counts_per_second,
            0.0,
            TOLERANCE,
        );
    }

    #[test]
    fn invalid_process_input_does_not_clear_accumulated_rotation() {
        let pitch_dps = 100.0;
        let imu_dt_seconds = 0.0025;
        let mut processor = FlowMeasurementProcessor::new();
        let measured_body_rate_dps = BodyAngularRateDps {
            pitch_dps,
            roll_dps: 0.0,
            yaw_dps: 0.0,
        };
        processor
            .accumulate_body_rotation(measured_body_rate_dps, imu_dt_seconds)
            .expect("rotation input should be valid");

        let invalid_flow_sample_dt_seconds = 0.0;

        let input = FlowMeasurementInput {
            sample: RawOpticalFlowSample {
                motion_detected: true,
                delta_x_counts: 0,
                delta_y_counts: 10,
                surface_quality: 255,
            },
            attitude_estimate: AttitudeEstimate {
                pitch_deg: 0.0,
                roll_deg: 0.0,
                yaw_heading_deg: 0.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            flow_sample_dt_seconds: invalid_flow_sample_dt_seconds,
            estimated_height_mm: 180.0,
            tof_age_seconds: Some(0.0),
        };
        let output = processor.process(input);

        assert!(matches!(
            output,
            Err(FlowMeasurementError::FlowSampleDtOutOfRange)
        ));
        assert_eq!(processor.pitch_rotation_since_flow_sample_deg, 0.25);
        assert_eq!(processor.roll_rotation_since_flow_sample_deg, 0.0);
    }

    #[test]
    fn pure_pitch_rotation_must_not_be_mistaken_for_forward_movement() {
        let pitch_dps = 74.41;
        let imu_dt_seconds = 0.0025;
        let mut processor = FlowMeasurementProcessor::new();
        for _ in 0..8 {
            let measured_body_rate_dps = BodyAngularRateDps {
                pitch_dps,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            };
            processor
                .accumulate_body_rotation(measured_body_rate_dps, imu_dt_seconds)
                .expect("rotation input should be valid");
        }

        let input = FlowMeasurementInput {
            sample: RawOpticalFlowSample {
                motion_detected: true,
                delta_x_counts: 0,
                delta_y_counts: -10,
                surface_quality: 255,
            },
            attitude_estimate: AttitudeEstimate {
                pitch_deg: 1.488,
                roll_deg: 0.0,
                yaw_heading_deg: 0.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            flow_sample_dt_seconds: FLOW_SAMPLE_DT_SECONDS,
            estimated_height_mm: 180.0,
            tof_age_seconds: Some(0.0),
        };
        let output = processor.process(input).expect("should be valid");
        let measurement = output.measurement.expect("sample should be accepted");

        assert_approximately_equal(output.body_forward_rotation_counts, -10.0, TOLERANCE);
        assert_approximately_equal(output.body_forward_translation_counts, 0.0, TOLERANCE);
        assert_approximately_equal(
            measurement.world_forward_velocity_counts_per_second,
            0.0,
            0.001,
        );
        assert_eq!(processor.pitch_rotation_since_flow_sample_deg, 0.0);
        assert_eq!(processor.roll_rotation_since_flow_sample_deg, 0.0);
    }

    #[test]
    fn low_surface_quality_rejects_measurement() {
        let low_surface_quality = 39;
        let input = FlowMeasurementInput {
            sample: RawOpticalFlowSample {
                motion_detected: true,
                delta_x_counts: 0,
                delta_y_counts: 10,
                surface_quality: low_surface_quality,
            },
            attitude_estimate: AttitudeEstimate {
                pitch_deg: 0.0,
                roll_deg: 0.0,
                yaw_heading_deg: 0.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            flow_sample_dt_seconds: FLOW_SAMPLE_DT_SECONDS,
            estimated_height_mm: 180.0,
            tof_age_seconds: Some(0.0),
        };
        let mut processor = FlowMeasurementProcessor::new();
        let output = processor.process(input).expect("input should be valid");

        assert!(matches!(
            output.measurement,
            Err(FlowMeasurementRejection::LowSurfaceQuality)
        ));
    }

    #[test]
    fn higher_height_scales_forward_velocity() {
        let input = FlowMeasurementInput {
            sample: RawOpticalFlowSample {
                motion_detected: true,
                delta_x_counts: 0,
                delta_y_counts: 10,
                surface_quality: 255,
            },
            attitude_estimate: AttitudeEstimate {
                pitch_deg: 0.0,
                roll_deg: 0.0,
                yaw_heading_deg: 0.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            flow_sample_dt_seconds: FLOW_SAMPLE_DT_SECONDS,
            estimated_height_mm: 360.0,
            tof_age_seconds: Some(0.0),
        };
        let mut processor = FlowMeasurementProcessor::new();
        let output = processor.process(input).expect("input should be valid");
        let measurement = output.measurement.expect("sample should be accepted");

        assert_eq!(output.body_forward_translation_counts, 10.0);
        assert_eq!(output.body_left_translation_counts, 0.0);
        assert_approximately_equal(
            measurement.world_forward_velocity_counts_per_second,
            1000.0,
            TOLERANCE,
        );
        assert_approximately_equal(
            measurement.world_left_velocity_counts_per_second,
            0.0,
            TOLERANCE,
        );
    }

    #[test]
    fn forward_body_movement_should_become_world_left_movement() {
        let input = FlowMeasurementInput {
            sample: RawOpticalFlowSample {
                motion_detected: true,
                delta_x_counts: 0,
                delta_y_counts: 10,
                surface_quality: 255,
            },
            attitude_estimate: AttitudeEstimate {
                pitch_deg: 0.0,
                roll_deg: 0.0,
                yaw_heading_deg: 90.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            flow_sample_dt_seconds: FLOW_SAMPLE_DT_SECONDS,
            estimated_height_mm: 180.0,
            tof_age_seconds: Some(0.0),
        };
        let mut processor = FlowMeasurementProcessor::new();
        let output = processor.process(input).expect("input should be valid");
        let measurement = output.measurement.expect("sample should be accepted");

        assert_eq!(output.body_forward_translation_counts, 10.0);
        assert_eq!(output.body_left_translation_counts, 0.0);
        assert_approximately_equal(
            measurement.world_forward_velocity_counts_per_second,
            0.0,
            TOLERANCE,
        );
        assert_approximately_equal(
            measurement.world_left_velocity_counts_per_second,
            500.0,
            TOLERANCE,
        );
    }

    #[test]
    fn pmw_counts_at_deadzone_limit_produce_zero_velocity() {
        let input = FlowMeasurementInput {
            sample: RawOpticalFlowSample {
                motion_detected: true,
                delta_x_counts: 0,
                delta_y_counts: 3,
                surface_quality: 255,
            },
            attitude_estimate: AttitudeEstimate {
                pitch_deg: 0.0,
                roll_deg: 0.0,
                yaw_heading_deg: 0.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            flow_sample_dt_seconds: FLOW_SAMPLE_DT_SECONDS,
            estimated_height_mm: 180.0,
            tof_age_seconds: Some(0.0),
        };
        let mut processor = FlowMeasurementProcessor::new();
        let output = processor.process(input).expect("input should be valid");
        let measurement = output.measurement.expect("sample should be accepted");

        assert_eq!(output.body_forward_translation_counts, 3.0);
        assert_eq!(output.body_left_translation_counts, 0.0);
        assert_approximately_equal(
            measurement.world_forward_velocity_counts_per_second,
            0.0,
            TOLERANCE,
        );
        assert_approximately_equal(
            measurement.world_left_velocity_counts_per_second,
            0.0,
            TOLERANCE,
        );
    }

    #[test]
    fn bad_pmw_sample_must_not_create_impossible_velocity_spike() {
        let input = FlowMeasurementInput {
            sample: RawOpticalFlowSample {
                motion_detected: true,
                delta_x_counts: 0,
                delta_y_counts: 100,
                surface_quality: 255,
            },
            attitude_estimate: AttitudeEstimate {
                pitch_deg: 0.0,
                roll_deg: 0.0,
                yaw_heading_deg: 0.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            flow_sample_dt_seconds: FLOW_SAMPLE_DT_SECONDS,
            estimated_height_mm: 180.0,
            tof_age_seconds: Some(0.0),
        };
        let mut processor = FlowMeasurementProcessor::new();
        let output = processor.process(input).expect("input should be valid");
        let measurement = output.measurement.expect("sample should be accepted");

        assert_eq!(output.body_forward_translation_counts, 100.0);
        assert_eq!(output.body_left_translation_counts, 0.0);
        assert_approximately_equal(
            measurement.world_forward_velocity_counts_per_second,
            3000.0,
            TOLERANCE,
        );
        assert_approximately_equal(
            measurement.world_left_velocity_counts_per_second,
            0.0,
            TOLERANCE,
        );
        assert_eq!(output.body_forward_counts_were_limited, true);
        assert_eq!(output.body_left_counts_were_limited, false);
    }

    #[test]
    fn measurement_uncertainty_from_surface_quality() {
        let high_quality_surface_input = FlowMeasurementInput {
            sample: RawOpticalFlowSample {
                motion_detected: true,
                delta_x_counts: 0,
                delta_y_counts: 10,
                surface_quality: 255,
            },
            attitude_estimate: AttitudeEstimate {
                pitch_deg: 0.0,
                roll_deg: 0.0,
                yaw_heading_deg: 0.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            flow_sample_dt_seconds: FLOW_SAMPLE_DT_SECONDS,
            estimated_height_mm: 180.0,
            tof_age_seconds: Some(0.0),
        };
        let mut processor = FlowMeasurementProcessor::new();
        let output = processor
            .process(high_quality_surface_input)
            .expect("input should be valid");
        let high_quality_surface_measurement =
            output.measurement.expect("sample should be accepted");

        assert_approximately_equal(
            high_quality_surface_measurement.world_forward_velocity_counts_per_second,
            500.0,
            TOLERANCE,
        );
        assert_approximately_equal(
            high_quality_surface_measurement.world_left_velocity_counts_per_second,
            0.0,
            TOLERANCE,
        );

        let low_quality_surface_input = FlowMeasurementInput {
            sample: RawOpticalFlowSample {
                motion_detected: true,
                delta_x_counts: 0,
                delta_y_counts: 10,
                surface_quality: 40,
            },
            attitude_estimate: AttitudeEstimate {
                pitch_deg: 0.0,
                roll_deg: 0.0,
                yaw_heading_deg: 0.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            flow_sample_dt_seconds: FLOW_SAMPLE_DT_SECONDS,
            estimated_height_mm: 180.0,
            tof_age_seconds: Some(0.0),
        };
        let mut processor = FlowMeasurementProcessor::new();
        let output = processor
            .process(low_quality_surface_input)
            .expect("input should be valid");
        let low_quality_surface_measurement =
            output.measurement.expect("sample should be accepted");

        assert_approximately_equal(
            low_quality_surface_measurement.world_forward_velocity_counts_per_second,
            500.0,
            TOLERANCE,
        );
        assert_approximately_equal(
            low_quality_surface_measurement.world_left_velocity_counts_per_second,
            0.0,
            TOLERANCE,
        );

        assert!(
            low_quality_surface_measurement.velocity_variance_counts_squared_per_second_squared
                > high_quality_surface_measurement
                    .velocity_variance_counts_squared_per_second_squared
        );
    }
}
