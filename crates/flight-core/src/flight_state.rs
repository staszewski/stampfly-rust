//! Explicit flight qualification, activation, recovery, and cutoff state.

use crate::types::{AttitudeEstimate, BodyAngularRateDps, CollectiveMode, CutoffReason};

const INITIAL_ATTITUDE_GATE_SAMPLES: u16 = 100;
const ALTITUDE_REARM_GATE_SAMPLES: u16 = 40;

const ATTITUDE_GATE_ENABLE_MAX_ANGLE_DEG: f32 = 10.0;
const ATTITUDE_GATE_MAX_RATE_DPS: f32 = 175.0;
const ATTITUDE_GATE_DISABLE_MAX_ANGLE_DEG: f32 = 12.0;

pub struct FlightState {
    attitude_stable_samples: u16,
    altitude_hold_has_activated: bool,
    altitude_control_qualified: bool,
}

pub struct FlightStateInput {
    pub motor_output_authorized: bool,
    pub cutoff_reason: CutoffReason,
    pub altitude_controller_active: bool,
    pub horizontal_control_enabled: bool,
    pub horizontal_controller_active: bool,
    pub base_thrust_ramp_complete: bool,
    pub attitude_estimate: AttitudeEstimate,
    pub measured_body_rate_dps: BodyAngularRateDps,
}

pub struct FlightStateOutput {
    pub collective_mode: CollectiveMode,
    pub attitude_qualified: bool,
    pub horizontal_control_authorized: bool,
    pub altitude_integral_learning_enabled: bool,
}

impl FlightState {
    pub fn new() -> Self {
        Self {
            attitude_stable_samples: 0,
            altitude_hold_has_activated: false,
            altitude_control_qualified: false,
        }
    }

    pub fn update(&mut self, input: FlightStateInput) -> FlightStateOutput {
        if !input.motor_output_authorized || input.cutoff_reason != CutoffReason::None {
            self.attitude_stable_samples = 0;
            self.altitude_hold_has_activated = false;
            self.altitude_control_qualified = false;
            return FlightStateOutput {
                collective_mode: CollectiveMode::Disabled,
                attitude_qualified: false,
                altitude_integral_learning_enabled: false,
                horizontal_control_authorized: false,
            };
        }

        let angle_limit_deg = if self.altitude_control_qualified {
            ATTITUDE_GATE_DISABLE_MAX_ANGLE_DEG
        } else {
            ATTITUDE_GATE_ENABLE_MAX_ANGLE_DEG
        };

        let required_attitude_gate_samples = if self.altitude_hold_has_activated {
            ALTITUDE_REARM_GATE_SAMPLES
        } else {
            INITIAL_ATTITUDE_GATE_SAMPLES
        };

        let attitude_qualified = Self::is_stable(&input, angle_limit_deg);
        if attitude_qualified {
            self.attitude_stable_samples = self
                .attitude_stable_samples
                .saturating_add(1)
                .min(required_attitude_gate_samples);
        } else {
            self.attitude_stable_samples = 0;
            self.altitude_control_qualified = false;
        }

        if self.attitude_stable_samples >= required_attitude_gate_samples {
            self.altitude_control_qualified = true;
        }

        let altitude_integral_learning_enabled =
            self.altitude_control_qualified && input.base_thrust_ramp_complete;

        if !input.altitude_controller_active
            || (self.attitude_stable_samples < required_attitude_gate_samples
                && !self.altitude_hold_has_activated)
        {
            return FlightStateOutput {
                collective_mode: CollectiveMode::BaseThrust,
                attitude_qualified,
                altitude_integral_learning_enabled,
                horizontal_control_authorized: false,
            };
        } else {
            self.altitude_hold_has_activated = true;
            return FlightStateOutput {
                collective_mode: CollectiveMode::AltitudeHold,
                attitude_qualified,
                altitude_integral_learning_enabled,
                horizontal_control_authorized: self.altitude_control_qualified
                    && input.horizontal_control_enabled
                    && input.base_thrust_ramp_complete
                    && input.horizontal_controller_active,
            };
        }
    }

    fn is_stable(input: &FlightStateInput, angle_limit_deg: f32) -> bool {
        input.attitude_estimate.pitch_deg.abs() <= angle_limit_deg
            && input.attitude_estimate.roll_deg.abs() <= angle_limit_deg
            && input.measured_body_rate_dps.pitch_dps.abs() <= ATTITUDE_GATE_MAX_RATE_DPS
            && input.measured_body_rate_dps.roll_dps.abs() <= ATTITUDE_GATE_MAX_RATE_DPS
            && input.measured_body_rate_dps.yaw_dps.abs() <= ATTITUDE_GATE_MAX_RATE_DPS
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unauthorized_state_inputs_selects_disabled_mode() {
        let test_cases = [
            (FlightStateInput {
                motor_output_authorized: false,
                cutoff_reason: CutoffReason::None,
                altitude_controller_active: true,
                base_thrust_ramp_complete: false,
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
                horizontal_control_enabled: true,
                horizontal_controller_active: false,
            }),
            (FlightStateInput {
                motor_output_authorized: true,
                cutoff_reason: CutoffReason::PitchAngle,
                altitude_controller_active: true,
                base_thrust_ramp_complete: false,
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
                horizontal_control_enabled: true,
                horizontal_controller_active: false,
            }),
        ];
        for input in test_cases {
            let mut flight_state = FlightState::new();
            let result = flight_state.update(input);
            assert_eq!(result.collective_mode, CollectiveMode::Disabled);
        }
    }

    #[test]
    fn initial_attitude_qualification_requires_100_stable_samples() {
        let mut flight_state = FlightState::new();

        let input = FlightStateInput {
            motor_output_authorized: true,
            cutoff_reason: CutoffReason::None,
            altitude_controller_active: true,
            base_thrust_ramp_complete: true,
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
            horizontal_control_enabled: true,
            horizontal_controller_active: false,
        };

        let result = flight_state.update(input);
        assert_eq!(result.collective_mode, CollectiveMode::BaseThrust);

        let unstable_input = FlightStateInput {
            motor_output_authorized: true,
            cutoff_reason: CutoffReason::None,
            altitude_controller_active: true,
            base_thrust_ramp_complete: true,
            attitude_estimate: AttitudeEstimate {
                pitch_deg: 11.0,
                roll_deg: 0.0,
                yaw_heading_deg: 0.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            horizontal_control_enabled: true,
            horizontal_controller_active: false,
        };

        let result = flight_state.update(unstable_input);
        assert_eq!(result.collective_mode, CollectiveMode::BaseThrust);

        for _ in 0..(INITIAL_ATTITUDE_GATE_SAMPLES - 1) {
            let input = FlightStateInput {
                motor_output_authorized: true,
                cutoff_reason: CutoffReason::None,
                altitude_controller_active: true,
                base_thrust_ramp_complete: true,
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
                horizontal_control_enabled: true,
                horizontal_controller_active: false,
            };

            let result = flight_state.update(input);
            assert_eq!(result.collective_mode, CollectiveMode::BaseThrust);
        }

        let input = FlightStateInput {
            motor_output_authorized: true,
            cutoff_reason: CutoffReason::None,
            altitude_controller_active: true,
            base_thrust_ramp_complete: true,
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
            horizontal_control_enabled: true,
            horizontal_controller_active: true,
        };
        let result = flight_state.update(input);
        assert_eq!(result.collective_mode, CollectiveMode::AltitudeHold);
        assert_eq!(result.attitude_qualified, true);
        assert_eq!(result.horizontal_control_authorized, true);

        let input = FlightStateInput {
            motor_output_authorized: true,
            cutoff_reason: CutoffReason::None,
            altitude_controller_active: true,
            horizontal_control_enabled: false,
            horizontal_controller_active: true,
            base_thrust_ramp_complete: true,
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
        };
        let result = flight_state.update(input);
        assert_eq!(result.collective_mode, CollectiveMode::AltitudeHold);
        assert_eq!(result.horizontal_control_authorized, false);

        let input = FlightStateInput {
            motor_output_authorized: true,
            cutoff_reason: CutoffReason::None,
            altitude_controller_active: true,
            base_thrust_ramp_complete: true,
            attitude_estimate: AttitudeEstimate {
                pitch_deg: 11.0,
                roll_deg: 0.0,
                yaw_heading_deg: 0.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            horizontal_control_enabled: true,
            horizontal_controller_active: true,
        };
        let result = flight_state.update(input);
        assert_eq!(result.collective_mode, CollectiveMode::AltitudeHold);
        assert_eq!(result.attitude_qualified, true);
        assert_eq!(result.horizontal_control_authorized, true);

        let unstable_input = FlightStateInput {
            motor_output_authorized: true,
            cutoff_reason: CutoffReason::None,
            altitude_controller_active: true,
            base_thrust_ramp_complete: true,
            attitude_estimate: AttitudeEstimate {
                pitch_deg: 15.0,
                roll_deg: 0.0,
                yaw_heading_deg: 0.0,
            },
            measured_body_rate_dps: BodyAngularRateDps {
                pitch_dps: 0.0,
                roll_dps: 0.0,
                yaw_dps: 0.0,
            },
            horizontal_control_enabled: true,
            horizontal_controller_active: true,
        };
        let result = flight_state.update(unstable_input);
        assert_eq!(result.collective_mode, CollectiveMode::AltitudeHold);
        assert_eq!(result.attitude_qualified, false);
        assert_eq!(result.altitude_integral_learning_enabled, false);

        for _ in 0..(ALTITUDE_REARM_GATE_SAMPLES - 1) {
            let input = FlightStateInput {
                motor_output_authorized: true,
                cutoff_reason: CutoffReason::None,
                altitude_controller_active: true,
                base_thrust_ramp_complete: true,
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
                horizontal_control_enabled: true,
                horizontal_controller_active: true,
            };

            let result = flight_state.update(input);
            assert_eq!(result.collective_mode, CollectiveMode::AltitudeHold);
            assert_eq!(result.attitude_qualified, true);
            assert_eq!(result.altitude_integral_learning_enabled, false);
        }

        let input = FlightStateInput {
            motor_output_authorized: true,
            cutoff_reason: CutoffReason::None,
            altitude_controller_active: true,
            base_thrust_ramp_complete: true,
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
            horizontal_control_enabled: true,
            horizontal_controller_active: true,
        };

        let result = flight_state.update(input);
        assert_eq!(result.collective_mode, CollectiveMode::AltitudeHold);
        assert_eq!(result.attitude_qualified, true);
        assert_eq!(result.altitude_integral_learning_enabled, true);
    }
}
