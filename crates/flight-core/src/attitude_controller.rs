//! Cascaded attitude-angle, body-rate, and yaw-heading control.

use crate::{
    math::wrap_degrees,
    pid::{RatePid, RatePidError},
    types::PidOutput,
};

const PITCH_RATE_KP: f32 = 0.45;
const ROLL_RATE_KP: f32 = 0.30;
const PITCH_RATE_KI: f32 = 0.04;
const ROLL_RATE_KI: f32 = 0.04;
const PITCH_RATE_KD: f32 = 0.00040;
const ROLL_RATE_KD: f32 = 0.00040;
const YAW_RATE_KP: f32 = 0.45;
const YAW_RATE_KI: f32 = 0.04;
const YAW_RATE_KD: f32 = 0.0;

const RATE_CORRECTION_LIMIT: f32 = 30.0;
const RATE_INTEGRAL_LIMIT: f32 = 8.0;

const PITCH_ANGLE_KP: f32 = 4.0;
const ROLL_ANGLE_KP: f32 = 4.0;
const YAW_HEADING_KP: f32 = 1.0;
const MAX_ANGLE_RATE_TARGET_DPS: f32 = 100.0;
const MAX_YAW_RATE_TARGET_DPS: f32 = 30.0;

const ATTITUDE_HIGH_ERROR_BRAKING_GAIN: f32 = 0.30;

pub struct AttitudeController {
    pitch_rate_pid: RatePid,
    roll_rate_pid: RatePid,
    yaw_rate_pid: RatePid,
}

pub struct AttitudeControllerInput {
    pub target_pitch_deg: f32,
    pub target_roll_deg: f32,
    pub target_yaw_heading_deg: f32,
    pub measured_pitch_deg: f32,
    pub measured_roll_deg: f32,
    pub measured_yaw_heading_deg: f32,
    pub measured_pitch_rate_dps: f32,
    pub measured_roll_rate_dps: f32,
    pub measured_yaw_rate_dps: f32,
    pub dt_seconds: f32,
}

#[derive(Debug, PartialEq, Eq)]
pub enum AttitudeAxis {
    Pitch,
    Roll,
    Yaw,
}

#[derive(Debug, PartialEq, Eq)]
pub enum AttitudeControllerError {
    InvalidConfig {
        axis: AttitudeAxis,
        source: RatePidError,
    },
    RatePidUpdate {
        axis: AttitudeAxis,
        source: RatePidError,
    },
}

impl AttitudeController {
    pub fn new() -> Result<Self, AttitudeControllerError> {
        let pitch_rate_pid = match RatePid::new(
            PITCH_RATE_KP,
            PITCH_RATE_KI,
            PITCH_RATE_KD,
            RATE_CORRECTION_LIMIT,
            RATE_INTEGRAL_LIMIT,
            ATTITUDE_HIGH_ERROR_BRAKING_GAIN,
        ) {
            Ok(pid) => pid,
            Err(err) => {
                return Err(AttitudeControllerError::InvalidConfig {
                    axis: AttitudeAxis::Pitch,
                    source: err,
                });
            }
        };
        let roll_rate_pid = match RatePid::new(
            ROLL_RATE_KP,
            ROLL_RATE_KI,
            ROLL_RATE_KD,
            RATE_CORRECTION_LIMIT,
            RATE_INTEGRAL_LIMIT,
            ATTITUDE_HIGH_ERROR_BRAKING_GAIN,
        ) {
            Ok(pid) => pid,
            Err(err) => {
                return Err(AttitudeControllerError::InvalidConfig {
                    axis: AttitudeAxis::Roll,
                    source: err,
                });
            }
        };
        let yaw_rate_pid = match RatePid::new(
            YAW_RATE_KP,
            YAW_RATE_KI,
            YAW_RATE_KD,
            RATE_CORRECTION_LIMIT,
            RATE_INTEGRAL_LIMIT,
            0.0,
        ) {
            Ok(pid) => pid,
            Err(err) => {
                return Err(AttitudeControllerError::InvalidConfig {
                    axis: AttitudeAxis::Yaw,
                    source: err,
                });
            }
        };

        Ok(Self {
            pitch_rate_pid,
            roll_rate_pid,
            yaw_rate_pid,
        })
    }

    pub fn update(
        &mut self,
        input: AttitudeControllerInput,
    ) -> Result<PidOutput, AttitudeControllerError> {
        let pitch_error = input.target_pitch_deg - input.measured_pitch_deg;
        let roll_error = input.target_roll_deg - input.measured_roll_deg;
        let yaw_error = wrap_degrees(input.target_yaw_heading_deg - input.measured_yaw_heading_deg);
        let pitch_rate_target_dps = (PITCH_ANGLE_KP * pitch_error)
            .clamp(-MAX_ANGLE_RATE_TARGET_DPS, MAX_ANGLE_RATE_TARGET_DPS);
        let roll_rate_target_dps = (ROLL_ANGLE_KP * roll_error)
            .clamp(-MAX_ANGLE_RATE_TARGET_DPS, MAX_ANGLE_RATE_TARGET_DPS);
        let yaw_rate_target_dps =
            (YAW_HEADING_KP * yaw_error).clamp(-MAX_YAW_RATE_TARGET_DPS, MAX_YAW_RATE_TARGET_DPS);
        let pitch_rate_error_dps = pitch_rate_target_dps - input.measured_pitch_rate_dps;
        let roll_rate_error_dps = roll_rate_target_dps - input.measured_roll_rate_dps;
        let mut candidate_pitch_rate_pid = self.pitch_rate_pid.clone();
        let mut candidate_roll_rate_pid = self.roll_rate_pid.clone();
        let mut candidate_yaw_rate_pid = self.yaw_rate_pid.clone();
        let pitch_correction = match candidate_pitch_rate_pid.update(
            pitch_rate_target_dps,
            input.measured_pitch_rate_dps,
            input.dt_seconds,
        ) {
            Ok(correction) => correction,
            Err(err) => {
                return Err(AttitudeControllerError::RatePidUpdate {
                    axis: AttitudeAxis::Pitch,
                    source: err,
                });
            }
        };
        let roll_correction = match candidate_roll_rate_pid.update(
            roll_rate_target_dps,
            input.measured_roll_rate_dps,
            input.dt_seconds,
        ) {
            Ok(correction) => correction,
            Err(err) => {
                return Err(AttitudeControllerError::RatePidUpdate {
                    axis: AttitudeAxis::Roll,
                    source: err,
                });
            }
        };

        let yaw_correction = match candidate_yaw_rate_pid.update(
            yaw_rate_target_dps,
            input.measured_yaw_rate_dps,
            input.dt_seconds,
        ) {
            Ok(correction) => correction,
            Err(err) => {
                return Err(AttitudeControllerError::RatePidUpdate {
                    axis: AttitudeAxis::Yaw,
                    source: err,
                });
            }
        };

        self.pitch_rate_pid = candidate_pitch_rate_pid;
        self.roll_rate_pid = candidate_roll_rate_pid;
        self.yaw_rate_pid = candidate_yaw_rate_pid;
        Ok(PidOutput {
            pitch_correction,
            roll_correction,
            yaw_correction: -yaw_correction,
            pitch_rate_integral_correction: self.pitch_rate_pid.integral_correction(),
            roll_rate_integral_correction: self.roll_rate_pid.integral_correction(),
            yaw_rate_integral_correction: -self.yaw_rate_pid.integral_correction(),
            pitch_rate_target_dps,
            roll_rate_target_dps,
            pitch_rate_error_dps,
            roll_rate_error_dps,
        })
    }
}

#[cfg(test)]
mod tests {
    use core::f32;

    use super::*;

    const DT: f32 = 0.0025;
    const TOLERANCE: f32 = 0.00001;
    #[test]
    fn pitch_angle_error_drives_only_pitch_axis() {
        let mut attitude_controller =
            AttitudeController::new().expect("should be constructed with no errors");
        let input = AttitudeControllerInput {
            target_pitch_deg: 5.0,
            target_roll_deg: 0.0,
            target_yaw_heading_deg: 0.0,
            measured_pitch_deg: 0.0,
            measured_roll_deg: 0.0,
            measured_yaw_heading_deg: 0.0,
            measured_pitch_rate_dps: 0.0,
            measured_roll_rate_dps: 0.0,
            measured_yaw_rate_dps: 0.0,
            dt_seconds: DT,
        };
        let output = attitude_controller
            .update(input)
            .expect("should be valid input to update");
        let expected = PidOutput {
            pitch_correction: 9.002,
            roll_correction: 0.0,
            yaw_correction: 0.0,
            pitch_rate_integral_correction: 0.002,
            roll_rate_integral_correction: 0.0,
            yaw_rate_integral_correction: 0.0,
            pitch_rate_target_dps: 20.0,
            roll_rate_target_dps: 0.0,
            pitch_rate_error_dps: 20.0,
            roll_rate_error_dps: 0.0,
        };

        assert!((output.pitch_correction - expected.pitch_correction).abs() <= TOLERANCE);
        assert_eq!(output.pitch_rate_target_dps, expected.pitch_rate_target_dps);
        assert_eq!(output.pitch_rate_error_dps, expected.pitch_rate_error_dps);
        assert!(
            (output.pitch_rate_integral_correction - expected.pitch_rate_integral_correction).abs()
                <= TOLERANCE
        );

        assert_eq!(output.roll_correction, expected.roll_correction);
        assert_eq!(output.yaw_correction, expected.yaw_correction);
        assert_eq!(
            output.roll_rate_integral_correction,
            expected.roll_rate_integral_correction
        );
        assert_eq!(
            output.yaw_rate_integral_correction,
            expected.yaw_rate_integral_correction
        );
        assert_eq!(output.roll_rate_target_dps, expected.roll_rate_target_dps);
        assert_eq!(output.roll_rate_error_dps, expected.roll_rate_error_dps);
    }

    #[test]
    fn roll_angle_error_drives_only_roll_axis() {
        let mut attitude_controller =
            AttitudeController::new().expect("should be constructed with no errors");
        let input = AttitudeControllerInput {
            target_pitch_deg: 0.0,
            target_roll_deg: 5.0,
            target_yaw_heading_deg: 0.0,
            measured_pitch_deg: 0.0,
            measured_roll_deg: 0.0,
            measured_yaw_heading_deg: 0.0,
            measured_pitch_rate_dps: 0.0,
            measured_roll_rate_dps: 0.0,
            measured_yaw_rate_dps: 0.0,
            dt_seconds: DT,
        };
        let output = attitude_controller
            .update(input)
            .expect("should be valid input to update");
        let expected = PidOutput {
            pitch_correction: 0.0,
            roll_correction: 6.002,
            yaw_correction: 0.0,
            pitch_rate_integral_correction: 0.0,
            roll_rate_integral_correction: 0.002,
            yaw_rate_integral_correction: 0.0,
            pitch_rate_target_dps: 0.0,
            roll_rate_target_dps: 20.0,
            pitch_rate_error_dps: 0.0,
            roll_rate_error_dps: 20.0,
        };

        assert!((output.roll_correction - expected.roll_correction).abs() <= TOLERANCE);
        assert_eq!(output.roll_rate_target_dps, expected.roll_rate_target_dps);
        assert_eq!(output.roll_rate_error_dps, expected.roll_rate_error_dps);
        assert!(
            (output.roll_rate_integral_correction - expected.roll_rate_integral_correction).abs()
                <= TOLERANCE
        );
    }

    #[test]
    fn failed_roll_update_does_not_commit_pid_state() {
        let mut attitude_controller =
            AttitudeController::new().expect("should be constructed with no errors");
        let roll_failure_input = AttitudeControllerInput {
            target_pitch_deg: 5.0,
            target_roll_deg: 5.0,
            target_yaw_heading_deg: 0.0,
            measured_pitch_deg: 0.0,
            measured_roll_deg: 0.0,
            measured_yaw_heading_deg: 0.0,
            measured_pitch_rate_dps: 0.0,
            measured_roll_rate_dps: f32::NAN,
            measured_yaw_rate_dps: 0.0,
            dt_seconds: DT,
        };
        let roll_failure_result = attitude_controller.update(roll_failure_input);
        assert!(matches!(
            roll_failure_result,
            Err(AttitudeControllerError::RatePidUpdate {
                axis: AttitudeAxis::Roll,
                source: RatePidError::NonFiniteInput
            })
        ));

        let next_valid_output = AttitudeControllerInput {
            target_pitch_deg: 5.0,
            target_roll_deg: 5.0,
            target_yaw_heading_deg: 0.0,
            measured_pitch_deg: 0.0,
            measured_roll_deg: 0.0,
            measured_yaw_heading_deg: 0.0,
            measured_pitch_rate_dps: 0.0,
            measured_roll_rate_dps: 0.0,
            measured_yaw_rate_dps: 0.0,
            dt_seconds: DT,
        };
        let next_valid_result = attitude_controller
            .update(next_valid_output)
            .expect("should be valid input to update");
        let expected_integral_correction = 0.002;
        assert!(
            (next_valid_result.pitch_rate_integral_correction - expected_integral_correction).abs()
                <= TOLERANCE
        );
        assert!(
            (next_valid_result.roll_rate_integral_correction - expected_integral_correction).abs()
                <= TOLERANCE
        );
    }

    #[test]
    fn positive_yaw_heading_produces_positive_braking_correction() {
        let mut attitude_controller =
            AttitudeController::new().expect("should be constructed with no errors");
        let input = AttitudeControllerInput {
            target_pitch_deg: 0.0,
            target_roll_deg: 0.0,
            target_yaw_heading_deg: 0.0,
            measured_pitch_deg: 0.0,
            measured_roll_deg: 0.0,
            measured_yaw_heading_deg: 10.0,
            measured_pitch_rate_dps: 0.0,
            measured_roll_rate_dps: 0.0,
            measured_yaw_rate_dps: 0.0,
            dt_seconds: DT,
        };
        let output = attitude_controller
            .update(input)
            .expect("should be valid input to update");
        let expected = PidOutput {
            pitch_correction: 0.0,
            roll_correction: 0.0,
            yaw_correction: 4.501,
            pitch_rate_integral_correction: 0.0,
            roll_rate_integral_correction: 0.0,
            yaw_rate_integral_correction: 0.001,
            pitch_rate_target_dps: 0.0,
            roll_rate_target_dps: 0.0,
            pitch_rate_error_dps: 0.0,
            roll_rate_error_dps: 0.0,
        };

        assert!((output.yaw_correction - expected.yaw_correction).abs() <= TOLERANCE);
        assert!(
            (output.yaw_rate_integral_correction - expected.yaw_rate_integral_correction).abs()
                <= TOLERANCE
        );
    }

    #[test]
    fn yaw_heading_wrap_uses_shortest_rotation() {
        let mut attitude_controller =
            AttitudeController::new().expect("should be constructed with no errors");
        let input = AttitudeControllerInput {
            target_pitch_deg: 0.0,
            target_roll_deg: 0.0,
            target_yaw_heading_deg: -179.0,
            measured_pitch_deg: 0.0,
            measured_roll_deg: 0.0,
            measured_yaw_heading_deg: 179.0,
            measured_pitch_rate_dps: 0.0,
            measured_roll_rate_dps: 0.0,
            measured_yaw_rate_dps: 0.0,
            dt_seconds: DT,
        };
        let output = attitude_controller
            .update(input)
            .expect("should be valid input to update");
        let expected = PidOutput {
            pitch_correction: 0.0,
            roll_correction: 0.0,
            yaw_correction: -0.9002,
            pitch_rate_integral_correction: 0.0,
            roll_rate_integral_correction: 0.0,
            yaw_rate_integral_correction: -0.0002,
            pitch_rate_target_dps: 0.0,
            roll_rate_target_dps: 0.0,
            pitch_rate_error_dps: 0.0,
            roll_rate_error_dps: 0.0,
        };

        assert!((output.yaw_correction - expected.yaw_correction).abs() <= TOLERANCE);
        assert!(
            (output.yaw_rate_integral_correction - expected.yaw_rate_integral_correction).abs()
                <= TOLERANCE
        );
    }

    #[test]
    fn pitch_angle_requests_limits_rate_target_and_correction() {
        let mut attitude_controller =
            AttitudeController::new().expect("should be constructed with no errors");
        let input = AttitudeControllerInput {
            target_pitch_deg: 100.0,
            target_roll_deg: 0.0,
            target_yaw_heading_deg: 0.0,
            measured_pitch_deg: 0.0,
            measured_roll_deg: 0.0,
            measured_yaw_heading_deg: 0.0,
            measured_pitch_rate_dps: 0.0,
            measured_roll_rate_dps: 0.0,
            measured_yaw_rate_dps: 0.0,
            dt_seconds: DT,
        };
        let output = attitude_controller
            .update(input)
            .expect("should be valid input to update");
        let expected = PidOutput {
            pitch_correction: 30.0,
            roll_correction: 0.0,
            yaw_correction: 0.0,
            pitch_rate_integral_correction: 0.0,
            roll_rate_integral_correction: 0.0,
            yaw_rate_integral_correction: 0.0,
            pitch_rate_target_dps: 100.0,
            roll_rate_target_dps: 0.0,
            pitch_rate_error_dps: 100.0,
            roll_rate_error_dps: 0.0,
        };
        assert_eq!(output, expected);
    }
}
