//! Cascaded horizontal position, velocity, and attitude-target control.

use crate::horizontal_estimator::HorizontalEstimate;

pub struct HorizontalControllerInput {
    pub horizontal_estimate: HorizontalEstimate,
    pub yaw_heading_deg: f32,
}

#[derive(Debug, PartialEq)]
pub struct HorizontalControllerOutput {
    pub active: bool,
    pub target_pitch_deg: f32,
    pub target_roll_deg: f32,
}

#[derive(Debug, PartialEq)]
pub enum HorizontalControllerError {
    NonFiniteInput,
    NonFiniteOutput,
    InvalidAge,
}

pub struct HorizontalController {}

const POSITION_TO_VELOCITY_GAIN_PER_SECOND: f32 = 0.4;
// (0.180 / 385) × (3.0 / 9.80665) × (180 / PI) ≈ 0.0082
const VELOCITY_ERROR_TO_ANGLE_GAIN_DEG_PER_COUNT_PER_SECOND: f32 = 0.0082;

const MAX_HORIZONTAL_MEASUREMENT_AGE_SECONDS: f32 = 0.25;
const MAX_POSITION_VELOCITY_TARGET_COUNTS_PER_SECOND: f32 = 600.0;
const MAX_HORIZONTAL_TARGET_DEG: f32 = 8.0;

impl HorizontalController {
    pub fn new() -> Self {
        Self {}
    }

    pub fn update(
        &self,
        input: HorizontalControllerInput,
    ) -> Result<HorizontalControllerOutput, HorizontalControllerError> {
        Self::validate_input(&input)?;

        if let Some(measurement_age_seconds) = input.horizontal_estimate.measurement_age_seconds {
            if !measurement_age_seconds.is_finite() || measurement_age_seconds < 0.0 {
                return Err(HorizontalControllerError::InvalidAge);
            }

            if measurement_age_seconds > MAX_HORIZONTAL_MEASUREMENT_AGE_SECONDS {
                return Ok(Self::inactive_output());
            }
        } else {
            return Ok(Self::inactive_output());
        }

        let world_forward_velocity_target_counts_per_second =
            (-POSITION_TO_VELOCITY_GAIN_PER_SECOND
                * input.horizontal_estimate.world_forward_position_counts)
                .clamp(
                    -MAX_POSITION_VELOCITY_TARGET_COUNTS_PER_SECOND,
                    MAX_POSITION_VELOCITY_TARGET_COUNTS_PER_SECOND,
                );
        let world_left_velocity_target_counts_per_second = (-POSITION_TO_VELOCITY_GAIN_PER_SECOND
            * input.horizontal_estimate.world_left_position_counts)
            .clamp(
                -MAX_POSITION_VELOCITY_TARGET_COUNTS_PER_SECOND,
                MAX_POSITION_VELOCITY_TARGET_COUNTS_PER_SECOND,
            );

        let world_forward_velocity_error = world_forward_velocity_target_counts_per_second
            - input
                .horizontal_estimate
                .world_forward_velocity_counts_per_second;
        let world_left_velocity_error = world_left_velocity_target_counts_per_second
            - input
                .horizontal_estimate
                .world_left_velocity_counts_per_second;

        let world_pitch_target_deg =
            VELOCITY_ERROR_TO_ANGLE_GAIN_DEG_PER_COUNT_PER_SECOND * world_forward_velocity_error;
        let world_roll_target_deg =
            VELOCITY_ERROR_TO_ANGLE_GAIN_DEG_PER_COUNT_PER_SECOND * world_left_velocity_error;

        let magnitude = world_pitch_target_deg.hypot(world_roll_target_deg);
        let scale = if magnitude <= MAX_HORIZONTAL_TARGET_DEG {
            1.0
        } else {
            MAX_HORIZONTAL_TARGET_DEG / magnitude
        };

        let scaled_target_pitch_deg = world_pitch_target_deg * scale;
        let scaled_target_roll_deg = world_roll_target_deg * scale;
        let yaw_heading_rad = input.yaw_heading_deg.to_radians();
        let (sin_yaw, cos_yaw) = yaw_heading_rad.sin_cos();
        let target_pitch_deg = cos_yaw * scaled_target_pitch_deg - sin_yaw * scaled_target_roll_deg;
        let target_roll_deg = sin_yaw * scaled_target_pitch_deg + cos_yaw * scaled_target_roll_deg;

        if !target_pitch_deg.is_finite() || !target_roll_deg.is_finite() {
            return Err(HorizontalControllerError::NonFiniteOutput);
        }

        Ok(HorizontalControllerOutput {
            active: true,
            target_pitch_deg,
            target_roll_deg,
        })
    }

    fn inactive_output() -> HorizontalControllerOutput {
        HorizontalControllerOutput {
            active: false,
            target_pitch_deg: 0.0,
            target_roll_deg: 0.0,
        }
    }

    fn validate_input(input: &HorizontalControllerInput) -> Result<(), HorizontalControllerError> {
        if !input
            .horizontal_estimate
            .world_forward_position_counts
            .is_finite()
            || !input
                .horizontal_estimate
                .world_left_position_counts
                .is_finite()
            || !input
                .horizontal_estimate
                .world_forward_velocity_counts_per_second
                .is_finite()
            || !input
                .horizontal_estimate
                .world_left_velocity_counts_per_second
                .is_finite()
            || !input.yaw_heading_deg.is_finite()
        {
            return Err(HorizontalControllerError::NonFiniteInput);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f32 = 0.001;

    #[test]
    fn positive_forward_and_left_drifts_request_opposing_targets_at_zero_yaw() {
        let input = HorizontalControllerInput {
            horizontal_estimate: HorizontalEstimate {
                world_forward_position_counts: 100.0,
                world_left_position_counts: 0.0,
                world_forward_velocity_counts_per_second: 20.0,
                world_left_velocity_counts_per_second: 0.0,
                measurement_age_seconds: Some(0.0),
            },
            yaw_heading_deg: 0.0,
        };

        let controller = HorizontalController::new();
        let result = controller.update(input).expect("to be valid");
        let actual = result.target_pitch_deg;
        let expected = -0.492;

        let difference = (actual - expected).abs();

        assert_eq!(result.active, true);
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected}, difference={difference},
      tolerance={TOLERANCE}"
        );
        assert_eq!(result.target_roll_deg, 0.0);

        let input = HorizontalControllerInput {
            horizontal_estimate: HorizontalEstimate {
                world_forward_position_counts: 0.0,
                world_left_position_counts: 100.0,
                world_forward_velocity_counts_per_second: 0.0,
                world_left_velocity_counts_per_second: 20.0,
                measurement_age_seconds: Some(0.0),
            },
            yaw_heading_deg: 0.0,
        };

        let controller = HorizontalController::new();
        let result = controller.update(input).expect("to be valid");
        let actual = result.target_roll_deg;
        let expected = -0.492;

        let difference = (actual - expected).abs();

        assert_eq!(result.active, true);
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected}, difference={difference},
      tolerance={TOLERANCE}"
        );
        assert_eq!(result.target_pitch_deg, 0.0);
    }

    #[test]
    fn diagonal_limiting_preserves_direction() {
        let input = HorizontalControllerInput {
            horizontal_estimate: HorizontalEstimate {
                world_forward_position_counts: 0.0,
                world_left_position_counts: 0.0,
                world_forward_velocity_counts_per_second: -2000.0,
                world_left_velocity_counts_per_second: -2000.0,
                measurement_age_seconds: Some(0.0),
            },
            yaw_heading_deg: 0.0,
        };

        let controller = HorizontalController::new();
        let result = controller.update(input).expect("to be valid");
        let expected_deg = 5.657;
        let actual = result.target_pitch_deg;
        let difference = (actual - expected_deg).abs();

        assert_eq!(result.active, true);
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected_deg}, difference={difference},
      tolerance={TOLERANCE}"
        );

        let actual = result.target_roll_deg;
        let difference = (actual - expected_deg).abs();

        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected_deg}, difference={difference},
      tolerance={TOLERANCE}"
        );
    }

    #[test]
    fn yaw_rotation() {
        let input = HorizontalControllerInput {
            horizontal_estimate: HorizontalEstimate {
                world_forward_position_counts: 100.0,
                world_left_position_counts: 0.0,
                world_forward_velocity_counts_per_second: 20.0,
                world_left_velocity_counts_per_second: 0.0,
                measurement_age_seconds: Some(0.0),
            },
            yaw_heading_deg: 90.0,
        };

        let controller = HorizontalController::new();
        let result = controller.update(input).expect("to be valid");

        let expected_deg = -0.492;
        let actual = result.target_roll_deg;
        let difference = (actual - expected_deg).abs();

        assert_eq!(result.active, true);
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected_deg}, difference={difference},
      tolerance={TOLERANCE}"
        );

        let expected_deg = 0.0;
        let actual = result.target_pitch_deg;
        let difference = (actual - expected_deg).abs();

        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected_deg}, difference={difference},
      tolerance={TOLERANCE}"
        );
    }
    #[test]
    fn old_or_unavailable_flow_data_cannot_command_tilt() {
        let input = HorizontalControllerInput {
            horizontal_estimate: HorizontalEstimate {
                world_forward_position_counts: 100.0,
                world_left_position_counts: 0.0,
                world_forward_velocity_counts_per_second: 20.0,
                world_left_velocity_counts_per_second: 0.0,
                measurement_age_seconds: None,
            },
            yaw_heading_deg: 0.0,
        };

        let controller = HorizontalController::new();
        let result = controller.update(input).expect("to be valid");
        assert_eq!(result.active, false);
        assert_eq!(result.target_pitch_deg, 0.0);
        assert_eq!(result.target_roll_deg, 0.0);

        let input = HorizontalControllerInput {
            horizontal_estimate: HorizontalEstimate {
                world_forward_position_counts: 100.0,
                world_left_position_counts: 0.0,
                world_forward_velocity_counts_per_second: 20.0,
                world_left_velocity_counts_per_second: 0.0,
                measurement_age_seconds: Some(0.26),
            },
            yaw_heading_deg: 0.0,
        };

        let controller = HorizontalController::new();
        let result = controller.update(input).expect("to be valid");
        assert_eq!(result.active, false);
        assert_eq!(result.target_pitch_deg, 0.0);
        assert_eq!(result.target_roll_deg, 0.0);
    }

    #[test]
    fn validation_for_non_finite_input() {
        let input = HorizontalControllerInput {
            horizontal_estimate: HorizontalEstimate {
                world_forward_position_counts: 100.0,
                world_left_position_counts: 0.0,
                world_forward_velocity_counts_per_second: 20.0,
                world_left_velocity_counts_per_second: 0.0,
                measurement_age_seconds: None,
            },
            yaw_heading_deg: f32::NAN,
        };

        let controller = HorizontalController::new();
        let result = controller.update(input);
        assert_eq!(result, Err(HorizontalControllerError::NonFiniteInput));
    }
}
