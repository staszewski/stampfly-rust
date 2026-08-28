//! Cascaded altitude-position and vertical-velocity control.

use crate::vertical_estimator::VerticalEstimate;

const HEIGHT_TO_VELOCITY_GAIN_PER_SECOND: f32 = 2.0;
const VELOCITY_PROPORTIONAL_GAIN_DUTY_PER_MM_PER_SECOND: f32 = 0.06;
const VELOCITY_INTEGRAL_GAIN_DUTY_PER_MM: f32 = 0.04;

const MAX_ASCENT_VELOCITY_MM_PER_SECOND: f32 = 40.0;
const MAX_DESCENT_VELOCITY_MM_PER_SECOND: f32 = 20.0;

const MAX_UPWARD_THRUST_OFFSET_DUTY: f32 = 24.0;
const MAX_DOWNWARD_THRUST_OFFSET_DUTY: f32 = 24.0;
const TOF_FRESHNESS_LIMIT_SECONDS: f32 = 0.15;

pub struct AltitudeController {
    velocity_integral_correction_duty: f32,
}

pub struct AltitudeControllerInput {
    pub target_height_mm: f32,
    pub vertical_estimate: VerticalEstimate,
    pub dt_seconds: f32,
    pub integral_learning_enabled: bool,
}

pub struct AltitudeControllerOutput {
    pub vertical_velocity_target_mm_per_second: f32,
    pub desired_collective_correction_duty: f32,
    pub active: bool,
}

#[derive(Debug)]
pub enum AltitudeControllerError {
    NonFiniteInput,
}

impl AltitudeController {
    pub fn new() -> Self {
        Self {
            velocity_integral_correction_duty: 0.0,
        }
    }
    pub fn update(
        &mut self,
        input: AltitudeControllerInput,
    ) -> Result<AltitudeControllerOutput, AltitudeControllerError> {
        Self::validate_input(&input)?;

        let Some(tof_age_seconds) = input.vertical_estimate.tof_age_seconds else {
            return Ok(Self::inactive_output());
        };

        if tof_age_seconds > TOF_FRESHNESS_LIMIT_SECONDS {
            return Ok(Self::inactive_output());
        }

        let height_error_mm = input.target_height_mm - input.vertical_estimate.height_mm;
        let desired_vertical_velocity_mm_per_second =
            (HEIGHT_TO_VELOCITY_GAIN_PER_SECOND * height_error_mm).clamp(
                -MAX_DESCENT_VELOCITY_MM_PER_SECOND,
                MAX_ASCENT_VELOCITY_MM_PER_SECOND,
            );
        let velocity_error_mm_per_second = desired_vertical_velocity_mm_per_second
            - input.vertical_estimate.vertical_velocity_mm_per_second;
        if input.integral_learning_enabled {
            self.update_velocity_integral(velocity_error_mm_per_second, input.dt_seconds);
        }
        let desired_collective_correction_duty = (VELOCITY_PROPORTIONAL_GAIN_DUTY_PER_MM_PER_SECOND
            * velocity_error_mm_per_second
            + self.velocity_integral_correction_duty)
            .clamp(
                -MAX_DOWNWARD_THRUST_OFFSET_DUTY,
                MAX_UPWARD_THRUST_OFFSET_DUTY,
            );
        Ok(AltitudeControllerOutput {
            vertical_velocity_target_mm_per_second: desired_vertical_velocity_mm_per_second,
            desired_collective_correction_duty,
            active: true,
        })
    }

    fn validate_input(input: &AltitudeControllerInput) -> Result<(), AltitudeControllerError> {
        if !input.target_height_mm.is_finite()
            || !input.vertical_estimate.height_mm.is_finite()
            || !input
                .vertical_estimate
                .vertical_velocity_mm_per_second
                .is_finite()
            || !input.dt_seconds.is_finite()
        {
            return Err(AltitudeControllerError::NonFiniteInput);
        }

        let Some(tof_age_seconds) = input.vertical_estimate.tof_age_seconds else {
            return Ok(());
        };
        if !tof_age_seconds.is_finite() {
            return Err(AltitudeControllerError::NonFiniteInput);
        }

        Ok(())
    }

    fn update_velocity_integral(&mut self, velocity_error_mm_per_second: f32, dt_seconds: f32) {
        let integral_increment_duty =
            VELOCITY_INTEGRAL_GAIN_DUTY_PER_MM * velocity_error_mm_per_second * dt_seconds;
        let integral_candidate_duty =
            (self.velocity_integral_correction_duty + integral_increment_duty).max(0.0);
        let proportional_correction_duty =
            VELOCITY_PROPORTIONAL_GAIN_DUTY_PER_MM_PER_SECOND * velocity_error_mm_per_second;
        let candidate_raw_output_duty = proportional_correction_duty + integral_candidate_duty;
        let should_reject_candidate = candidate_raw_output_duty >= MAX_UPWARD_THRUST_OFFSET_DUTY
            && velocity_error_mm_per_second > 0.0;
        if !should_reject_candidate {
            self.velocity_integral_correction_duty = integral_candidate_duty;
        }
    }

    fn inactive_output() -> AltitudeControllerOutput {
        AltitudeControllerOutput {
            vertical_velocity_target_mm_per_second: 0.0,
            desired_collective_correction_duty: 0.0,
            active: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_DT_SECONDS: f32 = 0.0025;
    const TOLERANCE: f32 = 0.0001;

    fn assert_approximately_equal(actual: f32, expected: f32) {
        let difference = (actual - expected).abs();
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected}, difference={difference}, tolerance={TOLERANCE}"
        );
    }

    #[test]
    fn below_target_and_rising_too_slow_requests_positive_collective_correction() {
        let target_height_mm = 100.0;
        let estimated_height = 80.0;
        let estimated_upward_velocity = 10.0;
        let fresh_tof = Some(0.02);

        let vertical_estimate = VerticalEstimate {
            height_mm: estimated_height,
            vertical_velocity_mm_per_second: estimated_upward_velocity,
            tof_age_seconds: fresh_tof,
        };
        let input = AltitudeControllerInput {
            target_height_mm,
            vertical_estimate,
            dt_seconds: SAMPLE_DT_SECONDS,
            integral_learning_enabled: false,
        };

        let mut controller = AltitudeController::new();
        let result = controller.update(input).expect("to be valid");

        let expected_vertical_velocity_target_mm_per_second = 40.0;
        let expected_desired_collective_correction_duty = 1.8;

        let actual = result.vertical_velocity_target_mm_per_second;
        assert_eq!(actual, expected_vertical_velocity_target_mm_per_second);
        let actual = result.desired_collective_correction_duty;
        assert_approximately_equal(actual, expected_desired_collective_correction_duty);
        assert_eq!(result.active, true);
    }

    #[test]
    fn below_target_but_rising_too_fast_requests_negative_correction() {
        let target_height_mm = 100.0;
        let estimated_height = 90.0;
        let estimated_upward_velocity = 30.0;
        let fresh_tof = Some(0.02);

        let vertical_estimate = VerticalEstimate {
            height_mm: estimated_height,
            vertical_velocity_mm_per_second: estimated_upward_velocity,
            tof_age_seconds: fresh_tof,
        };
        let input = AltitudeControllerInput {
            target_height_mm,
            vertical_estimate,
            dt_seconds: SAMPLE_DT_SECONDS,
            integral_learning_enabled: false,
        };

        let mut controller = AltitudeController::new();
        let result = controller.update(input).expect("to be valid");

        let expected_vertical_velocity_target_mm_per_second = 20.0;
        let expected_desired_collective_correction_duty = -0.6;

        let actual = result.vertical_velocity_target_mm_per_second;
        assert_eq!(actual, expected_vertical_velocity_target_mm_per_second);
        let actual = result.desired_collective_correction_duty;
        assert_approximately_equal(actual, expected_desired_collective_correction_duty);
        assert_eq!(result.active, true);
    }

    #[test]
    fn being_far_below_target_must_not_command_arbitrarily_fast_climb() {
        let target_height_mm = 200.0;
        let estimated_height = 100.0;
        let estimated_upward_velocity = 0.0;
        let fresh_tof = Some(0.02);

        let vertical_estimate = VerticalEstimate {
            height_mm: estimated_height,
            vertical_velocity_mm_per_second: estimated_upward_velocity,
            tof_age_seconds: fresh_tof,
        };
        let input = AltitudeControllerInput {
            target_height_mm,
            vertical_estimate,
            dt_seconds: SAMPLE_DT_SECONDS,
            integral_learning_enabled: false,
        };

        let mut controller = AltitudeController::new();
        let result = controller.update(input).expect("to be valid");

        let expected_vertical_velocity_target_mm_per_second = 40.0;
        let expected_desired_collective_correction_duty = 2.4;

        let actual = result.vertical_velocity_target_mm_per_second;
        assert_eq!(actual, expected_vertical_velocity_target_mm_per_second);
        let actual = result.desired_collective_correction_duty;
        assert_approximately_equal(actual, expected_desired_collective_correction_duty);
        assert_eq!(result.active, true);
    }

    #[test]
    fn fast_descent_must_not_request_excessive_thrust_correction() {
        let target_height_mm = 200.0;
        let estimated_height = 10.0;
        let estimated_upward_velocity = -400.0;
        let fresh_tof = Some(0.02);

        let vertical_estimate = VerticalEstimate {
            height_mm: estimated_height,
            vertical_velocity_mm_per_second: estimated_upward_velocity,
            tof_age_seconds: fresh_tof,
        };
        let input = AltitudeControllerInput {
            target_height_mm,
            vertical_estimate,
            dt_seconds: SAMPLE_DT_SECONDS,
            integral_learning_enabled: false,
        };

        let mut controller = AltitudeController::new();
        let result = controller.update(input).expect("to be valid");

        let expected_desired_collective_correction_duty = MAX_UPWARD_THRUST_OFFSET_DUTY;
        let actual = result.desired_collective_correction_duty;
        assert_eq!(actual, expected_desired_collective_correction_duty);
    }

    #[test]
    fn missing_or_stale_tof_keeps_altitude_control_inactive() {
        let target_height_mm = 100.0;
        let estimated_height = 10.0;
        let estimated_upward_velocity = 10.0;
        let missing_tof_seconds = None;

        let vertical_estimate = VerticalEstimate {
            height_mm: estimated_height,
            vertical_velocity_mm_per_second: estimated_upward_velocity,
            tof_age_seconds: missing_tof_seconds,
        };
        let test_cases = [
            AltitudeControllerInput {
                target_height_mm,
                vertical_estimate,
                dt_seconds: SAMPLE_DT_SECONDS,
                integral_learning_enabled: false,
            },
            AltitudeControllerInput {
                target_height_mm,
                vertical_estimate: VerticalEstimate {
                    height_mm: estimated_height,
                    vertical_velocity_mm_per_second: estimated_upward_velocity,
                    tof_age_seconds: Some(TOF_FRESHNESS_LIMIT_SECONDS + 0.01),
                },
                dt_seconds: SAMPLE_DT_SECONDS,
                integral_learning_enabled: false,
            },
        ];

        for input in test_cases {
            let mut controller = AltitudeController::new();
            let result = controller.update(input).expect("to be valid");

            assert_eq!(result.active, false);
            assert_eq!(result.vertical_velocity_target_mm_per_second, 0.0);
            assert_eq!(result.desired_collective_correction_duty, 0.0);
        }
    }

    #[test]
    fn integral_accumulates_when_enabled_and_holds_when_disabled() {
        let target_height_mm = 100.0;
        let estimated_height = 100.0;
        let estimated_upward_velocity = -10.0;
        let fresh_tof = Some(0.02);

        let vertical_estimate = VerticalEstimate {
            height_mm: estimated_height,
            vertical_velocity_mm_per_second: estimated_upward_velocity,
            tof_age_seconds: fresh_tof,
        };
        let input = AltitudeControllerInput {
            target_height_mm,
            vertical_estimate,
            dt_seconds: 0.0025,
            integral_learning_enabled: true,
        };

        let mut controller = AltitudeController::new();
        let result = controller.update(input).expect("to be valid");

        let expected_desired_collective_correction_duty = 0.601;
        let actual = result.desired_collective_correction_duty;
        assert_approximately_equal(actual, expected_desired_collective_correction_duty);
        assert_eq!(result.active, true);

        let input = AltitudeControllerInput {
            target_height_mm,
            vertical_estimate,
            dt_seconds: SAMPLE_DT_SECONDS,
            integral_learning_enabled: true,
        };

        let result = controller.update(input).expect("to be valid");
        let expected_desired_collective_correction_duty = 0.602;
        let actual = result.desired_collective_correction_duty;
        assert_approximately_equal(actual, expected_desired_collective_correction_duty);
        assert_eq!(result.active, true);

        let input = AltitudeControllerInput {
            target_height_mm,
            vertical_estimate,
            dt_seconds: SAMPLE_DT_SECONDS,
            integral_learning_enabled: false,
        };

        let result = controller.update(input).expect("to be valid");
        let expected_desired_collective_correction_duty = 0.602;
        let actual = result.desired_collective_correction_duty;
        assert_approximately_equal(actual, expected_desired_collective_correction_duty);
        assert_eq!(result.active, true);
    }

    #[test]
    fn conditional_anti_windup() {
        let target_height_mm = 200.0;
        let estimated_height = 10.0;
        let estimated_upward_velocity = -400.0;
        let fresh_tof = Some(0.02);

        let vertical_estimate = VerticalEstimate {
            height_mm: estimated_height,
            vertical_velocity_mm_per_second: estimated_upward_velocity,
            tof_age_seconds: fresh_tof,
        };
        let input = AltitudeControllerInput {
            target_height_mm,
            vertical_estimate,
            dt_seconds: SAMPLE_DT_SECONDS,
            integral_learning_enabled: false,
        };
        // 1. Fast descent, learning disabled
        let mut controller = AltitudeController::new();
        let result = controller.update(input).expect("to be valid");
        let expected_desired_collective_correction_duty = MAX_UPWARD_THRUST_OFFSET_DUTY;
        let actual = result.desired_collective_correction_duty;
        assert_eq!(actual, expected_desired_collective_correction_duty);
        assert_eq!(result.active, true);
        // 2. Same fast descent, learning enabled
        let vertical_estimate = VerticalEstimate {
            height_mm: estimated_height,
            vertical_velocity_mm_per_second: estimated_upward_velocity,
            tof_age_seconds: fresh_tof,
        };
        let input = AltitudeControllerInput {
            target_height_mm,
            vertical_estimate,
            dt_seconds: SAMPLE_DT_SECONDS,
            integral_learning_enabled: true,
        };
        let result = controller.update(input).expect("to be valid");
        let expected_desired_collective_correction_duty = MAX_UPWARD_THRUST_OFFSET_DUTY;
        let actual = result.desired_collective_correction_duty;
        assert_eq!(actual, expected_desired_collective_correction_duty);
        assert_eq!(result.active, true);
        // 3. Zero velocity error, learning disabled
        let vertical_estimate = VerticalEstimate {
            height_mm: 100.0,
            vertical_velocity_mm_per_second: 0.0,
            tof_age_seconds: fresh_tof,
        };
        let input = AltitudeControllerInput {
            target_height_mm: 100.0,
            vertical_estimate,
            dt_seconds: SAMPLE_DT_SECONDS,
            integral_learning_enabled: false,
        };

        let result = controller.update(input).expect("to be valid");
        assert_eq!(result.vertical_velocity_target_mm_per_second, 0.0);
        assert_eq!(result.desired_collective_correction_duty, 0.0);
        assert_eq!(result.active, true);
    }

    #[test]
    fn velocity_integral_does_not_become_negative() {
        let target_height_mm = 100.0;
        let estimated_height = 100.0;
        let estimated_upward_velocity = 10.0;
        let fresh_tof = Some(0.02);
        let vertical_estimate = VerticalEstimate {
            height_mm: estimated_height,
            vertical_velocity_mm_per_second: estimated_upward_velocity,
            tof_age_seconds: fresh_tof,
        };
        let input = AltitudeControllerInput {
            target_height_mm,
            vertical_estimate,
            dt_seconds: SAMPLE_DT_SECONDS,
            integral_learning_enabled: true,
        };

        let mut controller = AltitudeController::new();
        let result = controller.update(input).expect("to be valid");
        assert_eq!(result.vertical_velocity_target_mm_per_second, 0.0);
        assert_approximately_equal(result.desired_collective_correction_duty, -0.6);
        assert_eq!(result.active, true);
    }
}
