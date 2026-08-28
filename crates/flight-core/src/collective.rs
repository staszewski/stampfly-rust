//! Stateful collective-command arbitration, limiting, and recovery policy.

use crate::types::CollectiveMode;

const MAX_ALTITUDE_THRUST_OFFSET_SLEW_PER_SECOND: f32 = 240.0;

const BASE_COLLECTIVE_START_DUTY: f32 = 160.0;
const BASE_COLLECTIVE_TARGET_DUTY: f32 = 187.0;
const MAX_COLLECTIVE_DUTY: f32 = 255.0;
const BASE_THRUST_RAMP_DURATION_SECONDS: f32 = 0.5;

const MIN_UPDATE_DT_SECONDS: f32 = 0.001;
const MAX_UPDATE_DT_SECONDS: f32 = 0.01;

const MOTOR_VOLTAGE_REFERENCE_V: f32 = 3.88;
const MIN_MOTOR_VOLTAGE_SCALE: f32 = 0.88;
const MAX_MOTOR_VOLTAGE_SCALE: f32 = 1.0;

pub struct CollectivePolicy {
    applied_collective_correction_duty: f32,
    base_thrust_ramp_elapsed_seconds: f32,
    motor_voltage_scale: f32,
}

pub struct CollectivePolicyInput {
    pub desired_collective_correction_duty: f32,
    pub mode: CollectiveMode,
    pub dt_seconds: f32,
    pub vertical_velocity_mm_per_second: f32,
    pub attitude_qualified: bool,
}

pub struct CollectivePolicyOutput {
    pub collective_duty: f32,
    pub base_thrust_ramp_complete: bool,
}

#[derive(Debug)]
pub enum CollectivePolicyError {
    NonFiniteInput,
    InvalidDt,
}

impl CollectivePolicy {
    pub fn new(battery_voltage_v: f32) -> Self {
        Self {
            applied_collective_correction_duty: 0.0,
            base_thrust_ramp_elapsed_seconds: 0.0,
            motor_voltage_scale: (MOTOR_VOLTAGE_REFERENCE_V / battery_voltage_v)
                .clamp(MIN_MOTOR_VOLTAGE_SCALE, MAX_MOTOR_VOLTAGE_SCALE),
        }
    }

    pub fn update(
        &mut self,
        input: CollectivePolicyInput,
    ) -> Result<CollectivePolicyOutput, CollectivePolicyError> {
        if input.mode == CollectiveMode::Disabled {
            self.applied_collective_correction_duty = 0.0;
            self.base_thrust_ramp_elapsed_seconds = 0.0;
            return Ok(CollectivePolicyOutput {
                collective_duty: 0.0,
                base_thrust_ramp_complete: false,
            });
        }

        Self::validate_inputs(&input)?;

        self.base_thrust_ramp_elapsed_seconds = (self.base_thrust_ramp_elapsed_seconds
            + input.dt_seconds)
            .min(BASE_THRUST_RAMP_DURATION_SECONDS);
        let progress = self.base_thrust_ramp_elapsed_seconds / BASE_THRUST_RAMP_DURATION_SECONDS;
        let base_collective_duty = BASE_COLLECTIVE_START_DUTY
            + (BASE_COLLECTIVE_TARGET_DUTY - BASE_COLLECTIVE_START_DUTY) * progress;
        let base_thrust_ramp_complete =
            self.base_thrust_ramp_elapsed_seconds >= BASE_THRUST_RAMP_DURATION_SECONDS;
        if input.mode == CollectiveMode::BaseThrust {
            self.applied_collective_correction_duty = 0.0;

            return Ok(CollectivePolicyOutput {
                collective_duty: base_collective_duty * self.motor_voltage_scale,
                base_thrust_ramp_complete,
            });
        }

        let attitude_limited_desired_collective_correction_duty: f32 = if input.attitude_qualified {
            input.desired_collective_correction_duty
        } else if input.desired_collective_correction_duty <= 0.0 {
            input.desired_collective_correction_duty
        } else {
            input
                .desired_collective_correction_duty
                .min(self.applied_collective_correction_duty.max(0.0))
        };

        if attitude_limited_desired_collective_correction_duty.signum()
            == self.applied_collective_correction_duty.signum()
            && attitude_limited_desired_collective_correction_duty.abs()
                < self.applied_collective_correction_duty.abs()
        {
            self.applied_collective_correction_duty =
                attitude_limited_desired_collective_correction_duty;
            let logical_collective_duty =
                base_collective_duty + self.applied_collective_correction_duty;
            return Ok(CollectivePolicyOutput {
                collective_duty: Self::apply_bounds_to_collective_duty(logical_collective_duty)
                    * self.motor_voltage_scale,
                base_thrust_ramp_complete,
            });
        }

        if input.attitude_qualified
            && input.vertical_velocity_mm_per_second < 0.0
            && attitude_limited_desired_collective_correction_duty > 0.0
            && attitude_limited_desired_collective_correction_duty
                > self.applied_collective_correction_duty
        {
            self.applied_collective_correction_duty =
                attitude_limited_desired_collective_correction_duty;
            let logical_collective_duty =
                base_collective_duty + self.applied_collective_correction_duty;
            return Ok(CollectivePolicyOutput {
                collective_duty: Self::apply_bounds_to_collective_duty(logical_collective_duty)
                    * self.motor_voltage_scale,
                base_thrust_ramp_complete,
            });
        }

        if self.applied_collective_correction_duty != 0.0
            && self.applied_collective_correction_duty.signum()
                != attitude_limited_desired_collective_correction_duty.signum()
        {
            self.applied_collective_correction_duty = 0.0;
            let logical_collective_duty =
                base_collective_duty + self.applied_collective_correction_duty;
            return Ok(CollectivePolicyOutput {
                collective_duty: Self::apply_bounds_to_collective_duty(logical_collective_duty)
                    * self.motor_voltage_scale,
                base_thrust_ramp_complete,
            });
        }

        let max_change_this_tick = MAX_ALTITUDE_THRUST_OFFSET_SLEW_PER_SECOND * input.dt_seconds;
        let requested_change = (attitude_limited_desired_collective_correction_duty
            - self.applied_collective_correction_duty)
            .clamp(-max_change_this_tick, max_change_this_tick);
        self.applied_collective_correction_duty =
            self.applied_collective_correction_duty + requested_change;
        let logical_collective_duty =
            base_collective_duty + self.applied_collective_correction_duty;
        Ok(CollectivePolicyOutput {
            collective_duty: Self::apply_bounds_to_collective_duty(logical_collective_duty)
                * self.motor_voltage_scale,
            base_thrust_ramp_complete,
        })
    }

    fn validate_inputs(input: &CollectivePolicyInput) -> Result<(), CollectivePolicyError> {
        if !input.desired_collective_correction_duty.is_finite()
            || !input.vertical_velocity_mm_per_second.is_finite()
        {
            return Err(CollectivePolicyError::NonFiniteInput);
        }

        if !input.dt_seconds.is_finite()
            || input.dt_seconds <= 0.0
            || !(MIN_UPDATE_DT_SECONDS..=MAX_UPDATE_DT_SECONDS).contains(&input.dt_seconds)
        {
            return Err(CollectivePolicyError::InvalidDt);
        }

        Ok(())
    }

    fn apply_bounds_to_collective_duty(collective_duty: f32) -> f32 {
        collective_duty.clamp(0.0, MAX_COLLECTIVE_DUTY)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_DT_SECONDS: f32 = 0.0025;
    const TOLERANCE: f32 = 0.001;
    #[test]
    fn active_collective_correction_transitions_follow_policy() {
        let mut collective_policy = CollectivePolicy::new(MOTOR_VOLTAGE_REFERENCE_V);
        let input = CollectivePolicyInput {
            desired_collective_correction_duty: 10.0,
            mode: CollectiveMode::AltitudeHold,
            dt_seconds: SAMPLE_DT_SECONDS,
            vertical_velocity_mm_per_second: 0.0,
            attitude_qualified: true,
        };

        let result = collective_policy.update(input).expect("should be valid");
        let actual = result.collective_duty;
        let expected = 160.735;
        let difference = (actual - expected).abs();
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected}, difference={difference},
      tolerance={TOLERANCE}"
        );

        let input = CollectivePolicyInput {
            desired_collective_correction_duty: 10.0,
            mode: CollectiveMode::AltitudeHold,
            dt_seconds: SAMPLE_DT_SECONDS,
            vertical_velocity_mm_per_second: 0.0,
            attitude_qualified: true,
        };
        let result = collective_policy.update(input).expect("should be valid");
        let actual = result.collective_duty;
        let expected = 161.47;
        let difference = (actual - expected).abs();
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected}, difference={difference},
      tolerance={TOLERANCE}"
        );

        let input = CollectivePolicyInput {
            desired_collective_correction_duty: 0.2,
            mode: CollectiveMode::AltitudeHold,
            dt_seconds: SAMPLE_DT_SECONDS,
            vertical_velocity_mm_per_second: 0.0,
            attitude_qualified: true,
        };
        let result = collective_policy.update(input).expect("should be valid");
        let actual = result.collective_duty;
        let expected = 160.605;
        let difference = (actual - expected).abs();
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected}, difference={difference},
      tolerance={TOLERANCE}"
        );

        let input = CollectivePolicyInput {
            desired_collective_correction_duty: -10.0,
            mode: CollectiveMode::AltitudeHold,
            dt_seconds: SAMPLE_DT_SECONDS,
            vertical_velocity_mm_per_second: 0.0,
            attitude_qualified: true,
        };
        let result = collective_policy.update(input).expect("should be valid");
        let actual = result.collective_duty;
        let expected = 160.54;
        let difference = (actual - expected).abs();
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected}, difference={difference},
      tolerance={TOLERANCE}"
        );

        let input = CollectivePolicyInput {
            desired_collective_correction_duty: -10.0,
            mode: CollectiveMode::AltitudeHold,
            dt_seconds: SAMPLE_DT_SECONDS,
            vertical_velocity_mm_per_second: 0.0,
            attitude_qualified: true,
        };
        let result = collective_policy.update(input).expect("should be valid");
        let actual = result.collective_duty;
        let expected = 160.075;
        let difference = (actual - expected).abs();
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected}, difference={difference},
      tolerance={TOLERANCE}"
        );
    }

    #[test]
    fn base_thrust_mode_clears_altitude_correction() {
        let mut collective_policy = CollectivePolicy::new(MOTOR_VOLTAGE_REFERENCE_V);
        let input = CollectivePolicyInput {
            desired_collective_correction_duty: 10.0,
            mode: CollectiveMode::AltitudeHold,
            dt_seconds: SAMPLE_DT_SECONDS,
            vertical_velocity_mm_per_second: 0.0,
            attitude_qualified: true,
        };

        let result = collective_policy.update(input).expect("should be valid");
        assert_eq!(result.collective_duty, 160.735);

        let input = CollectivePolicyInput {
            desired_collective_correction_duty: 10.0,
            mode: CollectiveMode::BaseThrust,
            dt_seconds: SAMPLE_DT_SECONDS,
            vertical_velocity_mm_per_second: 0.0,
            attitude_qualified: true,
        };

        let result = collective_policy.update(input).expect("should be valid");
        assert_eq!(result.collective_duty, 160.27);

        let input = CollectivePolicyInput {
            desired_collective_correction_duty: 10.0,
            mode: CollectiveMode::AltitudeHold,
            dt_seconds: SAMPLE_DT_SECONDS,
            vertical_velocity_mm_per_second: 0.0,
            attitude_qualified: true,
        };
        let result = collective_policy.update(input).expect("should be valid");
        let actual = result.collective_duty;
        let expected = 161.005;
        let difference = (actual - expected).abs();
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected}, difference={difference},
      tolerance={TOLERANCE}"
        );
    }

    #[test]
    fn descent_recovery_requires_qualified_attitude() {
        let mut collective_policy = CollectivePolicy::new(MOTOR_VOLTAGE_REFERENCE_V);

        let input = CollectivePolicyInput {
            desired_collective_correction_duty: 10.0,
            mode: CollectiveMode::AltitudeHold,
            dt_seconds: SAMPLE_DT_SECONDS,
            vertical_velocity_mm_per_second: -100.0,
            attitude_qualified: false,
        };

        let result = collective_policy.update(input).expect("should be valid");
        let actual = result.collective_duty;
        let expected = 160.135;
        let difference = (actual - expected).abs();
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected}, difference={difference},
      tolerance={TOLERANCE}"
        );

        let input = CollectivePolicyInput {
            desired_collective_correction_duty: 10.0,
            mode: CollectiveMode::AltitudeHold,
            dt_seconds: SAMPLE_DT_SECONDS,
            vertical_velocity_mm_per_second: -100.0,
            attitude_qualified: true,
        };

        let result = collective_policy.update(input).expect("should be valid");
        let actual = result.collective_duty;
        let expected = 170.27;
        let difference = (actual - expected).abs();
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected}, difference={difference},
      tolerance={TOLERANCE}"
        );

        let input = CollectivePolicyInput {
            desired_collective_correction_duty: 1000.0,
            mode: CollectiveMode::AltitudeHold,
            dt_seconds: SAMPLE_DT_SECONDS,
            vertical_velocity_mm_per_second: -100.0,
            attitude_qualified: true,
        };

        let result = collective_policy.update(input).expect("should be valid");
        assert_eq!(result.collective_duty, 255.0);
    }

    #[test]
    fn base_thrust_mode_advances_collective_ramp() {
        let mut collective_policy = CollectivePolicy::new(MOTOR_VOLTAGE_REFERENCE_V);

        let input = CollectivePolicyInput {
            desired_collective_correction_duty: 0.0,
            mode: CollectiveMode::BaseThrust,
            dt_seconds: SAMPLE_DT_SECONDS,
            vertical_velocity_mm_per_second: 0.0,
            attitude_qualified: true,
        };

        let result = collective_policy.update(input).expect("should be valid");
        let actual = result.collective_duty;
        let expected = 160.135;
        let difference = (actual - expected).abs();
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected}, difference={difference},
      tolerance={TOLERANCE}"
        );
        assert_eq!(result.base_thrust_ramp_complete, false);

        let input = CollectivePolicyInput {
            desired_collective_correction_duty: 0.0,
            mode: CollectiveMode::BaseThrust,
            dt_seconds: SAMPLE_DT_SECONDS,
            vertical_velocity_mm_per_second: 0.0,
            attitude_qualified: true,
        };

        let result = collective_policy.update(input).expect("should be valid");
        let actual = result.collective_duty;
        let expected = 160.27;
        let difference = (actual - expected).abs();
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected}, difference={difference},
      tolerance={TOLERANCE}"
        );

        for _ in 0..197 {
            let input = CollectivePolicyInput {
                desired_collective_correction_duty: 0.0,
                mode: CollectiveMode::BaseThrust,
                dt_seconds: SAMPLE_DT_SECONDS,
                vertical_velocity_mm_per_second: 0.0,
                attitude_qualified: true,
            };

            let _ = collective_policy.update(input).expect("should be valid");
        }

        let input = CollectivePolicyInput {
            desired_collective_correction_duty: 0.0,
            mode: CollectiveMode::BaseThrust,
            dt_seconds: SAMPLE_DT_SECONDS,
            vertical_velocity_mm_per_second: 0.0,
            attitude_qualified: true,
        };

        let result = collective_policy.update(input).expect("should be valid");
        let actual = result.collective_duty;
        let expected = 187.0;
        let difference = (actual - expected).abs();
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected}, difference={difference},
      tolerance={TOLERANCE}"
        );

        let input = CollectivePolicyInput {
            desired_collective_correction_duty: 0.0,
            mode: CollectiveMode::BaseThrust,
            dt_seconds: SAMPLE_DT_SECONDS,
            vertical_velocity_mm_per_second: 0.0,
            attitude_qualified: true,
        };

        let result = collective_policy.update(input).expect("should be valid");
        let actual = result.collective_duty;
        let expected = 187.0;
        let difference = (actual - expected).abs();
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected}, difference={difference},
      tolerance={TOLERANCE}"
        );
        assert_eq!(result.base_thrust_ramp_complete, true);

        let input = CollectivePolicyInput {
            desired_collective_correction_duty: 0.0,
            mode: CollectiveMode::Disabled,
            dt_seconds: SAMPLE_DT_SECONDS,
            vertical_velocity_mm_per_second: 0.0,
            attitude_qualified: true,
        };

        let result = collective_policy.update(input).expect("should be valid");
        assert_eq!(result.collective_duty, 0.0);

        let input = CollectivePolicyInput {
            desired_collective_correction_duty: 0.0,
            mode: CollectiveMode::BaseThrust,
            dt_seconds: SAMPLE_DT_SECONDS,
            vertical_velocity_mm_per_second: 0.0,
            attitude_qualified: true,
        };

        let result = collective_policy.update(input).expect("should be valid");
        let actual = result.collective_duty;
        let expected = 160.135;
        let difference = (actual - expected).abs();
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected}, difference={difference},
      tolerance={TOLERANCE}"
        );
        assert_eq!(result.base_thrust_ramp_complete, false);
    }

    #[test]
    fn charged_battery_reduces_collective_without_boosting_low_battery() {
        let mut collective_policy = CollectivePolicy::new(4.2);

        let input = CollectivePolicyInput {
            desired_collective_correction_duty: 0.0,
            mode: CollectiveMode::BaseThrust,
            dt_seconds: SAMPLE_DT_SECONDS,
            vertical_velocity_mm_per_second: 0.0,
            attitude_qualified: true,
        };

        let result = collective_policy.update(input).expect("should be valid");
        let actual = result.collective_duty;
        let expected = 147.93425;
        let difference = (actual - expected).abs();
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected}, difference={difference},
      tolerance={TOLERANCE}"
        );

        let mut collective_policy = CollectivePolicy::new(3.6);

        let input = CollectivePolicyInput {
            desired_collective_correction_duty: 0.0,
            mode: CollectiveMode::BaseThrust,
            dt_seconds: SAMPLE_DT_SECONDS,
            vertical_velocity_mm_per_second: 0.0,
            attitude_qualified: true,
        };

        let result = collective_policy.update(input).expect("should be valid");
        let actual = result.collective_duty;
        let expected = 160.135;
        let difference = (actual - expected).abs();
        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected}, difference={difference},
      tolerance={TOLERANCE}"
        );
    }
}
