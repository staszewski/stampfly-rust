use crate::{
    math::{decay_for_sample_dt, wrap_degrees},
    types::{AttitudeEstimate, BodyAccelerationG, BodyAngularRateDps},
};

pub struct AttitudeEstimator {
    pitch_deg: f32,
    roll_deg: f32,
    yaw_heading_deg: f32,
    initialized: bool,
}

#[derive(Debug, PartialEq)]
pub enum AttitudeEstimatorError {
    NonFiniteInput,
    NonFiniteOutput,
    NotReadyForInitialization,
    InvalidDt,
    InvalidExternalCorrectionWeight,
}

const INITIAL_ACCEL_MIN_G: f32 = 0.8;
const INITIAL_ACCEL_MAX_G: f32 = 1.2;
const INITIAL_MAX_RATE_DPS: f32 = 10.0;

const ACCEL_CORRECTION_TIME_CONSTANT_SECONDS: f32 = 0.5;
const ACCEL_FULL_CONFIDENCE_ERROR_G: f32 = 0.05;
const ACCEL_ZERO_CONFIDENCE_ERROR_G: f32 = 0.20;
const ACCEL_ZERO_CONFIDENCE_RATE_DPS: f32 = 90.0;

const MIN_UPDATE_DT_SECONDS: f32 = 0.001;
const MAX_UPDATE_DT_SECONDS: f32 = 0.01;

impl AttitudeEstimator {
    pub fn new() -> Self {
        Self {
            pitch_deg: 0.0,
            roll_deg: 0.0,
            yaw_heading_deg: 0.0,
            initialized: false,
        }
    }
    pub fn update(
        &mut self,
        accel_g: BodyAccelerationG,
        gyro_dps: BodyAngularRateDps,
        dt_seconds: f32,
        external_accel_correction_weight: f32,
    ) -> Result<AttitudeEstimate, AttitudeEstimatorError> {
        if !accel_g.x_g.is_finite()
            || !accel_g.y_g.is_finite()
            || !accel_g.z_g.is_finite()
            || !gyro_dps.pitch_dps.is_finite()
            || !gyro_dps.roll_dps.is_finite()
            || !gyro_dps.yaw_dps.is_finite()
        {
            return Err(AttitudeEstimatorError::NonFiniteInput);
        }

        if !dt_seconds.is_finite()
            || !(MIN_UPDATE_DT_SECONDS..=MAX_UPDATE_DT_SECONDS).contains(&dt_seconds)
        {
            return Err(AttitudeEstimatorError::InvalidDt);
        }

        if !external_accel_correction_weight.is_finite()
            || !(0.0..=1.0).contains(&external_accel_correction_weight)
        {
            return Err(AttitudeEstimatorError::InvalidExternalCorrectionWeight);
        }

        let (accel_roll, accel_pitch) = Self::accel_angles_deg(accel_g);
        let accel_magnitude = accel_g.x_g.hypot(accel_g.y_g).hypot(accel_g.z_g);
        let max_pitch_roll_rate_dps = gyro_dps.pitch_dps.abs().max(gyro_dps.roll_dps.abs());

        if !self.initialized {
            if (INITIAL_ACCEL_MIN_G..=INITIAL_ACCEL_MAX_G).contains(&accel_magnitude)
                && max_pitch_roll_rate_dps <= INITIAL_MAX_RATE_DPS
            {
                self.pitch_deg = accel_pitch;
                self.roll_deg = accel_roll;
                self.initialized = true;

                return Ok(AttitudeEstimate {
                    pitch_deg: self.pitch_deg,
                    roll_deg: self.roll_deg,
                    yaw_heading_deg: self.yaw_heading_deg,
                });
            } else {
                return Err(AttitudeEstimatorError::NotReadyForInitialization);
            }
        }

        let predict_pitch = self.pitch_deg + gyro_dps.pitch_dps * dt_seconds;
        let predict_roll = self.roll_deg + gyro_dps.roll_dps * dt_seconds;

        let decay = decay_for_sample_dt(dt_seconds, ACCEL_CORRECTION_TIME_CONSTANT_SECONDS);
        let base_correction = 1.0 - decay;
        let acceleration_confidence = Self::accel_confidence(accel_magnitude);
        let rate_confidence = Self::rate_confidence(max_pitch_roll_rate_dps);
        let total_correction = base_correction
            * acceleration_confidence
            * rate_confidence
            * external_accel_correction_weight;

        let pitch_disagreement = wrap_degrees(accel_pitch - predict_pitch);
        let roll_disagreement = wrap_degrees(accel_roll - predict_roll);

        let candidate_pitch = predict_pitch + total_correction * pitch_disagreement;
        let candidate_roll = predict_roll + total_correction * roll_disagreement;
        let candidate_yaw_heading =
            wrap_degrees(self.yaw_heading_deg + gyro_dps.yaw_dps * dt_seconds);

        if !candidate_pitch.is_finite()
            || !candidate_roll.is_finite()
            || !candidate_yaw_heading.is_finite()
        {
            return Err(AttitudeEstimatorError::NonFiniteOutput);
        }

        self.pitch_deg = candidate_pitch;
        self.roll_deg = candidate_roll;
        self.yaw_heading_deg = candidate_yaw_heading;

        Ok(AttitudeEstimate {
            pitch_deg: self.pitch_deg,
            roll_deg: self.roll_deg,
            yaw_heading_deg: self.yaw_heading_deg,
        })
    }

    fn accel_angles_deg(accel_g: BodyAccelerationG) -> (f32, f32) {
        let roll_denom = (accel_g.y_g * accel_g.y_g + accel_g.z_g * accel_g.z_g).sqrt();
        let roll_rad = -(accel_g.x_g.atan2(roll_denom));
        let pitch_denom = (accel_g.x_g * accel_g.x_g + accel_g.z_g * accel_g.z_g).sqrt();
        let pitch_rad = accel_g.y_g.atan2(pitch_denom);

        (roll_rad.to_degrees(), pitch_rad.to_degrees())
    }

    fn accel_confidence(accel_magnitude: f32) -> f32 {
        let magnitude_error = (accel_magnitude - 1.0).abs();
        let confidence = (1.0
            - (magnitude_error - ACCEL_FULL_CONFIDENCE_ERROR_G)
                / (ACCEL_ZERO_CONFIDENCE_ERROR_G - ACCEL_FULL_CONFIDENCE_ERROR_G))
            .clamp(0.0, 1.0);
        return confidence;
    }

    fn rate_confidence(max_rate_dps: f32) -> f32 {
        (1.0 - max_rate_dps / ACCEL_ZERO_CONFIDENCE_RATE_DPS).clamp(0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f32 = 0.001;
    const DT_SECONDS: f32 = 0.01;
    const CONFIDENCE_TOLERANCE: f32 = 0.000001;
    #[test]
    fn stationary_nose_up_and_invalid_input_transaction() {
        let valid_input_accel = BodyAccelerationG {
            x_g: 0.0,
            y_g: 0.5,
            z_g: 0.8660254, // approx cos(30)
        };
        let input_gyro = BodyAngularRateDps {
            pitch_dps: 5.0,
            roll_dps: -5.0,
            yaw_dps: 0.0,
        };
        let expected = AttitudeEstimate {
            pitch_deg: 30.0,
            roll_deg: 0.0,
            yaw_heading_deg: 0.0,
        };
        let mut attitude_estimator = AttitudeEstimator::new();
        let result1 = attitude_estimator
            .update(valid_input_accel, input_gyro, DT_SECONDS, 0.0)
            .expect("should be a valid input to update");
        assert!((result1.pitch_deg - expected.pitch_deg).abs() <= TOLERANCE);
        assert_eq!(result1.roll_deg, 0.0);
        assert_eq!(result1.yaw_heading_deg, 0.0);

        let invalid_input_accel = BodyAccelerationG {
            x_g: f32::NAN,
            y_g: 0.5,
            z_g: 0.8660254,
        };
        let result2 = attitude_estimator.update(invalid_input_accel, input_gyro, DT_SECONDS, 0.0);
        assert_eq!(result2, Err(AttitudeEstimatorError::NonFiniteInput));
        assert!((attitude_estimator.pitch_deg - expected.pitch_deg).abs() <= TOLERANCE);
        assert_eq!(attitude_estimator.roll_deg, 0.0);
        assert_eq!(attitude_estimator.yaw_heading_deg, 0.0);
        assert_eq!(attitude_estimator.initialized, true);

        let result3 = attitude_estimator.update(valid_input_accel, input_gyro, -0.01, 0.0);
        assert_eq!(result3, Err(AttitudeEstimatorError::InvalidDt));
        assert!((attitude_estimator.pitch_deg - expected.pitch_deg).abs() <= TOLERANCE);
        assert_eq!(attitude_estimator.roll_deg, 0.0);
        assert_eq!(attitude_estimator.yaw_heading_deg, 0.0);
        assert_eq!(attitude_estimator.initialized, true);
    }

    #[test]
    fn gyro_rate_predicts_attitude_after_initialization() {
        let input_accel = BodyAccelerationG {
            x_g: 0.0,
            y_g: 0.0,
            z_g: 1.0,
        };
        let input_gyro = BodyAngularRateDps {
            pitch_dps: 0.0,
            roll_dps: 0.0,
            yaw_dps: 0.0,
        };
        let mut attitude_estimator = AttitudeEstimator::new();
        let result1 = attitude_estimator
            .update(input_accel, input_gyro, DT_SECONDS, 0.0)
            .expect("should be a valid input to update");
        assert_eq!(result1.pitch_deg, 0.0);
        assert_eq!(result1.roll_deg, 0.0);
        assert_eq!(result1.yaw_heading_deg, 0.0);

        let input_gyro2 = BodyAngularRateDps {
            pitch_dps: 100.0,
            roll_dps: -50.0,
            yaw_dps: 25.0,
        };
        let result2 = attitude_estimator
            .update(input_accel, input_gyro2, DT_SECONDS, 1.0)
            .expect("should be a valid input to update");

        assert!((result2.pitch_deg - 1.0).abs() <= TOLERANCE);
        assert!((result2.roll_deg - (-0.5)).abs() <= TOLERANCE);
        assert!((result2.yaw_heading_deg - 0.25).abs() <= TOLERANCE);

        let result3 = attitude_estimator
            .update(input_accel, input_gyro2, DT_SECONDS, 1.0)
            .expect("should be a valid input to update");

        assert!((result3.pitch_deg - 2.0).abs() <= TOLERANCE);
        assert!((result3.roll_deg - (-1.0)).abs() <= TOLERANCE);
        assert!((result3.yaw_heading_deg - 0.5).abs() <= TOLERANCE);
    }

    #[test]
    fn initialization_requires_calm_sample() {
        let input_accel = BodyAccelerationG {
            x_g: 0.0,
            y_g: 0.0,
            z_g: 0.5,
        };
        let input_gyro = BodyAngularRateDps {
            pitch_dps: 0.0,
            roll_dps: 0.0,
            yaw_dps: 0.0,
        };
        let mut attitude_estimator = AttitudeEstimator::new();
        let result = attitude_estimator.update(input_accel, input_gyro, DT_SECONDS, 0.0);
        assert_eq!(
            result,
            Err(AttitudeEstimatorError::NotReadyForInitialization)
        );

        let input_accel2 = BodyAccelerationG {
            z_g: 1.0,
            ..input_accel
        };

        let input_gyro2 = BodyAngularRateDps {
            pitch_dps: 11.0,
            ..input_gyro
        };
        let _result2 = attitude_estimator.update(input_accel2, input_gyro2, DT_SECONDS, 0.0);
        assert_eq!(
            result,
            Err(AttitudeEstimatorError::NotReadyForInitialization)
        );
    }

    #[test]
    fn yaw_heading_wraps_after_gyro_pred() {
        let input_accel = BodyAccelerationG {
            x_g: 0.0,
            y_g: 0.0,
            z_g: 1.0,
        };
        let input_gyro = BodyAngularRateDps {
            pitch_dps: 0.0,
            roll_dps: 0.0,
            yaw_dps: 0.0,
        };
        let mut attitude_estimator = AttitudeEstimator::new();
        attitude_estimator
            .update(input_accel, input_gyro, DT_SECONDS, 0.0)
            .expect("should be valid");

        // artificial update just to wrap the degree of yaw
        attitude_estimator.yaw_heading_deg = 179.9;

        let yaw_input_gyro = BodyAngularRateDps {
            yaw_dps: 25.0,
            ..input_gyro
        };

        let result2 = attitude_estimator
            .update(input_accel, yaw_input_gyro, DT_SECONDS, 0.0)
            .expect("should be valid");

        let actual = result2.yaw_heading_deg;
        let expected = -179.85;
        let difference = (actual - expected).abs();

        assert!(
            difference <= TOLERANCE,
            "yaw mismatch: actual={actual}, expected={expected},
      difference={difference}, tolerance={TOLERANCE}"
        );
    }

    #[test]
    fn base_accel_correction_moves_pitch_toward_gravity() {
        let input_accel = BodyAccelerationG {
            x_g: 0.0,
            y_g: 0.17364818, // sin(10)
            z_g: 0.98480775, // cos(10)
        };
        let input_gyro = BodyAngularRateDps {
            pitch_dps: 0.0,
            roll_dps: 0.0,
            yaw_dps: 0.0,
        };
        let mut attitude_estimator = AttitudeEstimator::new();

        let result1 = attitude_estimator
            .update(input_accel, input_gyro, DT_SECONDS, 1.0)
            .expect("should be valid");

        assert!((result1.pitch_deg - 10.0).abs() <= TOLERANCE);
        assert_eq!(result1.roll_deg, 0.0);
        assert_eq!(result1.yaw_heading_deg, 0.0);

        let level_accel = BodyAccelerationG {
            x_g: 0.0,
            y_g: 0.0,
            z_g: 1.0,
        };

        let result2 = attitude_estimator
            .update(level_accel, input_gyro, DT_SECONDS, 1.0)
            .expect("should be valid");

        let actual_pitch_deg = result2.pitch_deg;
        let expected_pitch_deg = 9.801987; // 10 * exp(-0.01 / 0.5)
        let difference = (actual_pitch_deg - expected_pitch_deg).abs();
        assert!(
            difference <= TOLERANCE,
            "pitch mismatch: actual={actual_pitch_deg}, expected={expected_pitch_deg},
      difference={difference}, tolerance={TOLERANCE}"
        );
        assert!(
            actual_pitch_deg > 0.0 && actual_pitch_deg < 10.0,
            "corrected pitch should be between level and initial pitch:
      actual={actual_pitch_deg}"
        );
        assert_eq!(result2.roll_deg, 0.0);
        assert_eq!(result2.yaw_heading_deg, 0.0);
    }

    #[test]
    fn wrapped_disagreement() {
        let init_accel = BodyAccelerationG {
            x_g: 0.0,
            y_g: 0.0,
            z_g: 1.0,
        };
        let input_accel = BodyAccelerationG {
            x_g: 0.0,
            y_g: -0.17364818,
            z_g: 0.98480775,
        };
        let input_gyro = BodyAngularRateDps {
            pitch_dps: 0.0,
            roll_dps: 0.0,
            yaw_dps: 0.0,
        };
        let mut attitude_estimator = AttitudeEstimator::new();
        attitude_estimator
            .update(init_accel, input_gyro, DT_SECONDS, 1.0)
            .expect("initial alignment should succeed");

        attitude_estimator.pitch_deg = 179.0;

        let result = attitude_estimator
            .update(input_accel, input_gyro, DT_SECONDS, 1.0)
            .expect("should be valid");

        assert!(
            result.pitch_deg > 179.0,
            "wrapped shortest-path correction should increase pitch: actual={}",
            result.pitch_deg
        );
    }

    #[test]
    fn accel_confidence_fades_with_magnitude_error() {
        let test_cases = [
            (1.0, 1.0),
            (0.95, 1.0),
            (1.125, 0.5),
            (0.875, 0.5),
            (1.2, 0.0),
            (0.80, 0.0),
        ];
        for (accel_magnitude, expected_confidence) in test_cases {
            let confidence = AttitudeEstimator::accel_confidence(accel_magnitude);
            let difference = (confidence - expected_confidence).abs();
            assert!(
                difference <= CONFIDENCE_TOLERANCE,
                "accel confidence mismatch: actual={confidence}, expected={expected_confidence},
          difference={difference}, tolerance={CONFIDENCE_TOLERANCE}"
            );
        }
    }

    #[test]
    fn zero_accel_confidence_disables_gravity_correction() {
        let input_accel = BodyAccelerationG {
            x_g: 0.0,
            y_g: 0.17364818, // sin(10)
            z_g: 0.98480775, // cos(10)
        };
        let input_gyro = BodyAngularRateDps {
            pitch_dps: 0.0,
            roll_dps: 0.0,
            yaw_dps: 0.0,
        };
        let mut attitude_estimator = AttitudeEstimator::new();

        let result1 = attitude_estimator
            .update(input_accel, input_gyro, DT_SECONDS, 1.0)
            .expect("should be valid");

        let zero_confidence_accel = BodyAccelerationG {
            x_g: 0.0,
            y_g: 0.0,
            z_g: 1.2,
        };

        let result2 = attitude_estimator
            .update(zero_confidence_accel, input_gyro, DT_SECONDS, 1.0)
            .expect("should be valid");

        let actual_pitch_deg = result2.pitch_deg;
        let expected_pitch_deg = result1.pitch_deg;
        let difference = (actual_pitch_deg - expected_pitch_deg).abs();
        assert!(
            difference <= TOLERANCE,
            "pitch mismatch: actual={actual_pitch_deg}, expected={expected_pitch_deg},
      difference={difference}, tolerance={TOLERANCE}"
        );
        assert_eq!(result2.roll_deg, 0.0);
        assert_eq!(result2.yaw_heading_deg, 0.0);
    }

    #[test]
    fn rate_confidence_fades_with_pitch_roll_rate() {
        let test_cases = [(0.0, 1.0), (45.0, 0.5), (90.0, 0.0), (120.0, 0.0)];
        for (max_rate_dps, expected_confidence) in test_cases {
            let confidence = AttitudeEstimator::rate_confidence(max_rate_dps);
            let difference = (confidence - expected_confidence).abs();
            assert!(
                difference <= CONFIDENCE_TOLERANCE,
                "rate confidence mismatch: actual={confidence}, expected={expected_confidence},
          difference={difference}, tolerance={CONFIDENCE_TOLERANCE}"
            );
        }
    }

    #[test]
    fn update_above_max_dt_is_rejected() {
        let input_accel = BodyAccelerationG {
            x_g: 0.0,
            y_g: 0.0,
            z_g: 1.0,
        };
        let input_gyro = BodyAngularRateDps {
            pitch_dps: 0.0,
            roll_dps: 0.0,
            yaw_dps: 0.0,
        };
        let dt = 0.0025;
        let mut attitude_estimator = AttitudeEstimator::new();
        let result1 = attitude_estimator
            .update(input_accel, input_gyro, dt, 0.5)
            .expect("valid result");

        let input_gyro2 = BodyAngularRateDps {
            yaw_dps: 25.0,
            ..input_gyro
        };

        let invalid_dt = 0.02;
        let result2 = attitude_estimator.update(input_accel, input_gyro2, invalid_dt, 0.5);

        assert_eq!(result2, Err(AttitudeEstimatorError::InvalidDt));
        assert_eq!(attitude_estimator.pitch_deg, result1.pitch_deg);
        assert_eq!(attitude_estimator.roll_deg, result1.roll_deg);
        assert_eq!(attitude_estimator.yaw_heading_deg, result1.yaw_heading_deg);
        assert!(attitude_estimator.initialized);
    }

    #[test]
    fn candidate_validation() {
        let input_accel = BodyAccelerationG {
            x_g: 0.0,
            y_g: 0.0,
            z_g: 1.0,
        };
        let input_gyro1 = BodyAngularRateDps {
            pitch_dps: 0.0,
            roll_dps: 0.0,
            yaw_dps: 0.0,
        };
        let input_gyro2 = BodyAngularRateDps {
            pitch_dps: f32::MAX,
            roll_dps: 0.0,
            yaw_dps: 25.0,
        };
        let mut attitude_estimator = AttitudeEstimator::new();
        attitude_estimator
            .update(input_accel, input_gyro1, DT_SECONDS, 0.0)
            .expect("should be valid update");

        attitude_estimator.pitch_deg = f32::MAX;

        let state_before = (
            attitude_estimator.pitch_deg,
            attitude_estimator.roll_deg,
            attitude_estimator.yaw_heading_deg,
            attitude_estimator.initialized,
        );

        let result2 = attitude_estimator.update(input_accel, input_gyro2, DT_SECONDS, 0.0);
        let state_after = (
            attitude_estimator.pitch_deg,
            attitude_estimator.roll_deg,
            attitude_estimator.yaw_heading_deg,
            attitude_estimator.initialized,
        );
        assert_eq!(result2, Err(AttitudeEstimatorError::NonFiniteOutput));
        assert_eq!(state_after, state_before);
    }
}
