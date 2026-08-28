use core::f32;

use crate::math::decay_for_sample_dt;

const MIN_TOF_HEIGHT_MM: f32 = 10.0;
const MAX_VALID_TOF_HEIGHT_MM: i16 = 700;
const TOF_POSITION_CORRECTION_GAIN: f32 = 0.35;
const GRAVITY_MM_PER_SECOND_SQUARED: f32 = 9_806.65;
const MAX_TOF_VERTICAL_ACCELERATION_MM_PER_SECOND_SQUARED: f32 =
    0.5 * GRAVITY_MM_PER_SECOND_SQUARED;
const MAX_TOF_VERTICAL_SPEED_MM_PER_SECOND: f32 = 600.0;
const TOF_INNOVATION_MARGIN_MM: f32 = 8.0;
const TOF_VELOCITY_FILTER_TIME_CONSTANT_SECONDS: f32 = 0.06;
const ACCELERATION_FILTER_TIME_CONSTANT_SECONDS: f32 = 0.06;
const ACCELERATION_VELOCITY_LEAK_TIME_CONSTANT_SECONDS: f32 = 0.12;
const MAX_ACCELERATION_VELOCITY_CORRECTION_MM_PER_SECOND: f32 = 40.0;
const MAX_OBSERVED_VERTICAL_SPEED_MM_PER_SECOND: f32 = 1_000.0;

const MIN_TOF_UPDATE_DT_SECONDS: f32 = 0.025;
const MAX_TOF_UPDATE_DT_SECONDS: f32 = 0.20;
const MIN_PREDICTION_DT_SECONDS: f32 = 0.001;
const MAX_PREDICTION_DT_SECONDS: f32 = 0.01;

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VerticalEstimate {
    pub height_mm: f32,
    pub vertical_velocity_mm_per_second: f32,
    pub tof_age_seconds: Option<f32>,
}

pub struct TofMeasurement {
    pub range_mm: i16,
    pub status: u8,
    pub objects: u8,
}

#[derive(Copy, Clone)]
pub struct VerticalEstimator {
    pub last_tof_height_mm: Option<f32>,
    pub estimate: VerticalEstimate,
    pub tof_vertical_velocity_mm_per_second: f32,
    pub acceleration_filter_initialized: bool,
    pub filtered_vertical_acceleration_mm_per_second_squared: f32,
    pub acceleration_velocity_correction_mm_per_second: f32,
}

#[derive(Debug, PartialEq)]
pub enum VerticalEstimatorError {
    InvalidDt,
    NonFiniteVerticalAcceleration,
    NonFiniteOutput,
}

impl VerticalEstimator {
    pub fn new() -> Self {
        Self {
            last_tof_height_mm: None,
            estimate: VerticalEstimate {
                height_mm: 0.0,
                vertical_velocity_mm_per_second: 0.0,
                tof_age_seconds: None,
            },
            tof_vertical_velocity_mm_per_second: 0.0,
            acceleration_filter_initialized: false,
            filtered_vertical_acceleration_mm_per_second_squared: 0.0,
            acceleration_velocity_correction_mm_per_second: 0.0,
        }
    }
    pub fn correct_tof(
        &mut self,
        tof_measurement: TofMeasurement,
        sample_dt_seconds: f32,
    ) -> Result<VerticalEstimate, VerticalEstimatorError> {
        if !sample_dt_seconds.is_finite()
            || sample_dt_seconds <= 0.0
            || !(MIN_TOF_UPDATE_DT_SECONDS..=MAX_TOF_UPDATE_DT_SECONDS).contains(&sample_dt_seconds)
        {
            return Err(VerticalEstimatorError::InvalidDt);
        }
        let Some(measured_height_mm) = valid_height_mm(tof_measurement) else {
            return Ok(self.estimate);
        };

        self.estimate.tof_age_seconds = Some(0.0);

        if measured_height_mm <= MIN_TOF_HEIGHT_MM {
            self.last_tof_height_mm = Some(MIN_TOF_HEIGHT_MM);
            self.tof_vertical_velocity_mm_per_second = 0.0;
            self.estimate = VerticalEstimate {
                height_mm: MIN_TOF_HEIGHT_MM,
                vertical_velocity_mm_per_second: 0.0,
                tof_age_seconds: Some(0.0),
            };
            self.acceleration_velocity_correction_mm_per_second = 0.0;
            self.sync_vertical_velocity();
            return Ok(self.estimate);
        }

        match self.last_tof_height_mm {
            Some(previous_height_mm) => {
                let accepted_height_mm = get_accepted_tof_height_mm(
                    previous_height_mm,
                    measured_height_mm,
                    sample_dt_seconds,
                );
                let raw_vertical_velocity_mm_per_second =
                    ((accepted_height_mm - previous_height_mm) / sample_dt_seconds).clamp(
                        -MAX_TOF_VERTICAL_SPEED_MM_PER_SECOND,
                        MAX_TOF_VERTICAL_SPEED_MM_PER_SECOND,
                    );
                let height_innovation_mm = accepted_height_mm - self.estimate.height_mm;
                self.estimate.height_mm =
                    self.estimate.height_mm + TOF_POSITION_CORRECTION_GAIN * height_innovation_mm;
                self.last_tof_height_mm = Some(accepted_height_mm);
                let next_tof_velocity = next_tof_velocity_mm_per_second(
                    self.tof_vertical_velocity_mm_per_second,
                    raw_vertical_velocity_mm_per_second,
                    sample_dt_seconds,
                );
                self.tof_vertical_velocity_mm_per_second = next_tof_velocity;
                self.sync_vertical_velocity();
            }
            None => {
                self.last_tof_height_mm = Some(measured_height_mm);
                self.estimate = VerticalEstimate {
                    height_mm: measured_height_mm,
                    vertical_velocity_mm_per_second: 0.0,
                    tof_age_seconds: Some(0.0),
                };
                self.tof_vertical_velocity_mm_per_second = 0.0;
                self.acceleration_velocity_correction_mm_per_second = 0.0;
                self.sync_vertical_velocity();
            }
        }

        Ok(self.estimate)
    }

    pub fn predict(
        &mut self,
        vertical_acceleration_mm_per_second_squared: f32,
        sample_dt_seconds: f32,
    ) -> Result<VerticalEstimate, VerticalEstimatorError> {
        if !sample_dt_seconds.is_finite()
            || sample_dt_seconds <= 0.0
            || !(MIN_PREDICTION_DT_SECONDS..=MAX_PREDICTION_DT_SECONDS).contains(&sample_dt_seconds)
        {
            return Err(VerticalEstimatorError::InvalidDt);
        }

        if !vertical_acceleration_mm_per_second_squared.is_finite() {
            return Err(VerticalEstimatorError::NonFiniteVerticalAcceleration);
        }

        if self.last_tof_height_mm.is_none() {
            return Ok(self.estimate);
        }

        let mut candidate = *self;

        candidate.estimate.tof_age_seconds = candidate
            .estimate
            .tof_age_seconds
            .map(|tof_age_seconds| tof_age_seconds + sample_dt_seconds);

        let bounded_vertical_acceleration_mm_per_second_squared =
            vertical_acceleration_mm_per_second_squared.clamp(
                -GRAVITY_MM_PER_SECOND_SQUARED,
                GRAVITY_MM_PER_SECOND_SQUARED,
            );

        if candidate.acceleration_filter_initialized {
            let acceleration_decay =
                decay_for_sample_dt(sample_dt_seconds, ACCELERATION_FILTER_TIME_CONSTANT_SECONDS);
            candidate.filtered_vertical_acceleration_mm_per_second_squared = candidate
                .filtered_vertical_acceleration_mm_per_second_squared
                * acceleration_decay
                + bounded_vertical_acceleration_mm_per_second_squared * (1.0 - acceleration_decay);
        } else {
            candidate.filtered_vertical_acceleration_mm_per_second_squared =
                bounded_vertical_acceleration_mm_per_second_squared;
            candidate.acceleration_filter_initialized = true;
        }

        let velocity_correction_decay = decay_for_sample_dt(
            sample_dt_seconds,
            ACCELERATION_VELOCITY_LEAK_TIME_CONSTANT_SECONDS,
        );

        candidate.acceleration_velocity_correction_mm_per_second =
            (candidate.acceleration_velocity_correction_mm_per_second * velocity_correction_decay
                + candidate.filtered_vertical_acceleration_mm_per_second_squared
                    * ACCELERATION_VELOCITY_LEAK_TIME_CONSTANT_SECONDS
                    * (1.0 - velocity_correction_decay))
                .clamp(
                    -MAX_ACCELERATION_VELOCITY_CORRECTION_MM_PER_SECOND,
                    MAX_ACCELERATION_VELOCITY_CORRECTION_MM_PER_SECOND,
                );
        candidate.sync_vertical_velocity();
        let predicted_height = candidate.estimate.height_mm
            + candidate.estimate.vertical_velocity_mm_per_second * sample_dt_seconds;
        candidate.estimate.height_mm = predicted_height;

        if !candidate.estimate.height_mm.is_finite()
            || !candidate
                .estimate
                .vertical_velocity_mm_per_second
                .is_finite()
            || candidate
                .estimate
                .tof_age_seconds
                .is_some_and(|age_seconds| !age_seconds.is_finite())
        {
            return Err(VerticalEstimatorError::NonFiniteOutput);
        }

        candidate.estimate.height_mm = candidate
            .estimate
            .height_mm
            .clamp(0.0, f32::from(MAX_VALID_TOF_HEIGHT_MM));

        *self = candidate;
        Ok(self.estimate)
    }

    fn sync_vertical_velocity(&mut self) {
        self.estimate.vertical_velocity_mm_per_second = (self.tof_vertical_velocity_mm_per_second
            + self.acceleration_velocity_correction_mm_per_second)
            .clamp(
                -MAX_OBSERVED_VERTICAL_SPEED_MM_PER_SECOND,
                MAX_OBSERVED_VERTICAL_SPEED_MM_PER_SECOND,
            );
    }
}

fn next_tof_velocity_mm_per_second(
    current_tof: f32,
    measured_tof: f32,
    sample_dt_seconds: f32,
) -> f32 {
    let decay = decay_for_sample_dt(sample_dt_seconds, TOF_VELOCITY_FILTER_TIME_CONSTANT_SECONDS);
    let filtered_velocity = current_tof * decay + measured_tof * (1.0 - decay);
    let max_velocity_change_mm_per_second =
        MAX_TOF_VERTICAL_ACCELERATION_MM_PER_SECOND_SQUARED * sample_dt_seconds;
    let request_change_mm_per_second = filtered_velocity - current_tof;
    current_tof
        + (request_change_mm_per_second).clamp(
            -max_velocity_change_mm_per_second,
            max_velocity_change_mm_per_second,
        )
}

fn get_accepted_tof_height_mm(
    previous_height_mm: f32,
    measured_height_mm: f32,
    sample_dt_seconds: f32,
) -> f32 {
    let measurement_change_mm = measured_height_mm - previous_height_mm;
    let innovation_limit_mm =
        TOF_INNOVATION_MARGIN_MM + MAX_TOF_VERTICAL_SPEED_MM_PER_SECOND * sample_dt_seconds;

    let accepted_height_mm = if measurement_change_mm.abs() > innovation_limit_mm {
        let limited_change_mm = measurement_change_mm.clamp(
            -MAX_TOF_VERTICAL_SPEED_MM_PER_SECOND * sample_dt_seconds,
            MAX_TOF_VERTICAL_SPEED_MM_PER_SECOND * sample_dt_seconds,
        );
        previous_height_mm + limited_change_mm
    } else {
        measured_height_mm
    };

    accepted_height_mm
}

fn valid_height_mm(tof_measurement: TofMeasurement) -> Option<f32> {
    if tof_measurement.status == 0
        && tof_measurement.objects > 0
        && (0..=MAX_VALID_TOF_HEIGHT_MM).contains(&tof_measurement.range_mm)
    {
        Some(f32::from(tof_measurement.range_mm))
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_DT_SECONDS: f32 = 0.05;
    const PREDICTION_DT: f32 = 0.01;
    const TOLERANCE: f32 = 0.001;
    #[test]
    fn prevent_division_by_zero() {
        let invalid_dt_seconds = 0.0;
        let mut estimator = VerticalEstimator::new();
        let result = estimator.correct_tof(
            TofMeasurement {
                range_mm: 120,
                status: 0,
                objects: 1,
            },
            invalid_dt_seconds,
        );

        assert_eq!(result, Err(VerticalEstimatorError::InvalidDt));
    }

    #[test]
    fn first_valid_tof_sample_initializes_height_and_zero_velocity() {
        let mut estimator = VerticalEstimator::new();
        let result = estimator
            .correct_tof(
                TofMeasurement {
                    range_mm: 120,
                    status: 0,
                    objects: 1,
                },
                SAMPLE_DT_SECONDS,
            )
            .expect("should be valid");

        assert_eq!(result.height_mm, 120.0);
        assert_eq!(result.vertical_velocity_mm_per_second, 0.0);
    }

    #[test]
    fn second_tof_sample_updates_height_and_vertical_velocity() {
        let mut estimator = VerticalEstimator::new();
        let first_sample = TofMeasurement {
            range_mm: 100,
            status: 0,
            objects: 1,
        };
        let second_sample = TofMeasurement {
            range_mm: 105,
            status: 0,
            objects: 1,
        };

        let _ = estimator
            .correct_tof(first_sample, SAMPLE_DT_SECONDS)
            .expect("should be valid");

        let result = estimator
            .correct_tof(second_sample, SAMPLE_DT_SECONDS)
            .expect("should be valid");
        assert!(
            result.height_mm > 100.0 && result.height_mm < 105.0,
            "expected height between 100 and 105 mm, actual={}",
            result.height_mm
        );
        assert!(
            result.vertical_velocity_mm_per_second > 0.0,
            "expected vertical velocity to be positive, actual={}",
            result.vertical_velocity_mm_per_second
        );
    }

    #[test]
    fn large_tof_jump_does_not_create_unbounded_estimate() {
        let mut estimator = VerticalEstimator::new();
        let first_sample = TofMeasurement {
            range_mm: 100,
            status: 0,
            objects: 1,
        };
        let second_sample = TofMeasurement {
            range_mm: 700,
            status: 0,
            objects: 1,
        };

        let _ = estimator
            .correct_tof(first_sample, SAMPLE_DT_SECONDS)
            .expect("should be valid");

        let result = estimator
            .correct_tof(second_sample, SAMPLE_DT_SECONDS)
            .expect("should be valid");
        assert!(
            result.height_mm > 100.0 && result.height_mm <= 110.5,
            "expected height between 100 and 105 mm, actual={}",
            result.height_mm
        );
        assert!(
            result.vertical_velocity_mm_per_second > 0.0
                && result.vertical_velocity_mm_per_second
                    <= MAX_TOF_VERTICAL_ACCELERATION_MM_PER_SECOND_SQUARED * SAMPLE_DT_SECONDS,
            "expected vertical velocity to be 0 < velocity <= around 245 mm/s, actual={}",
            result.vertical_velocity_mm_per_second
        );
    }

    #[test]
    fn floor_contact_resets_height_and_velocity() {
        let mut estimator = VerticalEstimator::new();
        let first_sample = TofMeasurement {
            range_mm: 100,
            status: 0,
            objects: 1,
        };
        let second_sample = TofMeasurement {
            range_mm: 105,
            status: 0,
            objects: 1,
        };
        let third_sample = TofMeasurement {
            range_mm: 0,
            status: 0,
            objects: 1,
        };

        let _ = estimator
            .correct_tof(first_sample, SAMPLE_DT_SECONDS)
            .expect("should be valid");

        let result = estimator
            .correct_tof(second_sample, SAMPLE_DT_SECONDS)
            .expect("should be valid");

        assert!(
            result.vertical_velocity_mm_per_second > 0.0,
            "expected velocity to be positive, actual={}",
            result.vertical_velocity_mm_per_second
        );

        let result_after_hitting_floor = estimator
            .correct_tof(third_sample, SAMPLE_DT_SECONDS)
            .expect("should be valid");

        assert_eq!(result_after_hitting_floor.height_mm, 10.0);
        assert_eq!(
            result_after_hitting_floor.vertical_velocity_mm_per_second,
            0.0
        );
    }

    #[test]
    fn predict_advances_height_between_tof_samples() {
        let mut estimator = VerticalEstimator::new();
        let first_sample = TofMeasurement {
            range_mm: 100,
            status: 0,
            objects: 1,
        };
        let second_sample = TofMeasurement {
            range_mm: 105,
            status: 0,
            objects: 1,
        };

        let _ = estimator
            .correct_tof(first_sample, SAMPLE_DT_SECONDS)
            .expect("should be valid");

        let estimate_before_prediction = estimator
            .correct_tof(second_sample, SAMPLE_DT_SECONDS)
            .expect("should be valid");

        let prediction = estimator
            .predict(0.0, PREDICTION_DT)
            .expect("should be valid");
        let actual = prediction.height_mm;
        let expected = estimate_before_prediction.height_mm
            + estimate_before_prediction.vertical_velocity_mm_per_second * PREDICTION_DT;

        let difference = (actual - expected).abs();

        assert!(
            difference <= TOLERANCE,
            "mismatch: actual={actual}, expected={expected}, difference={difference},
      tolerance={TOLERANCE}"
        );
    }

    #[test]
    fn positive_vertical_acceleration_increases_velocity_between_tof_samples() {
        let mut estimator = VerticalEstimator::new();
        let first_sample = TofMeasurement {
            range_mm: 100,
            status: 0,
            objects: 1,
        };
        let _ = estimator
            .correct_tof(first_sample, SAMPLE_DT_SECONDS)
            .expect("should be valid");
        let prediction = estimator
            .predict(1000.0, PREDICTION_DT)
            .expect("should be valid");
        assert!(
            prediction.vertical_velocity_mm_per_second > 0.0
                && prediction.vertical_velocity_mm_per_second <= 40.0,
            "expected predicted velocity between 0.0 and 40.0, actual={}",
            prediction.vertical_velocity_mm_per_second
        );
        assert!(
            prediction.height_mm > 100.0,
            "expected predicted height to be over 100, actual={}",
            prediction.height_mm
        );
    }

    #[test]
    fn acceleration_velocity_correction_is_bounded_and_leaks_toward_zero() {
        let mut estimator = VerticalEstimator::new();
        let first_sample = TofMeasurement {
            range_mm: 100,
            status: 0,
            objects: 1,
        };
        let _ = estimator
            .correct_tof(first_sample, SAMPLE_DT_SECONDS)
            .expect("should be valid");
        let first_prediction = estimator
            .predict(1000.0, PREDICTION_DT)
            .expect("should be valid");
        let mut latest_prediction = first_prediction;
        for _ in 0..100 {
            let predict = estimator
                .predict(0.0, PREDICTION_DT)
                .expect("should be valid");
            latest_prediction = predict;
            assert!(predict.vertical_velocity_mm_per_second.abs() <= 40.0);
        }
        assert!(
            latest_prediction.vertical_velocity_mm_per_second.abs()
                < first_prediction.vertical_velocity_mm_per_second.abs()
        );
        assert!(latest_prediction.vertical_velocity_mm_per_second.abs() < 1.0);
    }

    #[test]
    fn tof_age_increases_during_prediction_and_resets_after_valid_tof() {
        let mut estimator = VerticalEstimator::new();
        let first_sample = TofMeasurement {
            range_mm: 100,
            status: 0,
            objects: 1,
        };
        let second_sample = TofMeasurement {
            range_mm: 101,
            status: 0,
            objects: 1,
        };
        let invalid_tof_sample = TofMeasurement {
            range_mm: 101,
            status: 1,
            objects: 1,
        };
        let result = estimator
            .correct_tof(first_sample, SAMPLE_DT_SECONDS)
            .expect("should be valid");
        assert_eq!(result.tof_age_seconds, Some(0.0));

        let prediction = estimator
            .predict(0.0, PREDICTION_DT)
            .expect("should be valid");
        let actual_age_seconds = prediction.tof_age_seconds.expect("should exist");
        let expected_age_seconds = PREDICTION_DT;
        let difference = (actual_age_seconds - expected_age_seconds).abs();

        assert!(
            difference <= TOLERANCE,
            "ToF age mismatch: actual={actual_age_seconds}, expected={expected_age_seconds}, difference={difference}"
        );

        let after_invalid_tof = estimator
            .correct_tof(invalid_tof_sample, SAMPLE_DT_SECONDS)
            .expect("should be valid");

        let actual_age_seconds = after_invalid_tof.tof_age_seconds.expect("should exist");
        let expected_age_seconds = PREDICTION_DT;
        let difference = (actual_age_seconds - expected_age_seconds).abs();

        assert!(
            difference <= TOLERANCE,
            "ToF age mismatch after invalid sample: actual={actual_age_seconds}, expected={expected_age_seconds}, difference={difference}"
        );

        let second_valid_result = estimator
            .correct_tof(second_sample, SAMPLE_DT_SECONDS)
            .expect("should be valid");
        assert_eq!(second_valid_result.tof_age_seconds, Some(0.0));
    }

    #[test]
    fn out_of_range_prediction_dt_is_rejected_without_committing_state() {
        let mut estimator = VerticalEstimator::new();
        let first_sample = TofMeasurement {
            range_mm: 100,
            status: 0,
            objects: 1,
        };
        let second_sample = TofMeasurement {
            range_mm: 105,
            status: 0,
            objects: 1,
        };
        let _ = estimator
            .correct_tof(first_sample, SAMPLE_DT_SECONDS)
            .expect("should be valid");
        let estimate_before_failure = estimator
            .correct_tof(second_sample, SAMPLE_DT_SECONDS)
            .expect("should be valid");
        let prediction = estimator.predict(0.0, f32::MAX);
        assert_eq!(prediction, Err(VerticalEstimatorError::InvalidDt));
        assert_eq!(estimator.estimate, estimate_before_failure);
    }

    #[test]
    fn tof_velocity_changes_sign_when_motion_reverses() {
        let mut estimator = VerticalEstimator::new();
        let first_sample = TofMeasurement {
            range_mm: 100,
            status: 0,
            objects: 1,
        };
        let second_sample = TofMeasurement {
            range_mm: 110,
            status: 0,
            objects: 1,
        };
        let third_sample = TofMeasurement {
            range_mm: 100,
            status: 0,
            objects: 1,
        };
        let _ = estimator
            .correct_tof(first_sample, SAMPLE_DT_SECONDS)
            .expect("should be valid");
        let ascent_estimate = estimator
            .correct_tof(second_sample, SAMPLE_DT_SECONDS)
            .expect("should be valid");
        assert!(
            ascent_estimate.vertical_velocity_mm_per_second > 0.0,
            "expected positive ascent velocity, actual={}",
            ascent_estimate.vertical_velocity_mm_per_second
        );

        let descent_estimate = estimator
            .correct_tof(third_sample, SAMPLE_DT_SECONDS)
            .expect("should be valid");
        assert!(
            descent_estimate.vertical_velocity_mm_per_second < 0.0,
            "expected negative descent velocity, actual={}",
            descent_estimate.vertical_velocity_mm_per_second
        );
    }
}
