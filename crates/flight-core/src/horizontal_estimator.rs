//! World-forward and world-left position/velocity estimation.

use crate::flow_measurement::HorizontalVelocityMeasurement;

pub struct HorizontalEstimatorInput {
    pub dt_seconds: f32,
    pub measurement: Option<HorizontalVelocityMeasurement>,
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub struct HorizontalEstimate {
    pub world_forward_position_counts: f32,
    pub world_left_position_counts: f32,
    pub world_forward_velocity_counts_per_second: f32,
    pub world_left_velocity_counts_per_second: f32,
    pub measurement_age_seconds: Option<f32>,
}

#[derive(Clone, Copy)]
struct HorizontalAxisEstimator {
    position_counts: f32,
    velocity_counts_per_second: f32,
    position_variance_counts_squared: f32,
    position_velocity_covariance_counts_squared_per_second: f32,
    velocity_variance_counts_squared_per_second_squared: f32,
}

pub struct HorizontalEstimator {
    world_forward_axis: HorizontalAxisEstimator,
    world_left_axis: HorizontalAxisEstimator,
    has_position_reference: bool,
    measurement_age_seconds: Option<f32>,
}

#[derive(Debug, PartialEq)]
pub enum HorizontalEstimatorError {
    NonFiniteInput,
    InvalidDt,
    InvalidMeasurement,
}

const HORIZONTAL_ACCELERATION_STD_COUNTS_PER_SECOND_SQUARED: f32 = 8000.0;

const MIN_PREDICTION_DT_SECONDS: f32 = 0.001;
const MAX_PREDICTION_DT_SECONDS: f32 = 0.01;
const MAX_HORIZONTAL_POSITION_COUNTS: f32 = 2_000.0;
const MAX_HORIZONTAL_VELOCITY_COUNTS_PER_SECOND: f32 = 3_000.0;
const MAX_TRACKED_MEASUREMENT_AGE_SECONDS: f32 = 1.0;

impl HorizontalEstimator {
    pub fn new() -> Self {
        Self {
            world_forward_axis: HorizontalAxisEstimator::new(),
            world_left_axis: HorizontalAxisEstimator::new(),
            has_position_reference: false,
            measurement_age_seconds: None,
        }
    }

    pub fn update(
        &mut self,
        input: HorizontalEstimatorInput,
    ) -> Result<HorizontalEstimate, HorizontalEstimatorError> {
        if !input.dt_seconds.is_finite()
            || !(MIN_PREDICTION_DT_SECONDS..=MAX_PREDICTION_DT_SECONDS).contains(&input.dt_seconds)
        {
            return Err(HorizontalEstimatorError::InvalidDt);
        }
        let mut candidate_world_forward_axis = self.world_forward_axis;
        let mut candidate_world_left_axis = self.world_left_axis;
        let mut candidate_measurement_age_seconds = self.measurement_age_seconds;

        candidate_world_forward_axis.predict(input.dt_seconds)?;
        candidate_world_left_axis.predict(input.dt_seconds)?;

        if let Some(measurement) = input.measurement {
            if !measurement
                .world_forward_velocity_counts_per_second
                .is_finite()
                || !measurement
                    .world_left_velocity_counts_per_second
                    .is_finite()
                || !measurement
                    .velocity_variance_counts_squared_per_second_squared
                    .is_finite()
                || measurement.velocity_variance_counts_squared_per_second_squared <= 0.0
            {
                return Err(HorizontalEstimatorError::InvalidMeasurement);
            }
            if !self.has_position_reference {
                candidate_world_forward_axis.establish_position_reference();
                candidate_world_left_axis.establish_position_reference();
            }
            candidate_world_forward_axis.correct(
                measurement.world_forward_velocity_counts_per_second,
                measurement.velocity_variance_counts_squared_per_second_squared,
            )?;
            candidate_world_left_axis.correct(
                measurement.world_left_velocity_counts_per_second,
                measurement.velocity_variance_counts_squared_per_second_squared,
            )?;
            self.has_position_reference = true;
            candidate_measurement_age_seconds = Some(0.0)
        } else {
            candidate_measurement_age_seconds = candidate_measurement_age_seconds
                .map(|age| (age + input.dt_seconds).min(MAX_TRACKED_MEASUREMENT_AGE_SECONDS));
        }

        self.world_forward_axis = candidate_world_forward_axis;
        self.world_left_axis = candidate_world_left_axis;
        self.measurement_age_seconds = candidate_measurement_age_seconds;

        Ok(HorizontalEstimate {
            world_forward_position_counts: self.world_forward_axis.position_counts,
            world_left_position_counts: self.world_left_axis.position_counts,
            world_forward_velocity_counts_per_second: self
                .world_forward_axis
                .velocity_counts_per_second,
            world_left_velocity_counts_per_second: self.world_left_axis.velocity_counts_per_second,
            measurement_age_seconds: self.measurement_age_seconds,
        })
    }
}

impl HorizontalAxisEstimator {
    pub fn new() -> Self {
        Self {
            position_counts: 0.0,
            velocity_counts_per_second: 0.0,
            position_variance_counts_squared: 1.0,
            position_velocity_covariance_counts_squared_per_second: 0.0,
            velocity_variance_counts_squared_per_second_squared: 1_000_000.0,
        }
    }
    fn predict(&mut self, dt_seconds: f32) -> Result<(), HorizontalEstimatorError> {
        let dt_2 = dt_seconds * dt_seconds;
        let dt_3 = dt_2 * dt_seconds;
        let dt_4 = dt_3 * dt_seconds;
        let acceleration_variance = HORIZONTAL_ACCELERATION_STD_COUNTS_PER_SECOND_SQUARED.powi(2);

        let position_counts_candidate =
            self.position_counts + self.velocity_counts_per_second * dt_seconds;
        let position_variance_counts_squared_candidate = self.position_variance_counts_squared
            + 2.0 * dt_seconds * self.position_velocity_covariance_counts_squared_per_second
            + dt_2 * self.velocity_variance_counts_squared_per_second_squared
            + 0.25 * dt_4 * acceleration_variance;
        let position_velocity_covariance_counts_squared_per_second_candidate = self
            .position_velocity_covariance_counts_squared_per_second
            + dt_seconds * self.velocity_variance_counts_squared_per_second_squared
            + 0.5 * dt_3 * acceleration_variance;
        let velocity_variance_counts_squared_per_second_squared_candidate =
            self.velocity_variance_counts_squared_per_second_squared + dt_2 * acceleration_variance;

        if !position_counts_candidate.is_finite()
            || !position_variance_counts_squared_candidate.is_finite()
            || !position_velocity_covariance_counts_squared_per_second_candidate.is_finite()
            || !velocity_variance_counts_squared_per_second_squared_candidate.is_finite()
        {
            return Err(HorizontalEstimatorError::NonFiniteInput);
        }
        self.position_counts = position_counts_candidate.clamp(
            -MAX_HORIZONTAL_POSITION_COUNTS,
            MAX_HORIZONTAL_POSITION_COUNTS,
        );
        self.position_variance_counts_squared = position_variance_counts_squared_candidate;
        self.position_velocity_covariance_counts_squared_per_second =
            position_velocity_covariance_counts_squared_per_second_candidate;
        self.velocity_variance_counts_squared_per_second_squared =
            velocity_variance_counts_squared_per_second_squared_candidate;

        Ok(())
    }

    fn correct(
        &mut self,
        measured_velocity_counts_per_second: f32,
        measurement_variance_counts_squared_per_second_squared: f32,
    ) -> Result<(), HorizontalEstimatorError> {
        let velocity_innovation_counts_per_second =
            measured_velocity_counts_per_second - self.velocity_counts_per_second;
        let velocity_innovation_variance_counts_squared_per_second_squared = self
            .velocity_variance_counts_squared_per_second_squared
            + measurement_variance_counts_squared_per_second_squared;
        let position_kalman_gain_seconds = self
            .position_velocity_covariance_counts_squared_per_second
            / velocity_innovation_variance_counts_squared_per_second_squared;
        let velocity_kalman_gain_unitless = self
            .velocity_variance_counts_squared_per_second_squared
            / velocity_innovation_variance_counts_squared_per_second_squared;

        let position_counts_candidate = self.position_counts
            + position_kalman_gain_seconds * velocity_innovation_counts_per_second;
        let velocity_counts_per_second_candidate = self.velocity_counts_per_second
            + velocity_kalman_gain_unitless * velocity_innovation_counts_per_second;
        let position_variance_counts_squared_candidate = self.position_variance_counts_squared
            - position_kalman_gain_seconds
                * self.position_velocity_covariance_counts_squared_per_second;
        let position_velocity_covariance_counts_squared_per_second_candidate = self
            .position_velocity_covariance_counts_squared_per_second
            - velocity_kalman_gain_unitless
                * self.position_velocity_covariance_counts_squared_per_second;
        let velocity_variance_counts_squared_per_second_squared_candidate = self
            .velocity_variance_counts_squared_per_second_squared
            - velocity_kalman_gain_unitless
                * self.velocity_variance_counts_squared_per_second_squared;
        if !position_counts_candidate.is_finite()
            || !velocity_counts_per_second_candidate.is_finite()
            || !position_variance_counts_squared_candidate.is_finite()
            || !position_velocity_covariance_counts_squared_per_second_candidate.is_finite()
            || !velocity_variance_counts_squared_per_second_squared_candidate.is_finite()
        {
            return Err(HorizontalEstimatorError::NonFiniteInput);
        }
        // correct estimate
        self.position_counts = position_counts_candidate.clamp(
            -MAX_HORIZONTAL_POSITION_COUNTS,
            MAX_HORIZONTAL_POSITION_COUNTS,
        );
        self.velocity_counts_per_second = velocity_counts_per_second_candidate.clamp(
            -MAX_HORIZONTAL_VELOCITY_COUNTS_PER_SECOND,
            MAX_HORIZONTAL_VELOCITY_COUNTS_PER_SECOND,
        );
        // reduce covariance
        self.position_variance_counts_squared = position_variance_counts_squared_candidate.max(0.0);
        self.position_velocity_covariance_counts_squared_per_second =
            position_velocity_covariance_counts_squared_per_second_candidate;
        self.velocity_variance_counts_squared_per_second_squared =
            velocity_variance_counts_squared_per_second_squared_candidate.max(1.0);
        Ok(())
    }

    fn establish_position_reference(&mut self) {
        self.position_counts = 0.0;
        self.position_variance_counts_squared = 1.0;
        self.position_velocity_covariance_counts_squared_per_second = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT_SECONDS: f32 = 0.01;

    #[test]
    fn prediction_advances_position_using_current_velocity_without_changing_velocity() {
        let mut axis_estimator = HorizontalAxisEstimator::new();
        axis_estimator.position_counts = 10.0;
        axis_estimator.velocity_counts_per_second = 100.0;

        axis_estimator.predict(DT_SECONDS).expect("should be valid");
        assert_eq!(axis_estimator.position_counts, 11.0);
        assert_eq!(axis_estimator.velocity_counts_per_second, 100.0);

        assert_eq!(axis_estimator.position_variance_counts_squared, 101.16);
        assert_eq!(
            axis_estimator.position_velocity_covariance_counts_squared_per_second,
            10_032.0
        );
        assert_eq!(
            axis_estimator.velocity_variance_counts_squared_per_second_squared,
            1_006_400.0
        );
    }

    #[test]
    fn velocity_correction() {
        let mut axis_estimator = HorizontalAxisEstimator::new();
        axis_estimator.position_counts = 10.0;
        axis_estimator.velocity_counts_per_second = 100.0;
        axis_estimator.position_variance_counts_squared = 4.0;
        axis_estimator.position_velocity_covariance_counts_squared_per_second = 2.0;
        axis_estimator.velocity_variance_counts_squared_per_second_squared = 8.0;

        axis_estimator.correct(120.0, 8.0).expect("should be valid");

        assert_eq!(axis_estimator.position_counts, 12.5);
        assert_eq!(axis_estimator.velocity_counts_per_second, 110.0);
        assert_eq!(axis_estimator.position_variance_counts_squared, 3.75);
        assert_eq!(
            axis_estimator.position_velocity_covariance_counts_squared_per_second,
            1.0
        );
        assert_eq!(
            axis_estimator.velocity_variance_counts_squared_per_second_squared,
            4.0
        );
    }

    #[test]
    fn update_predicts_and_corrects_both_world_axes() {
        let mut estimator = HorizontalEstimator::new();
        estimator.has_position_reference = true;
        estimator.world_forward_axis.position_counts = 10.0;
        estimator.world_forward_axis.velocity_counts_per_second = 100.0;
        estimator
            .world_forward_axis
            .position_variance_counts_squared = 2.84;
        estimator
            .world_forward_axis
            .position_velocity_covariance_counts_squared_per_second = 32.0;
        estimator
            .world_forward_axis
            .velocity_variance_counts_squared_per_second_squared = 3600.0;

        estimator.world_left_axis.position_counts = -10.0;
        estimator.world_left_axis.velocity_counts_per_second = -100.0;
        estimator.world_left_axis.position_variance_counts_squared = 2.84;
        estimator
            .world_left_axis
            .position_velocity_covariance_counts_squared_per_second = 32.0;
        estimator
            .world_left_axis
            .velocity_variance_counts_squared_per_second_squared = 3600.0;

        let input = HorizontalEstimatorInput {
            dt_seconds: DT_SECONDS,
            measurement: Some(HorizontalVelocityMeasurement {
                velocity_variance_counts_squared_per_second_squared: 10_000.0,
                world_forward_velocity_counts_per_second: 200.0,
                world_left_velocity_counts_per_second: -200.0,
            }),
        };

        let result = estimator.update(input).expect("should be valid update");
        assert_eq!(result.world_forward_position_counts, 11.5);
        assert_eq!(result.world_left_position_counts, -11.5);

        assert_eq!(result.world_forward_velocity_counts_per_second, 150.0);
        assert_eq!(result.world_left_velocity_counts_per_second, -150.0);
    }

    #[test]
    fn failed_left_axis_prediction_does_not_commit_forward_axis() {
        let mut estimator = HorizontalEstimator::new();
        estimator.world_forward_axis.position_counts = 10.0;
        estimator.world_forward_axis.velocity_counts_per_second = 100.0;

        estimator.world_left_axis.position_counts = f32::MAX;
        estimator.world_left_axis.velocity_counts_per_second = f32::MAX;

        let input = HorizontalEstimatorInput {
            dt_seconds: DT_SECONDS,
            measurement: None,
        };

        let result = estimator.update(input);
        assert_eq!(result, Err(HorizontalEstimatorError::NonFiniteInput));
        assert_eq!(estimator.world_forward_axis.position_counts, 10.0);
        assert_eq!(
            estimator.world_forward_axis.velocity_counts_per_second,
            100.0
        );
    }

    #[test]
    fn first_measurement_establishes_position_zero_only_once() {
        let mut estimator = HorizontalEstimator::new();
        for _ in 0..10 {
            let input = HorizontalEstimatorInput {
                dt_seconds: DT_SECONDS,
                measurement: None,
            };
            let result = estimator.update(input).expect("to be a valid");
            assert_eq!(result.measurement_age_seconds, None);
        }

        let first_measurement = Some(HorizontalVelocityMeasurement {
            velocity_variance_counts_squared_per_second_squared: 10_000.0,
            world_forward_velocity_counts_per_second: 100.0,
            world_left_velocity_counts_per_second: -100.0,
        });
        let result = estimator
            .update(HorizontalEstimatorInput {
                dt_seconds: DT_SECONDS,
                measurement: first_measurement,
            })
            .expect("to be valid");

        assert_eq!(result.measurement_age_seconds, Some(0.0));
        assert_eq!(estimator.world_forward_axis.position_counts, 0.0);
        assert_eq!(estimator.world_left_axis.position_counts, 0.0);
        assert!(estimator.world_forward_axis.velocity_counts_per_second > 0.0);
        assert!(estimator.world_left_axis.velocity_counts_per_second < 0.0);

        let input = HorizontalEstimatorInput {
            dt_seconds: DT_SECONDS,
            measurement: None,
        };
        let result = estimator.update(input).expect("to be a valid");
        assert_eq!(result.measurement_age_seconds, Some(DT_SECONDS));

        let second_measurement = Some(HorizontalVelocityMeasurement {
            velocity_variance_counts_squared_per_second_squared: 10_000.0,
            world_forward_velocity_counts_per_second: 100.0,
            world_left_velocity_counts_per_second: -100.0,
        });

        let result = estimator
            .update(HorizontalEstimatorInput {
                dt_seconds: DT_SECONDS,
                measurement: second_measurement,
            })
            .expect("to be valid");

        assert_eq!(result.measurement_age_seconds, Some(0.0));
        assert!(estimator.world_forward_axis.position_counts > 0.0);
        assert!(estimator.world_left_axis.position_counts < 0.0);
    }
}
