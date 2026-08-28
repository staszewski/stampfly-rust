#[derive(Debug, PartialEq, Clone)]
pub(crate) struct RatePid {
    kp: f32,
    ki: f32,
    kd: f32,
    integral_correction: f32,
    integral_limit: f32,
    output_limit: f32,
    high_error_braking_gain: f32,
    previous_measured_rate_dps: Option<f32>,
}

#[derive(Debug, PartialEq, Eq)]
pub enum RatePidError {
    NonFiniteOutput,
    NonFiniteInput,
    InvalidConstructorParams,
    InvalidDt,
}

const MIN_DT_SECONDS: f32 = 0.001;
const MAX_DT_SECONDS: f32 = 0.01;
const HIGH_RATE_ERROR_BRAKING_START_DPS: f32 = 30.0;

impl RatePid {
    pub(crate) fn new(
        kp: f32,
        ki: f32,
        kd: f32,
        output_limit: f32,
        integral_limit: f32,
        high_error_braking_gain: f32,
    ) -> Result<Self, RatePidError> {
        if !kp.is_finite()
            || !ki.is_finite()
            || !kd.is_finite()
            || !output_limit.is_finite()
            || !integral_limit.is_finite()
            || !high_error_braking_gain.is_finite()
        {
            return Err(RatePidError::NonFiniteInput);
        }
        if kp < 0.0
            || ki < 0.0
            || kd < 0.0
            || output_limit < 0.0
            || integral_limit < 0.0
            || high_error_braking_gain < 0.0
        {
            return Err(RatePidError::InvalidConstructorParams);
        }
        if integral_limit > output_limit {
            return Err(RatePidError::InvalidConstructorParams);
        }
        Ok(Self {
            kp,
            ki,
            kd,
            integral_correction: 0.0,
            integral_limit,
            output_limit,
            high_error_braking_gain,
            previous_measured_rate_dps: None,
        })
    }

    pub(crate) fn update(
        &mut self,
        target_rate_dps: f32,
        measured_rate_dps: f32,
        dt_seconds: f32,
    ) -> Result<f32, RatePidError> {
        if !target_rate_dps.is_finite() || !measured_rate_dps.is_finite() {
            return Err(RatePidError::NonFiniteInput);
        }
        if !dt_seconds.is_finite() || dt_seconds < MIN_DT_SECONDS || dt_seconds > MAX_DT_SECONDS {
            return Err(RatePidError::InvalidDt);
        }
        let measured_rate_derivative_dps2 = match self.previous_measured_rate_dps {
            Some(previous) => (measured_rate_dps - previous) / dt_seconds,
            None => 0.0,
        };
        let rate_error_dps = target_rate_dps - measured_rate_dps;
        let amount_above_threshold =
            (rate_error_dps.abs() - HIGH_RATE_ERROR_BRAKING_START_DPS).max(0.0);
        let extra_braking =
            rate_error_dps.signum() * amount_above_threshold * self.high_error_braking_gain;
        let proportional_correction = self.kp * rate_error_dps;
        let integral_candidate = self.integral_correction + self.ki * rate_error_dps * dt_seconds;
        let derivative_correction = -self.kd * measured_rate_derivative_dps2;

        if !proportional_correction.is_finite()
            || !integral_candidate.is_finite()
            || !derivative_correction.is_finite()
            || !measured_rate_derivative_dps2.is_finite()
            || !extra_braking.is_finite()
        {
            return Err(RatePidError::NonFiniteOutput);
        }

        let bounded_integral_candidate =
            integral_candidate.clamp(-self.integral_limit, self.integral_limit);

        let combined_correction = proportional_correction
            + bounded_integral_candidate
            + derivative_correction
            + extra_braking;

        if !combined_correction.is_finite() {
            return Err(RatePidError::NonFiniteOutput);
        }

        if combined_correction.abs() <= self.output_limit
            || (combined_correction > self.output_limit && rate_error_dps < 0.0)
            || (combined_correction < -self.output_limit && rate_error_dps > 0.0)
        {
            self.integral_correction = bounded_integral_candidate;
        }
        let bounded_correction = combined_correction.clamp(-self.output_limit, self.output_limit);
        self.previous_measured_rate_dps = Some(measured_rate_dps);
        Ok(bounded_correction)
    }

    pub(crate) fn integral_correction(&self) -> f32 {
        self.integral_correction
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.0025;
    #[test]
    fn rate_pid_inputs() {
        let test_cases = [
            (
                f32::INFINITY,
                0.0,
                30.0,
                1.0,
                Err(RatePidError::NonFiniteInput),
            ),
            (
                -5.0,
                0.0,
                30.0,
                1.0,
                Err(RatePidError::InvalidConstructorParams),
            ),
            (f32::NAN, 0.0, 30.0, 1.0, Err(RatePidError::NonFiniteInput)),
            (
                5.0,
                0.0,
                30.0,
                1.0,
                Ok(RatePid {
                    kp: 5.0,
                    ki: 0.0,
                    kd: 0.0,
                    integral_correction: 0.0,
                    integral_limit: 1.0,
                    output_limit: 30.0,
                    high_error_braking_gain: 0.0,
                    previous_measured_rate_dps: None,
                }),
            ),
            (
                5.0,
                0.0,
                30.0,
                -1.0,
                Err(RatePidError::InvalidConstructorParams),
            ),
            (
                5.0,
                0.0,
                30.0,
                40.0,
                Err(RatePidError::InvalidConstructorParams),
            ),
        ];
        for (kp, ki, output_limit, integral_limit, output) in test_cases {
            let result = RatePid::new(kp, ki, 0.0, output_limit, integral_limit, 0.0);
            assert_eq!(result, output);
        }
    }

    #[test]
    fn rate_pid_update() {
        let test_cases = [
            (0.5, 0.0, 30.0, 20.0, -4.0, Ok(12.0)),
            (0.5, 0.0, 30.0, 100.0, 30.0, Ok(30.0)),
            (0.5, 0.0, 30.0, -100.0, 30.0, Ok(-30.0)),
            (
                1.0,
                0.0,
                30.0,
                f32::NAN,
                f32::INFINITY,
                Err(RatePidError::NonFiniteInput),
            ),
            (
                1.0,
                0.0,
                30.0,
                0.0,
                f32::INFINITY,
                Err(RatePidError::NonFiniteInput),
            ),
            (
                f32::MAX,
                0.0,
                30.0,
                2.0,
                0.0,
                Err(RatePidError::NonFiniteOutput),
            ),
        ];
        for (kp, ki, output_limit, target_rate_dps, measured_rate_dps, expected_correction) in
            test_cases
        {
            let mut rate_pid = RatePid::new(kp, ki, 0.0, output_limit, 3.0, 0.0)
                .expect("test gain should be valid");
            let result = rate_pid.update(target_rate_dps, measured_rate_dps, DT);
            assert_eq!(result, expected_correction);
        }
    }

    #[test]
    fn integral_accumulates() {
        let (kp, ki, output_limit, target_rate_dps, measured_rate_dps, first_result, second_result) =
            (0.0, 50.0, 30.0, 8.0, 0.0, Ok(1.0), Ok(2.0));
        let mut rate_pid =
            RatePid::new(kp, ki, 0.0, output_limit, 2.0, 0.0).expect("test gain should be valid");

        let first_update = rate_pid.update(target_rate_dps, measured_rate_dps, DT);
        assert_eq!(first_update, first_result);

        let second_update = rate_pid.update(target_rate_dps, measured_rate_dps, DT);
        assert_eq!(second_update, second_result);

        let third_update = rate_pid.update(target_rate_dps, measured_rate_dps, DT);
        assert_eq!(third_update, second_result);
    }

    #[test]
    fn combined_correction_overflow_is_rejected() {
        let (kp, ki, output_limit, target_rate_dps, measured_rate_dps, expected) = (
            f32::MAX,
            f32::MAX,
            f32::MAX,
            1.0,
            0.0,
            Err(RatePidError::NonFiniteOutput),
        );
        let mut rate_pid = RatePid::new(kp, ki, 0.0, output_limit, f32::MAX, 0.0)
            .expect("test gain should be valid");

        let result = rate_pid.update(target_rate_dps, measured_rate_dps, DT);
        assert_eq!(result, expected);
    }

    #[test]
    fn invalid_dt() {
        let invalid_dt = 0.05;
        let (kp, ki, output_limit, target_rate_dps, measured_rate_dps, expected) =
            (0.0, 5.0, 30.0, 1.0, 0.0, Err(RatePidError::InvalidDt));
        let mut rate_pid =
            RatePid::new(kp, ki, 0.0, output_limit, 3.0, 0.0).expect("test gain should be valid");

        let result = rate_pid.update(target_rate_dps, measured_rate_dps, invalid_dt);
        assert_eq!(result, expected);
    }

    #[test]
    fn anti_windup() {
        let (kp, ki, integral_limit, output_limit, target_rate_dps, measured_rate_dps) =
            (1.0, 100.0, 8.0, 10.0, 20.0, 0.0);
        let mut rate_pid = RatePid::new(kp, ki, 0.0, output_limit, integral_limit, 0.0)
            .expect("test gain should be valid");

        let first_update = rate_pid.update(target_rate_dps, measured_rate_dps, DT);
        assert_eq!(first_update, Ok(10.0));

        let second_update = rate_pid.update(0.0, measured_rate_dps, DT);
        assert_eq!(second_update, Ok(0.0));
    }

    #[test]
    fn derivative_stage() {
        let (kp, ki, kd, integral_limit, output_limit, target_rate_dps, measured_rate_dps) =
            (0.0, 0.0, 0.01, 8.0, 30.0, 0.0, 10.0);
        let mut rate_pid = RatePid::new(kp, ki, kd, output_limit, integral_limit, 0.0)
            .expect("test gain should be valid");

        let first_update = rate_pid.update(target_rate_dps, measured_rate_dps, DT);
        assert_eq!(first_update, Ok(0.0));

        let second_update = rate_pid.update(100.0, measured_rate_dps, DT);
        assert_eq!(second_update, Ok(0.0));

        let third_update = rate_pid.update(100.0, 12.0, DT);
        assert_eq!(third_update, Ok(-8.0));
    }

    #[test]
    fn high_rate_error_adds_progressive_braking() {
        let (
            kp,
            ki,
            kd,
            integral_limit,
            output_limit,
            high_braking_error_gain,
            target_rate_dps,
            measured_rate_dps,
        ) = (0.45, 0.0, 0.0, 8.0, 30.0, 0.30, 60.0, 10.0);
        let mut positive_rate_pid = RatePid::new(
            kp,
            ki,
            kd,
            output_limit,
            integral_limit,
            high_braking_error_gain,
        )
        .expect("test gain should be valid");

        let result = positive_rate_pid.update(target_rate_dps, measured_rate_dps, DT);
        assert_eq!(result, Ok(28.5));

        let (
            kp,
            ki,
            kd,
            integral_limit,
            output_limit,
            high_braking_error_gain,
            target_rate_dps,
            measured_rate_dps,
        ) = (0.45, 0.0, 0.0, 8.0, 30.0, 0.30, -60.0, -10.0);
        let mut negative_rate_pid = RatePid::new(
            kp,
            ki,
            kd,
            output_limit,
            integral_limit,
            high_braking_error_gain,
        )
        .expect("test gain should be valid");

        let result = negative_rate_pid.update(target_rate_dps, measured_rate_dps, DT);
        assert_eq!(result, Ok(-28.5));
    }
}
