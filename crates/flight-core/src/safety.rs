use crate::types::{BodyAngularRateDps, CutoffReason};

const ANGLE_LIMIT_DEG: f32 = 35.0;
const RATE_LIMIT_DPS: f32 = 350.0;
const PERSISTENCE_SAMPLES: u8 = 8;

#[derive(Default)]
pub struct SafetyCutoff {
    pitch_angle_count: u8,
    roll_angle_count: u8,
    pitch_rate_count: u8,
    roll_rate_count: u8,
    yaw_rate_count: u8,
    latched: Option<CutoffReason>,
}

impl SafetyCutoff {
    pub fn new() -> Self {
        Self {
            pitch_angle_count: 0,
            roll_angle_count: 0,
            pitch_rate_count: 0,
            roll_rate_count: 0,
            yaw_rate_count: 0,
            latched: None,
        }
    }

    pub fn update(
        &mut self,
        pitch_deg: f32,
        roll_deg: f32,
        measured_body_rate_dps: BodyAngularRateDps,
    ) -> CutoffReason {
        if let Some(reason) = self.latched {
            return reason;
        }
        if !pitch_deg.is_finite()
            || !roll_deg.is_finite()
            || !measured_body_rate_dps.pitch_dps.is_finite()
            || !measured_body_rate_dps.roll_dps.is_finite()
            || !measured_body_rate_dps.yaw_dps.is_finite()
        {
            self.latched = Some(CutoffReason::NonFinite);
            return CutoffReason::NonFinite;
        }

        Self::track(
            &mut self.pitch_angle_count,
            pitch_deg.abs() > ANGLE_LIMIT_DEG,
        );
        Self::track(&mut self.roll_angle_count, roll_deg.abs() > ANGLE_LIMIT_DEG);
        Self::track(
            &mut self.pitch_rate_count,
            measured_body_rate_dps.pitch_dps.abs() > RATE_LIMIT_DPS,
        );
        Self::track(
            &mut self.roll_rate_count,
            measured_body_rate_dps.roll_dps.abs() > RATE_LIMIT_DPS,
        );
        Self::track(
            &mut self.yaw_rate_count,
            measured_body_rate_dps.yaw_dps.abs() > RATE_LIMIT_DPS,
        );

        let reason = if self.pitch_angle_count >= PERSISTENCE_SAMPLES {
            CutoffReason::PitchAngle
        } else if self.roll_angle_count >= PERSISTENCE_SAMPLES {
            CutoffReason::RollAngle
        } else if self.pitch_rate_count >= PERSISTENCE_SAMPLES {
            CutoffReason::PitchRate
        } else if self.roll_rate_count >= PERSISTENCE_SAMPLES {
            CutoffReason::RollRate
        } else if self.yaw_rate_count >= PERSISTENCE_SAMPLES {
            CutoffReason::YawRate
        } else {
            CutoffReason::None
        };
        if reason != CutoffReason::None {
            self.latched = Some(reason);
        }

        reason
    }

    fn track(count: &mut u8, exceeded: bool) {
        *count = if exceeded { count.saturating_add(1) } else { 0 };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn non_finite_input_latches_immediately() {
        let mut safety = SafetyCutoff::new();
        let gyro_input = BodyAngularRateDps {
            pitch_dps: 0.0,
            roll_dps: 0.0,
            yaw_dps: 0.0,
        };
        let first_reason = safety.update(f32::NAN, 0.0, gyro_input);
        assert_eq!(first_reason, CutoffReason::NonFinite);

        let second_reason = safety.update(0.0, 0.0, gyro_input);
        assert_eq!(second_reason, CutoffReason::NonFinite);
    }

    #[test]
    fn pitch_angle_requires_consecutive_samples_and_resets_on_safe_sample() {
        let mut safety = SafetyCutoff::new();
        let zero_rates = BodyAngularRateDps {
            pitch_dps: 0.0,
            roll_dps: 0.0,
            yaw_dps: 0.0,
        };
        let angle_over_limit = ANGLE_LIMIT_DEG + 1.0;

        let boundary_reason = safety.update(ANGLE_LIMIT_DEG, 0.0, zero_rates);
        assert_eq!(boundary_reason, CutoffReason::None);

        for _ in 0..(PERSISTENCE_SAMPLES - 1) {
            let reason = safety.update(angle_over_limit, 0.0, zero_rates);
            assert_eq!(reason, CutoffReason::None);
        }

        let reset_reason = safety.update(0.0, 0.0, zero_rates);
        assert_eq!(reset_reason, CutoffReason::None);

        for _ in 0..(PERSISTENCE_SAMPLES - 1) {
            let reason = safety.update(angle_over_limit, 0.0, zero_rates);
            assert_eq!(reason, CutoffReason::None);
        }

        let trigger_reason = safety.update(angle_over_limit, 0.0, zero_rates);
        assert_eq!(trigger_reason, CutoffReason::PitchAngle);
    }

    #[test]
    fn conditions_map_to_expected_reasons() {
        let test_cases = [
            (
                0.0,
                -(ANGLE_LIMIT_DEG + 1.0),
                BodyAngularRateDps {
                    pitch_dps: 0.0,
                    roll_dps: 0.0,
                    yaw_dps: 0.0,
                },
                CutoffReason::RollAngle,
            ),
            (
                0.0,
                0.0,
                BodyAngularRateDps {
                    pitch_dps: RATE_LIMIT_DPS + 1.0,
                    roll_dps: 0.0,
                    yaw_dps: 0.0,
                },
                CutoffReason::PitchRate,
            ),
            (
                0.0,
                0.0,
                BodyAngularRateDps {
                    pitch_dps: 0.0,
                    roll_dps: -(RATE_LIMIT_DPS + 1.0),
                    yaw_dps: 0.0,
                },
                CutoffReason::RollRate,
            ),
            (
                0.0,
                0.0,
                BodyAngularRateDps {
                    pitch_dps: 0.0,
                    roll_dps: 0.0,
                    yaw_dps: RATE_LIMIT_DPS + 1.0,
                },
                CutoffReason::YawRate,
            ),
            (
                ANGLE_LIMIT_DEG + 1.0,
                ANGLE_LIMIT_DEG + 1.0,
                BodyAngularRateDps {
                    pitch_dps: RATE_LIMIT_DPS + 1.0,
                    roll_dps: RATE_LIMIT_DPS + 1.0,
                    yaw_dps: RATE_LIMIT_DPS + 1.0,
                },
                CutoffReason::PitchAngle,
            ),
        ];

        for (pitch_deg, roll_deg, body_rate_dps, expected_reason) in test_cases {
            let mut safety = SafetyCutoff::new();

            for _ in 0..(PERSISTENCE_SAMPLES - 1) {
                let reason = safety.update(pitch_deg, roll_deg, body_rate_dps);
                assert_eq!(reason, CutoffReason::None);
            }

            let trigger_reason = safety.update(pitch_deg, roll_deg, body_rate_dps);
            assert_eq!(trigger_reason, expected_reason);
        }
    }
}
