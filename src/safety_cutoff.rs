pub const ANGLE_LIMIT_DEG: f32 = 35.0;
pub const RATE_LIMIT_DPS: f32 = 350.0;
pub const PERSISTENCE_SAMPLES: u8 = 8;

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum CutoffReason {
    None = 0,
    PitchAngle = 1,
    RollAngle = 2,
    PitchRate = 3,
    RollRate = 4,
    YawRate = 5,
    NonFinite = 6,
}

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
    pub fn update(
        &mut self,
        pitch_deg: f32,
        roll_deg: f32,
        gyro_dps: (f32, f32, f32),
    ) -> CutoffReason {
        if let Some(reason) = self.latched {
            return reason;
        }
        if !pitch_deg.is_finite()
            || !roll_deg.is_finite()
            || !gyro_dps.0.is_finite()
            || !gyro_dps.1.is_finite()
            || !gyro_dps.2.is_finite()
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
            gyro_dps.0.abs() > RATE_LIMIT_DPS,
        );
        Self::track(&mut self.roll_rate_count, gyro_dps.1.abs() > RATE_LIMIT_DPS);
        Self::track(&mut self.yaw_rate_count, gyro_dps.2.abs() > RATE_LIMIT_DPS);

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
