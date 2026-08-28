pub struct AttitudeEstimator {
    roll_deg: f32,
    pitch_deg: f32,
    initialized: bool,
}

// Accelerometer tilt is a useful long-term gravity reference only while the
// vehicle is moving calmly. A fixed 0.98 blend at 400 Hz gave it an effective
// time constant of about 0.12 s, so translational acceleration could erase
// real gyro-integrated attitude during recovery. Use a dt-aware complementary
// correction and fade it out as body rate or non-gravity acceleration rises.
const ACCEL_CORRECTION_TIME_CONSTANT_SECONDS: f32 = 0.5;
const ACCEL_FULL_CONFIDENCE_ERROR_G: f32 = 0.05;
const ACCEL_ZERO_CONFIDENCE_ERROR_G: f32 = 0.20;
const ACCEL_ZERO_CONFIDENCE_RATE_DPS: f32 = 90.0;
const INITIAL_ACCEL_MIN_G: f32 = 0.8;
const INITIAL_ACCEL_MAX_G: f32 = 1.2;
const INITIAL_MAX_RATE_DPS: f32 = 10.0;

// Answers: What is current roll/pitch?
impl AttitudeEstimator {
    pub fn new() -> Self {
        Self {
            roll_deg: 0.0,
            pitch_deg: 0.0,
            initialized: false,
        }
    }

    pub fn update(
        &mut self,
        accel_g: (f32, f32, f32),
        gyro_dps: (f32, f32, f32),
        external_accel_correction_weight: f32,
        dt_seconds: f32,
    ) {
        let (accel_roll, accel_pitch) = Self::accel_angles_deg(accel_g);
        let accel_magnitude = accel_g.0.hypot(accel_g.1).hypot(accel_g.2);
        if !self.initialized
            && (INITIAL_ACCEL_MIN_G..=INITIAL_ACCEL_MAX_G).contains(&accel_magnitude)
            && gyro_dps.0.abs().max(gyro_dps.1.abs()) <= INITIAL_MAX_RATE_DPS
        {
            // Establish gravity while the vehicle is still stationary. During
            // flight, optical-flow motion weighting below prevents horizontal
            // acceleration from being mistaken for tilt.
            self.roll_deg = accel_roll;
            self.pitch_deg = accel_pitch;
            self.initialized = true;
            return;
        }
        let predict_pitch = self.pitch_deg + gyro_dps.0 * dt_seconds;
        let predict_roll = self.roll_deg + gyro_dps.1 * dt_seconds;

        let base_correction = 1.0 - (-dt_seconds / ACCEL_CORRECTION_TIME_CONSTANT_SECONDS).exp();
        let accel_confidence = Self::accel_confidence(accel_g);
        let rate_confidence = (1.0
            - gyro_dps.0.abs().max(gyro_dps.1.abs()) / ACCEL_ZERO_CONFIDENCE_RATE_DPS)
            .clamp(0.0, 1.0);
        let correction = base_correction
            * accel_confidence
            * rate_confidence
            * external_accel_correction_weight.clamp(0.0, 1.0);

        self.pitch_deg =
            predict_pitch + correction * Self::angle_error_deg(accel_pitch, predict_pitch);
        self.roll_deg = predict_roll + correction * Self::angle_error_deg(accel_roll, predict_roll);
    }

    fn accel_confidence(accel_g: (f32, f32, f32)) -> f32 {
        let (ax, ay, az) = accel_g;
        let magnitude_error_g = (ax.hypot(ay).hypot(az) - 1.0).abs();
        (1.0 - (magnitude_error_g - ACCEL_FULL_CONFIDENCE_ERROR_G)
            / (ACCEL_ZERO_CONFIDENCE_ERROR_G - ACCEL_FULL_CONFIDENCE_ERROR_G))
            .clamp(0.0, 1.0)
    }

    fn angle_error_deg(target_deg: f32, current_deg: f32) -> f32 {
        (target_deg - current_deg + 180.0).rem_euclid(360.0) - 180.0
    }

    fn accel_angles_deg(accel_g: (f32, f32, f32)) -> (f32, f32) {
        let (ax, ay, az) = accel_g;

        let roll_denom = (ay * ay + az * az).sqrt();
        let roll_rad = -(ax.atan2(roll_denom));
        let pitch_denom = (ax * ax + az * az).sqrt();
        let pitch_rad = ay.atan2(pitch_denom);

        (roll_rad.to_degrees(), pitch_rad.to_degrees())
    }

    pub fn pitch_deg(&self) -> f32 {
        self.pitch_deg
    }

    pub fn roll_deg(&self) -> f32 {
        self.roll_deg
    }
}
