pub struct PidOutput {
    pub pitch_correction: f32,
    pub roll_correction: f32,
    pub yaw_correction: f32,
    pub pitch_rate_integral_correction: f32,
    pub roll_rate_integral_correction: f32,
    pub yaw_rate_integral_correction: f32,
    pub pitch_rate_target_dps: f32,
    pub roll_rate_target_dps: f32,
    pub pitch_rate_error_dps: f32,
    pub roll_rate_error_dps: f32,
}

/// Inner-loop PID operating on measured body rate.
///
/// The derivative is taken from the measured rate rather than the rate error,
/// so changing the commanded angle does not create a derivative kick. The
/// output is bounded before accepting the integral update to avoid winding up
/// while a motor correction is saturated.
struct RatePid {
    kp: f32,
    ki: f32,
    kd: f32,
    high_error_braking_gain: f32,
    integral_output: f32,
    previous_rate_dps: Option<f32>,
}

impl RatePid {
    fn update(
        &mut self,
        rate_error_dps: f32,
        measured_rate_dps: f32,
        dt_seconds: f32,
        output_limit: f32,
    ) -> f32 {
        debug_assert!(output_limit.is_finite() && output_limit >= 0.0);
        let integral_candidate = (self.integral_output + self.ki * rate_error_dps * dt_seconds)
            .clamp(-MAX_RATE_INTEGRAL_OUTPUT, MAX_RATE_INTEGRAL_OUTPUT);
        let rate_derivative_dps2 = self
            .previous_rate_dps
            .map(|previous| (measured_rate_dps - previous) / dt_seconds)
            .unwrap_or(0.0);
        // Preserve the validated small-error gain around hover, but apply
        // progressively stronger braking once the measured rate falls far
        // behind the cascaded rate target. Run168 repeatedly had 60..80 dps
        // of rate error with only about half of the available correction, so
        // the vehicle crossed the flow attitude gate before full braking
        // arrived. This continuous term reaches the existing output limit
        // earlier without introducing a threshold step or changing authority.
        let high_error_magnitude =
            (rate_error_dps.abs() - HIGH_RATE_ERROR_BRAKING_START_DPS).max(0.0);
        let high_error_braking =
            rate_error_dps.signum() * self.high_error_braking_gain * high_error_magnitude;
        let unsaturated = self.kp * rate_error_dps + high_error_braking + integral_candidate
            - self.kd * rate_derivative_dps2;
        let correction = unsaturated.clamp(-output_limit, output_limit);

        // Do not integrate further into a saturated output. Keep integrating
        // when the integral would move the output back toward the valid range.
        if unsaturated.abs() <= output_limit
            || (unsaturated > output_limit && rate_error_dps < 0.0)
            || (unsaturated < -output_limit && rate_error_dps > 0.0)
        {
            self.integral_output = integral_candidate;
        }
        self.previous_rate_dps = Some(measured_rate_dps);
        correction
    }
}

pub const MAX_RATE_CORRECTION: f32 = 30.0;
const MAX_ANGLE_RATE_TARGET_DPS: f32 = 100.0;
const MAX_RATE_INTEGRAL_OUTPUT: f32 = 8.0;
const HIGH_RATE_ERROR_BRAKING_START_DPS: f32 = 30.0;
const ATTITUDE_HIGH_ERROR_BRAKING_GAIN: f32 = 0.30;

// Outer loop: attitude error becomes a desired body rate.
// Convert attitude error in degrees into a meaningful body-rate target.
// The previous 0.55 dps/deg target was too weak: at 18 degrees of tilt it
// requested only about 10 dps, allowing the vehicle to remain tilted while
// the inner rate loop was otherwise functioning.
const PITCH_ANGLE_KP: f32 = 4.0;
const ROLL_ANGLE_KP: f32 = 4.0;

// Inner loop: desired body rate is driven to the measured body rate.
// These gains are deliberately separate from the angle gains.
// Run202 reduced horizontal target saturation, but its clean interval still
// averaged 6.50 degrees of pitch error and 25.5 dps of pitch-rate error while
// pitch correction and every motor retained headroom. Increase only forward
// attitude response so the vehicle realizes the existing bounded braking
// target. Roll tracking was substantially better and remains unchanged;
// correction authority remains limited by MAX_RATE_CORRECTION below.
const PITCH_RATE_KP: f32 = 0.45;
const ROLL_RATE_KP: f32 = 0.30;
const PITCH_RATE_KI: f32 = 0.04;
const ROLL_RATE_KI: f32 = 0.04;
const PITCH_RATE_KD: f32 = 0.00040;
const ROLL_RATE_KD: f32 = 0.00040;

// Keep yaw inside the optical-flow acceptance gate instead of waiting until
// 100 dps to reach the correction limit.
const YAW_HEADING_KP: f32 = 1.0;
const MAX_YAW_RATE_TARGET_DPS: f32 = 30.0;
const YAW_RATE_DAMPING: f32 = 0.45;
const YAW_RATE_KI: f32 = 0.04;

pub struct PidController {
    target_pitch_deg: f32,
    target_roll_deg: f32,
    pitch: RatePid,
    roll: RatePid,
    yaw: RatePid,
}

impl PidController {
    pub fn new() -> Self {
        Self {
            target_pitch_deg: 0.0,
            target_roll_deg: 0.0,
            pitch: RatePid {
                kp: PITCH_RATE_KP,
                ki: PITCH_RATE_KI,
                kd: PITCH_RATE_KD,
                high_error_braking_gain: ATTITUDE_HIGH_ERROR_BRAKING_GAIN,
                // Run174 held a roughly -5 degree flow target for several
                // seconds, but the leaky integral settled near -1.7 and the
                // aircraft remained nose-up. Standard conditional integration
                // retains the trim needed to realize a persistent target.
                integral_output: 0.0,
                previous_rate_dps: None,
            },
            roll: RatePid {
                kp: ROLL_RATE_KP,
                ki: ROLL_RATE_KI,
                kd: ROLL_RATE_KD,
                high_error_braking_gain: ATTITUDE_HIGH_ERROR_BRAKING_GAIN,
                integral_output: 0.0,
                previous_rate_dps: None,
            },
            yaw: RatePid {
                kp: YAW_RATE_DAMPING,
                ki: YAW_RATE_KI,
                kd: 0.0,
                // Yaw is not part of the pitch/roll overshoot seen in run168.
                high_error_braking_gain: 0.0,
                // Run176 kept the correct positive yaw braking command while
                // heading drifted about 150 degrees. The two-second leak held
                // this integral near +1.9 instead of retaining the persistent
                // motor-torque trim. Use the same bounded conditional PI as
                // pitch and roll so heading bias can actually be cancelled.
                integral_output: 0.0,
                previous_rate_dps: None,
            },
        }
    }

    pub fn update_with_target_offsets(
        &mut self,
        current_pitch_deg: f32,
        current_roll_deg: f32,
        gyro_dps: (f32, f32, f32),
        pitch_target_offset_deg: f32,
        roll_target_offset_deg: f32,
        yaw_heading_deg: f32,
        dt_seconds: f32,
    ) -> PidOutput {
        let target_pitch = self.target_pitch_deg + pitch_target_offset_deg;
        let target_roll = self.target_roll_deg + roll_target_offset_deg;
        let pitch_error = target_pitch - current_pitch_deg;
        let roll_error = target_roll - current_roll_deg;
        let pitch_rate_target_dps = (PITCH_ANGLE_KP * pitch_error)
            .clamp(-MAX_ANGLE_RATE_TARGET_DPS, MAX_ANGLE_RATE_TARGET_DPS);
        let roll_rate_target_dps = (ROLL_ANGLE_KP * roll_error)
            .clamp(-MAX_ANGLE_RATE_TARGET_DPS, MAX_ANGLE_RATE_TARGET_DPS);
        let pitch_rate_error_dps = pitch_rate_target_dps - gyro_dps.0;
        let roll_rate_error_dps = roll_rate_target_dps - gyro_dps.1;

        let pitch_correction = self.pitch.update(
            pitch_rate_error_dps,
            gyro_dps.0,
            dt_seconds,
            MAX_RATE_CORRECTION,
        );
        let roll_correction = self.roll.update(
            roll_rate_error_dps,
            gyro_dps.1,
            dt_seconds,
            MAX_RATE_CORRECTION,
        );
        // Positive yaw correction was bench-validated to oppose positive yaw
        // rate, so yaw uses the measured rate directly as its damping error.
        // The mixer preserves every attitude differential by shifting only
        // collective. With pitch, roll, and yaw each bounded to +/-30 and all
        // mixed at 0.375, even the conservative sum of all three differential
        // magnitudes is only 67.5 duty, well inside the mixer's 0..220 range.
        // The former correction-
        // space budget therefore withheld yaw authority despite ample motor
        // headroom: in Run177 yaw repeatedly saturated at that artificial
        // limit while all four motors remained far from either bound. Keep
        // full pitch/roll authority and allow yaw its independently safe limit.
        let yaw_limit = MAX_RATE_CORRECTION;
        // Hold the heading captured at arming (the yaw estimator starts at
        // zero). Rate damping alone permits a constant motor-torque bias to
        // settle at a nonzero yaw rate, which starves optical flow. Convert
        // wrapped heading error into a bounded counter-rotation rate, then use
        // the existing bounded rate loop.
        let yaw_heading_error_deg = wrap_degrees(-yaw_heading_deg);
        let yaw_rate_target_dps = (YAW_HEADING_KP * yaw_heading_error_deg)
            .clamp(-MAX_YAW_RATE_TARGET_DPS, MAX_YAW_RATE_TARGET_DPS);
        let yaw_rate_error_dps = gyro_dps.2 - yaw_rate_target_dps;
        let yaw_correction =
            self.yaw
                .update(yaw_rate_error_dps, -gyro_dps.2, dt_seconds, yaw_limit);

        PidOutput {
            pitch_correction,
            roll_correction,
            yaw_correction,
            pitch_rate_integral_correction: self.pitch.integral_output,
            roll_rate_integral_correction: self.roll.integral_output,
            yaw_rate_integral_correction: self.yaw.integral_output,
            pitch_rate_target_dps,
            roll_rate_target_dps,
            pitch_rate_error_dps,
            roll_rate_error_dps,
        }
    }

    pub fn target_pitch_with_offset(&self, offset_deg: f32) -> f32 {
        self.target_pitch_deg + offset_deg
    }

    pub fn target_roll_with_offset(&self, offset_deg: f32) -> f32 {
        self.target_roll_deg + offset_deg
    }
}

fn wrap_degrees(angle_deg: f32) -> f32 {
    let wrapped = (angle_deg + 180.0).rem_euclid(360.0) - 180.0;
    if wrapped == -180.0 {
        180.0
    } else {
        wrapped
    }
}
