use crate::altitude_control::valid_height_mm;
use crate::pmw3901::FlowSample;

#[derive(Copy, Clone, Default)]
pub struct FlowState {
    pub pitch_offset_deg: f32,
    pub roll_offset_deg: f32,
    pub sample_accepted: bool,
    pub control_active: bool,
    pub flow_reject_reason: u8,
    pub world_forward_position_counts: f32,
    pub world_left_position_counts: f32,
    pub world_forward_velocity_counts_per_second: f32,
    pub world_left_velocity_counts_per_second: f32,
    pub world_forward_velocity_target_counts_per_second: f32,
    pub world_left_velocity_target_counts_per_second: f32,
    pub world_forward_velocity_innovation_counts_per_second: f32,
    pub world_left_velocity_innovation_counts_per_second: f32,
    pub body_forward_rotation_counts: f32,
    pub body_left_rotation_counts: f32,
    pub body_forward_translation_counts: f32,
    pub body_left_translation_counts: f32,
}

#[derive(Copy, Clone)]
struct HorizontalAxisEstimator {
    position_counts: f32,
    velocity_counts_per_second: f32,
    position_variance: f32,
    position_velocity_covariance: f32,
    velocity_variance: f32,
}

impl HorizontalAxisEstimator {
    fn new() -> Self {
        Self {
            // Position is defined to be zero when the first qualified flow
            // velocity arrives. Initial velocity is deliberately uncertain.
            position_counts: 0.0,
            velocity_counts_per_second: 0.0,
            position_variance: 1.0,
            position_velocity_covariance: 0.0,
            velocity_variance: 1_000_000.0,
        }
    }

    fn predict(&mut self, dt_seconds: f32) {
        self.position_counts += self.velocity_counts_per_second * dt_seconds;

        // Constant-velocity process model with horizontal acceleration as
        // process noise. This is the estimator equivalent of allowing the
        // vehicle to accelerate between 50 Hz optical-flow measurements.
        let acceleration_variance = HORIZONTAL_ACCELERATION_STD_COUNTS_PER_SECOND2.powi(2);
        let dt2 = dt_seconds * dt_seconds;
        let dt3 = dt2 * dt_seconds;
        let dt4 = dt2 * dt2;
        let old_position_variance = self.position_variance;
        let old_covariance = self.position_velocity_covariance;
        let old_velocity_variance = self.velocity_variance;
        self.position_variance = old_position_variance
            + 2.0 * dt_seconds * old_covariance
            + dt2 * old_velocity_variance
            + 0.25 * dt4 * acceleration_variance;
        self.position_velocity_covariance =
            old_covariance + dt_seconds * old_velocity_variance + 0.5 * dt3 * acceleration_variance;
        self.velocity_variance = old_velocity_variance + dt2 * acceleration_variance;
    }

    fn establish_position_reference(&mut self) {
        // Position hold is relative to the first qualified flow observation.
        // Prediction before that observation may increase velocity covariance,
        // but it must not allow the first velocity innovation to retroactively
        // move the newly established position origin.
        self.position_counts = 0.0;
        self.position_variance = 1.0;
        self.position_velocity_covariance = 0.0;
    }

    fn update_velocity(
        &mut self,
        measured_velocity_counts_per_second: f32,
        measurement_variance: f32,
    ) -> f32 {
        let innovation = measured_velocity_counts_per_second - self.velocity_counts_per_second;
        let innovation_variance = self.velocity_variance + measurement_variance;
        if !innovation_variance.is_finite() || innovation_variance <= 0.0 {
            return innovation;
        }

        let position_gain = self.position_velocity_covariance / innovation_variance;
        let velocity_gain = self.velocity_variance / innovation_variance;
        let old_covariance = self.position_velocity_covariance;
        let old_velocity_variance = self.velocity_variance;
        self.position_counts += position_gain * innovation;
        self.velocity_counts_per_second += velocity_gain * innovation;
        self.position_variance = (self.position_variance - position_gain * old_covariance).max(0.0);
        self.position_velocity_covariance -= position_gain * old_velocity_variance;
        self.velocity_variance =
            (self.velocity_variance - velocity_gain * old_velocity_variance).max(1.0);
        innovation
    }
}

pub struct FlowStateEstimator {
    forward: HorizontalAxisEstimator,
    left: HorizontalAxisEstimator,
    state: FlowState,
    has_position_reference: bool,
    seconds_since_measurement: f32,
    has_flow_diagnostic: bool,
}

const PMW_MOTION_BIT: u8 = 0x80;
const FLOW_REJECT_MOTION: u8 = 1 << 0;
const FLOW_REJECT_SQUAL: u8 = 1 << 1;
const FLOW_REJECT_ATTITUDE: u8 = 1 << 2;
const FLOW_REJECT_GYRO_XY: u8 = 1 << 3;
const FLOW_REJECT_YAW: u8 = 1 << 4;
const FLOW_REJECT_TOF: u8 = 1 << 5;
const FLOW_REJECT_DELTA_X: u8 = 1 << 6;
const FLOW_REJECT_DELTA_Y: u8 = 1 << 7;
// The motion bit and raw-delta flags remain diagnostics. A high-quality
// no-motion sample is a valid zero-velocity observation, while oversized
// deltas are bounded before entering the estimator.
const FLOW_MEASUREMENT_REJECT_MASK: u8 = FLOW_REJECT_SQUAL
    | FLOW_REJECT_ATTITUDE
    | FLOW_REJECT_GYRO_XY
    | FLOW_REJECT_YAW
    | FLOW_REJECT_TOF;
const MIN_SQUAL: u8 = 40;
const MIN_FLOW_HEIGHT_MM: f32 = 25.0;
const MAX_FLOW_HEIGHT_MM: f32 = 600.0;
const FLOW_REFERENCE_HEIGHT_MM: f32 = 180.0;
const FLOW_REFERENCE_HEIGHT_METERS: f32 = FLOW_REFERENCE_HEIGHT_MM / 1000.0;
const FLOW_DEADZONE_COUNTS: f32 = 3.0;
const MAX_RAW_COUNTS_PER_SAMPLE: f32 = 120.0;
const PMW_COUNTS_PER_RADIAN: f32 = 385.0;
const MAX_VELOCITY_COUNTS_PER_SECOND: f32 = 3000.0;
const MAX_POSITION_COUNTS: f32 = 2000.0;

// Measurement gates protect the estimator, not the controller. The previous
// +/-10 degree and 30 dps yaw gates repeatedly switched a valid position
// correction off while the vehicle was still recoverable.
const MAX_FLOW_MEASUREMENT_ATTITUDE_DEG: f32 = 20.0;
const MAX_FLOW_MEASUREMENT_GYRO_XY_DPS: f32 = 175.0;
pub const MAX_FLOW_MEASUREMENT_YAW_DPS: f32 = 90.0;
const MAX_MEASUREMENT_AGE_SECONDS: f32 = 0.25;

// A two-state Kalman filter replaces the velocity low-pass filter, position
// integrator, trust streak, capture mode, and dropout decay. Noise values are
// expressed in the retained reference-height count units.
const HORIZONTAL_ACCELERATION_STD_COUNTS_PER_SECOND2: f32 = 8000.0;
const FLOW_VELOCITY_MEASUREMENT_STD_COUNTS_PER_SECOND: f32 = 150.0;

// Conventional position -> velocity -> attitude cascade. Positive pitch
// opposes positive-forward velocity. Roll has the opposite actuator sign:
// negative roll opposes positive-left velocity. Vector limiting preserves the
// world correction direction before the actuator-specific roll sign is applied.
const POSITION_TO_VELOCITY_GAIN_PER_SECOND: f32 = 0.4;
const MAX_POSITION_VELOCITY_TARGET_COUNTS_PER_SECOND: f32 = 600.0;
// Run201's faster inner attitude loop captured the initial forward escape, but
// the 6/s velocity loop then drove repeated full-vector reversals: forward
// position recovered from 656 to 262 counts before the growing oscillation
// reached the +2000 clamp. Restore the velocity loop to its linear 3/s
// bandwidth while retaining the improved attitude response and the 8-degree
// ceiling as a safety bound for genuinely large errors.
const HORIZONTAL_VELOCITY_GAIN_PER_SECOND: f32 = 3.0;
const STANDARD_GRAVITY_METERS_PER_SECOND2: f32 = 9.80665;
const RADIANS_TO_DEGREES: f32 = 180.0 / core::f32::consts::PI;
// One reference-height count represents height / counts-per-radian meters.
// Convert velocity error to acceleration with a 3/s loop bandwidth, then use
// the small-angle acceleration ~= gravity * tilt relationship. This replaces
// the former empirical direct count-to-angle gains.
const VELOCITY_ERROR_TO_ANGLE_GAIN_DEG_PER_COUNT_PER_SECOND: f32 =
    FLOW_REFERENCE_HEIGHT_METERS / PMW_COUNTS_PER_RADIAN * HORIZONTAL_VELOCITY_GAIN_PER_SECOND
        / STANDARD_GRAVITY_METERS_PER_SECOND2
        * RADIANS_TO_DEGREES;
// Run188 spent 95.8% of its clean flow-controlled interval on the 6-degree
// vector limit while the measured return velocity still pointed away from the
// origin. The closed-loop response verified both actuator signs: negative
// pitch reduced negative-forward velocity and positive roll reduced
// negative-left velocity. Increase only the braking authority, not estimator
// or cascade gains. Eight degrees remains inside the altitude attitude gate.
const MAX_HORIZONTAL_TARGET_DEG: f32 = 8.0;
const _: () = assert!(MAX_HORIZONTAL_TARGET_DEG < MAX_FLOW_MEASUREMENT_ATTITUDE_DEG);

// Linear acceleration can have a one-g magnitude and therefore pass a norm
// check while corrupting accelerometer tilt. Optical flow supplies the missing
// observability: fade accelerometer correction out as corrected image
// translation rises, and restore it only when the image is actually still.
const ACCEL_CORRECTION_FULL_FLOW_COUNTS: f32 = 5.0;
const ACCEL_CORRECTION_ZERO_FLOW_COUNTS: f32 = 15.0;

impl FlowStateEstimator {
    pub fn new() -> Self {
        Self {
            forward: HorizontalAxisEstimator::new(),
            left: HorizontalAxisEstimator::new(),
            state: FlowState::default(),
            has_position_reference: false,
            seconds_since_measurement: MAX_MEASUREMENT_AGE_SECONDS,
            has_flow_diagnostic: false,
        }
    }

    pub fn update(
        &mut self,
        sample: FlowSample,
        current_pitch_deg: f32,
        current_roll_deg: f32,
        gyro_dps: (f32, f32, f32),
        gyro_delta_angle_deg: (f32, f32),
        yaw_heading_deg: f32,
        sample_dt_seconds: f32,
        tof_range_mm: i16,
        tof_status: u8,
        tof_objects: u8,
    ) -> FlowState {
        let dt_seconds = sample_dt_seconds.clamp(0.001, 0.1);
        self.forward.predict(dt_seconds);
        self.left.predict(dt_seconds);

        let flow_reject_reason = sample_reject_reason(
            sample,
            current_pitch_deg,
            current_roll_deg,
            gyro_dps,
            tof_range_mm,
            tof_status,
            tof_objects,
        );
        // Run197 provided a closed-loop translation sign test: visible
        // forward travel produced persistently positive dy, but mapping it to
        // negative forward motion commanded negative (nose-down) pitch and
        // reinforced the escape until position reached -2000 counts. Use
        // vehicle-forward = +dy so the same motion commands positive nose-up
        // braking. Flip the forward rotation model with the measurement axis
        // so pure pitch rotation still subtracts from translation. The
        // independently calibrated vehicle-left = +dx mapping is unchanged.
        let body_forward_rotation_counts =
            -gyro_delta_angle_deg.0.to_radians() * PMW_COUNTS_PER_RADIAN;
        let body_left_rotation_counts = gyro_delta_angle_deg.1.to_radians() * PMW_COUNTS_PER_RADIAN;
        let body_forward_translation_counts =
            f32::from(sample.delta_y) - body_forward_rotation_counts;
        let body_left_translation_counts = f32::from(sample.delta_x) - body_left_rotation_counts;
        self.has_flow_diagnostic = true;

        let sample_accepted = flow_reject_reason & FLOW_MEASUREMENT_REJECT_MASK == 0;
        let mut forward_innovation = 0.0;
        let mut left_innovation = 0.0;
        if sample_accepted {
            if !self.has_position_reference {
                self.forward.establish_position_reference();
                self.left.establish_position_reference();
            }
            let height_scale =
                flow_height_scale(tof_range_mm, tof_status, tof_objects).unwrap_or(1.0);
            let max_counts_per_sample = MAX_VELOCITY_COUNTS_PER_SECOND * dt_seconds;
            let body_forward_counts =
                bounded_counts(body_forward_translation_counts, max_counts_per_sample)
                    * height_scale;
            let body_left_counts =
                bounded_counts(body_left_translation_counts, max_counts_per_sample) * height_scale;
            let (world_forward_counts, world_left_counts) =
                rotate_body_to_world(body_forward_counts, body_left_counts, yaw_heading_deg);
            let world_forward_velocity = world_forward_counts / dt_seconds;
            let world_left_velocity = world_left_counts / dt_seconds;
            let measurement_variance = flow_velocity_measurement_variance(sample.squal);
            forward_innovation = self
                .forward
                .update_velocity(world_forward_velocity, measurement_variance);
            left_innovation = self
                .left
                .update_velocity(world_left_velocity, measurement_variance);
            self.has_position_reference = true;
            self.seconds_since_measurement = 0.0;
        } else {
            self.seconds_since_measurement = (self.seconds_since_measurement + dt_seconds).min(1.0);
        }

        self.forward.position_counts = self
            .forward
            .position_counts
            .clamp(-MAX_POSITION_COUNTS, MAX_POSITION_COUNTS);
        self.left.position_counts = self
            .left
            .position_counts
            .clamp(-MAX_POSITION_COUNTS, MAX_POSITION_COUNTS);

        let world_forward_velocity_target =
            (-POSITION_TO_VELOCITY_GAIN_PER_SECOND * self.forward.position_counts).clamp(
                -MAX_POSITION_VELOCITY_TARGET_COUNTS_PER_SECOND,
                MAX_POSITION_VELOCITY_TARGET_COUNTS_PER_SECOND,
            );
        let world_left_velocity_target =
            (-POSITION_TO_VELOCITY_GAIN_PER_SECOND * self.left.position_counts).clamp(
                -MAX_POSITION_VELOCITY_TARGET_COUNTS_PER_SECOND,
                MAX_POSITION_VELOCITY_TARGET_COUNTS_PER_SECOND,
            );
        // Run198 confirmed the forward actuator polarity directly: positive
        // pitch produced positive PMW dy and increasingly positive estimated
        // forward velocity. Use the standard target-minus-measurement error so
        // positive forward escape commands negative pitch and opposes motion.
        let world_pitch_target = VELOCITY_ERROR_TO_ANGLE_GAIN_DEG_PER_COUNT_PER_SECOND
            * (world_forward_velocity_target - self.forward.velocity_counts_per_second);
        let world_roll_target = VELOCITY_ERROR_TO_ANGLE_GAIN_DEG_PER_COUNT_PER_SECOND
            * (self.left.velocity_counts_per_second - world_left_velocity_target);
        let (limited_world_pitch, limited_world_roll) = vector_limit(
            world_pitch_target,
            world_roll_target,
            MAX_HORIZONTAL_TARGET_DEG,
        );
        let (body_pitch_target, body_left_braking_target) =
            rotate_world_to_body(limited_world_pitch, limited_world_roll, yaw_heading_deg);
        // Pitch and roll do not share the same controller sign on this
        // airframe. Forward pitch polarity is handled by the target-minus-
        // measurement error above. Run175 independently showed that negative
        // roll produced increasingly negative-left (physical rightward)
        // velocity, so retain the final body-left-to-roll inversion.
        let body_roll_target = -body_left_braking_target;
        let control_active = self.has_position_reference
            && self.seconds_since_measurement <= MAX_MEASUREMENT_AGE_SECONDS;

        self.state = FlowState {
            pitch_offset_deg: if control_active {
                body_pitch_target
            } else {
                0.0
            },
            roll_offset_deg: if control_active {
                body_roll_target
            } else {
                0.0
            },
            sample_accepted,
            control_active,
            flow_reject_reason,
            world_forward_position_counts: self.forward.position_counts,
            world_left_position_counts: self.left.position_counts,
            world_forward_velocity_counts_per_second: self.forward.velocity_counts_per_second,
            world_left_velocity_counts_per_second: self.left.velocity_counts_per_second,
            world_forward_velocity_target_counts_per_second: world_forward_velocity_target,
            world_left_velocity_target_counts_per_second: world_left_velocity_target,
            world_forward_velocity_innovation_counts_per_second: forward_innovation,
            world_left_velocity_innovation_counts_per_second: left_innovation,
            body_forward_rotation_counts,
            body_left_rotation_counts,
            body_forward_translation_counts,
            body_left_translation_counts,
        };
        self.state
    }

    pub fn attitude_accel_correction_weight(&self) -> f32 {
        if !self.has_flow_diagnostic {
            return 1.0;
        }
        let motion_counts = self
            .state
            .body_forward_translation_counts
            .hypot(self.state.body_left_translation_counts);
        (1.0 - (motion_counts - ACCEL_CORRECTION_FULL_FLOW_COUNTS)
            / (ACCEL_CORRECTION_ZERO_FLOW_COUNTS - ACCEL_CORRECTION_FULL_FLOW_COUNTS))
            .clamp(0.0, 1.0)
    }
}

fn sample_reject_reason(
    sample: FlowSample,
    current_pitch_deg: f32,
    current_roll_deg: f32,
    gyro_dps: (f32, f32, f32),
    tof_range_mm: i16,
    tof_status: u8,
    tof_objects: u8,
) -> u8 {
    let mut reason = 0;
    if sample.motion & PMW_MOTION_BIT == 0 {
        reason |= FLOW_REJECT_MOTION;
    }
    if sample.squal < MIN_SQUAL {
        reason |= FLOW_REJECT_SQUAL;
    }
    if current_pitch_deg.abs() > MAX_FLOW_MEASUREMENT_ATTITUDE_DEG
        || current_roll_deg.abs() > MAX_FLOW_MEASUREMENT_ATTITUDE_DEG
    {
        reason |= FLOW_REJECT_ATTITUDE;
    }
    if gyro_dps.0.abs() > MAX_FLOW_MEASUREMENT_GYRO_XY_DPS
        || gyro_dps.1.abs() > MAX_FLOW_MEASUREMENT_GYRO_XY_DPS
    {
        reason |= FLOW_REJECT_GYRO_XY;
    }
    if gyro_dps.2.abs() > MAX_FLOW_MEASUREMENT_YAW_DPS {
        reason |= FLOW_REJECT_YAW;
    }
    if flow_height_scale(tof_range_mm, tof_status, tof_objects).is_none() {
        reason |= FLOW_REJECT_TOF;
    }
    if f32::from(sample.delta_x).abs() > MAX_RAW_COUNTS_PER_SAMPLE {
        reason |= FLOW_REJECT_DELTA_X;
    }
    if f32::from(sample.delta_y).abs() > MAX_RAW_COUNTS_PER_SAMPLE {
        reason |= FLOW_REJECT_DELTA_Y;
    }
    reason
}

fn flow_height_scale(range_mm: i16, status: u8, objects: u8) -> Option<f32> {
    let height_mm = valid_height_mm(range_mm, status, objects)?;
    if !(MIN_FLOW_HEIGHT_MM..=MAX_FLOW_HEIGHT_MM).contains(&height_mm) {
        return None;
    }
    // PMW counts represent angular image motion. Multiplying by measured
    // height converts them to a consistent reference-height translation.
    // The previous 0.5 lower clamp overstated motion by up to 3.6x at the
    // 25 mm acquisition height and was not part of the sensor model.
    Some(height_mm / FLOW_REFERENCE_HEIGHT_MM)
}

fn bounded_counts(counts: f32, max_counts_per_sample: f32) -> f32 {
    if counts.abs() <= FLOW_DEADZONE_COUNTS {
        0.0
    } else {
        counts.clamp(-max_counts_per_sample, max_counts_per_sample)
    }
}

fn flow_velocity_measurement_variance(squal: u8) -> f32 {
    let quality = f32::from(squal) / f32::from(u8::MAX);
    let quality_scale = 1.0 + 3.0 * (1.0 - quality).powi(2);
    (FLOW_VELOCITY_MEASUREMENT_STD_COUNTS_PER_SECOND * quality_scale).powi(2)
}

fn vector_limit(forward: f32, left: f32, max_magnitude: f32) -> (f32, f32) {
    let magnitude = forward.hypot(left);
    if magnitude > max_magnitude && magnitude > 0.0 {
        let scale = max_magnitude / magnitude;
        (forward * scale, left * scale)
    } else {
        (forward, left)
    }
}

fn rotate_body_to_world(forward: f32, left: f32, yaw_heading_deg: f32) -> (f32, f32) {
    let (sin_yaw, cos_yaw) = yaw_heading_deg.to_radians().sin_cos();
    (
        cos_yaw * forward - sin_yaw * left,
        sin_yaw * forward + cos_yaw * left,
    )
}

fn rotate_world_to_body(forward: f32, left: f32, yaw_heading_deg: f32) -> (f32, f32) {
    let (sin_yaw, cos_yaw) = yaw_heading_deg.to_radians().sin_cos();
    (
        cos_yaw * forward + sin_yaw * left,
        -sin_yaw * forward + cos_yaw * left,
    )
}
