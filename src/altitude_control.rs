pub const ALTITUDE_TARGET_MM: f32 = 80.0;

const MIN_CONTROL_HEIGHT_MM: f32 = 10.0;
const MAX_VALID_HEIGHT_MM: i16 = 700;

// Cascaded altitude control: position error commands a bounded vertical
// velocity, and a conditionally integrated velocity PI loop commands
// collective. Keeping these two jobs separate lets the integral establish the
// steady collective needed for hover without integrating into saturation.
const HEIGHT_TO_VELOCITY_KP_PER_SECOND: f32 = 2.0;
const MAX_ASCENT_VELOCITY_MM_PER_SECOND: f32 = 70.0;
const MAX_DESCENT_VELOCITY_MM_PER_SECOND: f32 = 20.0;
const VELOCITY_KP_THRUST_PER_MM_PER_SECOND: f32 = 0.06;
const VELOCITY_KI_THRUST_PER_MM: f32 = 0.04;
const MAX_THRUST_OFFSET: f32 = 24.0;
const MAX_VELOCITY_INTEGRAL_OUTPUT: f32 = 24.0;

const MAX_OBSERVED_VERTICAL_SPEED_MM_PER_SECOND: f32 = 1_000.0;
const TOF_POSITION_CORRECTION_GAIN: f32 = 0.35;
const TOF_VELOCITY_FILTER_TIME_CONSTANT_SECONDS: f32 = 0.06;
const INVALID_TOF_COAST_SECONDS: f32 = 0.15;

// Acceleration contributes only a short-lived velocity correction. Run184's
// motor-on vertical specific force shifted by about -0.13 g, so direct
// integration falsely railed velocity at -1000 mm/s. A leaky integral cannot
// accumulate that bias, and the explicit authority cap limits its effect on
// collective to 40 mm/s * 0.06 duty/(mm/s) = 2.4 duty. ToF remains dominant.
const GRAVITY_MM_PER_SECOND_SQUARED: f32 = 9_806.65;
const ACCELERATION_FILTER_TIME_CONSTANT_SECONDS: f32 = 0.06;
const ACCELERATION_VELOCITY_LEAK_TIME_CONSTANT_SECONDS: f32 = 0.12;
const MAX_VERTICAL_ACCELERATION_MM_PER_SECOND_SQUARED: f32 = GRAVITY_MM_PER_SECOND_SQUARED;
const MAX_ACCELERATION_VELOCITY_CORRECTION_MM_PER_SECOND: f32 = 40.0;

// Bound each ToF update symmetrically. Run184 proved that rejecting a real
// upward trajectory and later rebasing height while retaining contradictory
// negative velocity can command maximum climb at more than 200 mm altitude.
// Rate-bounded assimilation lets repeated motion accumulate continuously in
// the observer without differentiating a single surface step.
const MAX_TOF_VERTICAL_SPEED_MM_PER_SECOND: f32 = 600.0;
// Run204 kept attitude, yaw, and flow qualified, but individual ToF
// publications still changed the filtered vertical velocity by several
// hundred mm/s and reversed collective in one update. A real vertical speed
// may remain large, but it cannot change discontinuously. Limit only the rate
// at which the ToF velocity state can move. Run206's half-gravity envelope
// visibly held position without floor contact while still allowing sustained
// motion to reach the existing +/-600 mm/s velocity bound.
const MAX_TOF_VERTICAL_ACCELERATION_MM_PER_SECOND_SQUARED: f32 =
    0.5 * GRAVITY_MM_PER_SECOND_SQUARED;
const TOF_INNOVATION_MARGIN_MM: f32 = 8.0;

#[derive(Copy, Clone, Debug, Default)]
pub struct AltitudeHoldState {
    pub thrust_offset: f32,
    pub hover_thrust_offset: f32,
    pub height_mm: f32,
    pub vertical_velocity_mm_per_second: f32,
    pub vertical_velocity_target_mm_per_second: f32,
    pub tof_innovation_rejected: bool,
    pub active: bool,
}

pub struct AltitudeHoldController {
    last_tof_height_mm: Option<f32>,
    seconds_since_valid_tof: f32,
    tof_vertical_velocity_mm_per_second: f32,
    filtered_vertical_acceleration_mm_per_second_squared: f32,
    acceleration_filter_initialized: bool,
    acceleration_velocity_correction_mm_per_second: f32,
    velocity_integral_output: f32,
    state: AltitudeHoldState,
}

impl AltitudeHoldController {
    pub fn new() -> Self {
        Self {
            last_tof_height_mm: None,
            seconds_since_valid_tof: f32::INFINITY,
            tof_vertical_velocity_mm_per_second: 0.0,
            filtered_vertical_acceleration_mm_per_second_squared: 0.0,
            acceleration_filter_initialized: false,
            acceleration_velocity_correction_mm_per_second: 0.0,
            velocity_integral_output: 0.0,
            state: AltitudeHoldState::default(),
        }
    }

    // ToF velocity provides the constant-velocity prediction. The acceleration
    // path may lead a reversal between ToF samples but cannot own the state.
    pub fn predict(
        &mut self,
        vertical_acceleration_mm_per_second_squared: f32,
        dt_seconds: f32,
    ) -> AltitudeHoldState {
        if !dt_seconds.is_finite() || dt_seconds <= 0.0 {
            return self.state;
        }

        self.seconds_since_valid_tof += dt_seconds;
        let bounded_acceleration = vertical_acceleration_mm_per_second_squared.clamp(
            -MAX_VERTICAL_ACCELERATION_MM_PER_SECOND_SQUARED,
            MAX_VERTICAL_ACCELERATION_MM_PER_SECOND_SQUARED,
        );
        if self.acceleration_filter_initialized {
            let acceleration_decay =
                decay_for_sample_dt(dt_seconds, ACCELERATION_FILTER_TIME_CONSTANT_SECONDS);
            self.filtered_vertical_acceleration_mm_per_second_squared =
                self.filtered_vertical_acceleration_mm_per_second_squared * acceleration_decay
                    + bounded_acceleration * (1.0 - acceleration_decay);
        } else {
            self.filtered_vertical_acceleration_mm_per_second_squared = bounded_acceleration;
            self.acceleration_filter_initialized = true;
        }

        let velocity_correction_decay =
            decay_for_sample_dt(dt_seconds, ACCELERATION_VELOCITY_LEAK_TIME_CONSTANT_SECONDS);
        self.acceleration_velocity_correction_mm_per_second =
            (self.acceleration_velocity_correction_mm_per_second * velocity_correction_decay
                + self.filtered_vertical_acceleration_mm_per_second_squared
                    * ACCELERATION_VELOCITY_LEAK_TIME_CONSTANT_SECONDS
                    * (1.0 - velocity_correction_decay))
                .clamp(
                    -MAX_ACCELERATION_VELOCITY_CORRECTION_MM_PER_SECOND,
                    MAX_ACCELERATION_VELOCITY_CORRECTION_MM_PER_SECOND,
                );
        self.sync_vertical_velocity();
        if self.last_tof_height_mm.is_some() {
            self.state.height_mm = (self.state.height_mm
                + self.state.vertical_velocity_mm_per_second * dt_seconds)
                .clamp(0.0, f32::from(MAX_VALID_HEIGHT_MM));
        }
        self.state
    }

    // ToF owns the long-term vertical state. Run186 showed that accumulating a
    // small fraction of position innovation into velocity introduced too much
    // phase lag: range had already reversed upward at ticks 2244..2316 while
    // the observer still reported -375..-66 mm/s and commanded more climb.
    // Use publication-timestamped range slope instead. One time constant of
    // filtering rejects single-sample range noise without hiding a real
    // direction change for several hundred milliseconds.
    pub fn correct_tof(
        &mut self,
        range_mm: i16,
        status: u8,
        objects: u8,
        sample_dt_seconds: f32,
    ) -> AltitudeHoldState {
        self.state.tof_innovation_rejected = false;
        let Some(measured_height_mm) = valid_height_mm(range_mm, status, objects) else {
            // Prediction bridges brief invalid publications. The bounded coast
            // timeout in update_control prevents indefinite blind control.
            return self.state;
        };

        self.seconds_since_valid_tof = 0.0;
        if measured_height_mm <= MIN_CONTROL_HEIGHT_MM {
            // A valid reading at the sensor floor is a contact-state
            // observation, not continued free fall. After the Run195 object
            // impact, retaining -160..-570 mm/s through repeated 0 mm ToF
            // publications held maximum positive recovery against the floor
            // and launched each subsequent bounce. Re-anchor the observer at
            // contact so the cascaded controller returns to its normal bounded
            // ascent command instead of carrying impact velocity into the next
            // flight segment. Recovery remains unrestricted until contact is
            // actually measured.
            self.last_tof_height_mm = Some(MIN_CONTROL_HEIGHT_MM);
            self.state.height_mm = MIN_CONTROL_HEIGHT_MM;
            self.tof_vertical_velocity_mm_per_second = 0.0;
            self.acceleration_velocity_correction_mm_per_second = 0.0;
            self.sync_vertical_velocity();
            return self.state;
        }
        let previous_height_mm = self.last_tof_height_mm;
        let accepted_height_mm = match previous_height_mm {
            None => {
                self.last_tof_height_mm = Some(measured_height_mm);
                self.state.height_mm = measured_height_mm;
                self.tof_vertical_velocity_mm_per_second = 0.0;
                self.acceleration_velocity_correction_mm_per_second = 0.0;
                self.sync_vertical_velocity();
                return self.state;
            }
            Some(previous_height_mm) => {
                let innovation_limit_mm = TOF_INNOVATION_MARGIN_MM
                    + MAX_TOF_VERTICAL_SPEED_MM_PER_SECOND * sample_dt_seconds;
                let measurement_change_mm = measured_height_mm - previous_height_mm;
                if measurement_change_mm.abs() > innovation_limit_mm {
                    self.state.tof_innovation_rejected = true;
                    previous_height_mm
                        + measurement_change_mm.clamp(
                            -MAX_TOF_VERTICAL_SPEED_MM_PER_SECOND * sample_dt_seconds,
                            MAX_TOF_VERTICAL_SPEED_MM_PER_SECOND * sample_dt_seconds,
                        )
                } else {
                    measured_height_mm
                }
            }
        };

        self.last_tof_height_mm = Some(accepted_height_mm);
        let measured_vertical_velocity_mm_per_second = previous_height_mm
            .map(|previous_height_mm| (accepted_height_mm - previous_height_mm) / sample_dt_seconds)
            .unwrap_or(0.0)
            .clamp(
                -MAX_TOF_VERTICAL_SPEED_MM_PER_SECOND,
                MAX_TOF_VERTICAL_SPEED_MM_PER_SECOND,
            );
        let velocity_decay =
            decay_for_sample_dt(sample_dt_seconds, TOF_VELOCITY_FILTER_TIME_CONSTANT_SECONDS);
        let filtered_vertical_velocity_mm_per_second = self.tof_vertical_velocity_mm_per_second
            * velocity_decay
            + measured_vertical_velocity_mm_per_second * (1.0 - velocity_decay);
        let max_velocity_change_mm_per_second =
            MAX_TOF_VERTICAL_ACCELERATION_MM_PER_SECOND_SQUARED * sample_dt_seconds;
        let requested_velocity_change_mm_per_second =
            filtered_vertical_velocity_mm_per_second - self.tof_vertical_velocity_mm_per_second;
        self.tof_vertical_velocity_mm_per_second += requested_velocity_change_mm_per_second.clamp(
            -max_velocity_change_mm_per_second,
            max_velocity_change_mm_per_second,
        );
        let observer_innovation_mm = accepted_height_mm - self.state.height_mm;
        self.state.height_mm += TOF_POSITION_CORRECTION_GAIN * observer_innovation_mm;
        self.sync_vertical_velocity();
        self.state
    }

    pub fn update_control(
        &mut self,
        dt_seconds: f32,
        control_enabled: bool,
        integral_enabled: bool,
    ) -> AltitudeHoldState {
        let tof_recent = self.last_tof_height_mm.is_some()
            && self.seconds_since_valid_tof <= INVALID_TOF_COAST_SECONDS;
        if !control_enabled || !tof_recent {
            self.state.thrust_offset = 0.0;
            self.state.vertical_velocity_target_mm_per_second = 0.0;
            self.state.hover_thrust_offset = self.velocity_integral_output;
            self.state.active = false;
            return self.state;
        }

        let height_error_mm = ALTITUDE_TARGET_MM - self.state.height_mm;
        let velocity_target_mm_per_second = (HEIGHT_TO_VELOCITY_KP_PER_SECOND * height_error_mm)
            .clamp(
                -MAX_DESCENT_VELOCITY_MM_PER_SECOND,
                MAX_ASCENT_VELOCITY_MM_PER_SECOND,
            );
        let velocity_error_mm_per_second =
            velocity_target_mm_per_second - self.state.vertical_velocity_mm_per_second;
        let integral_candidate = (self.velocity_integral_output
            + VELOCITY_KI_THRUST_PER_MM * velocity_error_mm_per_second * dt_seconds)
            .clamp(0.0, MAX_VELOCITY_INTEGRAL_OUTPUT);
        let unsaturated_thrust_offset = integral_candidate
            + VELOCITY_KP_THRUST_PER_MM_PER_SECOND * velocity_error_mm_per_second;

        let integrating_into_upper_limit =
            unsaturated_thrust_offset >= MAX_THRUST_OFFSET && velocity_error_mm_per_second > 0.0;
        let integrating_into_lower_limit =
            unsaturated_thrust_offset <= -MAX_THRUST_OFFSET && velocity_error_mm_per_second < 0.0;
        // Run196's hover trim remained effectively zero (0.005 duty mean,
        // 0.008 maximum) because the former learner required the aircraft to
        // already be within 10 mm of target and below 30 mm/s. The controller
        // therefore had no steady-state collective estimate and alternated
        // proportional recovery with braking. Use the velocity loop as a
        // genuine PI controller: integrate whenever altitude control is fully
        // qualified, except when the candidate would integrate farther into
        // an existing output limit. A velocity-error reversal is still free to
        // unwind the integral immediately.
        if integral_enabled && !integrating_into_upper_limit && !integrating_into_lower_limit {
            self.velocity_integral_output = integral_candidate;
        }

        self.state.thrust_offset = (self.velocity_integral_output
            + VELOCITY_KP_THRUST_PER_MM_PER_SECOND * velocity_error_mm_per_second)
            .clamp(-MAX_THRUST_OFFSET, MAX_THRUST_OFFSET);
        self.state.vertical_velocity_target_mm_per_second = velocity_target_mm_per_second;
        self.state.hover_thrust_offset = self.velocity_integral_output;
        self.state.active = true;
        self.state
    }

    fn sync_vertical_velocity(&mut self) {
        self.state.vertical_velocity_mm_per_second = (self.tof_vertical_velocity_mm_per_second
            + self.acceleration_velocity_correction_mm_per_second)
            .clamp(
                -MAX_OBSERVED_VERTICAL_SPEED_MM_PER_SECOND,
                MAX_OBSERVED_VERTICAL_SPEED_MM_PER_SECOND,
            );
    }
}

pub fn valid_height_mm(range_mm: i16, status: u8, objects: u8) -> Option<f32> {
    if status == 0 && objects > 0 && (0..=MAX_VALID_HEIGHT_MM).contains(&range_mm) {
        // A status=0/object-present reading below 10 mm is floor proximity,
        // not a reason to erase positive recovery collective.
        Some(f32::from(range_mm).max(MIN_CONTROL_HEIGHT_MM))
    } else {
        None
    }
}

pub fn earth_vertical_acceleration_mm_per_second_squared(
    accel_g: (f32, f32, f32),
    pitch_deg: f32,
    roll_deg: f32,
) -> f32 {
    let pitch_rad = pitch_deg.to_radians();
    let roll_rad = roll_deg.to_radians();
    let (ax, ay, az) = accel_g;

    // Pitch is gyro-X and roll is gyro-Y in this codebase. Yaw does not affect
    // the earth-up projection.
    let earth_vertical_specific_force_g = -roll_rad.sin() * ax
        + pitch_rad.sin() * roll_rad.cos() * ay
        + pitch_rad.cos() * roll_rad.cos() * az;
    (earth_vertical_specific_force_g - 1.0) * GRAVITY_MM_PER_SECOND_SQUARED
}

fn decay_for_sample_dt(sample_dt_seconds: f32, time_constant_seconds: f32) -> f32 {
    (-sample_dt_seconds / time_constant_seconds).exp()
}
