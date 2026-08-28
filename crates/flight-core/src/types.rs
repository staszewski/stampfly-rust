//! Shared control-domain types and unit conventions.

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

#[derive(Debug, PartialEq)]
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

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct BodyAccelerationG {
    pub x_g: f32,
    pub y_g: f32,
    pub z_g: f32,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct BodyAngularRateDps {
    pub pitch_dps: f32,
    pub roll_dps: f32,
    pub yaw_dps: f32,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AttitudeEstimate {
    pub pitch_deg: f32,
    pub roll_deg: f32,
    pub yaw_heading_deg: f32,
}

#[derive(Copy, Clone, Default)]
pub struct StateEstimate {
    pub pitch_deg: f32,
    pub roll_deg: f32,
    pub yaw_heading_deg: f32,
    pub gyro_dps: (f32, f32, f32),
    pub altitude: AltitudeHoldState,
    pub flow: FlowState,
    pub attitude_accel_correction_weight: f32,
    pub tof_sampled: bool,
    pub tof_range_mm: i16,
    pub tof_status: u8,
    pub tof_objects: u8,
}

#[derive(Debug)]
pub struct MotorInput {
    pub thrust: f32,
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
}

#[derive(Debug, PartialEq)]
pub struct MotorMix {
    pub front_left: f32,
    pub front_right: f32,
    pub rear_left: f32,
    pub rear_right: f32,
    pub collective_shift: f32,
    pub pitch_mix_factor: f32,
    pub roll_mix_factor: f32,
}

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

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CollectiveMode {
    Disabled,
    BaseThrust,
    AltitudeHold,
}
