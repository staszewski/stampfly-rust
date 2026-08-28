use crate::altitude_control::{
    earth_vertical_acceleration_mm_per_second_squared, AltitudeHoldController, AltitudeHoldState,
};
use crate::attitude::AttitudeEstimator;
use crate::bmi270::ImuSample;
use crate::flow_control::{FlowState, FlowStateEstimator};
use crate::pmw3901::CachedFlowSample;
use crate::tof::CachedTofSample;

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

pub struct StateEstimator {
    attitude: AttitudeEstimator,
    altitude: AltitudeHoldController,
    flow: FlowStateEstimator,
    yaw_heading_deg: f32,
    latest_gyro_dps: (f32, f32, f32),
    latest_tof_range_mm: i16,
    latest_tof_status: u8,
    latest_tof_objects: u8,
    tof_sampled: bool,
    altitude_state: AltitudeHoldState,
    flow_state: FlowState,
    flow_gyro_delta_angle_deg: (f32, f32),
}

impl StateEstimator {
    pub fn new() -> Self {
        Self {
            attitude: AttitudeEstimator::new(),
            altitude: AltitudeHoldController::new(),
            flow: FlowStateEstimator::new(),
            yaw_heading_deg: 0.0,
            latest_gyro_dps: (0.0, 0.0, 0.0),
            latest_tof_range_mm: 0,
            latest_tof_status: 255,
            latest_tof_objects: 0,
            tof_sampled: false,
            altitude_state: AltitudeHoldState::default(),
            flow_state: FlowState::default(),
            flow_gyro_delta_angle_deg: (0.0, 0.0),
        }
    }

    pub fn update_imu(&mut self, imu: &ImuSample, dt_seconds: f32) {
        self.tof_sampled = false;
        self.latest_gyro_dps = imu.gyro_dps;
        self.flow_gyro_delta_angle_deg.0 += imu.gyro_dps.0 * dt_seconds;
        self.flow_gyro_delta_angle_deg.1 += imu.gyro_dps.1 * dt_seconds;
        self.attitude.update(
            imu.accel_g,
            imu.gyro_dps,
            self.flow.attitude_accel_correction_weight(),
            dt_seconds,
        );
        self.yaw_heading_deg = wrap_degrees(self.yaw_heading_deg + imu.gyro_dps.2 * dt_seconds);
        let vertical_acceleration_mm_per_second_squared =
            earth_vertical_acceleration_mm_per_second_squared(
                imu.accel_g,
                self.attitude.pitch_deg(),
                self.attitude.roll_deg(),
            );
        self.altitude_state = self
            .altitude
            .predict(vertical_acceleration_mm_per_second_squared, dt_seconds);
    }

    pub fn update_tof(&mut self, sample: CachedTofSample, sample_dt_seconds: f32) {
        self.tof_sampled = sample.sampled;
        if !sample.sampled {
            return;
        }

        self.latest_tof_range_mm = sample.range_mm;
        self.latest_tof_status = sample.status;
        self.latest_tof_objects = sample.objects;
        self.altitude_state = self.altitude.correct_tof(
            sample.range_mm,
            sample.status,
            sample.objects,
            sample_dt_seconds,
        );
    }

    pub fn update_altitude_control(
        &mut self,
        dt_seconds: f32,
        control_enabled: bool,
        integral_enabled: bool,
    ) {
        self.altitude_state =
            self.altitude
                .update_control(dt_seconds, control_enabled, integral_enabled);
    }

    pub fn update_flow(&mut self, sample: CachedFlowSample, sample_dt_seconds: f32) {
        if !sample.sampled {
            return;
        }

        let gyro_delta_angle_deg = self.flow_gyro_delta_angle_deg;
        self.flow_gyro_delta_angle_deg = (0.0, 0.0);
        self.flow_state = self.flow.update(
            sample.sample,
            self.attitude.pitch_deg(),
            self.attitude.roll_deg(),
            self.latest_gyro_dps,
            gyro_delta_angle_deg,
            self.yaw_heading_deg,
            sample_dt_seconds,
            self.latest_tof_range_mm,
            self.latest_tof_status,
            self.latest_tof_objects,
        );
    }

    pub fn reset_flow_gyro_integration(&mut self) {
        self.flow_gyro_delta_angle_deg = (0.0, 0.0);
    }

    pub fn estimate(&self) -> StateEstimate {
        StateEstimate {
            pitch_deg: self.attitude.pitch_deg(),
            roll_deg: self.attitude.roll_deg(),
            yaw_heading_deg: self.yaw_heading_deg,
            gyro_dps: self.latest_gyro_dps,
            altitude: self.altitude_state,
            flow: self.flow_state,
            attitude_accel_correction_weight: self.flow.attitude_accel_correction_weight(),
            tof_sampled: self.tof_sampled,
            tof_range_mm: self.latest_tof_range_mm,
            tof_status: self.latest_tof_status,
            tof_objects: self.latest_tof_objects,
        }
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
