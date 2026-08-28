use std::ffi::CString;

use esp_idf_hal::{
    delay::FreeRtos,
    sys::{
        esp_partition_erase_range, esp_partition_find_first, esp_partition_read, esp_partition_t,
        esp_partition_write, esp_rom_crc32_le,
    },
};

// The control loop remains 400 Hz. Trace at 100 Hz so the payload
// buffer can cover nine seconds without exhausting the ESP32-S3 heap.
const TRACE_CAPACITY: usize = 800;
const TRACE_SAMPLE_DIVISOR: u64 = 4;
const TRACE_DUMP_YIELD_EVERY_SAMPLES: usize = 4;
const MAGIC_BYTES: [u8; 4] = [0x54, 0x52, 0x43, 0x31];
const FORMAT_VERSION: u16 = 28;
const HEADER_SIZE: u16 = 24;
const HEADER_WITHOUT_MARKER_SIZE: usize = 20;
const COMPLETION_MARKER: u32 = 0xC0DE_CAFE;
const PAYLOAD_OFFSET: usize = HEADER_SIZE as usize;
pub const ENCODED_SAMPLE_SIZE: usize = 276;
const _: () = assert!(ENCODED_SAMPLE_SIZE == 276);
const TRACE_ERASE_BYTES: usize = 0x80000;
const _: () =
    assert!(HEADER_SIZE as usize + ENCODED_SAMPLE_SIZE * TRACE_CAPACITY <= TRACE_ERASE_BYTES);

#[derive(Copy, Clone, Default)]
pub struct ControlTraceSample {
    pub missed_deadline: bool,
    pub missed_deadline_count_since_last_trace: u8,
    pub tick: u64,
    pub control_dt_seconds: f32,
    pub maximum_control_dt_seconds_since_last_trace: f32,
    pub maximum_loop_execution_seconds_since_last_trace: f32,
    pub maximum_imu_read_seconds_since_last_trace: f32,
    pub maximum_tof_handling_seconds_since_last_trace: f32,
    pub maximum_flight_controller_update_seconds_since_last_trace: f32,
    pub maximum_motor_output_seconds_since_last_trace: f32,
    pub terminal_error_code: u8,
    pub battery_voltage_v: f32,
    pub gyro_x_dps: f32,
    pub gyro_y_dps: f32,
    pub gyro_z_dps: f32,
    pub accel_x_g: f32,
    pub accel_y_g: f32,
    pub accel_z_g: f32,
    pub pitch_deg: f32,
    pub roll_deg: f32,
    pub pitch_correction: f32,
    pub roll_correction: f32,
    pub yaw_correction: f32,
    pub pitch_rate_integral_correction: f32,
    pub roll_rate_integral_correction: f32,
    pub yaw_rate_integral_correction: f32,
    pub thrust_command: f32,
    pub mixer_collective_shift: f32,
    pub mixer_pitch_factor: f32,
    pub mixer_roll_factor: f32,
    pub motor_mix_front_left: f32,
    pub motor_mix_front_right: f32,
    pub motor_mix_rear_left: f32,
    pub motor_mix_rear_right: f32,
    pub motor_duty_front_left: f32,
    pub motor_duty_front_right: f32,
    pub motor_duty_rear_left: f32,
    pub motor_duty_rear_right: f32,
    pub pmw_motion: u8,
    pub pmw_dx_forward: i16,
    pub pmw_dy_left: i16,
    pub pmw_squal: u8,
    pub target_pitch_deg: f32,
    pub target_roll_deg: f32,
    pub horizontal_requested_target_pitch_deg: f32,
    pub horizontal_requested_target_roll_deg: f32,
    pub yaw_heading_deg: f32,
    pub attitude_accel_correction_weight: f32,
    pub flow_sample_accepted: bool,
    pub flow_control_active: bool,
    pub flow_reject_reason: u8,
    pub flow_world_forward_position_counts: f32,
    pub flow_world_left_position_counts: f32,
    pub flow_world_forward_velocity_counts_per_second: f32,
    pub flow_world_left_velocity_counts_per_second: f32,
    pub flow_world_forward_velocity_target_counts_per_second: f32,
    pub flow_world_left_velocity_target_counts_per_second: f32,
    pub flow_world_forward_velocity_innovation_counts_per_second: f32,
    pub flow_world_left_velocity_innovation_counts_per_second: f32,
    pub flow_body_forward_rotation_counts: f32,
    pub flow_body_left_rotation_counts: f32,
    pub flow_body_forward_translation_counts: f32,
    pub flow_body_left_translation_counts: f32,
    pub tof_sampled: bool,
    pub tof_range_mm: i16,
    pub tof_status: u8,
    pub tof_objects: u8,
    pub pitch_rate_target_dps: f32,
    pub roll_rate_target_dps: f32,
    pub pitch_rate_error_dps: f32,
    pub roll_rate_error_dps: f32,
    pub altitude_control_height_mm: f32,
    pub altitude_vertical_velocity_mm_per_second: f32,
    pub altitude_vertical_velocity_target_mm_per_second: f32,
    pub altitude_tof_innovation_rejected: bool,
    pub altitude_hover_thrust_offset: f32,
    pub altitude_desired_thrust_offset: f32,
    pub altitude_applied_thrust_offset: f32,
    pub altitude_active: bool,
    pub cutoff_reason: u8,
}

pub struct ControlTrace {
    samples: Box<[[u8; ENCODED_SAMPLE_SIZE]]>,
    len: usize,
    frozen: bool,
    missed_deadline_count_since_last_trace: u8,
    maximum_control_dt_seconds_since_last_trace: f32,
    maximum_loop_execution_seconds_since_last_trace: f32,
    maximum_imu_read_seconds_since_last_trace: f32,
    maximum_tof_handling_seconds_since_last_trace: f32,
    maximum_flight_controller_update_seconds_since_last_trace: f32,
    maximum_motor_output_seconds_since_last_trace: f32,
}

pub struct ControlTraceHeader {
    pub magic: [u8; 4],
    pub format_version: u16,
    pub header_bytes: u16,
    pub sample_count: u32,
    pub payload_bytes: u32,
    pub payload_crc32: u32,
    pub completion_marker: u32,
}

impl ControlTrace {
    pub fn codec_self_check() -> bool {
        let sample = ControlTraceSample {
            missed_deadline: true,
            missed_deadline_count_since_last_trace: 3,
            tick: 0x0102_0304_0506_0708,
            control_dt_seconds: 0.0025,
            maximum_control_dt_seconds_since_last_trace: 0.0035,
            maximum_loop_execution_seconds_since_last_trace: 0.0015,
            maximum_imu_read_seconds_since_last_trace: 0.0005,
            maximum_tof_handling_seconds_since_last_trace: 0.0006,
            maximum_flight_controller_update_seconds_since_last_trace: 0.0007,
            maximum_motor_output_seconds_since_last_trace: 0.0008,
            terminal_error_code: 80,
            battery_voltage_v: 1.25,
            gyro_x_dps: 2.25,
            gyro_y_dps: 3.25,
            gyro_z_dps: 4.25,
            accel_x_g: 5.25,
            accel_y_g: 6.25,
            accel_z_g: 7.25,
            pitch_deg: 8.25,
            roll_deg: 9.25,
            pitch_correction: 10.25,
            roll_correction: 11.25,
            yaw_correction: 12.25,
            pitch_rate_integral_correction: 12.50,
            roll_rate_integral_correction: 12.75,
            yaw_rate_integral_correction: 13.0,
            thrust_command: 13.25,
            mixer_collective_shift: -1.25,
            mixer_pitch_factor: 0.25,
            mixer_roll_factor: 0.375,
            motor_mix_front_left: 13.50,
            motor_mix_front_right: 13.75,
            motor_mix_rear_left: 14.0,
            motor_mix_rear_right: 14.125,
            motor_duty_front_left: 14.25,
            motor_duty_front_right: 15.25,
            motor_duty_rear_left: 16.25,
            motor_duty_rear_right: 17.25,
            target_pitch_deg: 18.25,
            target_roll_deg: 19.25,
            horizontal_requested_target_pitch_deg: -2.25,
            horizontal_requested_target_roll_deg: 3.25,
            yaw_heading_deg: 20.25,
            attitude_accel_correction_weight: 0.75,
            flow_control_active: true,
            tof_sampled: true,
            tof_range_mm: -1234,
            tof_status: 21,
            tof_objects: 22,
            pitch_rate_target_dps: 23.25,
            roll_rate_target_dps: 24.25,
            pitch_rate_error_dps: 25.25,
            roll_rate_error_dps: 26.25,
            altitude_control_height_mm: 27.25,
            altitude_vertical_velocity_mm_per_second: 27.50,
            altitude_vertical_velocity_target_mm_per_second: 27.75,
            altitude_tof_innovation_rejected: true,
            altitude_hover_thrust_offset: 28.25,
            altitude_desired_thrust_offset: 28.50,
            altitude_applied_thrust_offset: 28.75,
            altitude_active: true,
            cutoff_reason: 6,
            ..Default::default()
        };
        let encoded = Self::encode_sample(&sample);
        let Some(decoded) = Self::decode_sample(&encoded) else {
            return false;
        };
        Self::encode_sample(&decoded) == encoded
    }

    pub fn new() -> Self {
        Self {
            samples: vec![[0u8; ENCODED_SAMPLE_SIZE]; TRACE_CAPACITY].into_boxed_slice(),
            len: 0,
            frozen: false,
            missed_deadline_count_since_last_trace: 0,
            maximum_control_dt_seconds_since_last_trace: 0.0,
            maximum_loop_execution_seconds_since_last_trace: 0.0,
            maximum_imu_read_seconds_since_last_trace: 0.0,
            maximum_tof_handling_seconds_since_last_trace: 0.0,
            maximum_flight_controller_update_seconds_since_last_trace: 0.0,
            maximum_motor_output_seconds_since_last_trace: 0.0,
        }
    }

    pub fn record(&mut self, mut sample: ControlTraceSample, loop_execution_seconds: f32) {
        self.missed_deadline_count_since_last_trace = self
            .missed_deadline_count_since_last_trace
            .saturating_add(u8::from(sample.missed_deadline));
        self.maximum_control_dt_seconds_since_last_trace = self
            .maximum_control_dt_seconds_since_last_trace
            .max(sample.control_dt_seconds);
        self.maximum_loop_execution_seconds_since_last_trace = self
            .maximum_loop_execution_seconds_since_last_trace
            .max(loop_execution_seconds);
        self.maximum_imu_read_seconds_since_last_trace = self
            .maximum_imu_read_seconds_since_last_trace
            .max(sample.maximum_imu_read_seconds_since_last_trace);
        self.maximum_tof_handling_seconds_since_last_trace = self
            .maximum_tof_handling_seconds_since_last_trace
            .max(sample.maximum_tof_handling_seconds_since_last_trace);
        self.maximum_flight_controller_update_seconds_since_last_trace = self
            .maximum_flight_controller_update_seconds_since_last_trace
            .max(sample.maximum_flight_controller_update_seconds_since_last_trace);
        self.maximum_motor_output_seconds_since_last_trace = self
            .maximum_motor_output_seconds_since_last_trace
            .max(sample.maximum_motor_output_seconds_since_last_trace);

        // Always retain the cutoff sample, even when it falls between trace
        // frames, so the saved partial trace contains the triggering reason.
        if sample.tick % TRACE_SAMPLE_DIVISOR != 0
            && sample.cutoff_reason == 0
            && sample.terminal_error_code == 0
        {
            return;
        }
        if !self.frozen && self.len < TRACE_CAPACITY {
            sample.missed_deadline_count_since_last_trace =
                self.missed_deadline_count_since_last_trace;
            sample.maximum_control_dt_seconds_since_last_trace =
                self.maximum_control_dt_seconds_since_last_trace;
            sample.maximum_loop_execution_seconds_since_last_trace =
                self.maximum_loop_execution_seconds_since_last_trace;
            sample.maximum_imu_read_seconds_since_last_trace =
                self.maximum_imu_read_seconds_since_last_trace;
            sample.maximum_tof_handling_seconds_since_last_trace =
                self.maximum_tof_handling_seconds_since_last_trace;
            sample.maximum_flight_controller_update_seconds_since_last_trace =
                self.maximum_flight_controller_update_seconds_since_last_trace;
            sample.maximum_motor_output_seconds_since_last_trace =
                self.maximum_motor_output_seconds_since_last_trace;
            self.samples[self.len] = Self::encode_sample(&sample);
            self.len += 1;
            self.missed_deadline_count_since_last_trace = 0;
            self.maximum_control_dt_seconds_since_last_trace = 0.0;
            self.maximum_loop_execution_seconds_since_last_trace = 0.0;
            self.maximum_imu_read_seconds_since_last_trace = 0.0;
            self.maximum_tof_handling_seconds_since_last_trace = 0.0;
            self.maximum_flight_controller_update_seconds_since_last_trace = 0.0;
            self.maximum_motor_output_seconds_since_last_trace = 0.0;
            if self.len == TRACE_CAPACITY {
                self.freeze();
            }
        }
    }

    pub fn freeze(&mut self) {
        self.frozen = true;
    }

    pub fn is_frozen(&self) -> bool {
        self.frozen
    }

    fn samples(&self) -> &[[u8; ENCODED_SAMPLE_SIZE]] {
        &self.samples[..self.len]
    }

    fn dump_header() {
        log::info!(
            "missed_deadline;missed_deadline_count_since_last_trace;tick;control_dt_seconds;maximum_control_dt_seconds_since_last_trace;maximum_loop_execution_seconds_since_last_trace;terminal_error_code;battery_voltage_v;gyro_x_dps;gyro_y_dps;gyro_z_dps;accel_x_g;accel_y_g;accel_z_g;pitch_deg;roll_deg;pitch_correction;roll_correction;yaw_correction;pitch_rate_integral_correction;roll_rate_integral_correction;yaw_rate_integral_correction;thrust_command;mixer_collective_shift;mixer_pitch_factor;mixer_roll_factor;motor_mix_front_left;motor_mix_front_right;motor_mix_rear_left;motor_mix_rear_right;motor_duty_front_left;motor_duty_front_right;motor_duty_rear_left;motor_duty_rear_right;pmw_motion;pmw_delta_x_counts;pmw_delta_y_counts;pmw_squal;effective_target_pitch_deg;effective_target_roll_deg;horizontal_requested_target_pitch_deg;horizontal_requested_target_roll_deg;yaw_heading_deg;attitude_accel_correction_weight;flow_sample_accepted;flow_control_active;flow_reject_reason;flow_world_forward_position_counts;flow_world_left_position_counts;flow_world_forward_velocity_counts_per_second;flow_world_left_velocity_counts_per_second;flow_world_forward_velocity_target_counts_per_second;flow_world_left_velocity_target_counts_per_second;flow_world_forward_velocity_innovation_counts_per_second;flow_world_left_velocity_innovation_counts_per_second;flow_body_forward_rotation_counts;flow_body_left_rotation_counts;flow_body_forward_translation_counts;flow_body_left_translation_counts;tof_sampled;tof_range_mm;tof_status;tof_objects;pitch_rate_target_dps;roll_rate_target_dps;pitch_rate_error_dps;roll_rate_error_dps;altitude_control_height_mm;altitude_vertical_velocity_mm_per_second;altitude_vertical_velocity_target_mm_per_second;altitude_tof_innovation_rejected;altitude_hover_thrust_offset;altitude_desired_thrust_offset;altitude_applied_thrust_offset;altitude_active;cutoff_reason;maximum_imu_read_seconds_since_last_trace;maximum_tof_handling_seconds_since_last_trace;maximum_flight_controller_update_seconds_since_last_trace;maximum_motor_output_seconds_since_last_trace"
        );
    }

    fn dump_sample(bytes: &[u8]) {
        if let Some(s) = Self::decode_sample(bytes) {
            log::info!(
                concat!(
                    "{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{}",
                    ";{};{};{};{};{};{}"
                ),
                s.missed_deadline,
                s.missed_deadline_count_since_last_trace,
                s.tick,
                s.control_dt_seconds,
                s.maximum_control_dt_seconds_since_last_trace,
                s.maximum_loop_execution_seconds_since_last_trace,
                s.terminal_error_code,
                s.battery_voltage_v,
                s.gyro_x_dps,
                s.gyro_y_dps,
                s.gyro_z_dps,
                s.accel_x_g,
                s.accel_y_g,
                s.accel_z_g,
                s.pitch_deg,
                s.roll_deg,
                s.pitch_correction,
                s.roll_correction,
                s.yaw_correction,
                s.pitch_rate_integral_correction,
                s.roll_rate_integral_correction,
                s.yaw_rate_integral_correction,
                s.thrust_command,
                s.mixer_collective_shift,
                s.mixer_pitch_factor,
                s.mixer_roll_factor,
                s.motor_mix_front_left,
                s.motor_mix_front_right,
                s.motor_mix_rear_left,
                s.motor_mix_rear_right,
                s.motor_duty_front_left,
                s.motor_duty_front_right,
                s.motor_duty_rear_left,
                s.motor_duty_rear_right,
                s.pmw_motion,
                s.pmw_dx_forward,
                s.pmw_dy_left,
                s.pmw_squal,
                s.target_pitch_deg,
                s.target_roll_deg,
                s.horizontal_requested_target_pitch_deg,
                s.horizontal_requested_target_roll_deg,
                s.yaw_heading_deg,
                s.attitude_accel_correction_weight,
                s.flow_sample_accepted,
                s.flow_control_active,
                s.flow_reject_reason,
                s.flow_world_forward_position_counts,
                s.flow_world_left_position_counts,
                s.flow_world_forward_velocity_counts_per_second,
                s.flow_world_left_velocity_counts_per_second,
                s.flow_world_forward_velocity_target_counts_per_second,
                s.flow_world_left_velocity_target_counts_per_second,
                s.flow_world_forward_velocity_innovation_counts_per_second,
                s.flow_world_left_velocity_innovation_counts_per_second,
                s.flow_body_forward_rotation_counts,
                s.flow_body_left_rotation_counts,
                s.flow_body_forward_translation_counts,
                s.flow_body_left_translation_counts,
                s.tof_sampled,
                s.tof_range_mm,
                s.tof_status,
                s.tof_objects,
                s.pitch_rate_target_dps,
                s.roll_rate_target_dps,
                s.pitch_rate_error_dps,
                s.roll_rate_error_dps,
                s.altitude_control_height_mm,
                s.altitude_vertical_velocity_mm_per_second,
                s.altitude_vertical_velocity_target_mm_per_second,
                s.altitude_tof_innovation_rejected,
                s.altitude_hover_thrust_offset,
                s.altitude_desired_thrust_offset,
                s.altitude_applied_thrust_offset,
                s.altitude_active,
                s.cutoff_reason,
                s.maximum_imu_read_seconds_since_last_trace,
                s.maximum_tof_handling_seconds_since_last_trace,
                s.maximum_flight_controller_update_seconds_since_last_trace,
                s.maximum_motor_output_seconds_since_last_trace
            );
        }
    }

    pub fn clear_saved_trace() -> bool {
        let partition = Self::find_partition();
        if partition.is_null() {
            return false;
        }
        let marker = 0u32.to_le_bytes();
        unsafe {
            esp_partition_write(
                partition,
                HEADER_WITHOUT_MARKER_SIZE,
                marker.as_ptr().cast(),
                marker.len(),
            ) == 0
        }
    }

    pub fn save_trace_to_partition(&self) -> bool {
        let partition = Self::find_partition();
        if !self.frozen || partition.is_null() {
            return false;
        }
        let payload_bytes = self.len * ENCODED_SAMPLE_SIZE;
        if PAYLOAD_OFFSET + payload_bytes > TRACE_ERASE_BYTES {
            return false;
        }
        let mut crc = 0;
        for sample in self.samples() {
            crc = unsafe { esp_rom_crc32_le(crc, sample.as_ptr(), sample.len() as u32) };
        }
        let header = ControlTraceHeader {
            magic: MAGIC_BYTES,
            format_version: FORMAT_VERSION,
            header_bytes: HEADER_SIZE,
            sample_count: self.len as u32,
            payload_bytes: payload_bytes as u32,
            payload_crc32: crc,
            completion_marker: COMPLETION_MARKER,
        };
        let encoded_header = Self::encode_header(&header);
        unsafe {
            if esp_partition_erase_range(partition, 0, TRACE_ERASE_BYTES) != 0 {
                return false;
            }
            for (index, sample) in self.samples().iter().enumerate() {
                if esp_partition_write(
                    partition,
                    PAYLOAD_OFFSET + index * ENCODED_SAMPLE_SIZE,
                    sample.as_ptr().cast(),
                    sample.len(),
                ) != 0
                {
                    return false;
                }
            }
            if esp_partition_write(
                partition,
                0,
                encoded_header.as_ptr().cast(),
                encoded_header.len(),
            ) != 0
            {
                return false;
            }
            let marker = header.completion_marker.to_le_bytes();
            esp_partition_write(
                partition,
                HEADER_WITHOUT_MARKER_SIZE,
                marker.as_ptr().cast(),
                marker.len(),
            ) == 0
        }
    }

    pub fn read_header() -> Option<ControlTraceHeader> {
        let partition = Self::find_partition();
        if partition.is_null() {
            return None;
        }
        let mut bytes = [0u8; HEADER_SIZE as usize];
        if unsafe { esp_partition_read(partition, 0, bytes.as_mut_ptr().cast(), bytes.len()) } != 0
        {
            return None;
        }
        let header = ControlTraceHeader {
            magic: bytes[0..4].try_into().ok()?,
            format_version: u16::from_le_bytes(bytes[4..6].try_into().ok()?),
            header_bytes: u16::from_le_bytes(bytes[6..8].try_into().ok()?),
            sample_count: u32::from_le_bytes(bytes[8..12].try_into().ok()?),
            payload_bytes: u32::from_le_bytes(bytes[12..16].try_into().ok()?),
            payload_crc32: u32::from_le_bytes(bytes[16..20].try_into().ok()?),
            completion_marker: u32::from_le_bytes(bytes[20..24].try_into().ok()?),
        };
        if header.magic != MAGIC_BYTES
            || header.format_version != FORMAT_VERSION
            || header.header_bytes != HEADER_SIZE
            || header.sample_count > TRACE_CAPACITY as u32
            || header.payload_bytes != header.sample_count * ENCODED_SAMPLE_SIZE as u32
            || header.completion_marker != COMPLETION_MARKER
        {
            None
        } else {
            Some(header)
        }
    }

    pub fn validate_saved_payload(header: &ControlTraceHeader) -> bool {
        let partition = Self::find_partition();
        if partition.is_null() {
            return false;
        }
        let mut sample = [0u8; ENCODED_SAMPLE_SIZE];
        let mut crc = 0;
        for index in 0..header.sample_count as usize {
            if unsafe {
                esp_partition_read(
                    partition,
                    PAYLOAD_OFFSET + index * ENCODED_SAMPLE_SIZE,
                    sample.as_mut_ptr().cast(),
                    sample.len(),
                )
            } != 0
            {
                return false;
            }
            crc = unsafe { esp_rom_crc32_le(crc, sample.as_ptr(), sample.len() as u32) };
        }
        crc == header.payload_crc32
    }

    pub fn dump_saved(header: &ControlTraceHeader) {
        let partition = Self::find_partition();
        if partition.is_null() || header.format_version != FORMAT_VERSION {
            return;
        }
        Self::dump_header();
        let mut bytes = [0u8; ENCODED_SAMPLE_SIZE];
        for index in 0..header.sample_count as usize {
            if unsafe {
                esp_partition_read(
                    partition,
                    PAYLOAD_OFFSET + index * ENCODED_SAMPLE_SIZE,
                    bytes.as_mut_ptr().cast(),
                    bytes.len(),
                )
            } != 0
            {
                log::info!("Trace dump stopped at sample {index}: partition read failed");
                return;
            }
            Self::dump_sample(&bytes);
            if (index + 1) % TRACE_DUMP_YIELD_EVERY_SAMPLES == 0 {
                // Formatting a v25 row performs many floating-point-to-text
                // conversions. Briefly block the high-priority main task so
                // CPU0's idle task can run and service its watchdog while a
                // saved trace is printed over the serial logger.
                FreeRtos::delay_ms(1);
            }
        }
    }

    pub fn decode_sample(bytes: &[u8]) -> Option<ControlTraceSample> {
        if bytes.len() != ENCODED_SAMPLE_SIZE {
            return None;
        }
        let mut i = 0usize;
        macro_rules! u8v {
            () => {{
                let v = bytes[i];
                i += 1;
                v
            }};
        }
        macro_rules! i16v {
            () => {{
                let v = i16::from_le_bytes(bytes[i..i + 2].try_into().ok()?);
                i += 2;
                v
            }};
        }
        macro_rules! u64v {
            () => {{
                let v = u64::from_le_bytes(bytes[i..i + 8].try_into().ok()?);
                i += 8;
                v
            }};
        }
        macro_rules! f32v {
            () => {{
                let v = f32::from_le_bytes(bytes[i..i + 4].try_into().ok()?);
                i += 4;
                v
            }};
        }
        let boolv = |v| match v {
            0 => Some(false),
            1 => Some(true),
            _ => None,
        };
        let sample = ControlTraceSample {
            missed_deadline: boolv(u8v!())?,
            missed_deadline_count_since_last_trace: u8v!(),
            tick: u64v!(),
            control_dt_seconds: f32v!(),
            maximum_control_dt_seconds_since_last_trace: f32v!(),
            maximum_loop_execution_seconds_since_last_trace: f32v!(),
            terminal_error_code: u8v!(),
            battery_voltage_v: f32v!(),
            gyro_x_dps: f32v!(),
            gyro_y_dps: f32v!(),
            gyro_z_dps: f32v!(),
            accel_x_g: f32v!(),
            accel_y_g: f32v!(),
            accel_z_g: f32v!(),
            pitch_deg: f32v!(),
            roll_deg: f32v!(),
            pitch_correction: f32v!(),
            roll_correction: f32v!(),
            yaw_correction: f32v!(),
            pitch_rate_integral_correction: f32v!(),
            roll_rate_integral_correction: f32v!(),
            yaw_rate_integral_correction: f32v!(),
            thrust_command: f32v!(),
            mixer_collective_shift: f32v!(),
            mixer_pitch_factor: f32v!(),
            mixer_roll_factor: f32v!(),
            motor_mix_front_left: f32v!(),
            motor_mix_front_right: f32v!(),
            motor_mix_rear_left: f32v!(),
            motor_mix_rear_right: f32v!(),
            motor_duty_front_left: f32v!(),
            motor_duty_front_right: f32v!(),
            motor_duty_rear_left: f32v!(),
            motor_duty_rear_right: f32v!(),
            pmw_motion: u8v!(),
            pmw_dx_forward: i16v!(),
            pmw_dy_left: i16v!(),
            pmw_squal: u8v!(),
            target_pitch_deg: f32v!(),
            target_roll_deg: f32v!(),
            horizontal_requested_target_pitch_deg: f32v!(),
            horizontal_requested_target_roll_deg: f32v!(),
            yaw_heading_deg: f32v!(),
            attitude_accel_correction_weight: f32v!(),
            flow_sample_accepted: boolv(u8v!())?,
            flow_control_active: boolv(u8v!())?,
            flow_reject_reason: u8v!(),
            flow_world_forward_position_counts: f32v!(),
            flow_world_left_position_counts: f32v!(),
            flow_world_forward_velocity_counts_per_second: f32v!(),
            flow_world_left_velocity_counts_per_second: f32v!(),
            flow_world_forward_velocity_target_counts_per_second: f32v!(),
            flow_world_left_velocity_target_counts_per_second: f32v!(),
            flow_world_forward_velocity_innovation_counts_per_second: f32v!(),
            flow_world_left_velocity_innovation_counts_per_second: f32v!(),
            flow_body_forward_rotation_counts: f32v!(),
            flow_body_left_rotation_counts: f32v!(),
            flow_body_forward_translation_counts: f32v!(),
            flow_body_left_translation_counts: f32v!(),
            tof_sampled: boolv(u8v!())?,
            tof_range_mm: i16v!(),
            tof_status: u8v!(),
            tof_objects: u8v!(),
            pitch_rate_target_dps: f32v!(),
            roll_rate_target_dps: f32v!(),
            pitch_rate_error_dps: f32v!(),
            roll_rate_error_dps: f32v!(),
            altitude_control_height_mm: f32v!(),
            altitude_vertical_velocity_mm_per_second: f32v!(),
            altitude_vertical_velocity_target_mm_per_second: f32v!(),
            altitude_tof_innovation_rejected: boolv(u8v!())?,
            altitude_hover_thrust_offset: f32v!(),
            altitude_desired_thrust_offset: f32v!(),
            altitude_applied_thrust_offset: f32v!(),
            altitude_active: boolv(u8v!())?,
            cutoff_reason: u8v!(),
            maximum_imu_read_seconds_since_last_trace: f32v!(),
            maximum_tof_handling_seconds_since_last_trace: f32v!(),
            maximum_flight_controller_update_seconds_since_last_trace: f32v!(),
            maximum_motor_output_seconds_since_last_trace: f32v!(),
            ..Default::default()
        };
        (i == ENCODED_SAMPLE_SIZE).then_some(sample)
    }

    fn encode_sample(sample: &ControlTraceSample) -> [u8; ENCODED_SAMPLE_SIZE] {
        let mut bytes = [0u8; ENCODED_SAMPLE_SIZE];
        let mut i = 0usize;
        macro_rules! put {
            ($v:expr) => {{
                let b = $v.to_le_bytes();
                bytes[i..i + b.len()].copy_from_slice(&b);
                i += b.len();
            }};
        }
        macro_rules! byte {
            ($v:expr) => {{
                bytes[i] = $v;
                i += 1;
            }};
        }
        byte!(sample.missed_deadline as u8);
        byte!(sample.missed_deadline_count_since_last_trace);
        put!(sample.tick);
        put!(sample.control_dt_seconds);
        put!(sample.maximum_control_dt_seconds_since_last_trace);
        put!(sample.maximum_loop_execution_seconds_since_last_trace);
        byte!(sample.terminal_error_code);
        put!(sample.battery_voltage_v);
        put!(sample.gyro_x_dps);
        put!(sample.gyro_y_dps);
        put!(sample.gyro_z_dps);
        put!(sample.accel_x_g);
        put!(sample.accel_y_g);
        put!(sample.accel_z_g);
        put!(sample.pitch_deg);
        put!(sample.roll_deg);
        put!(sample.pitch_correction);
        put!(sample.roll_correction);
        put!(sample.yaw_correction);
        put!(sample.pitch_rate_integral_correction);
        put!(sample.roll_rate_integral_correction);
        put!(sample.yaw_rate_integral_correction);
        put!(sample.thrust_command);
        put!(sample.mixer_collective_shift);
        put!(sample.mixer_pitch_factor);
        put!(sample.mixer_roll_factor);
        put!(sample.motor_mix_front_left);
        put!(sample.motor_mix_front_right);
        put!(sample.motor_mix_rear_left);
        put!(sample.motor_mix_rear_right);
        put!(sample.motor_duty_front_left);
        put!(sample.motor_duty_front_right);
        put!(sample.motor_duty_rear_left);
        put!(sample.motor_duty_rear_right);
        byte!(sample.pmw_motion);
        put!(sample.pmw_dx_forward);
        put!(sample.pmw_dy_left);
        byte!(sample.pmw_squal);
        put!(sample.target_pitch_deg);
        put!(sample.target_roll_deg);
        put!(sample.horizontal_requested_target_pitch_deg);
        put!(sample.horizontal_requested_target_roll_deg);
        put!(sample.yaw_heading_deg);
        put!(sample.attitude_accel_correction_weight);
        byte!(sample.flow_sample_accepted as u8);
        byte!(sample.flow_control_active as u8);
        byte!(sample.flow_reject_reason);
        put!(sample.flow_world_forward_position_counts);
        put!(sample.flow_world_left_position_counts);
        put!(sample.flow_world_forward_velocity_counts_per_second);
        put!(sample.flow_world_left_velocity_counts_per_second);
        put!(sample.flow_world_forward_velocity_target_counts_per_second);
        put!(sample.flow_world_left_velocity_target_counts_per_second);
        put!(sample.flow_world_forward_velocity_innovation_counts_per_second);
        put!(sample.flow_world_left_velocity_innovation_counts_per_second);
        put!(sample.flow_body_forward_rotation_counts);
        put!(sample.flow_body_left_rotation_counts);
        put!(sample.flow_body_forward_translation_counts);
        put!(sample.flow_body_left_translation_counts);
        byte!(sample.tof_sampled as u8);
        put!(sample.tof_range_mm);
        byte!(sample.tof_status);
        byte!(sample.tof_objects);
        put!(sample.pitch_rate_target_dps);
        put!(sample.roll_rate_target_dps);
        put!(sample.pitch_rate_error_dps);
        put!(sample.roll_rate_error_dps);
        put!(sample.altitude_control_height_mm);
        put!(sample.altitude_vertical_velocity_mm_per_second);
        put!(sample.altitude_vertical_velocity_target_mm_per_second);
        byte!(sample.altitude_tof_innovation_rejected as u8);
        put!(sample.altitude_hover_thrust_offset);
        put!(sample.altitude_desired_thrust_offset);
        put!(sample.altitude_applied_thrust_offset);
        byte!(sample.altitude_active as u8);
        byte!(sample.cutoff_reason);
        put!(sample.maximum_imu_read_seconds_since_last_trace);
        put!(sample.maximum_tof_handling_seconds_since_last_trace);
        put!(sample.maximum_flight_controller_update_seconds_since_last_trace);
        put!(sample.maximum_motor_output_seconds_since_last_trace);
        debug_assert_eq!(i, ENCODED_SAMPLE_SIZE);
        bytes
    }

    fn encode_header(header: &ControlTraceHeader) -> [u8; HEADER_WITHOUT_MARKER_SIZE] {
        let mut bytes = [0u8; HEADER_WITHOUT_MARKER_SIZE];
        bytes[0..4].copy_from_slice(&header.magic);
        bytes[4..6].copy_from_slice(&header.format_version.to_le_bytes());
        bytes[6..8].copy_from_slice(&header.header_bytes.to_le_bytes());
        bytes[8..12].copy_from_slice(&header.sample_count.to_le_bytes());
        bytes[12..16].copy_from_slice(&header.payload_bytes.to_le_bytes());
        bytes[16..20].copy_from_slice(&header.payload_crc32.to_le_bytes());
        bytes
    }

    fn find_partition() -> *const esp_partition_t {
        let label = CString::new("trace").expect("trace partition label");
        unsafe { esp_partition_find_first(0x40, 0x00, label.as_ptr()) }
    }
}
