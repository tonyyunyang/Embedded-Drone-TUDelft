use core::time::Duration;

use crate::control::pid_controller::{map_p_to_fixed, PIDController};
use crate::control::state_machine::{execute_state_function, JoystickControl, StateMachine};
use crate::yaw_pitch_roll::YawPitchRoll;
use alloc::vec::Vec;
// use heapless::Vec as HVec;
use protocol::format::{DeviceProtocol, HostProtocol};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::block;
use tudelft_quadrupel::fixed::types::I16F16;
use tudelft_quadrupel::fixed::{types, FixedI32};
use tudelft_quadrupel::led::Led::Blue;
use tudelft_quadrupel::led::Yellow;
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::structs::{Accel, Gyro, Quaternion};
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{receive_bytes, send_bytes};

use self::pid_controller::{map_p1_to_fixed, map_p2_to_fixed, GeneralController};
use self::state_machine::State;
mod motor_control;
mod pid_controller;
mod state_machine;

#[allow(unused_assignments)]
pub fn control_loop() -> ! {
    // Initialize the variables for the control loop
    set_tick_frequency(100);
    let mut safety_counter = SafetyCounter::new();
    let mut sensor_data = SensorData::new();
    let mut sensor_data_calibration_offset = SensorOffset::new();
    let mut state_machine = StateMachine::new();
    let mut joystick_control = JoystickControl::new();
    let mut nice_received_message = HostProtocol::new(0, 0, 0, 0, 0, 0, 0, 0);
    let mut ack = 0b0000_0000;
    let mut buf = [0u8; 257];
    // let mut command_buf: HVec<u8, 1024> = HVec::new();
    let mut command_buf: Vec<u8> = Vec::new();
    let mut mode = 0b0000_0000;

    // initialize the struct for stable controls
    let yaw_pid = PIDController::new(
        I16F16::from_num(1),
        I16F16::from_num(0),
        I16F16::from_num(0),
        I16F16::from_num(0),
        I16F16::from_num(0),
    );
    let pitch_pid = PIDController::new(
        I16F16::from_num(1),
        I16F16::from_num(0),
        I16F16::from_num(0),
        I16F16::from_num(0),
        I16F16::from_num(0),
    );
    let roll_pid = PIDController::new(
        I16F16::from_num(1),
        I16F16::from_num(0),
        I16F16::from_num(0),
        I16F16::from_num(0),
        I16F16::from_num(1),
    );
    let height_pid = PIDController::new(
        I16F16::from_num(1.5),
        I16F16::from_num(0),
        I16F16::from_num(0),
        I16F16::from_num(0),
        I16F16::from_num(5),
    );
    let yaw_control = pid_controller::YawController::new(yaw_pid);
    let pitch_control = pid_controller::PitchController::new(pitch_pid);
    let roll_control = pid_controller::RollController::new(roll_pid);
    let height_control = pid_controller::HeightController::new(height_pid);
    let mut general_controllers = pid_controller::GeneralController::new(
        yaw_control,
        pitch_control,
        roll_control,
        height_control,
    );

    for i in 0.. {
        // update the sensor data
        sensor_data.update_all(&sensor_data_calibration_offset);

        // the code below is an algorithm for receiving the message from the host
        // first read 'num' bytes from the uart
        let num = receive_bytes(&mut buf);
        // command_buf.extend_from_slice(&buf[0..num]).unwrap();
        // push what we read into the command buffer
        command_buf.extend_from_slice(&buf[0..num]);
        // if the command buffer is long enough, then we check if the message is valid
        if command_buf.len() >= 12 {
            let temp = command_buf.clone();
            // due to the fact that the length of the message is already long enough, we can check if the first and last byte are correct
            if temp[0] != 0x7b || temp[11] != 0x7d {
                // if the first or last byte is not correct, we directly flush out everything in the command buffer
                command_buf.clear();
            } else {
                // if the first and last byte are correct, we split the command buffer into two parts, the first part is the message, the second part is the rest
                let (message, rest) = temp.split_at(12);
                // then we clear the command buffer
                command_buf.clear();
                // and push the rest into the command buffer
                // command_buf.extend_from_slice(rest).unwrap();
                command_buf.extend_from_slice(rest);
                // then we form the message and check if it is valid
                nice_received_message = HostProtocol::format_message_not_mut(message);
                ack = verify_message(&nice_received_message);
            }
        }

        // if the code received by the drone is acknowledged, then we transition to the next state, and execute corresponding function
        if ack == 0b1111_1111 {
            Yellow.on();
            // Update global struct.
            mode = nice_received_message.get_mode();
            let next_state = map_to_state(mode);
            update_joystick_control_and_controller(
                &mut joystick_control,
                &mut general_controllers,
                &nice_received_message,
            );
            // Assume that transition is false before transition, will become true if transition is successful.
            let mut transition_result = false;
            // After updating, check if the stick is in a neutral state before transition.
            // The OR statement is added for panic state, since drone should always be able to panic.
            (transition_result, ack) = state_machine.transition(
                next_state,
                &mut joystick_control,
                &mut general_controllers,
                &mut sensor_data_calibration_offset,
                &mut sensor_data,
            );

            let current_state = state_machine.state();
            mode = map_to_mode(&current_state);
            // Reset time out counter, since message was received successfully.
            safety_counter.reset_command_timeout();
            if transition_result && ack != 0b0000_1111 {
                execute_state_function(
                    &current_state,
                    &joystick_control,
                    &mut general_controllers,
                    &sensor_data,
                );
            }
        }

        if i % 100 == 0 {
            // Create an instance of the Drone Protocol struct
            let message_to_host = DeviceProtocol::new(
                mode,
                sensor_data.get_dt().as_millis() as u16,
                sensor_data.get_motors(),
                sensor_data.get_ypr_data(),
                sensor_data.get_accel_data(),
                sensor_data.get_bat(),
                sensor_data.get_pres(),
                ack,
            );
            // Form the message waiting to be sent to the host
            let mut message: Vec<u8> = Vec::new();
            message_to_host.form_message(&mut message);
            send_bytes(&message);
        }

        // safety checks
        safety_counter.increment_command_timeout();
        // Check if the time limit has been reached for no message received.
        if safety_counter.is_command_timeout() {
            // Panic because connection timed out.
            state_machine.transition(
                State::Panic,
                &mut joystick_control,
                &mut general_controllers,
                &mut sensor_data_calibration_offset,
                &mut sensor_data,
            );
            // Reset the timeout counter, since it's going to go back to safe mode.
            safety_counter.reset_command_timeout();
        }
        // Check if battery level is low, if positive then go to panic state.
        if sensor_data.get_bat() < 120 {
            safety_counter.increment_battery_danger();
        }
        if safety_counter.is_battery_danger() {
            state_machine.transition(
                State::Panic,
                &mut joystick_control,
                &mut general_controllers,
                &mut sensor_data_calibration_offset,
                &mut sensor_data,
            );
            // then end the function
            panic!();
        }

        Blue.off();
        Yellow.off();
        wait_for_next_tick();
    }
    unreachable!();
}

fn update_joystick_control_and_controller(
    joystick_control: &mut JoystickControl,
    controller: &mut GeneralController,
    nice_received_message: &HostProtocol,
) {
    joystick_control.set_lift(nice_received_message.get_lift());
    joystick_control.set_yaw(nice_received_message.get_yaw());
    joystick_control.set_pitch(nice_received_message.get_pitch());
    joystick_control.set_roll(nice_received_message.get_roll());
    joystick_control.set_p(map_p_to_fixed(nice_received_message.get_p()));
    joystick_control.set_p1(map_p1_to_fixed(nice_received_message.get_p1()));
    joystick_control.set_p2(map_p2_to_fixed(nice_received_message.get_p2()));

    controller.yaw_control.set_kp(joystick_control.get_p());
    controller.pitch_control.set_kp1(joystick_control.get_p1());
    controller.pitch_control.set_kp2(joystick_control.get_p2());
    controller.roll_control.set_kp1(joystick_control.get_p1());
    controller.roll_control.set_kp2(joystick_control.get_p2());
}

/// verify the message received from the host
fn verify_message(message: &HostProtocol) -> u8 {
    // we check the start bit and the end bit first
    if message.get_start_flag() == 0x7b && message.get_end_flag() == 0x7d {
        if verify_crc(message) {
            0b1111_1111
        } else {
            0b0000_0000
        }
    } else {
        0b0000_0000
    }
}

/// verify the crc of the message received from the host
fn verify_crc(message: &HostProtocol) -> bool {
    let verification_crc = HostProtocol::calculate_crc16(message);
    verification_crc == message.get_crc()
}

/// map the mode received from the host to the state of the drone
fn map_to_state(mode_received: u8) -> State {
    match mode_received {
        0b1111_0000 => State::Panic,
        0b0000_1111 => State::Panic,
        0b0000_0000 => State::Safety,
        0b0000_0001 => State::Panic,
        0b0000_0010 => State::Manual,
        0b0000_0011 => State::Calibrate,
        0b0000_0100 => State::Yaw,
        0b0000_0101 => State::Full,
        0b0000_0110 => State::Raw,
        0b0000_0111 => State::Height,
        0b0000_1000 => State::Wireless,
        _ => State::Panic,
    }
}

/// map the state of the drone to the mode to be sent to the host
fn map_to_mode(current_state: &State) -> u8 {
    match current_state {
        State::Safety => 0b0000_0000,
        State::Panic => 0b0000_0001,
        State::Manual => 0b0000_0010,
        State::Calibrate => 0b0000_0011,
        State::Yaw => 0b0000_0100,
        State::Full => 0b0000_0101,
        State::Raw => 0b0000_0110,
        State::Height => 0b0000_0111,
        State::Wireless => 0b0000_1000,
    }
}

pub struct SafetyCounter {
    pub command_timeout: u64,
    pub battery_danger: u64,
}

impl SafetyCounter {
    pub fn new() -> Self {
        SafetyCounter {
            command_timeout: 0,
            battery_danger: 0,
        }
    }

    pub fn reset_command_timeout(&mut self) {
        self.command_timeout = 0;
    }

    pub fn increment_command_timeout(&mut self) {
        self.command_timeout += 1;
    }

    pub fn increment_battery_danger(&mut self) {
        self.battery_danger += 1;
    }

    pub fn is_command_timeout(&self) -> bool {
        self.command_timeout > 100
    }

    pub fn is_battery_danger(&self) -> bool {
        self.battery_danger > 100
    }
}

pub struct SensorData {
    motors: [u16; 4],
    quaternion: Quaternion,
    ypr: YawPitchRoll,
    non_offset_ypr: YawPitchRoll,
    accel: Accel,
    gyro: Gyro,
    bat: u16,
    pres: i32,
    non_offset_pres: i32,
    last: Instant,
    now: Instant,
    dt: Duration,
}

#[allow(dead_code)]
impl SensorData {
    pub fn new() -> Self {
        let zero_u30 = FixedI32::<types::extra::U30>::from_num(0.0);
        let zero_i6 = I16F16::from_num(0.0);
        let quaternion: Quaternion = Quaternion {
            w: zero_u30,
            x: zero_u30,
            y: zero_u30,
            z: zero_u30,
        };
        let ypr = YawPitchRoll {
            yaw: zero_i6,
            pitch: zero_i6,
            roll: zero_i6,
        };
        let accel = Accel { x: 0, y: 0, z: 0 };
        let gyro = Gyro { x: 0, y: 0, z: 0 };
        let bat: u16 = 0;
        let pres: i32 = 0;
        let motors: [u16; 4] = [0; 4];
        let last = Instant::now();
        let now = Instant::now();
        let dt = Duration::from_secs(0);
        SensorData {
            motors,
            quaternion,
            ypr,
            non_offset_ypr: ypr,
            accel,
            gyro,
            bat,
            pres,
            non_offset_pres: pres,
            last,
            now,
            dt,
        }
    }

    pub fn update_dt(&mut self) {
        self.now = Instant::now();
        self.dt = self.now.duration_since(self.last);
        self.last = self.now;
    }

    pub fn update_motors(&mut self) {
        self.motors = get_motors();
    }

    pub fn update_quaternion(&mut self) {
        self.quaternion = block!(read_dmp_bytes()).unwrap();
    }

    pub fn update_ypr(&mut self, sensor_data_offset: &SensorOffset) {
        self.ypr = YawPitchRoll::from(self.get_quaternion());
        self.non_offset_ypr = self.ypr;
        self.ypr.yaw = self.ypr.yaw.saturating_sub(sensor_data_offset.yaw_offset);
        self.ypr.pitch = self
            .ypr
            .pitch
            .saturating_sub(sensor_data_offset.pitch_offset);
        self.ypr.roll = self.ypr.roll.saturating_sub(sensor_data_offset.roll_offset);
        // self.ypr.yaw -= sensor_data_offset.yaw_offset;
        // self.ypr.pitch -= sensor_data_offset.pitch_offset;
        // self.ypr.roll -= sensor_data_offset.roll_offset;
    }

    pub fn update_accel_gyro(&mut self) {
        (self.accel, self.gyro) = read_raw().unwrap();
    }

    pub fn update_bat(&mut self) {
        self.bat = read_battery();
        // self.bat = 500;
    }

    pub fn update_pres(&mut self, sensor_data_offset: &SensorOffset) {
        self.non_offset_pres = read_pressure() as i32;
        self.pres = self
            .non_offset_pres
            .saturating_sub(sensor_data_offset.lift_offset);
    }

    pub fn get_dt(&self) -> Duration {
        self.dt
    }

    pub fn get_motors(&self) -> [u16; 4] {
        self.motors
    }

    pub fn get_quaternion(&self) -> Quaternion {
        self.quaternion
    }

    pub fn get_ypr(&self) -> YawPitchRoll {
        self.ypr
    }

    pub fn get_ypr_data(&self) -> [I16F16; 3] {
        [
            self.get_ypr().yaw,
            self.get_ypr().pitch,
            self.get_ypr().roll,
        ]
    }

    pub fn get_accel(&self) -> Accel {
        self.accel
    }

    pub fn get_accel_data(&self) -> [i16; 3] {
        [self.get_accel().x, self.get_accel().y, self.get_accel().z]
    }

    pub fn get_gyro(&self) -> Gyro {
        self.gyro
    }

    pub fn get_gyro_data(&self) -> [i16; 3] {
        [self.get_gyro().x, self.get_gyro().y, self.get_gyro().z]
    }

    pub fn get_bat(&self) -> u16 {
        self.bat
    }

    pub fn get_pres(&self) -> i32 {
        self.pres
    }

    pub fn update_all(&mut self, sensor_data_offset: &SensorOffset) {
        self.update_dt();
        self.update_motors();
        self.update_quaternion();
        self.update_ypr(sensor_data_offset);
        self.update_accel_gyro();
        self.update_bat();
        self.update_pres(sensor_data_offset);
    }

    pub fn resume_non_offset(&mut self) {
        self.ypr = self.non_offset_ypr;
        self.pres = self.non_offset_pres;
    }
}

pub struct SensorOffset {
    yaw_offset: I16F16,
    pitch_offset: I16F16,
    roll_offset: I16F16,
    lift_offset: i32,
    sample_count: u32,
}

impl SensorOffset {
    pub fn new() -> Self {
        SensorOffset {
            yaw_offset: I16F16::from_num(0.0),
            pitch_offset: I16F16::from_num(0.0),
            roll_offset: I16F16::from_num(0.0),
            lift_offset: 0,
            sample_count: 0,
        }
    }

    pub fn reset_offset(&mut self) {
        self.yaw_offset = I16F16::from_num(0.0);
        self.pitch_offset = I16F16::from_num(0.0);
        self.roll_offset = I16F16::from_num(0.0);
        self.lift_offset = 0;
        self.sample_count = 0;
    }

    pub fn reset_sample_count(&mut self) {
        self.sample_count = 0;
    }

    pub fn update_sample_count(&mut self) {
        self.sample_count += 1;
    }

    pub fn get_sample_count(&self) -> u32 {
        self.sample_count
    }

    pub fn update_yaw_offset(&mut self, yaw_offset: I16F16) {
        self.yaw_offset += yaw_offset;
    }

    pub fn update_pitch_offset(&mut self, pitch_offset: I16F16) {
        self.pitch_offset += pitch_offset;
    }

    pub fn update_roll_offset(&mut self, roll_offset: I16F16) {
        self.roll_offset += roll_offset;
    }

    pub fn update_lift_offset(&mut self, lift_offset: i32) {
        self.lift_offset += lift_offset;
    }

    pub fn calculate_offset(&mut self) {
        self.yaw_offset /= I16F16::from_num(self.sample_count);
        self.pitch_offset /= I16F16::from_num(self.sample_count);
        self.roll_offset /= I16F16::from_num(self.sample_count);
        self.lift_offset /= self.sample_count as i32;
    }
}
