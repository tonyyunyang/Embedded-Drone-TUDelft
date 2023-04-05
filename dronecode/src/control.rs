use core::time::Duration;

use crate::control::state_machine::{execute_state_function, JoystickControl, StateMachine};
use crate::storage::Storage;
use crate::yaw_pitch_roll::YawPitchRoll;
use alloc::vec;
use alloc::vec::Vec;
// use heapless::Vec as HVec;
use protocol::format::{DeviceProtocol, HostProtocol};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::block;
use tudelft_quadrupel::fixed::types::I16F16;
use tudelft_quadrupel::fixed::{types, FixedI32};
use tudelft_quadrupel::flash::FlashError;
use tudelft_quadrupel::led::Led::{Blue, Green};
use tudelft_quadrupel::led::Yellow;
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::structs::{Accel, Gyro, Quaternion};
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{receive_bytes, send_bytes};

use self::state_machine::State;

mod motor_control;
mod state_machine;

#[allow(unused_assignments)]
pub fn control_loop() -> ! {
    // Initialize the variables for the control loop
    set_tick_frequency(100);
    let mut safety_counter = SafetyCounter::new();
    let mut log_data = LogData::new();
    Green.on();
    if log_data.storage.erase_flash().is_ok() {
        Green.off();
    }
    let mut sensor_data = SensorData::new();
    let mut state_machine = StateMachine::new();
    let mut joystick_control = JoystickControl::new();
    let mut nice_received_message = HostProtocol::new(0, 0, 0, 0, 0, 0, 0, 0);
    let mut ack = 0b0000_0000;
    let mut buf = [0u8; 257];
    // let mut command_buf: HVec<u8, 1024> = HVec::new();
    let mut command_buf: Vec<u8> = Vec::new();
    let mut mode = 0b0000_0000;

    for i in 0.. {
        // update the sensor data
        sensor_data.update_all();

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
            joystick_control.set_lift(nice_received_message.get_lift());
            joystick_control.set_yaw(nice_received_message.get_yaw());
            joystick_control.set_pitch(nice_received_message.get_pitch());
            joystick_control.set_roll(nice_received_message.get_roll());
            joystick_control.set_p(nice_received_message.get_p());
            joystick_control.set_p1(nice_received_message.get_p1());
            joystick_control.set_p2(nice_received_message.get_p2());

            // Assume that transition is false before transition, will become true if transition is successful.
            let mut transition_result = false;
            // After updating, check if the stick is in a neutral state before transition.
            // The OR statement is added for panic state, since drone should always be able to panic.
            (transition_result, ack) = state_machine.transition(next_state, &mut joystick_control);

            let current_state = state_machine.state();
            mode = map_to_mode(&current_state);
            // Reset time out counter, since message was received successfully.
            safety_counter.reset_command_timeout();
            if transition_result && ack != 0b0000_1111 {
                execute_state_function(&current_state, &nice_received_message);
            }
        }

        if i % 20 == 0 {
            // 5 Hz
            if mode < 10 {
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

                let message_to_log = DeviceProtocol::new(
                    10 + mode,
                    sensor_data.get_dt().as_millis() as u16,
                    sensor_data.get_motors(),
                    sensor_data.get_ypr_data(),
                    sensor_data.get_accel_data(),
                    sensor_data.get_bat(),
                    sensor_data.get_pres(),
                    ack,
                );

                let mut message: Vec<u8> = Vec::new();
                message_to_host.form_message(&mut message);

                let mut log_message: Vec<u8> = Vec::new();
                message_to_log.form_message(&mut log_message);
                Green.on();
                if log_data.save_data(&log_message).is_ok() {
                    Green.off();
                }
                send_bytes(&message);
            } else {
                Green.on();
                let data = log_data.load_data();
                if let Ok(data) = data {
                    send_bytes(&data);
                    Green.off();
                }
            }
        }

        // safety checks
        safety_counter.increment_command_timeout();
        // Check if the time limit has been reached for no message received.
        if safety_counter.is_command_timeout() {
            // Panic because connection timed out.
            state_machine.transition(State::Panic, &mut joystick_control);
            // Reset the timeout counter, since it's going to go back to safe mode.
            safety_counter.reset_command_timeout();
        }
        // Check if battery level is low, if positive then go to panic state.
        if sensor_data.get_bat() < 120 {
            safety_counter.increment_battery_danger();
        }
        if safety_counter.is_battery_danger() {
            state_machine.transition(State::Panic, &mut joystick_control);
            // then end the function
            panic!();
        }

        Blue.off();
        Yellow.off();
        wait_for_next_tick();
    }
    unreachable!();
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
        0b0000_1010 => State::ReadLogs,
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
        State::ReadLogs => 0b0000_1010,
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
        self.command_timeout > 300
    }

    pub fn is_battery_danger(&self) -> bool {
        self.battery_danger > 500
    }
}

pub struct SensorData {
    motors: [u16; 4],
    quaternion: Quaternion,
    ypr: YawPitchRoll,
    accel: Accel,
    gyro: Gyro,
    bat: u16,
    pres: u32,
    last: Instant,
    now: Instant,
    dt: Duration,
}

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
        let pres: u32 = 0;
        let motors: [u16; 4] = [0; 4];
        let last = Instant::now();
        let now = Instant::now();
        let dt = Duration::from_secs(0);
        SensorData {
            motors,
            quaternion,
            ypr,
            accel,
            gyro,
            bat,
            pres,
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

    pub fn update_ypr(&mut self) {
        self.ypr = YawPitchRoll::from(self.quaternion);
    }

    pub fn update_accel_gyro(&mut self) {
        (self.accel, self.gyro) = read_raw().unwrap();
    }

    pub fn update_bat(&mut self) {
        self.bat = read_battery();
        self.bat = 500;
    }

    pub fn update_pres(&mut self) {
        self.pres = read_pressure();
    }

    pub fn get_dt(&self) -> Duration {
        self.dt
    }

    pub fn get_motors(&self) -> [u16; 4] {
        self.motors
    }

    // pub fn get_quaternion(&self) -> Quaternion {
    //     self.quaternion
    // }

    // pub fn get_ypr(&self) -> YawPitchRoll {
    //     self.ypr
    // }

    pub fn get_ypr_data(&self) -> [I16F16; 3] {
        [self.ypr.yaw, self.ypr.pitch, self.ypr.roll]
    }

    // pub fn get_accel(&self) -> Accel {
    //     self.accel
    // }

    pub fn get_accel_data(&self) -> [i16; 3] {
        [self.accel.x, self.accel.y, self.accel.z]
    }

    // pub fn get_gyro(&self) -> Gyro {
    //     self.gyro
    // }

    // pub fn get_gyro_data(&self) -> [i16; 3] {
    //     [self.gyro.x, self.gyro.y, self.gyro.z]
    // }

    pub fn get_bat(&self) -> u16 {
        self.bat
    }

    pub fn get_pres(&self) -> u32 {
        self.pres
    }

    pub fn update_all(&mut self) {
        self.update_dt();
        self.update_motors();
        self.update_quaternion();
        self.update_ypr();
        self.update_accel_gyro();
        self.update_bat();
        self.update_pres();
    }
}

pub struct LogData {
    storage: Storage,
}

impl LogData {
    pub fn new() -> Self {
        LogData {
            storage: Storage::new(0x000000, 0x01FFFF),
        }
    }

    pub fn save_data(&mut self, message: &[u8]) -> Result<(), FlashError> {
        self.storage.write(message)
    }

    pub fn load_data(&mut self) -> Result<Vec<u8>, FlashError> {
        let mut message: Vec<u8> = vec![0; 40]; // Pre-allocate the buffer with an arbitrary size
        match self.storage.read(&mut message) {
            Ok(_bytes_read) => {
                // message.truncate(bytes_read);
                Ok(message)
            }
            Err(e) => Err(e),
        }
    }
}
