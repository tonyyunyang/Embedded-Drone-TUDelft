use crate::control::state_machine::{execute_state_function, JoystickControl, StateMachine};
use crate::yaw_pitch_roll::YawPitchRoll;
use alloc::vec::Vec;
use protocol::format::{DeviceProtocol, HostProtocol};
use tudelft_quadrupel::barometer::read_pressure;
// use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::block;
use tudelft_quadrupel::fixed::types::I16F16;
use tudelft_quadrupel::fixed::{types, FixedI32};
use tudelft_quadrupel::led::Led::Blue;
use tudelft_quadrupel::led::Yellow;
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::structs::{Accel, Quaternion};
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{receive_bytes, send_bytes};

use self::state_machine::State;
mod motor_control;
mod state_machine;

#[allow(unused_assignments)]
pub fn control_loop() -> ! {
    // Save the tick frequency as a variable so it can be used for multiple things.
    let tick_frequency = 100;
    // let battery_low_counter_limit = 500;
    let timeout_limit = 5 * tick_frequency;
    let mut timeout_counter = 0;
    // let mut battery_low_counter = 0;
    set_tick_frequency(tick_frequency);
    // Create the variables for the read values from the sensors
    let mut motors: [u16; 4] = [0; 4];
    let zero_u30 = FixedI32::<types::extra::U30>::from_num(0.0);
    let zero_i6 = I16F16::from_num(0.0);
    let mut quaternion: Quaternion = Quaternion {
        w: zero_u30,
        x: zero_u30,
        y: zero_u30,
        z: zero_u30,
    };
    let mut ypr = YawPitchRoll {
        yaw: zero_i6,
        pitch: zero_i6,
        roll: zero_i6,
    };
    let mut accel = Accel { x: 0, y: 0, z: 0 };
    let mut bat: u16 = 0;
    let mut pres: u32 = 0;
    let mut last = Instant::now();
    let mut state_machine = StateMachine::new();
    let mut joystick_control = JoystickControl::new();
    let mut nice_received_message = HostProtocol::new(0, 0, 0, 0, 0, 0, 0, 0);
    let mut ack = 0b0000_0000;
    let mut buf = [0u8; 257];
    let mut command_buf: Vec<u8> = Vec::new();
    let mut mode = 0b0000_0000;
    // let mut p: u8 = 0;
    // let mut p1: u8 = 0;
    // let mut p2: u8 = 0;

    for i in 0.. {
        // the code below is for sending the message to the host
        // reads the data from the sensors
        let now = Instant::now();
        let dt = now.duration_since(last);
        last = now;

        // the code below is for receiving the message from the host
        let num = receive_bytes(&mut buf);
        command_buf.extend_from_slice(&buf[0..num]);
        if command_buf.len() >= 12 {
            let temp = command_buf.clone();
            let (message, rest) = temp.split_at(12);
            command_buf.clear();
            command_buf.extend_from_slice(rest);
            nice_received_message = HostProtocol::format_message_not_mut(message);
            // p = nice_received_message.get_p();
            // p1 = nice_received_message.get_p1();
            // p2 = nice_received_message.get_p2();
            ack = verify_message(&nice_received_message);
        }

        // if the code received by the drone is acknowledged, then we transition to the next state, and execute corresponding function
        if ack == 0b1111_1111 {
            Yellow.on();
            // Update global struct.
            mode = nice_received_message.get_mode();
            let lift = nice_received_message.get_lift();
            let yaw = nice_received_message.get_yaw();
            let pitch = nice_received_message.get_pitch();
            let roll = nice_received_message.get_roll();
            let next_state = map_to_state(mode);
            joystick_control.set_lift(lift);
            joystick_control.set_yaw(yaw);
            joystick_control.set_pitch(pitch);
            joystick_control.set_roll(roll);

            // Assume that transition is false before transition, will become true if transition is successful.
            let mut transition_result = false;
            // After updating, check if the stick is in a neutral state before transition.
            // The OR statement is added for panic state, since drone should always be able to panic.
            (transition_result, ack) = state_machine.transition(next_state, &mut joystick_control);

            let current_state = state_machine.state();
            mode = map_to_mode(&current_state);
            // Reset time out counter, since message was received successfully.
            timeout_counter = 0;
            if transition_result && ack != 0b0000_1111 {
                execute_state_function(&current_state, &nice_received_message);
            }
        }

        if i % 100 == 0 {
            motors = get_motors();
            quaternion = block!(read_dmp_bytes()).unwrap();
            ypr = YawPitchRoll::from(quaternion);
            (accel, _) = read_raw().unwrap();
            // bat = read_battery();
            bat = 99;
            pres = read_pressure();
        }

        if i % 100 == 0 {
            // Create an instance of the Drone Protocol struct
            let message_to_host = DeviceProtocol::new(
                mode,
                dt.as_millis() as u16,
                motors,
                [ypr.yaw, ypr.pitch, ypr.roll],
                [accel.x, accel.y, accel.z],
                bat,
                pres,
                ack,
            );

            // Form the message waiting to be sent to the host
            let mut message: Vec<u8> = Vec::new();
            message_to_host.form_message(&mut message);
            send_bytes(&message);
        }
        timeout_counter += 1;
        // Check if the time limit has been reached for no message received.
        if timeout_counter > timeout_limit {
            // Panic because connection timed out.
            state_machine.transition(State::Panic, &mut joystick_control);
            // Reset the timeout counter, since it's going to go back to safe mode.
            timeout_counter = 0;
        }
        // Check if battery level is low, if positive then go to panic state.
        // if bat < 120 {
        //     battery_low_counter += 1;
        // }
        // if battery_low_counter > battery_low_counter_limit {
        //     state_machine.transition(State::Panic, &mut joystick_control);
        //     // then end the function
        //     panic!();
        // }

        Blue.off();
        Yellow.off();
        wait_for_next_tick();
    }
    unreachable!();
}

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

fn verify_crc(message: &HostProtocol) -> bool {
    let verification_crc = HostProtocol::calculate_crc16(message);
    verification_crc == message.get_crc()
}

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
