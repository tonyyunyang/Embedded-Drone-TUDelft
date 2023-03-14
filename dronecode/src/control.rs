use crate::control::state_machine::{execute_state_function, JoystickControl, StateMachine};
use crate::yaw_pitch_roll::YawPitchRoll;
use alloc::vec::Vec;
use protocol::format::{DeviceProtocol, HostProtocol};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::block;
use tudelft_quadrupel::led::Yellow;
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{receive_bytes, send_bytes};

use self::state_machine::State;
mod motor_control;
mod state_machine;

pub fn control_loop() -> ! {
    // Save the tick frequency as a variable so it can be used for multiple things.
    let tick_frequency = 100;
    let battery_low_counter_limit = 500;
    set_tick_frequency(tick_frequency);
    // tick_frequency is how many ticks per second.
    // The timeout counter goes up with each tick. This means it times out after x seconds.
    let timeout_limit = 3 * tick_frequency;
    let mut last = Instant::now();
    let mut state_machine = StateMachine::new();
    let mut joystick_control = JoystickControl::new();
    let mut nice_received_message = HostProtocol::new(0, 0, 0, 0, 0, 0, 0, 0);
    let mut ack = 0b0000_0000;
    let mut buf = [0u8; 64];
    let mut message_buffer: Vec<u8> = Vec::new();
    let mut received_bytes_count = 0;
    let mut start_receiving = false;
    let mut mode = 0b0000_0000;
    let mut timeout_counter = 0;
    let mut battery_low_counter = 0;
    let mut lift: u8 = 0;
    let mut yaw: u8 = 0;
    let mut pitch: u8 = 0;
    let mut roll: u8 = 0;
    // let mut p: u8 = 0;
    // let mut p1: u8 = 0;
    // let mut p2: u8 = 0;

    // let mut logger = logger::BlackBoxLogger::new();
    // logger.start_logging();

    for i in 0.. {
        // Check if the time limit has been reached for no message received.
        if timeout_counter > timeout_limit {
            // Panic because connection timed out.
            state_machine.transition(State::Panic);
            // Reset the timeout counter, since it's going to go back to safe mode.
            timeout_counter = 0;

            // TODO Might want to communicate something to the PC here.

            // Wait for the next tick from the timer interrupt.
            wait_for_next_tick();
            // Go to next loop cycle.
            continue;
        }
        // reads the data from the sensors
        let now = Instant::now();
        let dt = now.duration_since(last);
        last = now;
        let motors = get_motors();
        let quaternion = block!(read_dmp_bytes()).unwrap();
        let ypr = YawPitchRoll::from(quaternion);
        let (accel, _) = read_raw().unwrap();
        let bat = read_battery();
        let pres = read_pressure();
        // Check if battery level is low, if positive then go to panic state.
        if bat < 7 {
            battery_low_counter += 1;
        }
        if battery_low_counter > battery_low_counter_limit {
            state_machine.transition(State::Panic);
            // then end the function
            // panic!();
        }

        // the code below is for receiving the message from the host
        let num = receive_bytes(&mut buf);
        if num != 0 {
            if num == 12 {
                for i in buf.iter().take(num) {
                    let received_byte = *i;
                    if received_byte == 0x7b && !start_receiving {
                        message_buffer.clear();
                        start_receiving = true;
                    }
                    if start_receiving {
                        message_buffer.push(received_byte);
                        received_bytes_count += 1;
                    }
                    if received_bytes_count < 12 {
                        continue;
                    }
                }
                nice_received_message = HostProtocol::format_message_alloc(&mut message_buffer);
                mode = nice_received_message.get_mode();
                lift = nice_received_message.get_lift();
                yaw = nice_received_message.get_yaw();
                pitch = nice_received_message.get_pitch();
                roll = nice_received_message.get_roll();
                // p = nice_received_message.get_p();
                // p1 = nice_received_message.get_p1();
                // p2 = nice_received_message.get_p2();
                ack = verify_message(&nice_received_message);
                received_bytes_count = 0;
                message_buffer.clear();
            } else {
                received_bytes_count = 0;
                message_buffer.clear();
            }
        }

        // if the code received by the drone is acknowledged, then we transition to the next state, and execute corresponding function
        if ack == 0b1111_1111 {
            Yellow.on();
            let next_state = map_to_state(mode);

            // Update global struct.
            joystick_control.set_lift(lift);
            joystick_control.set_yaw(yaw);
            joystick_control.set_pitch(pitch);
            joystick_control.set_roll(roll);
            // Assume that transition is false before transition, will become true if transition is successful.
            let mut transition_result = false;
            // After updating, check if the stick is in a neutral state before transition.
            // The OR statement is added for panic state, since drone should always be able to panic.
            if (joystick_control.joystick_neutral_check()) || next_state == State::Panic {
                transition_result = state_machine.transition(next_state);
            } else {
                ack = 0b1111_0000; // this means that the joystick is not in neutral state
            }

            let current_state = state_machine.state();
            mode = map_to_mode(&current_state);
            // Reset time out counter, since message was received successfully.
            timeout_counter = 0;
            if transition_result {
                execute_state_function(&current_state, &nice_received_message);
            }
        }

        // the code below is for sending the message to the host
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

            // reset the ack
            ack = 0b0000_0000;
            Yellow.off();
        }
        timeout_counter += 1;
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
