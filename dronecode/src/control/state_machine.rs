// This file implements the state machine for the drone's control module.
// The state machine is a finite state machine (FSM) that is used to control the drone.

use protocol::format::HostProtocol;
use tudelft_quadrupel::{fixed::types::I16F16, motor::set_motors};

use crate::control::state_machine::State::Safety;
use core::clone::Clone;

use super::motor_control::*;

// Define the possible states of the state machine.
#[derive(Clone, PartialEq)]
pub enum State {
    /// Mode 0: Ignore all PC commands, keep motors off.
    Safety,

    /// Mode 1: Moderate RPM to land safely, then enter safe mode.
    Panic,

    /// Mode 2: Pass joystick and keyboard commands to drone.
    Manual,

    /// Mode 3: Calibrate sensor readings for level attitude and zero movement.
    Calibrate,

    /// Mode 4: Control drone yaw using twist handle, adjust rotor thrusts to counteract disturbance.
    Yaw,

    /// Mode 5: Control roll, pitch, and yaw using all three controllers.
    Full,

    /// Mode 6: Raw data collection for debugging.
    Raw,

    /// Mode 7: Maintain drone height.
    Height,

    /// Mode 8: Wireless communication with ground station.
    Wireless,
}

// Define the state machine.
#[derive(Clone)]
pub struct StateMachine {
    state: State,
    pub operation_ready: bool,
    pub permissions: Permissions,
    // Add more fields here if needed such as data to be stored in the state machine.
}

// Yaw, Pitch, Roll and Height are all for the controller.
// Wireless can also be read from the mode, but having it as Permission might be useful.
// Sensors specify whether the mode should process the sensor readings into the calculations.
// For example, manual mode would not use sensor data -> sensors: false.
#[derive(Clone)]
pub struct Permissions {
    pub controller: bool,
    pub calibration: bool,
    pub yaw_control: bool,
    pub pitch_roll_control: bool,
    pub height_control: bool,
    pub wireless: bool,
    pub sensors: bool,
}

// Implement methods for the state machine.
impl StateMachine {
    // Create a new state machine in the Safety state.
    pub fn new() -> StateMachine {
        StateMachine {
            state: State::Safety,
            operation_ready: false,
            permissions: Permissions {
                controller: false,
                calibration: false,
                yaw_control: false,
                pitch_roll_control: false,
                height_control: false,
                wireless: false,
                sensors: false,
            },
        }
    }

    // Get the current state of the state machine.
    pub fn state(&self) -> State {
        self.state.clone()
    }

    // Transition the state machine to a new state.
    // Returns true for a proper transition and false for an illegal transition.
    // This value can then be communicated back to the PC.
    pub fn transition(&mut self, next_state: State) -> bool {
        if self.state() != next_state {
            match next_state {
                State::Safety => self.transition_safe(),
                State::Panic => self.transition_panic(),
                State::Manual => self.transition_manual(),
                State::Calibrate => self.transition_calibrate(),
                State::Yaw | State::Full | State::Raw | State::Height | State::Wireless => {
                    self.transition_operation(next_state)
                }
            }
        } else {
            false
        }
    }

    // Safe mode should do nothing so everything is false.
    fn transition_safe(&mut self) -> bool {
        self.state = State::Safety;
        self.permissions.controller = false;
        self.permissions.calibration = false;
        self.permissions.yaw_control = false;
        self.permissions.pitch_roll_control = false;
        self.permissions.height_control = false;
        self.permissions.wireless = false;
        self.permissions.sensors = false;
        true
    }

    // Panic mode should also do nothing from the controller, might need the sensors (for now false).
    fn transition_panic(&mut self) -> bool {
        self.state = State::Panic;
        self.permissions.controller = false;
        self.permissions.calibration = false;
        self.permissions.yaw_control = false;
        self.permissions.pitch_roll_control = false;
        self.permissions.height_control = false;
        self.permissions.wireless = false;
        self.permissions.sensors = false;
        // Reset the calibration flag if there was a panic.
        self.operation_ready = false;
        // TODO Proper power down instead of instant

        // Power down motors, due to panic.
        let no_motor_power = [0, 0, 0, 0];
        set_motors(no_motor_power);

        // Automatically go back to safe mode.
        self.transition_safe()
    }

    // Manual mode should accept all controller movements, but not use any sensor data.
    fn transition_manual(&mut self) -> bool {
        // Can only go into manual mode from safe mode.
        if self.state == State::Safety {
            self.state = State::Manual;
            self.permissions.controller = true;
            self.permissions.calibration = false;
            self.permissions.yaw_control = false;
            self.permissions.pitch_roll_control = false;
            self.permissions.height_control = false;
            self.permissions.wireless = false;
            self.permissions.sensors = false;
            true
        } else {
            false
        }
    }

    // Calibration mode should only accept sensor data, no controller movements.
    fn transition_calibrate(&mut self) -> bool {
        // Can only go into calibration mode from safe mode.
        if self.state == State::Safety {
            self.state = State::Calibrate;
            self.permissions.controller = false;
            self.permissions.calibration = true;
            self.permissions.yaw_control = false;
            self.permissions.pitch_roll_control = false;
            self.permissions.height_control = false;
            self.permissions.wireless = false;
            self.permissions.sensors = true;
            true
        } else {
            false
        }
    }

    // Checks whether calibration is done and then redirects to the required transition.
    // All operating modes use the sensors for control loops.
    fn transition_operation(&mut self, next_state: State) -> bool {
        // Calibration flag check
        if self.operation_ready && self.state == Safety {
            match next_state {
                State::Yaw => self.transition_yaw(),
                State::Full => self.transition_full(),
                State::Raw => self.transition_raw(),
                State::Height => self.transition_height(),
                State::Wireless => self.transition_wireless(),
                _ => unreachable!(), // Match needs to be exhaustive, but this is unreachable due to match in transition()
            }
        } else {
            false
        }
    }

    // Yaw control mode should enable the yaw control loop.
    fn transition_yaw(&mut self) -> bool {
        self.state = State::Yaw;
        self.permissions.controller = true;
        self.permissions.yaw_control = true;
        self.permissions.pitch_roll_control = false;
        self.permissions.height_control = false;
        self.permissions.wireless = false;
        self.permissions.sensors = true;
        true
    }

    // Full control mode should enable the yaw control loop and the pitch-roll control loop.
    fn transition_full(&mut self) -> bool {
        self.state = State::Full;
        self.permissions.controller = true;
        self.permissions.calibration = false;
        self.permissions.yaw_control = true;
        self.permissions.pitch_roll_control = true;
        self.permissions.height_control = false;
        self.permissions.wireless = false;
        self.permissions.sensors = true;
        true
    }

    // Raw sensor reading mode should add the filters, to the other control loops.
    fn transition_raw(&mut self) -> bool {
        self.state = State::Raw;
        self.permissions.controller = true;
        self.permissions.calibration = false;
        self.permissions.yaw_control = true;
        self.permissions.pitch_roll_control = true;
        self.permissions.height_control = false;
        self.permissions.wireless = false;
        self.permissions.sensors = true;
        true
    }

    // On top of the two control loops and the filters, a height control loop is added.
    fn transition_height(&mut self) -> bool {
        self.state = State::Height;
        self.permissions.controller = true;
        self.permissions.calibration = false;
        self.permissions.yaw_control = true;
        self.permissions.pitch_roll_control = true;
        self.permissions.height_control = true;
        self.permissions.wireless = false;
        self.permissions.sensors = true;
        true
    }

    // After all control loops and filters work, wireless mode can be used to fly the drone without the USB cable.
    // Requires high stabilization of drone, since wireless bandwidth is lower.
    // Only use this mode after all previous modes have been implemented.
    fn transition_wireless(&mut self) -> bool {
        self.state = State::Wireless;
        self.permissions.controller = true;
        self.permissions.calibration = false;
        self.permissions.yaw_control = true;
        self.permissions.pitch_roll_control = true;
        self.permissions.height_control = true;
        self.permissions.wireless = true;
        self.permissions.sensors = true;
        true
    }
}

pub fn execute_state_function(current_state: &State, command: &HostProtocol) {
    match current_state {
        State::Safety => {
            safety_mode();
        }
        State::Panic => {
            panic_mode();
        }
        State::Manual => {
            manual_mode(
                command.get_lift(),
                command.get_yaw(),
                command.get_pitch(),
                command.get_roll(),
            );
        }
        State::Calibrate => {
            calibrate_mode();
        }
        State::Yaw => {
            yaw_mode(
                command.get_lift(),
                command.get_yaw(),
                command.get_pitch(),
                command.get_roll(),
            );
        }
        State::Full => {
            full_mode(
                command.get_lift(),
                command.get_yaw(),
                command.get_pitch(),
                command.get_roll(),
            );
        }
        State::Raw => {
            raw_mode(
                command.get_lift(),
                command.get_yaw(),
                command.get_pitch(),
                command.get_roll(),
            );
        }
        State::Height => {
            height_mode();
        }
        State::Wireless => {
            wireless_mode();
        }
    }
}

fn safety_mode() {
    // TODO
}

fn panic_mode() {
    // TODO
}

fn manual_mode(lift: u8, yaw: u8, pitch: u8, roll: u8) {
    let lift: u16 = map_lift_command_manual(lift);
    let yaw: u16 = map_yaw_command_manual(yaw);
    let pitch: u16 = map_pitch_command_manual(pitch);
    let roll: u16 = map_roll_command_manual(roll);
    set_motor_speeds_manual(lift, yaw, pitch, roll);
}

fn calibrate_mode() {
    // TODO
}

#[allow(unused_variables)]
fn yaw_mode(lift: u8, yaw: u8, pitch: u8, roll: u8) {
    let lift: u16 = 200; // this should be the value that keeps the drone in the air stable
    let yaw: I16F16 = map_yaw_command(yaw);
    let pitch: u16 = 0;
    let roll: u16 = 0;
}

#[allow(unused_variables)]
fn full_mode(lift: u8, yaw: u8, pitch: u8, roll: u8) {
    // directly map the lift to the motor speeds
    let lift = map_lift_command(lift);
    // map roll and pitch to angles, map yaw to angular velocity
    let yaw = map_yaw_command(yaw);
    let pitch = map_pitch_command(pitch);
    let roll = map_roll_command(roll);
}

#[allow(unused_variables)]
fn raw_mode(lift: u8, yaw: u8, pitch: u8, roll: u8) {
    // TODO
}

fn height_mode() {
    // TODO
}

fn wireless_mode() {
    // TODO
}
