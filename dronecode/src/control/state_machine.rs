// This file implements the state machine for the drone's control module.
// The state machine is a finite state machine (FSM) that is used to control the drone.

use crate::control::state_machine::State::Safety;
use crate::panic;
use alloc::string::String;
use core::clone::Clone;

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
    operation_ready: bool,
    // Add more fields here if needed such as data to be stored in the state machine.
}

// Implement methods for the state machine.
impl StateMachine {
    // Create a new state machine in the Safety state.
    pub fn new() -> StateMachine {
        StateMachine {
            state: State::Safety,
            operation_ready: false,
        }
    }

    // Get the current state of the state machine.
    pub fn state(&self) -> State {
        self.state.clone()
    }

    // Transition the state machine to a new state.
    pub fn transition(&mut self, next_state: State) {
        if self.state != next_state {
            match next_state {
                State::Safety => self.state = Safety,
                State::Panic => self.transition_panic(),
                State::Manual => self.state = State::Manual,
                State::Calibrate => self.transition_calibrate(),
                State::Yaw | State::Full | State::Raw | State::Height | State::Wireless => {
                    self.transition_operation(next_state)
                }
            }
        }
    }

    fn transition_panic(&mut self) {
        self.state = State::Panic;
        // TODO Panic handling
    }

    fn transition_calibrate(&mut self) {
        if self.state == State::Safety {
            self.state = State::Calibrate;
        } else {
            // TODO proper error handling
            panic!("Improper transition, can only go to calibrate from safety.")
        }
    }

    fn transition_operation(&mut self, next_state: State) {
        if self.operation_ready && self.state == Safety {
            match next_state {
                State::Yaw => self.state = State::Yaw,
                State::Full => self.state = State::Full,
                State::Raw => self.state = State::Raw,
                State::Height => self.state = State::Height,
                State::Wireless => self.state = State::Wireless,
                _ => unreachable!(),
            }
        } else {
            // TODO proper error handling
            panic!("Improper transition, can only operate if calibration is done from safety mode.")
        }
    }
}
