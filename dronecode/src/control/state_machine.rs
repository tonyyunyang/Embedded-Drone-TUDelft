// This file implements the state machine for the drone's control module.
// The state machine is a finite state machine (FSM) that is used to control the drone.

use alloc::string::String;

// Define the possible states of the state machine.
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

// Define the transitions between states.
pub trait Transition {
    fn safety(&self) -> Option<State>;
    fn panic(&self) -> Option<State>;
    fn manual(&self) -> Option<State>;
    fn calibrate(&self) -> Option<State>;
    fn yaw(&self) -> Option<State>;
    fn full(&self) -> Option<State>;
    fn raw(&self) -> Option<State>;
    fn height(&self) -> Option<State>;
    fn wireless(&self) -> Option<State>;
}

// Define the state machine.
pub struct StateMachine<T: Transition> {
    transition: T,
    state: State,
    operation_ready: bool,
    // Add more fields here if needed such as data to be stored in the state machine.
}

// Implement methods for the state machine.
impl<T: Transition> StateMachine<T> {
    // Create a new state machine in the Safety state.
    pub fn new(transition: T) -> StateMachine<T> {
        StateMachine {
            transition,
            state: State::Safety,
            operation_ready: false,
        }
    }

    // Get the current state of the state machine.
    pub fn state(&self) -> State {
        self.state.clone()
    }

    // Transition the state machine to a new state.
    pub fn transition(&mut self, state: State) -> Result<(), String> {
        match state {
            State::Safety => self.transition_to_safety(),
            State::Panic => self.transition_to_panic(),
            State::Manual => self.transition_to_manual(),
            State::Calibrate => self.transition_to_calibrate(),
            State::Yaw => self.transition_to_operation(state),
            State::Full => self.transition_to_operation(state),
            State::Raw => self.transition_to_operation(state),
            State::Height => self.transition_to_operation(state),
            State::Wireless => self.transition_to_operation(state),
        }
    }

    // Transition to the Safety state.
    fn transition_to_safety(&mut self) -> Result<(), String> {
        if let Some(next_state) = self.transition.safety() {
            self.state = next_state;
            Ok(())
        } else {
            Err(String::from("Invalid transition to Safety state"))
        }
    }

    // Transition to the Panic state.
    fn transition_to_panic(&mut self) -> Result<(), String> {
        if let Some(next_state) = self.transition.panic() {
            self.state = next_state;
            Ok(())
        } else {
            Err(String::from("Invalid transition to Panic state"))
        }
    }

    // Transition to the Manual state.
    fn transition_to_manual(&mut self) -> Result<(), String> {
        if let Some(next_state) = self.transition.manual() {
            self.state = next_state;
            Ok(())
        } else {
            Err(String::from("Invalid transition to Manual state"))
        }
    }

    // Transition to the Calibrate state.
    fn transition_to_calibrate(&mut self) -> Result<(), String> {
        if let Some(next_state) = self.transition.calibrate() {
            self.state = next_state;
            Ok(())
        } else {
            Err(String::from("Invalid transition to Calibrate state"))
        }
    }

    fn transition_to_operation(&mut self, state: State) -> Result<(), String> {
        if self.operation_ready {
            match state {
                State::Yaw => self.transition_to_yaw(),
                State::Full => self.transition_to_full(),
                State::Raw => self.transition_to_raw(),
                State::Height => self.transition_to_height(),
                State::Wireless => self.transition_to_wireless(),
                _ => unreachable!()
            }
        } else {
            Err(String::from("Invalid transition to calibration, no calibration done yet."))
        }
    }

    // Transition to the Yaw state.
    fn transition_to_yaw(&mut self) -> Result<(), String> {
        if let Some(next_state) = self.transition.yaw() {
            self.state = next_state;
            Ok(())
        } else {
            Err(String::from("Invalid transition to Yaw state"))
        }
    }

    // Transition to the Full state.
    fn transition_to_full(&mut self) -> Result<(), String> {
        if let Some(next_state) = self.transition.full() {
            self.state = next_state;
            Ok(())
        } else {
            Err(String::from("Invalid transition to Full state"))
        }
    }

    // Transition to the Raw state.
    fn transition_to_raw(&mut self) -> Result<(), String> {
        if let Some(next_state) = self.transition.raw() {
            self.state = next_state;
            Ok(())
        } else {
            Err(String::from("Invalid transition to Raw state"))
        }
    }

    // Transition to the Height state.
    fn transition_to_height(&mut self) -> Result<(), String> {
        if let Some(next_state) = self.transition.height() {
            self.state = next_state;
            Ok(())
        } else {
            Err(String::from("Invalid transition to Height state"))
        }
    }

    // Transition to the Wireless state.
    fn transition_to_wireless(&mut self) -> Result<(), String> {
        if let Some(next_state) = self.transition.wireless() {
            self.state = next_state;
            Ok(())
        } else {
            Err(String::from("Invalid transition to Wireless state"))
        }
    }
}
