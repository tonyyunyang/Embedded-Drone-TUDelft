// This file implements the state machine for the drone's control module.
// The state machine is a finite state machine (FSM) that is used to control the drone.

use tudelft_quadrupel::{
    fixed::types::I16F16,
    led::Led::{Blue, Red},
};

use crate::control::state_machine::State::Safety;
use core::clone::Clone;

use super::{motor_control::*, pid_controller::GeneralController, SensorData, SensorOffset};

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
    pub controller_ready: bool,
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

#[derive(Clone)]
pub struct JoystickControl {
    pub lift: u8,
    pub yaw: u8,
    pub pitch: u8,
    pub roll: u8,
    pub p: I16F16,
    pub p1: I16F16,
    pub p2: I16F16,
}

impl JoystickControl {
    pub fn new() -> Self {
        Self {
            lift: 90,
            yaw: 50,
            pitch: 50,
            roll: 50,
            p: I16F16::from_num(50),
            p1: I16F16::from_num(50),
            p2: I16F16::from_num(50),
        }
    }

    pub fn set_lift(&mut self, lift: u8) {
        self.lift = lift;
    }

    pub fn set_yaw(&mut self, yaw: u8) {
        self.yaw = yaw;
    }

    pub fn set_pitch(&mut self, pitch: u8) {
        self.pitch = pitch;
    }

    pub fn set_roll(&mut self, roll: u8) {
        self.roll = roll;
    }

    pub fn set_p(&mut self, p: I16F16) {
        self.p = p;
    }

    pub fn set_p1(&mut self, p1: I16F16) {
        self.p1 = p1;
    }

    pub fn set_p2(&mut self, p2: I16F16) {
        self.p2 = p2;
    }

    pub fn get_lift(&self) -> u8 {
        self.lift
    }

    pub fn get_yaw(&self) -> u8 {
        self.yaw
    }

    pub fn get_pitch(&self) -> u8 {
        self.pitch
    }

    pub fn get_roll(&self) -> u8 {
        self.roll
    }

    pub fn get_p(&self) -> I16F16 {
        self.p
    }

    pub fn get_p1(&self) -> I16F16 {
        self.p1
    }

    pub fn get_p2(&self) -> I16F16 {
        self.p2
    }

    // Check if lift, yaw, pitch and roll are all neutral on the controller.
    pub fn joystick_neutral_check(&mut self, state_machine: &mut StateMachine) {
        let flag = (self.get_lift() <= 90 && self.get_lift() >= 75)
            && (self.get_yaw() >= 40 && self.get_yaw() <= 60)
            && (self.get_pitch() >= 40 && self.get_pitch() <= 60)
            && (self.get_roll() >= 40 && self.get_roll() <= 60);
        if flag {
            state_machine.controller_ready = true;
        } else {
            state_machine.controller_ready = false;
        }
    }
}

// Implement methods for the state machine.
impl StateMachine {
    // Create a new state machine in the Safety state.
    pub fn new() -> StateMachine {
        StateMachine {
            state: State::Safety,
            operation_ready: false,
            controller_ready: false,
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
    pub fn transition(
        &mut self,
        next_state: State,
        joystick: &mut JoystickControl,
        general_controllers: &mut GeneralController,
        sensor_data_offset: &mut SensorOffset,
        sensor_data: &mut SensorData,
    ) -> (bool, u8) {
        joystick.joystick_neutral_check(self);
        if self.state() != next_state {
            match next_state {
                State::Safety => self.transition_safe(false, sensor_data_offset),
                State::Panic => {
                    self.transition_panic(general_controllers, sensor_data_offset, sensor_data)
                }
                State::Manual => self.transition_manual(),
                State::Calibrate => self.transition_calibrate(sensor_data_offset, sensor_data),
                State::Yaw | State::Full | State::Raw | State::Height | State::Wireless => {
                    self.transition_operation(next_state)
                } // | State::Manual => self.transition_operation(next_state, joystick),
            }
        } else if self.state() == next_state {
            match next_state {
                State::Manual => (true, 0b0000_0001),
                State::Yaw => (true, 0b0000_0001),
                State::Full => (true, 0b0000_0001),
                State::Raw => (true, 0b0000_0001),
                State::Height => (true, 0b0000_0001),
                State::Wireless => (true, 0b0000_0001),
                _ => (false, 0b0000_0001),
            }
        } else {
            (false, 0b0000_1000) // not defined
        }
    }

    // Safe mode should do nothing so everything is false.
    fn transition_safe(
        &mut self,
        through_panic: bool,
        sensor_data_offset: &mut SensorOffset,
    ) -> (bool, u8) {
        self.state = State::Safety;
        if sensor_data_offset.get_sample_count() > 0 {
            sensor_data_offset.calculate_offset();
        }
        self.permissions.controller = false;
        self.permissions.calibration = false;
        self.permissions.yaw_control = false;
        self.permissions.pitch_roll_control = false;
        self.permissions.height_control = false;
        self.permissions.wireless = false;
        self.permissions.sensors = false;
        // self.operation_ready = false; // TODO: this line should be commented out, and then after the calibration, it should be put to true (btw, it was originally false)
        if through_panic {
            (true, 0b0000_0010)
        } else {
            (true, 0b0011_1100)
        }
    }

    // Panic mode should also do nothing from the controller, might need the sensors (for now false).
    fn transition_panic(
        &mut self,
        general_controllers: &mut GeneralController,
        sensor_data_offset: &mut SensorOffset,
        sensor_data: &mut SensorData,
    ) -> (bool, u8) {
        self.state = State::Panic;
        Blue.on();
        self.permissions.controller = false;
        self.permissions.calibration = false;
        self.permissions.yaw_control = false;
        self.permissions.pitch_roll_control = false;
        self.permissions.height_control = false;
        self.permissions.wireless = false;
        self.permissions.sensors = false;
        // Reset the calibration flag if there was a panic.
        self.operation_ready = false;
        // Power down motors, due to panic.
        gradually_slow_down_motors();
        // Reset all the values in the general PID controllers
        general_controllers.yaw_control.reset_values();
        general_controllers.pitch_control.reset_values();
        general_controllers.roll_control.reset_values();
        general_controllers.height_control.reset_values();
        sensor_data.height_filter.reset();
        sensor_data_offset.reset_sample_count();
        sensor_data_offset.reset_offset();
        sensor_data.resume_non_offset();
        // Automatically go back to safe mode.
        self.transition_safe(true, sensor_data_offset)
    }

    // Manual mode should accept all controller movements, but not use any sensor data.
    fn transition_manual(&mut self) -> (bool, u8) {
        // Can only go into manual mode from safe mode.
        if self.controller_ready {
            if self.state == State::Safety {
                self.state = State::Manual;
                self.permissions.controller = true;
                self.permissions.calibration = false;
                self.permissions.yaw_control = false;
                self.permissions.pitch_roll_control = false;
                self.permissions.height_control = false;
                self.permissions.wireless = false;
                self.permissions.sensors = false;
                (true, 0b0011_1100)
            } else {
                (false, 0b0000_1111)
            }
        } else {
            self.state = State::Safety;
            (false, 0b0000_1111)
        }
    }

    // Calibration mode should only accept sensor data, no controller movements.
    fn transition_calibrate(
        &mut self,
        sensor_data_offset: &mut SensorOffset,
        sensor_data: &mut SensorData,
    ) -> (bool, u8) {
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
            if calibrate_mode(sensor_data_offset, sensor_data) {
                self.operation_ready = true;
            }
            // The code below is the original code
            (true, 0b0011_1100)
        } else {
            Red.on();
            (false, 0b0000_1111)
        }
    }

    // Checks whether calibration is done and then redirects to the required transition.
    // All operating modes use the sensors for control loops.
    fn transition_operation(&mut self, next_state: State) -> (bool, u8) {
        // Calibration flag check
        if self.operation_ready && self.state == Safety {
            if self.controller_ready {
                match next_state {
                    State::Manual => self.transition_manual(),
                    State::Yaw => self.transition_yaw(),
                    State::Full => self.transition_full(),
                    State::Raw => self.transition_raw(),
                    State::Height => self.transition_height(),
                    State::Wireless => self.transition_wireless(),
                    _ => (false, 0b0000_1000), // not defined, Match needs to be exhaustive, but this is unreachable due to match in transition()
                }
            } else {
                Red.on();
                self.state = State::Safety;
                (false, 0b0000_1111)
            }
        } else {
            (false, 0b0000_1111)
        }
    }

    // Yaw control mode should enable the yaw control loop.
    fn transition_yaw(&mut self) -> (bool, u8) {
        self.state = State::Yaw;
        self.permissions.controller = true;
        self.permissions.yaw_control = true;
        self.permissions.pitch_roll_control = false;
        self.permissions.height_control = false;
        self.permissions.wireless = false;
        self.permissions.sensors = true;
        (true, 0b0011_1100)
    }

    // Full control mode should enable the yaw control loop and the pitch-roll control loop.
    fn transition_full(&mut self) -> (bool, u8) {
        self.state = State::Full;
        self.permissions.controller = true;
        self.permissions.calibration = false;
        self.permissions.yaw_control = true;
        self.permissions.pitch_roll_control = true;
        self.permissions.height_control = false;
        self.permissions.wireless = false;
        self.permissions.sensors = true;
        (true, 0b0011_1100)
    }

    // Raw sensor reading mode should add the filters, to the other control loops.
    fn transition_raw(&mut self) -> (bool, u8) {
        self.state = State::Raw;
        self.permissions.controller = true;
        self.permissions.calibration = false;
        self.permissions.yaw_control = true;
        self.permissions.pitch_roll_control = true;
        self.permissions.height_control = false;
        self.permissions.wireless = false;
        self.permissions.sensors = true;
        (true, 0b0011_1100)
    }

    // On top of the two control loops and the filters, a height control loop is added.
    fn transition_height(&mut self) -> (bool, u8) {
        self.state = State::Height;
        self.permissions.controller = true;
        self.permissions.calibration = false;
        self.permissions.yaw_control = true;
        self.permissions.pitch_roll_control = true;
        self.permissions.height_control = true;
        self.permissions.wireless = false;
        self.permissions.sensors = true;
        (true, 0b0011_1100)
    }

    // After all control loops and filters work, wireless mode can be used to fly the drone without the USB cable.
    // Requires high stabilization of drone, since wireless bandwidth is lower.
    // Only use this mode after all previous modes have been implemented.
    fn transition_wireless(&mut self) -> (bool, u8) {
        self.state = State::Wireless;
        self.permissions.controller = true;
        self.permissions.calibration = false;
        self.permissions.yaw_control = true;
        self.permissions.pitch_roll_control = true;
        self.permissions.height_control = true;
        self.permissions.wireless = true;
        self.permissions.sensors = true;
        (true, 0b0011_1100)
    }
}

pub fn execute_state_function(
    current_state: &State,
    command: &JoystickControl,
    general_controllers: &mut GeneralController,
    sensor_data: &mut SensorData,
    sensor_data_offset: &SensorOffset
) {
    match current_state {
        State::Safety => {
            safety_mode();
        }
        State::Manual => {
            manual_mode(command);
        }
        // State::Calibrate => {
        //     calibrate_mode(sensor_data_offset);
        // }
        State::Yaw => {
            yaw_mode(command, general_controllers, sensor_data);
        }
        State::Full => {
            full_mode(command, general_controllers, sensor_data);
        }
        State::Raw => {
            raw_mode(command, general_controllers, sensor_data, sensor_data_offset);
        }
        State::Height => {
            height_mode(command, general_controllers, sensor_data);
        }
        State::Wireless => {
            wireless_mode();
        }
        _ => {
            // mode such as Panic is dealt during transition
        }
    }
}

#[allow(unused_variables)]
fn safety_mode() {
    // TODO: Nothing to implement in safety mode
}

fn manual_mode(command: &JoystickControl) {
    let lift: i16 = map_lift_command_manual(command.get_lift());
    let yaw: i16 = map_yaw_command_manual(command.get_yaw());
    let pitch: i16 = map_pitch_command_manual(command.get_pitch());
    let roll: i16 = map_roll_command_manual(command.get_roll());
    set_motor_speeds_manual(lift, yaw, pitch, roll);
}

fn calibrate_mode(sensor_data_offset: &mut SensorOffset, sensor_data: &mut SensorData) -> bool {
    if sensor_data_offset.get_sample_count() == 0 {
        sensor_data.height_filter.reset();
        sensor_data.resume_non_offset();
        sensor_data_offset.reset_offset();
    }
    sensor_data_offset.update_sample_count();
    sensor_data_offset.update_yaw_offset(sensor_data.get_ypr().yaw);
    sensor_data_offset.update_pitch_offset(sensor_data.get_ypr().pitch);
    sensor_data_offset.update_roll_offset(sensor_data.get_ypr().roll);
    sensor_data_offset.update_lift_offset(sensor_data.get_pres());
    sensor_data_offset.update_gyro_offset(sensor_data.get_gyro_data());
    sensor_data_offset.update_acc_offset(sensor_data.get_accel_data());
    true
}

fn yaw_mode(
    command: &JoystickControl,
    general_controllers: &mut GeneralController,
    sensor_data: &SensorData,
) {
    let lift: i16 = map_lift_command_control(command.get_lift()); // this should be the value that keeps the drone in the air stable
    let yaw: i16 = map_yaw_command_manual(command.get_yaw());
    let yaw_rate: I16F16 = map_yaw_command(command.get_yaw());
    let pitch: i16 = map_pitch_command_manual(command.get_pitch());
    let roll: i16 = map_roll_command_manual(command.get_roll());
    general_controllers
        .yaw_control
        .go_through_process(yaw_rate, sensor_data);
    let yaw_compensate: i16 =
        determine_yaw_compensate(yaw_rate, general_controllers.yaw_control.new_yaw);
    set_motor_speeds_yaw(lift, yaw, pitch, roll, yaw_compensate);
}

fn full_mode(
    command: &JoystickControl,
    general_controllers: &mut GeneralController,
    sensor_data: &SensorData,
) {
    // directly map the lift to the motor speeds
    let lift: i16 = map_lift_command_control(command.get_lift()); // this should be the value that keeps the drone in the air stable
    let yaw: i16 = map_yaw_command_manual(command.get_yaw());
    let yaw_rate: I16F16 = map_yaw_command(command.get_yaw());
    let pitch: i16 = map_pitch_command_manual(command.get_pitch());
    let roll: i16 = map_roll_command_manual(command.get_roll());
    let pitch_angle: I16F16 = map_pitch_command(command.get_pitch());
    let roll_angle: I16F16 = map_roll_command(command.get_roll());

    general_controllers
        .yaw_control
        .go_through_process(yaw_rate, sensor_data);
    let yaw_compensate: i16 =
        determine_yaw_compensate(yaw_rate, general_controllers.yaw_control.new_yaw);
    // let yaw_compensate: i16 = 0;

    general_controllers
        .pitch_control
        .go_through_process(pitch_angle, sensor_data);
    let pitch_compensate: i16 =
        determine_pitch_compensate(pitch_angle, general_controllers.pitch_control.new_pitch);

    general_controllers
        .roll_control
        .go_through_process(roll_angle, sensor_data);
    let roll_compensate: i16 =
        determine_roll_compensate(roll_angle, general_controllers.roll_control.new_roll);
    // let roll_compensate: i16 = 0;

    set_motor_speeds_full(
        lift,
        yaw,
        pitch,
        roll,
        yaw_compensate,
        pitch_compensate,
        roll_compensate,
    );
}

#[allow(unused_variables)]
fn raw_mode(command: &JoystickControl, general_controllers: &mut GeneralController, sensor_data: &mut SensorData, sensor_data_offset: &SensorOffset) {
    let offseted_gyro = [I16F16::from_num(sensor_data.get_gyro_data()[0].saturating_sub(sensor_data_offset.gyro_offset[0] as i16)),
    I16F16::from_num(sensor_data.get_gyro_data()[1].saturating_sub(sensor_data_offset.gyro_offset[1] as i16)),
    I16F16::from_num(sensor_data.get_gyro_data()[2].saturating_sub(sensor_data_offset.gyro_offset[2] as i16))];

    let offseted_acc = [I16F16::from_num(sensor_data.get_accel_data()[0].saturating_sub(sensor_data_offset.acc_offset[0] as i16)),
    I16F16::from_num(sensor_data.get_accel_data()[1].saturating_sub(sensor_data_offset.acc_offset[1] as i16)),
    I16F16::from_num(sensor_data.get_accel_data()[2].saturating_sub(sensor_data_offset.acc_offset[2] as i16))];

    let (acc, gyro) = general_controllers.raw_control.low_pass_filter.low_pass_one(offseted_gyro, offseted_acc);
    let kf_ypr = general_controllers.raw_control.kalman_filter.get_kalman_data(acc, gyro, &sensor_data_offset);
    sensor_data.update_ypr_filtered(kf_ypr);
}

#[allow(unused_variables)]
fn height_mode(
    command: &JoystickControl,
    general_controllers: &mut GeneralController,
    sensor_data: &SensorData,
) {
    let lift: i16 = map_lift_command_height(command.get_lift()); // this should be the value that keeps the drone in the air stable
    let target_lift: I16F16 = map_lift_command(command.get_lift());
    let yaw: i16 = map_yaw_command_manual(command.get_yaw());
    let pitch: i16 = map_pitch_command_manual(command.get_pitch());
    let roll: i16 = map_roll_command_manual(command.get_roll());
    general_controllers
        .height_control
        .go_through_process(target_lift, sensor_data);
    let lift_compensate: i16 =
        determine_lift_compensate(target_lift, general_controllers.height_control.new_throttle);
    set_motor_speeds_lift(lift, yaw, pitch, roll, lift_compensate);
}

#[allow(unused_variables)]
fn wireless_mode() {
    // TODO
}
