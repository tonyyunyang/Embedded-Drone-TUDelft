use tudelft_quadrupel::{
    fixed::types::I16F16,
    motor::{get_motors, set_motors},
    time::delay_us_assembly,
};

// This function sets motor speeds manually for a drone or similar device.
// It takes four input parameters: lift, yaw, pitch, and roll. Each parameter is an i16 value.
pub fn set_motor_speeds_manual(lift: i16, yaw: i16, pitch: i16, roll: i16) {
    // If the lift value is 200, it indicates that the motors should be set to a safe state (all motors off).
    if lift == 200 {
        // Set all motor speeds to 0 (safe state)
        let ae1_safe: u16 = 0;
        let ae2_safe: u16 = 0;
        let ae3_safe: u16 = 0;
        let ae4_safe: u16 = 0;

        // Call the set_motors function with the safe motor speeds array
        set_motors([ae1_safe, ae2_safe, ae3_safe, ae4_safe]);
    } else {
        // Calculate the individual motor speeds based on the input parameters (lift, yaw, pitch, and roll)
        let ae1: u16 = (lift - pitch - yaw) as u16;
        let ae2: u16 = (lift - roll + yaw) as u16;
        let ae3: u16 = (lift + pitch - yaw) as u16;
        let ae4: u16 = (lift + roll + yaw) as u16;

        // Call the set_motors function with the calculated motor speeds array
        set_motors([ae1, ae2, ae3, ae4]);
    }
}

/// Maps a lift command value received from a joystick
/// to a corresponding lift value used by a drone
///
/// The function assumes that the lift values from the joystick range from 10 to 90.
///
/// # Arguments
///
/// * `command` - A `u8` value representing the lift command received from the joystick.
///
/// # Returns
///
/// * An `i16` value representing the corresponding lift value for the drone
///
/// # Notes
///
/// * If the `command` value is either 10 or an invalid value, the function will return 360,
///   setting the motor to 0 in both situations.
pub fn map_lift_command_manual(command: u8) -> i16 {
    // the mapping might be wrong, for now, I will assume the lift from the joystick starts at -1, and goes to 1
    if command == 90 {
        200
    } else if command == 85 {
        210
    } else if command == 80 {
        220
    } else if command == 75 {
        230
    } else if command == 70 {
        240
    } else if command == 65 {
        250
    } else if command == 60 {
        260
    } else if command == 55 {
        270
    } else if command == 50 {
        280
    } else if command == 45 {
        290
    } else if command == 40 {
        300
    } else if command == 35 {
        310
    } else if command == 30 {
        320
    } else if command == 25 {
        330
    } else if command == 20 {
        340
    } else if command == 15 {
        350
    } else {
        // either 10? Or an invalid value, we set motor to 0 under both situations
        360
    }
}

pub fn map_yaw_command_manual(command: u8) -> i16 {
    // the mapping might be wrong, for now, I will assume the initial value is 0, twisting left is negative, turning right is positive
    if command == 90 {
        70
    } else if command == 85 {
        60
    } else if command == 80 {
        50
    } else if command == 75 {
        40
    } else if command == 70 {
        30
    } else if command == 65 {
        20
    } else if command == 60 {
        10
    } else if command == 55 {
        5
    } else if command == 50 {
        0
    } else if command == 45 {
        -5
    } else if command == 40 {
        -10
    } else if command == 35 {
        -20
    } else if command == 30 {
        -30
    } else if command == 25 {
        -40
    } else if command == 20 {
        -50
    } else if command == 15 {
        -60
    } else if command == 10 {
        -70
    } else {
        // not a valid command, we set motor to 0
        0
    }
}

/// Maps a pitch command value to an angle.
///
/// # Arguments
///
/// * `command` - A u8 representing the input command value
///
/// # Returns
///
/// * An I16F16 representing the mapped angle in radians
pub fn map_pitch_command_manual(command: u8) -> i16 {
    // the mapping might be wrong, for now, I will assume the initial value is 0, pushing forward is positive, pushing backward is negative
    if command == 90 {
        40
    } else if command == 85 {
        35
    } else if command == 80 {
        30
    } else if command == 75 {
        25
    } else if command == 70 {
        20
    } else if command == 65 {
        15
    } else if command == 60 {
        10
    } else if command == 55 {
        5
    } else if command == 50 {
        0
    } else if command == 45 {
        -5
    } else if command == 40 {
        -10
    } else if command == 35 {
        -15
    } else if command == 30 {
        -20
    } else if command == 25 {
        -25
    } else if command == 20 {
        -30
    } else if command == 15 {
        -35
    } else if command == 10 {
        -40
    } else {
        // not a valid command, we set motor to 0
        0
    }
}

/// Maps a roll command value to an angle.
///
/// # Arguments
///
/// * `command` - A u8 representing the input command value
///
/// # Returns
///
/// * An I16F16 representing the mapped angle in radians
pub fn map_roll_command_manual(command: u8) -> i16 {
    // the mapping might be wrong, for now, I will assume the initial value is 0, pushing left is negative, pushing right is positive
    if command == 90 {
        40
    } else if command == 85 {
        35
    } else if command == 80 {
        30
    } else if command == 75 {
        25
    } else if command == 70 {
        20
    } else if command == 65 {
        15
    } else if command == 60 {
        10
    } else if command == 55 {
        5
    } else if command == 50 {
        0
    } else if command == 45 {
        -5
    } else if command == 40 {
        -10
    } else if command == 35 {
        -15
    } else if command == 30 {
        -20
    } else if command == 25 {
        -25
    } else if command == 20 {
        -30
    } else if command == 15 {
        -35
    } else if command == 10 {
        -40
    } else {
        //not a valid command, we set motor to 0
        0
    }
}
/// Maps a lift command value to a motor speed.
///
/// # Arguments
///
/// * `command` - A u8 representing the input command value
///
/// # Returns
///
/// * A u16 representing the mapped motor speed
pub fn map_lift_command(command: u8) -> u16 {
    // the mapping might be wrong, for now, I will assume the lift from the joystick starts at -1, and goes to 1
    if command == 90 {
        200
    } else if command == 85 {
        210
    } else if command == 80 {
        220
    } else if command == 75 {
        230
    } else if command == 70 {
        240
    } else if command == 65 {
        250
    } else if command == 60 {
        260
    } else if command == 55 {
        270
    } else if command == 50 {
        280
    } else if command == 45 {
        290
    } else if command == 40 {
        300
    } else if command == 35 {
        310
    } else if command == 30 {
        320
    } else if command == 25 {
        330
    } else if command == 20 {
        340
    } else if command == 15 {
        350
    } else {
        // either 10? Or an invalid value, we set motor to 0 under both situations
        360
    }
}

/// Maps a yaw command value to an angular velocity.
///
/// # Arguments
///
/// * `command` - A u8 representing the input command value
///
/// # Returns
///
/// * An i16 representing the mapped angular velocity
#[allow(clippy::approx_constant)]
pub fn map_yaw_command(command: u8) -> I16F16 {
    // the mapping might be wrong, for now, I will assume the initial value is 0, twisting left is negative, turning right is positive
    // this is the angular velocity below
    if command == 90 {
        I16F16::from_num(3.1415926) // 180 degrees in radians
    } else if command == 85 {
        I16F16::from_num(2.7488935) // 157.5 degrees in radians
    } else if command == 80 {
        I16F16::from_num(2.3561945) // 135 degrees in radians
    } else if command == 75 {
        I16F16::from_num(1.9634954) // 112.5 degrees in radians
    } else if command == 70 {
        I16F16::from_num(1.5707963) // 90 degrees in radians
    } else if command == 65 {
        I16F16::from_num(1.1780972) // 67.5 degrees in radians
    } else if command == 60 {
        I16F16::from_num(0.7853981) // 45 degrees in radians
    } else if command == 55 {
        I16F16::from_num(0.392699) // 22.5 degrees in radians
    } else if command == 50 {
        I16F16::from_num(0.0) // 0 degrees in radians
    } else if command == 45 {
        I16F16::from_num(-0.392699) // 22.5 degrees in radians
    } else if command == 40 {
        I16F16::from_num(-0.7853981) // 45 degrees in radians
    } else if command == 35 {
        I16F16::from_num(-1.1780972) // 67.5 degrees in radians
    } else if command == 30 {
        I16F16::from_num(-1.5707963) // 90 degrees in radians
    } else if command == 25 {
        I16F16::from_num(-1.9634954) // 112.5 degrees in radians
    } else if command == 20 {
        I16F16::from_num(-2.3561945) // 135 degrees in radians
    } else if command == 15 {
        I16F16::from_num(-2.7488935) // 157.5 degrees in radians
    } else if command == 10 {
        I16F16::from_num(-3.1415926) // 180 degrees in radians
    } else {
        // not a valid command, we set motor to 0
        I16F16::from_num(0.0) // 0 degrees in radians
    }
}

#[allow(clippy::approx_constant)]
pub fn map_pitch_command(command: u8) -> I16F16 {
    // the mapping might be wrong, for now, I will assume the initial value is 0, pushing forward is positive, pushing backward is negative
    // this is the pitch angle below, this is not the angular velocity
    if command == 90 {
        I16F16::from_num(0.5235987) // 30 degrees in radians
    } else if command == 85 {
        I16F16::from_num(0.4581489) // 26.25 degrees in radians
    } else if command == 80 {
        I16F16::from_num(0.3926991) // 22.5 degrees in radians
    } else if command == 75 {
        I16F16::from_num(0.3272492) // 18.75 degrees in radians
    } else if command == 70 {
        I16F16::from_num(0.2617993) // 15 degrees in radians
    } else if command == 65 {
        I16F16::from_num(0.1963495) // 11.25 degrees in radians
    } else if command == 60 {
        I16F16::from_num(0.1308996) // 7.5 degrees in radians
    } else if command == 55 {
        I16F16::from_num(0.0654498) // 3.75 degrees in radians
    } else if command == 50 {
        I16F16::from_num(0.0) // 0 degrees in radians
    } else if command == 45 {
        I16F16::from_num(-0.0654498) // 3.75 degrees in radians
    } else if command == 40 {
        I16F16::from_num(-0.1308996) // 7.5 degrees in radians
    } else if command == 35 {
        I16F16::from_num(-0.1963495) // 11.25 degrees in radians
    } else if command == 30 {
        I16F16::from_num(-0.2617993) // 15 degrees in radians
    } else if command == 25 {
        I16F16::from_num(-0.3272492) // 18.75 degrees in radians
    } else if command == 20 {
        I16F16::from_num(-0.3926991) // 22.5 degrees in radians
    } else if command == 15 {
        I16F16::from_num(-0.4581489) // 26.25 degrees in radians
    } else if command == 10 {
        I16F16::from_num(-0.5235987) // 30 degrees in radians
    } else {
        // not a valid command, we set motor to 0
        I16F16::from_num(0.0) // 0 degrees in radians
    }
}

#[allow(clippy::approx_constant)]
pub fn map_roll_command(command: u8) -> I16F16 {
    // the mapping might be wrong, for now, I will assume the initial value is 0, pushing forward is positive, pushing backward is negative
    // this is the roll angle below, this is not the angular velocity
    if command == 90 {
        I16F16::from_num(0.5235987) // 30 degrees in radians
    } else if command == 85 {
        I16F16::from_num(0.4581489) // 26.25 degrees in radians
    } else if command == 80 {
        I16F16::from_num(0.3926991) // 22.5 degrees in radians
    } else if command == 75 {
        I16F16::from_num(0.3272492) // 18.75 degrees in radians
    } else if command == 70 {
        I16F16::from_num(0.2617993) // 15 degrees in radians
    } else if command == 65 {
        I16F16::from_num(0.1963495) // 11.25 degrees in radians
    } else if command == 60 {
        I16F16::from_num(0.1308996) // 7.5 degrees in radians
    } else if command == 55 {
        I16F16::from_num(0.0654498) // 3.75 degrees in radians
    } else if command == 50 {
        I16F16::from_num(0.0) // 0 degrees in radians
    } else if command == 45 {
        I16F16::from_num(-0.0654498) // 3.75 degrees in radians
    } else if command == 40 {
        I16F16::from_num(-0.1308996) // 7.5 degrees in radians
    } else if command == 35 {
        I16F16::from_num(-0.1963495) // 11.25 degrees in radians
    } else if command == 30 {
        I16F16::from_num(-0.2617993) // 15 degrees in radians
    } else if command == 25 {
        I16F16::from_num(-0.3272492) // 18.75 degrees in radians
    } else if command == 20 {
        I16F16::from_num(-0.3926991) // 22.5 degrees in radians
    } else if command == 15 {
        I16F16::from_num(-0.4581489) // 26.25 degrees in radians
    } else if command == 10 {
        I16F16::from_num(-0.5235987) // 30 degrees in radians
    } else {
        // not a valid command, we set motor to 0
        I16F16::from_num(0.0) // 0 degrees in radians
    }
}

/// Gradually slows down motors until they stop.
///
/// This function will get the current motor speeds, and then gradually decrease them
/// by 1 unit every 2500 microseconds until all motors have stopped.
pub fn gradually_slow_down_motors() {
    let motors_speed = get_motors();
    let mut motor0 = motors_speed[0];
    let mut motor1 = motors_speed[1];
    let mut motor2 = motors_speed[2];
    let mut motor3 = motors_speed[3];

    while (motor0 > 0) || (motor1 > 0) || (motor2 > 0) || (motor3 > 0) {
        motor0 = motor0.saturating_sub(1);
        motor1 = motor1.saturating_sub(1);
        motor2 = motor2.saturating_sub(1);
        motor3 = motor3.saturating_sub(1);

        delay_us_assembly(2500);

        set_motors([motor0, motor1, motor2, motor3]);
    }
}
