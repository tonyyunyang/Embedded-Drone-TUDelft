use tudelft_quadrupel::{fixed::types::I16F16, motor::set_motors};

pub fn set_motor_speeds_manual(lift: u16, yaw: u16, pitch: u16, roll: u16) {
    if lift > roll + yaw {
        let ae1: u16 = lift + yaw - pitch;
        let ae2: u16 = lift + roll - yaw;
        let ae3: u16 = lift + yaw + pitch;
        let ae4: u16 = lift - roll - yaw;
        set_motors([ae1, ae2, ae3, ae4]);
    } else {
        set_motors([lift; 4]);
    }
}

pub fn map_lift_command_manual(command: u8) -> u16 {
    // the mapping might be wrong, for now, I will assume the lift from the joystick starts at -1, and goes to 1
    if command == 90 {
        360
    } else if command == 85 {
        350
    } else if command == 80 {
        340
    } else if command == 75 {
        330
    } else if command == 70 {
        320
    } else if command == 65 {
        310
    } else if command == 60 {
        300
    } else if command == 55 {
        290
    } else if command == 50 {
        280
    } else if command == 45 {
        270
    } else if command == 40 {
        260
    } else if command == 35 {
        250
    } else if command == 30 {
        240
    } else if command == 25 {
        230
    } else if command == 20 {
        220
    } else if command == 15 {
        210
    } else {
        // either 10? Or an invalid value, we set motor to 0 under both situations
        200
    }
}

pub fn map_yaw_command_manual(command: u8) -> u16 {
    // the mapping might be wrong, for now, I will assume the initial value is 0, twisting left is negative, turning right is positive
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
        5
    } else if command == 40 {
        10
    } else if command == 35 {
        15
    } else if command == 30 {
        20
    } else if command == 25 {
        25
    } else if command == 20 {
        30
    } else if command == 15 {
        35
    } else if command == 10 {
        40
    } else {
        // not a valid command, we set motor to 0
        0
    }
}

pub fn map_pitch_command_manual(command: u8) -> u16 {
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
        5
    } else if command == 40 {
        10
    } else if command == 35 {
        15
    } else if command == 30 {
        20
    } else if command == 25 {
        25
    } else if command == 20 {
        30
    } else if command == 15 {
        35
    } else if command == 10 {
        40
    } else {
        // not a valid command, we set motor to 0
        0
    }
}

pub fn map_roll_command_manual(command: u8) -> u16 {
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
        5
    } else if command == 40 {
        10
    } else if command == 35 {
        15
    } else if command == 30 {
        20
    } else if command == 25 {
        25
    } else if command == 20 {
        30
    } else if command == 15 {
        35
    } else if command == 10 {
        40
    } else {
        //not a valid command, we set motor to 0
        0
    }
}

pub fn map_lift_command(command: u8) -> u16 {
    // the mapping might be wrong, for now, I will assume the lift from the joystick starts at -1, and goes to 1
    if command == 90 {
        360
    } else if command == 85 {
        350
    } else if command == 80 {
        340
    } else if command == 75 {
        330
    } else if command == 70 {
        320
    } else if command == 65 {
        310
    } else if command == 60 {
        300
    } else if command == 55 {
        290
    } else if command == 50 {
        280
    } else if command == 45 {
        270
    } else if command == 40 {
        260
    } else if command == 35 {
        250
    } else if command == 30 {
        240
    } else if command == 25 {
        230
    } else if command == 20 {
        220
    } else if command == 15 {
        210
    } else {
        // either 10? Or an invalid value, we set motor to 0 under both situations
        200
    }
}

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
