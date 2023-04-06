use tudelft_quadrupel::{
    fixed::types::I16F16,
    motor::{get_motors, set_motor_max, set_motors},
    time::delay_us_assembly,
};

pub fn set_motor_speeds_manual(lift: i16, yaw: i16, pitch: i16, roll: i16) {
    if lift == 200 {
        let ae1_safe: u16 = 0;
        let ae2_safe: u16 = 0;
        let ae3_safe: u16 = 0;
        let ae4_safe: u16 = 0;
        set_motors([ae1_safe, ae2_safe, ae3_safe, ae4_safe]);
    } else {
        let ae1: u16 = (lift - pitch - yaw) as u16;
        let ae2: u16 = (lift - roll + yaw) as u16;
        let ae3: u16 = (lift + pitch - yaw) as u16;
        let ae4: u16 = (lift + roll + yaw) as u16;
        set_motors([ae1, ae2, ae3, ae4]);
    }
}

#[allow(clippy::approx_constant)]
pub fn determine_yaw_compensate(old: I16F16, new: I16F16) -> i16 {
    let difference: I16F16 = new - old;
    let percentage: I16F16 = difference / I16F16::from_num(3.1415926);
    let result: i16 = I16F16::to_num(percentage * 80);
    let mut result_max = get_motors();
    result_max.sort();
    let max = (result_max[3] as i16) / 4;
    if result > max {
        max
    } else if result < -max {
        -max
    } else {
        result
    }
}

pub fn determine_pitch_compensate(old: I16F16, new: I16F16) -> i16 {
    let difference: I16F16 = new - old;
    let percentage: I16F16 = difference / I16F16::from_num(0.27925268 * 40.0);
    // the magic factor below might need to be adjusted
    let result: i16 = I16F16::to_num(percentage * I16F16::from_num(10));
    let mut result_max = get_motors();
    result_max.sort();
    let max = (result_max[3] as i16) / 10;
    if result > max {
        max
    } else if result < -max {
        -max
    } else {
        result
    }
}

pub fn determine_roll_compensate(old: I16F16, new: I16F16) -> i16 {
    let difference: I16F16 = new - old;
    let percentage: I16F16 = difference / I16F16::from_num(0.27925268 * 40.0);
    // the magic factor below might need to be adjusted
    let result: i16 = I16F16::to_num(percentage * I16F16::from_num(10));
    let mut result_max = get_motors();
    result_max.sort();
    let max = (result_max[3] as i16) / 10;
    if result > max {
        max
    } else if result < -max {
        -max
    } else {
        result
    }
}

pub fn determine_lift_compensate(old: I16F16, new: I16F16) -> i16 {
    let difference: I16F16 = new - old;
    let percentage: I16F16 = difference / I16F16::from_num(5);
    // the magic factor below might need to be adjusted
    let result: i16 = I16F16::to_num(percentage * I16F16::from_num(15.0));
    let mut result_max = get_motors();
    result_max.sort();
    let max = (result_max[3] as i16) / 5;
    if result > max {
        max
    } else if result < -max {
        -max
    } else {
        result
    }
}

pub fn set_motor_speeds_yaw(lift: i16, yaw: i16, pitch: i16, roll: i16, yaw_compensate: i16) {
    if lift == 200 {
        let ae1_safe: u16 = 0;
        let ae2_safe: u16 = 0;
        let ae3_safe: u16 = 0;
        let ae4_safe: u16 = 0;
        set_motors([ae1_safe, ae2_safe, ae3_safe, ae4_safe]);
    } else {
        let mut ae1: u16 = (lift - pitch - yaw - yaw_compensate) as u16;
        let mut ae2: u16 = (lift - roll + yaw + yaw_compensate) as u16;
        let mut ae3: u16 = (lift + pitch - yaw - yaw_compensate) as u16;
        let mut ae4: u16 = (lift + roll + yaw + yaw_compensate) as u16;

        let motor_minimum = 220;
        if ae1 < motor_minimum {
            ae1 = motor_minimum;
        }
        if ae2 < motor_minimum {
            ae2 = motor_minimum;
        }
        if ae3 < motor_minimum {
            ae3 = motor_minimum;
        }
        if ae4 < motor_minimum {
            ae4 = motor_minimum;
        }
        set_motor_max(600);
        set_motors([ae1, ae2, ae3, ae4]);
    }
}

pub fn set_motor_speeds_full(
    lift: i16,
    yaw: i16,
    pitch: i16,
    roll: i16,
    yaw_compensate: i16,
    pitch_compensate: i16,
    roll_compensate: i16,
) {
    if lift == 200 {
        let ae1_safe: u16 = 0;
        let ae2_safe: u16 = 0;
        let ae3_safe: u16 = 0;
        let ae4_safe: u16 = 0;
        set_motors([ae1_safe, ae2_safe, ae3_safe, ae4_safe]);
    } else {
        let mut ae1: u16 = (lift - pitch + pitch_compensate - yaw - yaw_compensate) as u16;
        let mut ae2: u16 = (lift - roll - roll_compensate + yaw + yaw_compensate) as u16;
        let mut ae3: u16 = (lift + pitch - pitch_compensate - yaw - yaw_compensate) as u16;
        let mut ae4: u16 = (lift + roll + roll_compensate + yaw + yaw_compensate) as u16;

        let motor_minimum = 220;
        if ae1 < motor_minimum {
            ae1 = motor_minimum;
        }
        if ae2 < motor_minimum {
            ae2 = motor_minimum;
        }
        if ae3 < motor_minimum {
            ae3 = motor_minimum;
        }
        if ae4 < motor_minimum {
            ae4 = motor_minimum;
        }
        set_motor_max(1000);
        set_motors([ae1, ae2, ae3, ae4]);
    }
}

pub fn set_motor_speeds_lift(lift: i16, yaw: i16, pitch: i16, roll: i16, lift_compensate: i16) {
    if lift == 200 {
        let ae1_safe: u16 = 0;
        let ae2_safe: u16 = 0;
        let ae3_safe: u16 = 0;
        let ae4_safe: u16 = 0;
        set_motors([ae1_safe, ae2_safe, ae3_safe, ae4_safe]);
    } else {
        let lift_temp = lift - lift_compensate;

        let mut ae1: u16 = (lift_temp - pitch - yaw) as u16;
        let mut ae2: u16 = (lift_temp - roll + yaw) as u16;
        let mut ae3: u16 = (lift_temp + pitch - yaw) as u16;
        let mut ae4: u16 = (lift_temp + roll + yaw) as u16;

        let motor_minimum = 220;
        if ae1 < motor_minimum {
            ae1 = motor_minimum;
        }
        if ae2 < motor_minimum {
            ae2 = motor_minimum;
        }
        if ae3 < motor_minimum {
            ae3 = motor_minimum;
        }
        if ae4 < motor_minimum {
            ae4 = motor_minimum;
        }
        set_motor_max(1000);
        set_motors([ae1, ae2, ae3, ae4]);
    }
}

pub fn map_lift_command_manual(command: u8) -> i16 {
    // the mapping might be wrong, for now, I will assume the lift from the joystick starts at -1, and goes to 1
    if command == 90 {
        200
    } else if command == 85 {
        205
    } else if command == 80 {
        210
    } else if command == 75 {
        220
    } else if command == 70 {
        230
    } else if command == 65 {
        240
    } else if command == 60 {
        250
    } else if command == 55 {
        260
    } else if command == 50 {
        270
    } else if command == 45 {
        280
    } else if command == 40 {
        290
    } else if command == 35 {
        300
    } else if command == 30 {
        310
    } else if command == 25 {
        320
    } else if command == 20 {
        330
    } else if command == 15 {
        340
    } else {
        // either 10? Or an invalid value, we set motor to 0 under both situations
        350
    }
}

pub fn map_lift_command_control(command: u8) -> i16 {
    // the mapping might be wrong, for now, I will assume the lift from the joystick starts at -1, and goes to 1
    if command == 90 {
        200
    } else if command == 85 {
        230
    } else if command == 80 {
        260
    } else if command == 75 {
        290
    } else if command == 70 {
        320
    } else if command == 65 {
        350
    } else if command == 60 {
        380
    } else if command == 55 {
        410
    } else if command == 50 {
        440
    } else if command == 45 {
        470
    } else if command == 40 {
        500
    } else if command == 35 {
        520
    } else if command == 30 {
        540
    } else if command == 25 {
        560
    } else if command == 20 {
        580
    } else if command == 15 {
        590
    } else {
        // either 10? Or an invalid value, we set motor to 0 under both situations
        600
    }
}

pub fn map_lift_command_height(command: u8) -> i16 {
    // the mapping might be wrong, for now, I will assume the lift from the joystick starts at -1, and goes to 1
    if command == 90 {
        200
    } else if command == 85 {
        240
    } else if command == 80 {
        280
    } else if command == 75 {
        320
    } else if command == 70 {
        360
    } else if command == 65 {
        400
    } else if command == 60 {
        440
    } else if command == 55 {
        480
    } else if command == 50 {
        520
    } else if command == 45 {
        560
    } else if command == 40 {
        600
    } else if command == 35 {
        640
    } else if command == 30 {
        680
    } else if command == 25 {
        720
    } else if command == 20 {
        760
    } else if command == 15 {
        800
    } else {
        // either 10? Or an invalid value, we set motor to 0 under both situations
        840
    }
}

pub fn map_yaw_command_manual(command: u8) -> i16 {
    // the mapping might be wrong, for now, I will assume the initial value is 0, twisting left is negative, turning right is positive
    if command == 90 {
        80
    } else if command == 85 {
        70
    } else if command == 80 {
        60
    } else if command == 75 {
        50
    } else if command == 70 {
        40
    } else if command == 65 {
        30
    } else if command == 60 {
        20
    } else if command == 55 {
        10
    } else if command == 50 {
        0
    } else if command == 45 {
        -10
    } else if command == 40 {
        -20
    } else if command == 35 {
        -30
    } else if command == 30 {
        -40
    } else if command == 25 {
        -50
    } else if command == 20 {
        -60
    } else if command == 15 {
        -70
    } else if command == 10 {
        -80
    } else {
        // not a valid command, we set motor to 0
        0
    }
}

pub fn map_pitch_command_manual(command: u8) -> i16 {
    // the mapping might be wrong, for now, I will assume the initial value is 0, pushing forward is positive, pushing backward is negative
    if command == 90 {
        24
    } else if command == 85 {
        21
    } else if command == 80 {
        18
    } else if command == 75 {
        15
    } else if command == 70 {
        12
    } else if command == 65 {
        9
    } else if command == 60 {
        6
    } else if command == 55 {
        3
    } else if command == 50 {
        0
    } else if command == 45 {
        -3
    } else if command == 40 {
        -6
    } else if command == 35 {
        -9
    } else if command == 30 {
        -12
    } else if command == 25 {
        -15
    } else if command == 20 {
        -18
    } else if command == 15 {
        -21
    } else if command == 10 {
        -24
    } else {
        // not a valid command, we set motor to 0
        0
    }
}

pub fn map_roll_command_manual(command: u8) -> i16 {
    // the mapping might be wrong, for now, I will assume the initial value is 0, pushing forward is positive, pushing backward is negative
    if command == 90 {
        24
    } else if command == 85 {
        21
    } else if command == 80 {
        18
    } else if command == 75 {
        15
    } else if command == 70 {
        12
    } else if command == 65 {
        9
    } else if command == 60 {
        6
    } else if command == 55 {
        3
    } else if command == 50 {
        0
    } else if command == 45 {
        -3
    } else if command == 40 {
        -6
    } else if command == 35 {
        -9
    } else if command == 30 {
        -12
    } else if command == 25 {
        -15
    } else if command == 20 {
        -18
    } else if command == 15 {
        -21
    } else if command == 10 {
        -24
    } else {
        // not a valid command, we set motor to 0
        0
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
    // if command == 90 {
    //     I16F16::from_num(0.5235987) // 30 degrees in radians
    // } else if command == 85 {
    //     I16F16::from_num(0.4581489) // 26.25 degrees in radians
    // } else if command == 80 {
    //     I16F16::from_num(0.3926991) // 22.5 degrees in radians
    // } else if command == 75 {
    //     I16F16::from_num(0.3272492) // 18.75 degrees in radians
    // } else if command == 70 {
    //     I16F16::from_num(0.2617993) // 15 degrees in radians
    // } else if command == 65 {
    //     I16F16::from_num(0.1963495) // 11.25 degrees in radians
    // } else if command == 60 {
    //     I16F16::from_num(0.1308996) // 7.5 degrees in radians
    // } else if command == 55 {
    //     I16F16::from_num(0.0654498) // 3.75 degrees in radians
    // } else if command == 50 {
    //     I16F16::from_num(0.0) // 0 degrees in radians
    // } else if command == 45 {
    //     I16F16::from_num(-0.0654498) // 3.75 degrees in radians
    // } else if command == 40 {
    //     I16F16::from_num(-0.1308996) // 7.5 degrees in radians
    // } else if command == 35 {
    //     I16F16::from_num(-0.1963495) // 11.25 degrees in radians
    // } else if command == 30 {
    //     I16F16::from_num(-0.2617993) // 15 degrees in radians
    // } else if command == 25 {
    //     I16F16::from_num(-0.3272492) // 18.75 degrees in radians
    // } else if command == 20 {
    //     I16F16::from_num(-0.3926991) // 22.5 degrees in radians
    // } else if command == 15 {
    //     I16F16::from_num(-0.4581489) // 26.25 degrees in radians
    // } else if command == 10 {
    //     I16F16::from_num(-0.5235987) // 30 degrees in radians
    // } else {
    //     // not a valid command, we set motor to 0
    //     I16F16::from_num(0.0) // 0 degrees in radians
    // }

    if command == 90 {
        I16F16::from_num(0.27925268) // 16 degrees in radians
    } else if command == 85 {
        I16F16::from_num(0.244346095) // 14 degrees in radians
    } else if command == 80 {
        I16F16::from_num(0.20943951) // 12 degrees in radians
    } else if command == 75 {
        I16F16::from_num(0.174532925) // 10 degrees in radians
    } else if command == 70 {
        I16F16::from_num(0.13962634) // 8 degrees in radians
    } else if command == 65 {
        I16F16::from_num(0.104719755) // 6 degrees in radians
    } else if command == 60 {
        I16F16::from_num(0.06981317) // 4 degrees in radians
    } else if command == 55 {
        I16F16::from_num(0.034906585) // 2 degrees in radians
    } else if command == 50 {
        I16F16::from_num(0.0) // 0 degrees in radians
    } else if command == 45 {
        I16F16::from_num(-0.034906585) // 2 degrees in radians
    } else if command == 40 {
        I16F16::from_num(-0.06981317) // 4 degrees in radians
    } else if command == 35 {
        I16F16::from_num(-0.104719755) // 6 degrees in radians
    } else if command == 30 {
        I16F16::from_num(-0.13962634) // 8 degrees in radians
    } else if command == 25 {
        I16F16::from_num(-0.174532925) // 10 degrees in radians
    } else if command == 20 {
        I16F16::from_num(-0.20943951) // 12 degrees in radians
    } else if command == 15 {
        I16F16::from_num(-0.244346095) // 14 degrees in radians
    } else if command == 10 {
        I16F16::from_num(-0.27925268) // 16 degrees in radians
    } else {
        // not a valid command, we set motor to 0
        I16F16::from_num(0.0) // 0 degrees in radians
    }
}

#[allow(clippy::approx_constant)]
pub fn map_roll_command(command: u8) -> I16F16 {
    // the mapping might be wrong, for now, I will assume the initial value is 0, pushing forward is positive, pushing backward is negative
    // this is the roll angle below, this is not the angular velocity
    // if command == 90 {
    //     I16F16::from_num(0.5235987) // 30 degrees in radians
    // } else if command == 85 {
    //     I16F16::from_num(0.4581489) // 26.25 degrees in radians
    // } else if command == 80 {
    //     I16F16::from_num(0.3926991) // 22.5 degrees in radians
    // } else if command == 75 {
    //     I16F16::from_num(0.3272492) // 18.75 degrees in radians
    // } else if command == 70 {
    //     I16F16::from_num(0.2617993) // 15 degrees in radians
    // } else if command == 65 {
    //     I16F16::from_num(0.1963495) // 11.25 degrees in radians
    // } else if command == 60 {
    //     I16F16::from_num(0.1308996) // 7.5 degrees in radians
    // } else if command == 55 {
    //     I16F16::from_num(0.0654498) // 3.75 degrees in radians
    // } else if command == 50 {
    //     I16F16::from_num(0.0) // 0 degrees in radians
    // } else if command == 45 {
    //     I16F16::from_num(-0.0654498) // 3.75 degrees in radians
    // } else if command == 40 {
    //     I16F16::from_num(-0.1308996) // 7.5 degrees in radians
    // } else if command == 35 {
    //     I16F16::from_num(-0.1963495) // 11.25 degrees in radians
    // } else if command == 30 {
    //     I16F16::from_num(-0.2617993) // 15 degrees in radians
    // } else if command == 25 {
    //     I16F16::from_num(-0.3272492) // 18.75 degrees in radians
    // } else if command == 20 {
    //     I16F16::from_num(-0.3926991) // 22.5 degrees in radians
    // } else if command == 15 {
    //     I16F16::from_num(-0.4581489) // 26.25 degrees in radians
    // } else if command == 10 {
    //     I16F16::from_num(-0.5235987) // 30 degrees in radians
    // } else {
    //     // not a valid command, we set motor to 0
    //     I16F16::from_num(0.0) // 0 degrees in radians
    // }

    if command == 90 {
        I16F16::from_num(0.27925268) // 16 degrees in radians
    } else if command == 85 {
        I16F16::from_num(0.244346095) // 14 degrees in radians
    } else if command == 80 {
        I16F16::from_num(0.20943951) // 12 degrees in radians
    } else if command == 75 {
        I16F16::from_num(0.174532925) // 10 degrees in radians
    } else if command == 70 {
        I16F16::from_num(0.13962634) // 8 degrees in radians
    } else if command == 65 {
        I16F16::from_num(0.104719755) // 6 degrees in radians
    } else if command == 60 {
        I16F16::from_num(0.06981317) // 4 degrees in radians
    } else if command == 55 {
        I16F16::from_num(0.034906585) // 2 degrees in radians
    } else if command == 50 {
        I16F16::from_num(0.0) // 0 degrees in radians
    } else if command == 45 {
        I16F16::from_num(-0.034906585) // 2 degrees in radians
    } else if command == 40 {
        I16F16::from_num(-0.06981317) // 4 degrees in radians
    } else if command == 35 {
        I16F16::from_num(-0.104719755) // 6 degrees in radians
    } else if command == 30 {
        I16F16::from_num(-0.13962634) // 8 degrees in radians
    } else if command == 25 {
        I16F16::from_num(-0.174532925) // 10 degrees in radians
    } else if command == 20 {
        I16F16::from_num(-0.20943951) // 12 degrees in radians
    } else if command == 15 {
        I16F16::from_num(-0.244346095) // 14 degrees in radians
    } else if command == 10 {
        I16F16::from_num(-0.27925268) // 16 degrees in radians
    } else {
        // not a valid command, we set motor to 0
        I16F16::from_num(0.0) // 0 degrees in radians
    }
}

#[allow(clippy::if_same_then_else)]
pub fn map_lift_command(command: u8) -> I16F16 {
    if command == 90 {
        I16F16::from_num(0)
    } else if command == 85 {
        I16F16::from_num(0)
    } else if command == 80 {
        I16F16::from_num(0)
    } else if command == 75 {
        I16F16::from_num(0)
    } else if command == 70 {
        I16F16::from_num(0)
    } else if command == 65 {
        I16F16::from_num(0)
    } else if command == 60 {
        I16F16::from_num(0)
    } else if command == 55 {
        I16F16::from_num(0)
    } else if command == 50 {
        I16F16::from_num(0)
    } else if command == 45 {
        I16F16::from_num(5)
    } else if command == 40 {
        I16F16::from_num(10)
    } else if command == 35 {
        I16F16::from_num(15)
    } else if command == 30 {
        I16F16::from_num(20)
    } else if command == 25 {
        I16F16::from_num(25)
    } else if command == 20 {
        I16F16::from_num(30)
    } else if command == 15 {
        I16F16::from_num(35)
    } else if command == 10 {
        I16F16::from_num(40)
    } else {
        I16F16::from_num(45)
    }
}

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

        delay_us_assembly(5000);

        set_motors([motor0, motor1, motor2, motor3]);
    }
}
