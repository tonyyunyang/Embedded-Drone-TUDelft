use crate::yaw_pitch_roll::YawPitchRoll;
use alloc::format;
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::fixed::FixedI16;
use tudelft_quadrupel::fixed::types::I16F16;
use tudelft_quadrupel::led::Led::Blue;
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::send_bytes;
use protocol::format::DeviceProtocol;

pub fn control_loop() -> ! {
    set_tick_frequency(100);
    let mut last = Instant::now();

    for i in 0.. {
        let _ = Blue.toggle();
        let now = Instant::now();
        let dt = now.duration_since(last);
        last = now;

        let motors = get_motors();
        let quaternion = read_dmp_bytes().unwrap();
        let ypr = YawPitchRoll::from(quaternion);
        let (accel, _) = read_raw().unwrap();
        let bat = read_battery();
        let pres = read_pressure();

        // the code below is for debugging purpose
        if i % 100 == 0 {
            // Create an instance of the Drone Protocol struct
            let current_mode:u8 = 0b00000000;
            let mut message_to_pc = DeviceProtocol::new(current_mode, motors, [I16F16::from_bits(ypr.yaw.to_bits().try_into().unwrap()), I16F16::from_bits(ypr.pitch.to_bits().try_into().unwrap()),I16F16::from_bits(ypr.roll.to_bits().try_into().unwrap())], [I16F16::from_bits(accel.x as i32), I16F16::from_bits(accel.y as i32), I16F16::from_bits(accel.z as i32)], bat, pres);
        }

        // wait until the timer interrupt goes off again
        // based on the frequency set above
        wait_for_next_tick();
    }
    unreachable!();
}

// // Optimized version
//         if i % 100 == 0 {
//             let output_string = format!(
//                 "DTT: {:?}ms\nMTR: {} {} {} {}\nYPR {} {} {}\nACC {} {} {}\nBAT {}\nBAR {} \n",
//                 dt.as_millis(),
//                 motors[0],
//                 motors[1],
//                 motors[2],
//                 motors[3],
//                 ypr.yaw,
//                 ypr.pitch,
//                 ypr.roll,
//                 accel.x,
//                 accel.y,
//                 accel.z,
//                 bat,
//                 pres
//             );
//             send_bytes(output_string.as_bytes());
//         }
