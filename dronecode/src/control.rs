use crate::logger::{self, DroneLogData};
use crate::yaw_pitch_roll::YawPitchRoll;

use protocol::format::DeviceProtocol;
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;

use tudelft_quadrupel::led::Led::Blue;
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::send_bytes;

pub fn control_loop() -> ! {
    set_tick_frequency(100);
    let mut last = Instant::now();
    let mut test: u8 = 0;

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

        let mut logger = logger::BlackBoxLogger::new();

        logger.start_logging();

        // the code below is for sending the message to the PC
        if i % 100 == 0 {
            // Create an instance of the Drone Protocol struct
            test += 1;
            let mut message_to_pc = DeviceProtocol::new(
                test,
                dt.as_millis(),
                motors,
                [ypr.yaw.to_bits(), ypr.pitch.to_bits(), ypr.roll.to_bits()],
                [accel.x, accel.y, accel.z],
                bat,
                pres,
            );

            // Calculate the CRC and serialize the message
            message_to_pc.set_crc(DeviceProtocol::calculate_crc8(&message_to_pc));

            // Serialize the message
            let serialized_message_to_pc = DeviceProtocol::serialize(&message_to_pc);

            // Send the message to the PC, if the message is not serialized correctly, send an error message
            if let Ok(message) = serialized_message_to_pc {
                // The & character in &serialized_message_to_pc creates a reference to the vector that can be passed to send_bytes().
                send_bytes(&message);
            } else {
                let error_bytes: [u8; 60] = [0; 60];
                send_bytes(&error_bytes);
            }

            /**********************
             **** Data logging ****
             **********************/

            // First fill in the data log form
            // Then do logger.log_data(&data_log_form);
            let data_log_form = DroneLogData {
                timestamp: todo!(),
                accel: todo!(),
                motor: todo!(),
                ypr: todo!(),
                bat,
                pres,
                cpu_usage: todo!(),
                ram_usage: todo!(),
                flash_usage: todo!(),
                mode: todo!(),
                gyro: todo!(),
            };

            logger.log_data(&data_log_form);
        }

        // Stop the logger
        logger.stop_logging();

        // wait until the timer interrupt goes off again
        // based on the frequency set above
        wait_for_next_tick();
    }
    unreachable!();
}
