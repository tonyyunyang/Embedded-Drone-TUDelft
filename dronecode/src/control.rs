
use crate::yaw_pitch_roll::YawPitchRoll;

use alloc::vec::Vec;
use protocol::format::{DeviceProtocol, HostProtocol};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;

use tudelft_quadrupel::block;

use tudelft_quadrupel::led::Led::{Blue, Red, Green, Yellow};
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{send_bytes, receive_bytes};

use postcard::Error;

pub fn control_loop() -> ! {
    set_tick_frequency(100);
    let mut last = Instant::now();
    let mut test: u8 = 0;
    let mut ack = 0b0000_0000;

    // let mut logger = logger::BlackBoxLogger::new();
    // logger.start_logging();

    for i in 0.. {
        let _ = Blue.toggle();
        let now = Instant::now();
        let dt = now.duration_since(last);
        last = now;

        let motors = get_motors();
        let quaternion = block!(read_dmp_bytes()).unwrap();
        let ypr = YawPitchRoll::from(quaternion);
        let (accel, _) = read_raw().unwrap();
        let bat = read_battery();
        let pres = read_pressure();

        // the code below is for receiving command from the PC
        // if i % 10 == 0 {
        //     // a buffer to store the received bytes
        //     let mut buffer: [u8; 255] = [0; 255];
        //     receive_bytes(&mut buffer);
        //     let message_from_host = HostProtocol::deserialize(&mut buffer); // here might be a problem
        //     match message_from_host {
        //         Ok(message) => {
        //             // verfiy the message
        //             for counter in 0..10 {
        //                 let _ = Green.toggle();
        //             }
        //             ack = verify_message(&message);
        //         }
        //         Err(error) => {
        //             // if the message seems to be incomplete, read once more from the buffer
        //             if error == Error::DeserializeUnexpectedEnd {
        //                 let mut full_message = Vec::new();
        //                 full_message.extend_from_slice(&buffer);
        //                 // we read once again from the buffer here
        //                 receive_bytes(&mut buffer);
        //                 full_message.extend_from_slice(&buffer);
        //                 let message_from_host = HostProtocol::deserialize(&mut full_message);
        //                 match message_from_host {
        //                     Ok(message) => {
        //                         // verfiy the full message
        //                         ack = verify_message(&message);
        //                     }
        //                     Err(_) => {
        //                         // if the message is still incomplete, print out the error
        //                         ack = 0b0000_0000;
        //                     }
        //                 }
        //             } else {
        //                 // if the message is still incomplete, print out the error
        //                 ack = 0b0000_0000;
        //             }
        //         }
        //     }
        // }

        // the code below is for sending the message to the PC
        if i % 100 == 0 {
            // Create an instance of the Drone Protocol struct
            test += 1;
            ack += 1;
            let mut message_to_host = DeviceProtocol::new(
                test,
                dt.as_millis() as u16,
                motors,
                [ypr.yaw, ypr.pitch, ypr.roll],
                [accel.x, accel.y, accel.z],
                bat,
                pres,
                ack,
            );

            // Form the message waiting to be sent to the host
            let mut message = Vec::new();
            message_to_host.form_message(&mut message);
            send_bytes(&message);

            /**********************
             **** Data logging ****
             **********************/

            // First fill in the data log form
            // Then do logger.log_data(&data_log_form);
            // let data_log_form = DroneLogData {
            //     timestamp: now.ns_since_start(),
            //     accel: [accel.x, accel.y, accel.z],
            //     motor: motors,
            //     ypr: [ypr.yaw.to_bits(), ypr.pitch.to_bits(), ypr.roll.to_bits()],
            //     bat,
            //     pres,
            //     cpu_usage: 0,
            //     ram_usage: 0,
            //     flash_usage: 0,
            //     mode: test,
            // };

            // logger.log(&data_log_form);

            // if (i % 1000) == 0 {
            //     // Stop logging every 10 seconds
            //     logger.stop_logging();
            // }
        }

        // Stop the logger

        // Read from the flash memory

        // wait until the timer interrupt goes off again
        // based on the frequency set above
        wait_for_next_tick();
    }
    unreachable!();
}

fn verify_message(message: &HostProtocol) -> u8  {
    // we check the start bit and the end bit first
    if message.get_start_flag() != 0x7b || message.get_end_flag() != 0x7d {
        0b0000_0000
    } else if verify_crc(message) {
        0b1111_1111
    } else {
        0b0000_0000
    }
}

fn verify_crc(message: &HostProtocol) -> bool {
    let verification_crc = HostProtocol::calculate_crc16(message);
    verification_crc == message.get_crc()
}
