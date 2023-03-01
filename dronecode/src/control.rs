
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
        let now = Instant::now();
        let dt = now.duration_since(last);
        last = now;
        let motors = get_motors();
        let quaternion = block!(read_dmp_bytes()).unwrap();
        let ypr = YawPitchRoll::from(quaternion);
        let (accel, _) = read_raw().unwrap();
        let bat = read_battery();
        let pres = read_pressure();

        // the code below is for receiving the message from the host
        let mut buf: [u8; 255] = [0; 255];
        let mut messsage_buffer: Vec<u8> = Vec::new();
        let mut received_bytes_count = 0; // the size of the message should be exactly 40 bytes, since we are using fixed size
        let mut start_receiving = false;

        let num = receive_bytes(&mut buf);
        if num != 0 {
            for _ in 0..10 {
                Red.toggle();
            }
            for i in 0..num {
                let received_byte = buf[i];
                if received_byte == 0x7b && start_receiving == false {
                    messsage_buffer.clear();
                    start_receiving = true;
                }
                if start_receiving == true {
                    messsage_buffer.push(received_byte);
                    received_bytes_count += 1;
                }
                if received_byte == 0x7d && start_receiving == true {
                    if received_bytes_count != 12{
                        ack = 0b0000_0000;
                    } else if received_bytes_count == 12 {
                        let nice_received_message = HostProtocol::format_message(&mut messsage_buffer);
                        ack = verify_message(&nice_received_message);
                        received_bytes_count = 0;
                        start_receiving = false;
                    }
                }
            }
        }

        // the code below is for sending the message to the host
        if i % 100 == 0 {
            // Create an instance of the Drone Protocol struct
            test += 1;
            let message_to_host = DeviceProtocol::new(
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
        ack = 0b0000_0000;
        wait_for_next_tick();
    }
    unreachable!();
}

fn verify_message(message: &HostProtocol) -> u8  {
    // we check the start bit and the end bit first
    if message.get_start_flag() != 0x7b || message.get_end_flag() != 0x7d {
        if verify_crc(message) {
            0b1111_1111
        } else {
            0b0000_0000
        }
    }else {
        0b0000_0000
    }
}

fn verify_crc(message: &HostProtocol) -> bool {
    let verification_crc = HostProtocol::calculate_crc16(message);
    verification_crc == message.get_crc()
}
