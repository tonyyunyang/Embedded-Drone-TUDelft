

use protocol::format::{DeviceProtocol, HostProtocol};
use serial2::SerialPort;

enum DeviceModes {
    Panic,
    Manual,
    Safety,
}

enum HostModes {
    SendMessage,
    ReceiveMessage,
    Idle,
}

pub fn uart_handler(serial: SerialPort) {
    let mut buf = [0u8; 255];
    let mut received_bytes_count = 0; // the size of the message should be exactly 40 bytes, since we are using fixed size
    let mut message_buffer = Vec::new();
    let mut start_receiving = false;
    let mut command_ready = true;
    let mut repeat_flag = false;

    'outer: loop {
        // 'read: while finish_receiving == false {
        let read_result = serial.read(&mut buf);
        match read_result {
            Ok(num) => {
                if num != 0 {
                    'inner: for i in 0..num {
                        let received_byte = buf[i];
                        if received_byte == 0x7b && !start_receiving {
                            message_buffer.clear();
                            start_receiving = true;
                        }
                        if start_receiving {
                            message_buffer.push(received_byte);
                            received_bytes_count += 1;
                        }
                        if received_bytes_count < 40 {
                            continue 'inner;
                        }
                        if message_buffer.len() < 40 && !repeat_flag {
                            repeat_flag = true;
                            continue 'outer;
                        }

                        // when it reaches here, the bytes recieved is already >= 40
                        if received_byte == 0x7d && start_receiving {
                            if received_bytes_count != 40 {
                                message_buffer.clear();
                                received_bytes_count = 0;
                                start_receiving = false;
                                repeat_flag = false;
                                continue 'outer;
                            } else if received_bytes_count == 40 {
                                // send the ready state and the message to the uart handler
                                // // format the message
                                let nice_received_message =
                                    DeviceProtocol::format_message(&mut message_buffer);
                                // // verify the message, and print out the message
                                verify_message(&nice_received_message);
                                // clean everything, initialize everything and start receiving again
                                message_buffer.clear();
                                received_bytes_count = 0;
                                start_receiving = false;
                                repeat_flag = false;
                                continue 'outer;
                            }
                        }
                    }
                } else if num == 0 {
                    message_buffer.clear();
                    received_bytes_count = 0;
                    start_receiving = false;
                    repeat_flag = false;
                    // nothing is received
                    println!("\n-------------------------Nothing is received----------------------------\n");
                    continue 'outer;
                }
            }
            Err(_) => {
                message_buffer.clear();
                received_bytes_count = 0;
                start_receiving = false;
                // if nothing is received on the host, we send the message to the device
                // we also check if there is commands to be sent to the device
                // in this case, we check the channel connect this thread and the thread that monitors the input from users
                command_ready = false; // this state should be received from the channel
                command_ready = true;
                if command_ready {
                    let message_to_device = HostProtocol::new(1, 1, 1, 1, 1, 1, 1, 1);
                    let mut message = Vec::new();
                    message_to_device.form_message(&mut message);
                    serial.write(&message).unwrap();
                } else {
                    continue 'outer;
                }
            }
        }
    }
}

fn verify_message(message: &DeviceProtocol) {
    // we check the start bit and the end bit first
    if message.get_start_flag() != 0x7b || message.get_end_flag() != 0x7d {
        println!("Start or End flag is wrong");
    } else if verify_crc(message) {
        print_verified_message(message);
    } else {
        println!("CRC verification failed");
    }
}

fn verify_crc(message: &DeviceProtocol) -> bool {
    let verification_crc = DeviceProtocol::calculate_crc16(message);
    verification_crc == message.get_crc()
}

fn print_verified_message(message: &DeviceProtocol) {
    println!("DTT: {:?}ms", message.get_duration());
    println!("MODE: {:?}", message.get_mode());
    println!(
        "MTR: {} {} {} {}",
        message.get_motor()[0],
        message.get_motor()[1],
        message.get_motor()[2],
        message.get_motor()[3]
    );
    println!(
        "YPR {} {} {}",
        message.get_ypr()[0],
        message.get_ypr()[1],
        message.get_ypr()[2]
    );
    println!(
        "ACC {} {} {}",
        message.get_acc()[0],
        message.get_acc()[1],
        message.get_acc()[2]
    );
    println!("BAT {bat}", bat = message.get_bat());
    println!("BAR {pres} ", pres = message.get_pres());
    println!("ACK {ack}", ack = message.get_ack());
    println!("CRC {crc}", crc = message.get_crc());
    println!("--------------------------------");
}
