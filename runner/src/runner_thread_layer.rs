use protocol::format::{DeviceProtocol, HostProtocol};
use serial2::SerialPort;
use std::{
    sync::mpsc::{Receiver, Sender},
    thread::sleep,
    time::Duration,
};

// enum DeviceModes {
//     Panic,
//     Manual,
//     Safety,
// }

pub fn uart_handler(serial: SerialPort, user_input: Receiver<HostProtocol>) {
    let mut buf = [0u8; 255];
    let mut received_bytes_count = 0; // the size of the message should be exactly 40 bytes, since we are using fixed size
    let mut message_buffer = Vec::new();
    let mut start_receiving = false;
    // let mut command_ready = true;
    let mut repeat_flag = false;

    'outer: loop {
        let read_result = serial.read(&mut buf);
        match read_result {
            Ok(num) => {
                if num != 0 {
                    'inner: for i in buf.iter().take(num) {
                        let received_byte = *i;
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
                // if there is nothing to read, we check if there is something to be sent, if there is, we send it, if not, we continue
                let read_user = user_input.try_recv();
                match read_user {
                    Ok(message_to_device) => {
                        let mut message = Vec::new();
                        message_to_device.form_message(&mut message);
                        let write_result = serial.write(&message);
                        match write_result {
                            Ok(_) => {
                                println!("Message sent to device");
                            }
                            Err(_) => {
                                println!("Message not sent to device");
                            }
                        }
                        continue 'outer;
                    }
                    Err(_) => {
                        continue 'outer;
                    }
                }
            }
        }
    }
}

pub fn user_input(user_input: Sender<HostProtocol>) {
    loop {
        // form the messages by monitoring the user input (either joystick of keyboard)
        let protocol = HostProtocol::new(54, 65, 89, 89, 78, 54, 65, 78);
        let feedback = user_input.send(protocol);
        match feedback {
            Ok(_) => {
                println!("Message sent to handler");
            }
            Err(_) => {
                println!("Message not sent to handler");
            }
        }
        sleep(Duration::from_millis(100));
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
    println!("--------------------------------");
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
