use postcard::Error;
use protocol::format::{DeviceProtocol, HostProtocol};
use serial2::SerialPort;
use core::num;
use std::env::args;
use std::path::{Path, PathBuf};
use std::process::{exit, Command};
use std::time::{Duration, self};
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};
use std::thread::{self, sleep};
use fixed::{
    types::{I6F26, I16F16},
};
use std::sync::mpsc::{channel, Receiver, Sender};

fn main() {
    let file = args().nth(1);
    let port = upload_file_or_stop(PortSelector::AutoManufacturer, file);
    let port_ref1 = port.clone();
    let port_ref2 = port.clone();
    let port_ref3 = port.clone();
    let port_ref4 = port.clone();

    // set up channels for communication between threads
    let (uart_state_tx, uart_state_rx): (Sender<host_state>, Receiver<host_state>) = channel();
    let (receiver_state_tx, receiver_state_rx): (Sender<host_state>, Receiver<host_state>) = channel();
    let (sender_state_tx, sender_state_rx): (Sender<host_state>, Receiver<host_state>) = channel();
    let (received_message_tx, received_message_rx): (Sender<Vec<u8>>, Receiver<Vec<u8>>) = channel();
    let (sending_message_tx, sending_message_rx): (Sender<Vec<u8>>, Receiver<Vec<u8>>) = channel();
    let (censor_tx, censor_rx): (Sender<HostProtocol>, Receiver<HostProtocol>) = channel();

    let receiver_uart_state_tx = uart_state_tx.clone();
    let censor_uart_state_tx = uart_state_tx.clone();
    let sender_uart_state_tx = uart_state_tx.clone();

    // start_interface(&port);

    // below is for debugging purpose
    // loop {
    //     if let Ok(num) = serial.read(&mut buf) {
    //         print!("\n The size of the messgae is: {} \n", num);

    //         print!("The Message is:\n");

    //         for i in 0..num {
    //             print!("{:02x} ", buf[i]);
    //         }

    //         print!("\n");

    //     }
    // }

    let uart_handler = thread::spawn(move || {
        loop_uart_handler(port_ref1, uart_state_rx, receiver_state_tx, sender_state_tx, received_message_rx, censor_rx, sending_message_tx);
    });
    
    let receive_device_message = thread::spawn(move || {
        loop_receive_device_message(port_ref2, receiver_state_rx, receiver_uart_state_tx, received_message_tx);
    });

    let read_user_input = thread::spawn(move || {
        loop_read_user_input(port_ref3, censor_uart_state_tx, censor_tx);
    });
    
    let send_host_command = thread::spawn(move || {
        loop_send_host_command(port_ref4, sender_state_rx, sender_uart_state_tx, sending_message_rx);
    });

    uart_handler.join().unwrap();
    receive_device_message.join().unwrap();
    read_user_input.join().unwrap();
    send_host_command.join().unwrap();

}

enum host_state {
    idle,
    ready_to_receive,
    ready_to_send,
}

fn loop_uart_handler(port: PathBuf, state: Receiver<host_state>, receiver_state: Sender<host_state>, sender_state: Sender<host_state>, message: Receiver<Vec<u8>>, command: Receiver<HostProtocol>, sending_message: Sender<Vec<u8>>) {
    loop {
        receiver_state.send(host_state::ready_to_receive).unwrap();
        match state.recv() {
            Ok(current_state) => {
                match current_state {
                    host_state::idle => {
                        // println!("\n----------------------------\nThe state channel is idle.\n----------------------------\n");
                        continue;
                    },
                    host_state::ready_to_receive => {
                        println!("\n----------------------------\nThe state channel is ready to receive.\n----------------------------\n");
                        // read the message via the channel
                        match message.recv() {
                            Ok(received_message) => {
                                // form the message
                                let mut received_message_buffer = received_message.clone();
                                let nice_received_message = DeviceProtocol::format_message(&mut received_message_buffer);
                                // verify the message, and print out the message
                                verify_message(&nice_received_message);
                                // done receiving the message, change the state to idle
                                receiver_state.send(host_state::idle).unwrap();
                            },
                            Err(_) => {
                                println!("\n----------------------------\nThere is no message from the device yet.\n----------------------------\n");
                                continue;
                            },
                        }
                    },
                    host_state::ready_to_send => {
                        println!("\n----------------------------\nThe state channel is ready to send.\n----------------------------\n");
                        // read the message via the channel
                        match command.recv() {
                            Ok(command) => {
                                // form the message
                                let mut message_sending = Vec::new();
                                command.form_message(&mut message_sending);
                                // send the message via the channel
                                sending_message.send(message_sending).unwrap();
                                //notice the sender of the state to send
                                sender_state.send(host_state::ready_to_send).unwrap();
                            },
                            Err(_) => {
                                println!("\n----------------------------\nThere is no message from the user yet.\n----------------------------\n");
                                continue;
                            },
                        }
                    },
                }
            },
            Err(_) => {
                println!("\n----------------------------\nThere is no message from the state channel yet.\n----------------------------\n");
                continue;
            },
        }
    }
}

fn loop_receive_device_message(port: PathBuf, state: Receiver<host_state>, uart_state: Sender<host_state>, message: Sender<Vec<u8>>) {
    let mut serial = SerialPort::open(port, 115200).unwrap();
    serial.set_read_timeout(Duration::from_secs(4)).unwrap();
    let mut received_bytes_count = 0; // the size of the message should be exactly 40 bytes, since we are using fixed size
    let mut buf = [0u8; 255];
    let mut message_buffer = Vec::new();
    let mut start_receiving = false;
    'outer: loop {
        uart_state.send(host_state::idle).unwrap();
        let receiver_state = state.recv();
        match receiver_state {
            Ok(current_state) => {
                match current_state {
                    host_state::ready_to_receive => {
                        // read the serial port
                        let read_result = serial.read(&mut buf);
                        match read_result {
                            Ok(num) => {
                                if num != 0 {
                                    'inner: for i in 0..num {
                                        let received_byte = buf[i];
                                        if received_byte == 0x7b && start_receiving == false {
                                            message_buffer.clear();
                                            start_receiving = true;
                                        }
                                        if start_receiving == true {
                                            message_buffer.push(received_byte);
                                            received_bytes_count += 1;
                                        }
                                        if received_bytes_count < 40 {
                                            continue 'inner;
                                        }
                                        // when it reaches here, the bytes recieved is already >= 40
                                        if received_byte == 0x7d && start_receiving == true {
                                            if received_bytes_count > 40 {
                                                message_buffer.clear();
                                                received_bytes_count = 0;
                                                start_receiving = false;
                                                continue 'outer;
                                            } else if received_bytes_count == 40 {
                                                // send the ready state and the message to the uart handler
                                                message.send(message_buffer.clone()).unwrap();
                                                uart_state.send(host_state::ready_to_receive).unwrap();
                                                // // format the message
                                                // let nice_received_message = DeviceProtocol::format_message(&mut message_buffer);
                                                // // verify the message, and print out the message
                                                // verify_message(&nice_received_message);
                                                // clean everything, initialize everything and start receiving again
                                                message_buffer.clear();
                                                received_bytes_count = 0;
                                                start_receiving = false;
                                                continue 'outer;
                                            }
                                                
                                        }
                                    }
                                } else if num == 0 {
                                    // nothing is received, so we just continue
                                    continue 'outer;
                                }
                                
                            },
                            Err(err) => {
                                println!("\nAn error occured: {}\n", err);
                                continue 'outer;
                            },
                        }
                    },
                    _ => {
                        continue 'outer;
                    },
                }
            },
            Err(_) => {
                continue 'outer;
            },
        }
    }
}


fn loop_read_user_input(port: PathBuf, uart_state: Sender<host_state>, command: Sender<HostProtocol>){
    loop {
        sleep(Duration::from_millis(100));
        let message_to_device = HostProtocol::new(1, 1, 1, 1,  1, 1, 1, 1);
        let mut message = Vec::new();
        message_to_device.form_message(&mut message);
        command.send(message_to_device).unwrap();
        uart_state.send(host_state::ready_to_send).unwrap();
    }
}

fn loop_send_host_command(port: PathBuf, state: Receiver<host_state>, uart_state: Sender<host_state>, sending_message: Receiver<Vec<u8>>) {
    let mut serial = SerialPort::open(port, 115200).unwrap();
    serial.set_write_timeout(Duration::from_secs(1)).unwrap();
    'outer: loop {
        uart_state.send(host_state::idle).unwrap();
        let sender_state = state.recv();
        match sender_state {
            Ok(current_state) => {
                match current_state {
                    host_state::ready_to_send => {
                        // read the message via the channel
                        match sending_message.recv() {
                            Ok(message) => {
                                // send the message via the serial port
                                let write_result = serial.write_all(&message);
                                match write_result {
                                    Ok(_) => {
                                        println!("\n----------------------------\n{} bytes are sent to the device.\n----------------------------\n", message.len());
                                        continue 'outer;
                                    },
                                    Err(err) => {
                                        println!("\nAn error occured: {}\n", err);
                                        continue 'outer;
                                    },
                                }
                            },
                            Err(_) => {
                                println!("\n----------------------------\nThere is no message from the user yet.\n----------------------------\n");
                                continue 'outer;
                            },
                        }
                    },
                    _ => {
                        continue 'outer;
                    },
                }
            },
            Err(_) => {
                continue 'outer;
            },
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

#[allow(unused)]
fn start_interface(port: &Path) {
    let mut cmd = Command::new("python");
    cmd
        // there must be a `my_interface.py` file of course
        .arg("my_interface.py")
        // pass the serial port as a command line parameter to the python program
        .arg(port.to_str().unwrap());

    match cmd.output() {
        Err(e) => {
            eprintln!("{}", e);
            exit(1);
        }
        Ok(i) if !i.status.success() => exit(i.status.code().unwrap_or(1)),
        Ok(_) => {}
    }
}
