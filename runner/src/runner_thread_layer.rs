use protocol::format::{DeviceProtocol, HostProtocol};
use serial2::SerialPort;
use std::io::{stdin, stdout, Write};
use std::{
    sync::mpsc::{Receiver, Sender},
    thread::sleep,
    time::Duration,
};
use termion::event::Key;
use termion::input::TermRead;
use termion::raw::IntoRawMode;

pub enum KeyboardControl {
    SafeMode,
    PanicMode,
    Mode0,
    Mode1,
    Mode2,
    Mode3,
    Mode4,
    Mode5,
    Mode6,
    Mode7,
    Mode8,
    Mode9,
    LiftUp,
    LiftDown,
    RollUp,
    RollDown,
    PitchUp,
    PitchDown,
    YawUp,
    YawDown,
    YawPUp,
    YawPDown,
    RollPitchP1Up,
    RollPitchP1Down,
    RollPitchP2Up,
    RollPitchP2Down,
}

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

pub fn user_input(user_input: Sender<HostProtocol>, _keyboard_input: Receiver<KeyboardControl>) {
    // let mut mode = 0b0000_0000;
    // let mut lift = 0;
    // let mut yaw = 0;
    // let mut pitch = 0;
    // let mut roll = 0;
    // let mut p = 0;
    // let mut p1 = 0;
    // let mut p2 = 0;

    loop {
        // form the messages by monitoring the user input (either joystick of keyboard)
        // Read the joystick input in this thread and send commands continuously, when keyboard is pressed, then add the command to the current one
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

pub fn keyboard_monitor(keyboard_input: Sender<KeyboardControl>) {
    // end_flag is used to exit the Raw Terminal Mode
    let mut end_flag = false;
    // define the values that we want
    // Read keys pressed from the keyboard and send them to the device
    while !end_flag {
        let stdin = stdin();
        //setting up stdout and going into raw mode
        let mut stdout = stdout().into_raw_mode().unwrap();
        //detecting keydown events
        for c in stdin.keys() {
            //clearing the screen and going to top left corner
            write!(
                stdout,
                "{}{}",
                termion::cursor::Goto(1, 1),
                termion::clear::All
            )
            .unwrap();

            //i reckon this speaks for itself
            match c.unwrap() {
                // below are the keys related to mode switch, come back to this after modes on the drone side are implemented
                Key::Esc => {
                    switch_safe_mode(keyboard_input.clone());
                }
                Key::Char(' ') => {
                    switch_panic_mode(keyboard_input.clone());
                }
                Key::Char('0') => {
                    switch_mode_0(keyboard_input.clone());
                }
                Key::Char('1') => {
                    switch_mode_1(keyboard_input.clone());
                }
                Key::Char('2') => {
                    switch_mode_2(keyboard_input.clone());
                }
                Key::Char('3') => {
                    switch_mode_3(keyboard_input.clone());
                }
                Key::Char('4') => {
                    switch_mode_4(keyboard_input.clone());
                }
                Key::Char('5') => {
                    switch_mode_5(keyboard_input.clone());
                }
                Key::Char('6') => {
                    switch_mode_6(keyboard_input.clone());
                }
                Key::Char('7') => {
                    switch_mode_7(keyboard_input.clone());
                }
                Key::Char('8') => {
                    switch_mode_8(keyboard_input.clone());
                }
                Key::Char('9') => {
                    switch_mode_9(keyboard_input.clone());
                }
                // below are keys related to drone control
                Key::Char('a') => {
                    lift_up(keyboard_input.clone());
                }
                Key::Char('z') => {
                    lift_down(keyboard_input.clone());
                }
                Key::Char('A') => {
                    lift_up(keyboard_input.clone());
                }
                Key::Char('Z') => {
                    lift_down(keyboard_input.clone());
                }
                Key::Left => {
                    roll_up(keyboard_input.clone());
                }
                Key::Right => {
                    roll_down(keyboard_input.clone());
                }
                Key::Up => {
                    pitch_down(keyboard_input.clone());
                }
                Key::Down => {
                    pitch_up(keyboard_input.clone());
                }
                Key::Char('q') => {
                    yaw_down(keyboard_input.clone());
                }
                Key::Char('w') => {
                    yaw_up(keyboard_input.clone());
                }
                Key::Char('Q') => {
                    yaw_down(keyboard_input.clone());
                }
                Key::Char('W') => {
                    yaw_up(keyboard_input.clone());
                }
                // below are keys related to P control
                Key::Char('u') => {
                    yaw_p_up(keyboard_input.clone());
                }
                Key::Char('j') => {
                    yaw_p_down(keyboard_input.clone());
                }
                Key::Char('U') => {
                    yaw_p_up(keyboard_input.clone());
                }
                Key::Char('J') => {
                    yaw_p_down(keyboard_input.clone());
                }
                Key::Char('i') => {
                    roll_pitch_p1_up(keyboard_input.clone());
                }
                Key::Char('k') => {
                    roll_pitch_p1_down(keyboard_input.clone());
                }
                Key::Char('I') => {
                    roll_pitch_p1_up(keyboard_input.clone());
                }
                Key::Char('K') => {
                    roll_pitch_p1_down(keyboard_input.clone());
                }
                Key::Char('o') => {
                    roll_pitch_p2_up(keyboard_input.clone());
                }
                Key::Char('l') => {
                    roll_pitch_p2_down(keyboard_input.clone());
                }
                Key::Char('O') => {
                    roll_pitch_p2_up(keyboard_input.clone());
                }
                Key::Char('L') => {
                    roll_pitch_p2_down(keyboard_input.clone());
                }
                /*PLEASE READ THIS! ALWAYS REMEMBER THAT THE WAY TO EXIT IS: CTRL + Q */
                Key::Ctrl('q') => break,
                _ => { //Invalid key
                }
            }
            stdout.flush().unwrap();
        }
        drop(stdout);
        end_flag = true;
    }
}

fn switch_safe_mode(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::SafeMode).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn switch_panic_mode(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::PanicMode).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn switch_mode_0(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::Mode0).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn switch_mode_1(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::Mode1).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn switch_mode_2(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::Mode2).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn switch_mode_3(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::Mode3).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn switch_mode_4(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::Mode4).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn switch_mode_5(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::Mode5).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn switch_mode_6(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::Mode6).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn switch_mode_7(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::Mode7).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn switch_mode_8(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::Mode8).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn switch_mode_9(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::Mode9).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn lift_up(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::LiftUp).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn lift_down(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::LiftDown).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn roll_up(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::RollUp).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn roll_down(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::RollDown).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn pitch_up(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::PitchUp).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn pitch_down(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::PitchDown).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn yaw_up(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::YawUp).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn yaw_down(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::YawDown).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn yaw_p_up(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::YawPUp).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn yaw_p_down(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::YawPDown).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn roll_pitch_p1_up(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::RollPitchP1Up).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn roll_pitch_p1_down(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input
        .send(KeyboardControl::RollPitchP1Down)
        .is_ok()
    {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn roll_pitch_p2_up(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input.send(KeyboardControl::RollPitchP2Up).is_ok() {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
    }
}

fn roll_pitch_p2_down(keyboard_input: Sender<KeyboardControl>) {
    if keyboard_input
        .send(KeyboardControl::RollPitchP2Down)
        .is_ok()
    {
        println!("Message sent to message formatter");
    } else {
        println!("Message not sent to message formatter");
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
