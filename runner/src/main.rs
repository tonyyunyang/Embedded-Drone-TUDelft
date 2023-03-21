mod runner_thread_layer;
use gilrs::Gilrs;
use protocol::format::HostProtocol;
use runner_thread_layer::{
    joystick_monitor, keyboard_monitor, uart_handler, user_input, JoystickControl, KeyboardControl,
};
use serial2::SerialPort;
use std::env::args;
use std::sync::mpsc::channel;
use std::thread::{self, sleep};
use std::time::Duration;
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};

fn main() {
    let mut gilrs = Gilrs::new().unwrap();

    let file = args().nth(1);
    let port = upload_file_or_stop(PortSelector::AutoManufacturer, file);
    let serial = SerialPort::open(port, 115200).unwrap();

    sleep(Duration::from_millis(1000));

    let (user_input_tx, user_input_rx) = channel::<HostProtocol>();
    let (keyboard_input_tx, keyboard_input_rx) = channel::<KeyboardControl>();
    let (joystick_input_tx, joystick_input_rx) = channel::<JoystickControl>();
    let (ack_tx, ack_rx) = channel::<bool>();

    let uart_handler = thread::spawn(move || {
        uart_handler(serial, user_input_rx, ack_tx);
    });

    let user_input = thread::spawn(move || {
        user_input(user_input_tx, keyboard_input_rx, joystick_input_rx, ack_rx);
    });

    let keyboard_monitor = thread::spawn(move || {
        keyboard_monitor(keyboard_input_tx);
    });

    let joystick_monitor = thread::spawn(move || {
        joystick_monitor(joystick_input_tx, &mut gilrs);
    });

    uart_handler.join().unwrap();

    user_input.join().unwrap();

    keyboard_monitor.join().unwrap();

    joystick_monitor.join().unwrap();
}
