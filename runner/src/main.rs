mod runner_thread_layer;
use protocol::format::HostProtocol;
use runner_thread_layer::{keyboard_monitor, uart_handler, user_input, KeyboardControl};
use serial2::SerialPort;
use std::env::args;
use std::sync::mpsc::channel;
use std::thread::{self, sleep};
use std::time::Duration;
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};

fn main() {
    let file = args().nth(1);
    let port = upload_file_or_stop(PortSelector::AutoManufacturer, file);
    let serial = SerialPort::open(port, 115200).unwrap();

    sleep(Duration::from_millis(1000));

    let (user_input_tx, user_input_rx) = channel::<HostProtocol>();
    let (keyboard_input_tx, keyboard_input_rx) = channel::<KeyboardControl>();

    let uart_handler = thread::spawn(move || {
        uart_handler(serial, user_input_rx);
    });

    let user_input = thread::spawn(move || {
        user_input(user_input_tx, keyboard_input_rx);
    });

    let keyboard_monitor = thread::spawn(move || {
        keyboard_monitor(keyboard_input_tx);
    });

    uart_handler.join().unwrap();

    user_input.join().unwrap();

    keyboard_monitor.join().unwrap();
}
