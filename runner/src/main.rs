mod runner_thread_layer;
use app::App;
use gilrs::Gilrs;
use protocol::format::{DeviceProtocol, HostProtocol};
use runner_thread_layer::{
    joystick_monitor, keyboard_monitor, uart_handler, user_input, JoystickControl, KeyboardControl,
};
use serial2::SerialPort;
use std::env::args;
use std::io;
use std::sync::mpsc::channel;
use std::thread::{self, sleep};
use std::time::Duration;
use termion::raw::IntoRawMode;
use termion_ui::run_app;
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};
use tui::backend::TermionBackend;
use tui::Terminal;

mod file_writer;
mod app;
mod termion_ui;
mod ui;

fn main() {
    let mut gilrs = Gilrs::new().unwrap();

    let file = args().nth(1);
    let port = upload_file_or_stop(PortSelector::AutoManufacturer, file);
    let serial = SerialPort::open(port, 115200).unwrap();

    sleep(Duration::from_millis(1000));

    let (user_input_tx, user_input_rx) = channel::<HostProtocol>();
    let (keyboard_input_tx, keyboard_input_rx) = channel::<KeyboardControl>();
    let (joystick_input_tx, joystick_input_rx) = channel::<JoystickControl>();
    let (user_input_gui_tx, user_input_gui_rx) = channel::<HostProtocol>();
    let (device_data_tx, device_data_rx) = channel::<DeviceProtocol>();
    let (ack_tx, ack_rx) = channel::<bool>();

    let stdout = io::stdout().into_raw_mode().unwrap();
    let backend = TermionBackend::new(stdout);
    let mut terminal = Terminal::new(backend).unwrap();
    let app = App::new(" Group 5 Drone Demo!!!", true);

    let uart_handler = thread::spawn(move || {
        uart_handler(serial, user_input_rx, ack_tx, device_data_tx);
    });

    let user_input = thread::spawn(move || {
        user_input(
            user_input_tx,
            keyboard_input_rx,
            joystick_input_rx,
            ack_rx,
            user_input_gui_tx,
        );
    });

    let keyboard_monitor = thread::spawn(move || {
        keyboard_monitor(keyboard_input_tx);
    });

    let joystick_monitor = thread::spawn(move || {
        joystick_monitor(joystick_input_tx, &mut gilrs);
    });

    let gui = thread::spawn(move || {
        // run( true, user_input_gui_rx, device_data_rx).unwrap();
        run_app(&mut terminal, app, user_input_gui_rx, device_data_rx).unwrap();
    });

    uart_handler.join().unwrap();

    user_input.join().unwrap();

    keyboard_monitor.join().unwrap();

    joystick_monitor.join().unwrap();

    gui.join().unwrap();
}
