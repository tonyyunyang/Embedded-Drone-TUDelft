use postcard::Error;
use protocol::format::{DeviceProtocol, HostProtocol};
mod runner_thread_layer;
use core::num;
use fixed::types::{I16F16, I6F26};
use runner_thread_layer::uart_handler;
use serial2::SerialPort;
use std::env::args;
use std::path::{Path, PathBuf};
use std::process::{exit, Command};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::thread::{self, sleep};
use std::time::{self, Duration};
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};

fn main() {
    let file = args().nth(1);
    let port = upload_file_or_stop(PortSelector::AutoManufacturer, file);
    let serial = SerialPort::open(&port, 115200).unwrap();

    sleep(Duration::from_millis(1000));

    let uart_handler = thread::spawn(move || {
        uart_handler(serial);
    });

    uart_handler.join().unwrap();
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
