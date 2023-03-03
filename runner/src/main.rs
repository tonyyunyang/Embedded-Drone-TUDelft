mod runner_thread_layer;

use runner_thread_layer::uart_handler;
use serial2::SerialPort;
use std::env::args;
use std::path::Path;
use std::process::{exit, Command};

use std::thread::{self, sleep};
use std::time::Duration;
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};
// use pyo3::prelude::*;
// use inline_python::python;

fn main() {
    let file = args().nth(1);
    let port = upload_file_or_stop(PortSelector::AutoManufacturer, file);
    let serial = SerialPort::open(port, 115200).unwrap();

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
        .arg("my_interface.py");
        // pass the serial port as a command line parameter to the python program
        // .arg(port.to_str().unwrap());
        unsafe
        {
            eprint!("{:?}", cmd.output().unwrap().stdout);
        }
   
    match cmd.output() {
        Err(e) => {
            eprintln!("{}", e);
            exit(1);
        }
        Ok(i)=>
        {
            unsafe{

                eprintln!("{}", String::from_utf8_unchecked(i.stdout));
                exit(1);
            }
         if !i.status.success() 
         {
            exit(i.status.code().unwrap_or(1))
         }
        }
        Ok(_) => {}
    }
}
#[allow(unused)]
fn get_joystick_data_rust_og() {
    let mut cmd = Command::new("python");
    cmd
        // there must be a `my_interface.py` file of course
        .arg("-c")
        .arg("'from my_interface import *; get_joystick_data()'");
        // pass the serial port as a command line parameter to the python program
       //get back pitch, roll, yaw, throttle from python;
    match cmd.output() {
        Err(e) => {
            eprintln!("{}", e);
            exit(1);
        }
        Ok(i) if !i.status.success() =>
        {
            i.stdout;
            exit(i.status.code().unwrap_or(1));
        }
        Ok(_) => {}
    }
    
}


// fn get_joystick_data_rust()
// {
//     python! {
//         print("hello world")
//     }
// }
