use serial2::SerialPort;
use std::env::args;
use std::path::PathBuf;
use std::process::{exit, Command};
use std::time::Duration;
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};
// use pyo3::prelude::*;
// use inline_python::python;

fn main() {
    // get a filename from the command line. This filename will be uploaded to the drone
    // note that if no filename is given, the upload to the drone does not fail.
    // `upload_file_or_stop` will still try to detect the serial port on which the drone
    // is attached. This may be useful if you don't want to actually change the code on the
    // drone, but you do want to rerun your UI. In that case you simply don't provide any
    // command line parameter.
    let file = args().nth(1);
    let port = upload_file_or_stop(PortSelector::AutoManufacturer, file);

    // The code below shows a very simple start to a PC-side receiver of data from the drone.
    // You can extend this into an entire interface to the drone written in Rust. However,
    // if you are more comfortable writing such an interface in any other programming language
    // you like (for example, python isn't a bad choice), you can also call that here. The
    // commented function below gives an example of how to do that with python, but again
    // you don't need to choose python.
    //eprint!("I am trying to get joystick data");
    //get_joystick_data_rust();
    start_interface();
    // open the serial port we got back from `upload_file_or_stop`. This is the same port
    // as the upload occurred on, so we know that we can communicate with the drone over
    // this port.
    let mut serial = SerialPort::open(port, 115200).unwrap();
    serial.set_read_timeout(Duration::from_secs(1)).unwrap();

    // infinitely print whatever the drone sends us
    let mut buf = [0u8; 255];
    loop {
        if let Ok(num) = serial.read(&mut buf) {
            print!("{}", String::from_utf8_lossy(&buf[0..num]));
        }
        // get_joystick_data_rust_og();
        // start_interface();
    }
}

#[allow(unused)]
fn start_interface() {
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
