use protocol::format::DeviceProtocol;
use serial2::SerialPort;
use std::env::args;
use std::path::PathBuf;
use std::process::{exit, Command};
use std::time::Duration;
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};

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

    // start_interface(&port);

    // open the serial port we got back from `upload_file_or_stop`. This is the same port
    // as the upload occurred on, so we know that we can communicate with the drone over
    // this port.
    let mut serial = SerialPort::open(port, 115200).unwrap();
    serial.set_read_timeout(Duration::from_secs(1)).unwrap();

    // infinitely print whatever the drone sends us
    let mut buf = [0u8; 255];

    // below is the original code
    loop {
        if let Ok(num) = serial.read(&mut buf) {
            print!("{}", String::from_utf8_lossy(&buf[0..num]));
        }
    }

    // below is the code for debug purposes
    // loop {
    //     if let Ok(num) = serial.read(&mut buf) {
    //         let message_from_drone = DeviceProtocol::deserialize(&buf[0..num]);
    //         if let Ok(message) = message_from_drone {
    //             let verification_crc = DeviceProtocol::calculate_crc8(&message);
    //             if verification_crc != message.get_crc() {
    //                 print!("\nCRC verification failed: {} != {}\n", verification_crc, message.get_crc());
    //                 print!("\n");
    //             } else {
    //                 print!("DTT: {:?}ms\n", message.get_duration());
    //                 print!("MODE: {:?}\n", message.get_mode());
    //                 print!("MTR: {} {} {} {}\n", message.get_motor()[0], message.get_motor()[1], message.get_motor()[2], message.get_motor()[3]);
    //                 print!("YPR {} {} {}\n", f32::from_bits(message.get_ypr()[0]), f32::from_bits(message.get_ypr()[1]), f32::from_bits(message.get_ypr()[2]));
    //                 print!("ACC {} {} {}\n", message.get_acc()[0], message.get_acc()[1], message.get_acc()[2]);
    //                 print!("BAT {bat}\n", bat = message.get_bat());
    //                 print!("BAR {pres} \n", pres = message.get_pres());
    //                 print!("CRC {crc}\n", crc = message.get_crc());
    //                 print!("\n");
    //             }
    //         }
    //     }
    // }

}

#[allow(unused)]
fn start_interface(port: &PathBuf) {
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
