use postcard::Error;
use protocol::format::DeviceProtocol;
use serial2::SerialPort;
use std::env::args;
use std::path::Path;
use std::process::{exit, Command};
use std::time::Duration;
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};
use std::thread;

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
    // loop {
    //     if let Ok(num) = serial.read(&mut buf) {
    //         print!("{}", String::from_utf8_lossy(&buf[0..num]));
    //     }
    // }

    // below is for debugging purpose
    // loop {
    //     if let Ok(num) = serial.read(&mut buf) {
    //         print!("\n The size of the messgae is: {} \n", num);

    //         print!("The Message is:\n");

    //         for i in 0..num {
    //             print!("{} ", buf[i]);
    //         }

    //         print!("\n");

    //     }
    // }

    let receive_device_message = thread::spawn(move || {
        loop_receive_device_message(&mut serial, &mut buf);
    });

    let read_user_input = thread::spawn(move || {
        loop_read_user_input();
    });

    let send_host_command = thread::spawn(move || {
        loop_send_host_command();
    });

    send_host_command.join().unwrap();
    receive_device_message.join().unwrap();
    read_user_input.join().unwrap();
}

fn loop_receive_device_message(serial: &mut SerialPort, buf: &mut [u8; 255]) {
    // below is the code receiving data from the drone via protocol and printing them out
    loop {
        // read the serial port
        if let Ok(num) = serial.read(buf) {
            // when the program starts, the pc will read garbage data, so we need to ignore them
            if buf[0] != 123 {
                continue;
            }
            // deserialize the message
            let message_from_drone = DeviceProtocol::deserialize(&buf[0..num]);
            // capture whether the message is complete
            match message_from_drone {
                Ok(message) => {
                    // verfiy the message
                    verify_message(&message);
                }
                Err(error) => {
                    // if the message seems to be incomplete, read once more from the buffer
                    if error == Error::DeserializeUnexpectedEnd {
                        let mut full_message = Vec::new();
                        full_message.extend_from_slice(&buf[0..num]);
                        // we read once again from the buffer here
                        if let Ok(num) = serial.read(buf) {
                            full_message.extend_from_slice(&buf[0..num]);
                            let message_from_drone = DeviceProtocol::deserialize(&full_message);
                            match message_from_drone {
                                Ok(message) => {
                                    // verfiy the full message
                                    verify_message(&message);
                                }
                                Err(error) => {
                                    println!("Deserialize Error: {:?}", error);
                                }
                            }
                        }
                    } else {
                        println!("Deserialize Error: {:?}", error);
                    }
                }
            }
        }
    }
}

fn loop_read_user_input(){

}

fn loop_send_host_command(){

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
    let verification_crc = DeviceProtocol::calculate_crc8(message);
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
    println!("CRC {crc}", crc = message.get_crc());
    println!();
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
