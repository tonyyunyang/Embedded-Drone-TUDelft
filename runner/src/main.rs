use postcard::Error;
use protocol::format::{DeviceProtocol, HostProtocol};
use serial2::SerialPort;
use std::env::args;
use std::path::{Path, PathBuf};
use std::process::{exit, Command};
use std::time::{Duration, self};
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};
use std::thread::{self, sleep};
use fixed::{
    types::{I6F26, I16F16},
};

fn main() {
    // get a filename from the command line. This filename will be uploaded to the drone
    // note that if no filename is given, the upload to the drone does not fail.
    // `upload_file_or_stop` will still try to detect the serial port on which the drone
    // is attached. This may be useful if you don't want to actually change the code on the
    // drone, but you do want to rerun your UI. In that case you simply don't provide any
    // command line parameter.
    let file = args().nth(1);
    let port = upload_file_or_stop(PortSelector::AutoManufacturer, file);
    let port_ref1 = port.clone();
    let port_ref2 = port.clone();
    let port_ref3 = port.clone();

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
    //             print!("{:02x} ", buf[i]);
    //         }

    //         print!("\n");

    //     }
    // }

    
    let receive_device_message = thread::spawn(move || {
        loop_receive_device_message(port_ref1, &mut buf);
    });

    let read_user_input = thread::spawn(move || {
        // loop_read_user_input();
    });
    
    let send_host_command = thread::spawn(move || {
        loop_send_host_command(port_ref2);
    });

    send_host_command.join().unwrap();
    receive_device_message.join().unwrap();
    read_user_input.join().unwrap();

}

fn loop_receive_device_message(port: PathBuf, buf: &mut [u8;255]) {
    let mut serial = SerialPort::open(port, 115200).unwrap();
    serial.set_read_timeout(Duration::from_secs(1)).unwrap();
    let mut messsage_buffer: Vec<u8> = Vec::new();
    let mut received_bytes_count = 0; // the size of the message should be exactly 40 bytes, since we are using fixed size
    let mut start_receiving = false;
    'outer: loop {
        // read the serial port
        if let Ok(num) = serial.read(buf) {
            'inner: for i in 0..num {
                let received_byte = buf[i];
                if received_byte == 0x7b && start_receiving == false {
                    messsage_buffer.clear();
                    start_receiving = true;
                }
                if start_receiving == true {
                    messsage_buffer.push(received_byte);
                    received_bytes_count += 1;
                }
                if received_bytes_count < 40 {
                    continue 'inner;
                }
                // when it reaches here, the bytes recieved is already >= 40
                if received_byte == 0x7d && start_receiving == true {
                    if received_bytes_count > 40 {
                        messsage_buffer.clear();
                        received_bytes_count = 0;
                        start_receiving = false;
                        continue 'outer;
                    } else if received_bytes_count == 40 {
                        // format the message
                        let nice_received_message = DeviceProtocol::format_message(&mut messsage_buffer);
                        // verify the message, and print out the message
                        verify_message(&nice_received_message);
                        // clean everything, initialize everything and start receiving again
                        messsage_buffer.clear();
                        received_bytes_count = 0;
                        start_receiving = false;
                        continue 'outer;
                    }
                        
                }
            }
        }
    }
}

fn loop_read_user_input(){

}

fn loop_send_host_command(port: PathBuf){
    let mut serial = SerialPort::open(port, 115200).unwrap();
    serial.set_write_timeout(Duration::from_secs(1)).unwrap();
    loop {
        // send a message to the drone
        let message_to_device = HostProtocol::new(1, 1, 1, 1,  1, 1, 1, 1);
        let mut message = Vec::new();
        message_to_device.form_message(&mut message);
        let _size = serial.write(&message).unwrap();
        // println!("---------------------------------\nSent {} bytes\n----------------------", size);
        sleep(time::Duration::from_millis(200));
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
