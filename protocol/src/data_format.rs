use serde::{Serialize, Deserialize};
use postcard::{from_bytes, to_vec};
use heapless::Vec;

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct protocol_pc {        // this is the data format for the data sent from the PC to the drone
    // header
    start_flag: u8,             // by default, this would be set to 0b01111011 = 0x7b, in ASCII, it is "{"

    // payload                  // the payload is a 6-byte array, which is the data we want to send
    mode_selection: [u8; 2],    // the first 9 bits: mode selection, each bit represents a mode
                                // the next 7 bits are for redundancy
    joystick_lift: u8,          // the next 4 bytes are for the data of output from the joystick, sequence: Lift, Yaw, Pitch, Roll
    joystick_yaw: u8,
    joystick_pitch: u8,
    joystick_roll: u8,

    // footer
    crc: u16,                   // we are using the crc-16 method to verify the message
    end_flag: u8                // by default, this would be set to 0b01111101 = 0x7d, in ASCII, it is "}"
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct protocol_drone {     // this is the data format for the data sent from the drone to the PC
    // header
    start_flag: u8,             // by default, this would be set to 0b01111011 = 0x7b, in ASCII, it is "{"

    // payload
    motor: [u16; 4],            // this is the data of the 4 motors on the drone, each motor has 2 bytes
    ypr: [f32; 3],              // this is the data of the yaw, pitch and roll of the drone, each has 4 bytes
    acc: [i16; 3],              // this is the data of the acceleration of the drone (x, y and z), each has 2 bytes
    bat: u16,                   // this is the data of the battery of the drone, 2 bytes
    pres: u32,                  // this is the data of the pressure of the drone, 4 bytes

    // footer
    crc: u16,                   // we are using the crc-16 method to verify the message
    end_flag: u8                // by default, this would be set to 0b01111101 = 0x7d, in ASCII, it is "}"
}

impl protocol_pc {
    pub fn new() -> Self {
        protocol_pc {
            start_flag: 0x7b,
            mode_selection: [0; 2],
            joystick_lift: 0,
            joystick_yaw: 0,
            joystick_pitch: 0,
            joystick_roll: 0,
            crc: 0,
            end_flag: 0x7d,
        }
    }

    pub fn encode(&self) -> Vec<u8, 32> {
        let mut buf = Vec::new();
        to_vec(&self, &mut buf).unwrap();
    }

    pub fn decode(buf: &[u8]) -> Self {
        from_bytes(buf).unwrap()
    }

    pub fn set_mode_selection(&mut self, mode_selection: [u8; 2]) {
        self.mode_selection = mode_selection;
    }
    
    pub fn set_joystick_lift(&mut self, joystick_lift: u8) {
        self.joystick_lift = joystick_lift;
    } 
}