use fixed::types::I16F16;
use heapless::Vec;
use postcard::{from_bytes, to_vec};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct HostProtocol {
    // this is the data format for the data sent from the PC to the drone
    start_flag: u8,     // Start of frame indicator
    mode: u8,           // Two bytes to represent 9 modes
    joystick_lift: u8,  // Lift up/down control
    joystick_yaw: u8,   // Yaw left/right control
    joystick_pitch: u8, // Pitch up/down control
    joystick_roll: u8,  // Roll left/right control
    crc: u16,           // Cyclic redundancy check
    end_flag: u8,       // End of frame indicator
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct DroneProtocol {
    // This is the data format for the data sent from the drone to the PC

    // Header
    start_flag: u8, // By default, this would be set to 0b01111011 = 0x7b, in ASCII, it is "{"

    // Payload
    mode: u8,         // Two bytes to represent 9 modes
    motor: [u16; 4],  // This is the data of the 4 motors on the drone, each motor has 2 bytes
    ypr: [I16F16; 3], // This is the data of the yaw, pitch and roll
    acc: [I16F16; 3], // This is the data of the acceleration of the drone (x, y and z), each has 2 bytes
    bat: u16,         // This is the data of the battery of the drone, 2 bytes
    pres: u32,        // This is the data of the pressure of the drone, 4 bytes

    // Footer
    crc: u16,     // We are using the crc-16 method to verify the message
    end_flag: u8, // By default, this would be set to 0b01111101 = 0x7d, in ASCII, it is "}"
}

impl HostProtocol {
    // Construct a new HostProtocol from its fields
    pub fn new(
        mode: u8,
        joystick_lift: u8,
        joystick_yaw: u8,
        joystick_pitch: u8,
        joystick_roll: u8,
        crc: u16,
    ) -> Self {
        Self {
            start_flag: 0x7b,
            mode,
            joystick_lift,
            joystick_yaw,
            joystick_pitch,
            joystick_roll,
            crc,
            end_flag: 0x7d,
        }
    }

    // Serializes this protocol and creates a Vec of bytes
    pub fn serialize(&self) -> Result<Vec<u8, 32>, postcard::Error> {
        let payload = to_vec(self)?;
        Ok(payload)
    }

    // Deserializes a byte vector into this protocol
    pub fn deserialize(payload: &[u8]) -> Result<Self, postcard::Error> {
        let prot = from_bytes::<HostProtocol>(payload)?;
        Ok(prot)
    }

    pub fn set_mode(&mut self, mode: u8) {
        self.mode = mode;
    }

    pub fn set_joystick_lift(&mut self, joystick_lift: u8) {
        self.joystick_lift = joystick_lift;
    }

    pub fn set_joystick_yaw(&mut self, joystick_yaw: u8) {
        self.joystick_yaw = joystick_yaw;
    }

    pub fn set_joystick_pitch(&mut self, joystick_pitch: u8) {
        self.joystick_pitch = joystick_pitch;
    }

    pub fn set_joystick_roll(&mut self, joystick_roll: u8) {
        self.joystick_roll = joystick_roll;
    }

    pub fn set_crc(&mut self, crc: u16) {
        self.crc = crc;
    }
}

impl DroneProtocol {
    // Construct a new DroneProtocol from its fields
    pub fn new(
        mode: u8,
        motor: [u16; 4],
        ypr: [I16F16; 3],
        acc: [I16F16; 3],
        bat: u16,
        pres: u32,
        crc: u16,
    ) -> Self {
        Self {
            start_flag: 0x7b,
            mode,
            motor,
            ypr,
            acc,
            bat,
            pres,
            crc,
            end_flag: 0x7d,
        }
    }

    // Serializes this protocol and creates a Vec of bytes
    pub fn serialize(&self) -> Result<Vec<u8, 32>, postcard::Error> {
        let payload = to_vec(self)?;
        Ok(payload)
    }

    // Deserializes a byte vector into this protocol
    pub fn deserialize(payload: &[u8]) -> Result<Self, postcard::Error> {
        let prot = from_bytes::<DroneProtocol>(payload)?;
        Ok(prot)
    }

    pub fn set_mode(&mut self, mode: u8) {
        self.mode = mode;
    }

    pub fn set_motor(&mut self, motor: [u16; 4]) {
        self.motor = motor;
    }

    pub fn set_ypr(&mut self, ypr: [I16F16; 3]) {
        self.ypr = ypr;
    }

    pub fn set_acc(&mut self, acc: [I16F16; 3]) {
        self.acc = acc;
    }

    pub fn set_bat(&mut self, bat: u16) {
        self.bat = bat;
    }

    pub fn set_pres(&mut self, pres: u32) {
        self.pres = pres;
    }

    pub fn set_crc(&mut self, crc: u16) {
        self.crc = crc;
    }
}
