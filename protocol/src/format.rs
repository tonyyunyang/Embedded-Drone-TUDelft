use fixed::{types::I0F32, FixedI32};
use crc16::{State, XMODEM};
use crc_any::CRCu8;
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
    crc: u8,           // Cyclic redundancy check
    end_flag: u8,       // End of frame indicator
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct DeviceProtocol {
    // This is the data format for the data sent from the drone to the PC

    // Header
    start_flag: u8, // By default, this would be set to 0b01111011 = 0x7b, in ASCII, it is "{"

    // Payload
    mode: u8,         // Two bytes to represent 9 modes
    duration: u128,   // This is the duration of the tramision, 16 bytes
    motor: [u16; 4],  // This is the data of the 4 motors on the drone, each motor has 2 bytes
    ypr: [u32; 3], // This is the data of the yaw, pitch and roll (Keep in mind that this is originally f32, but we are using u32), each has 4 bytes
    acc: [i16; 3], // This is the data of the acceleration of the drone (x, y and z), each has 2 bytes
    bat: u16,         // This is the data of the battery of the drone, 2 bytes
    pres: u32,        // This is the data of the pressure of the drone, 4 bytes

    // Footer
    crc: u8,     // Cyclic redundancy check
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
    ) -> Self {
        Self {
            start_flag: 0x7b,
            mode,
            joystick_lift,
            joystick_yaw,
            joystick_pitch,
            joystick_roll,
            crc: 0x0000,    // This is the default value of the CRC, it will be calculated later. If the CRC is 0x0000, it means that the CRC has not been calculated yet.
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

    // The function below calculates the CRC-16 value for the struct, the value used for calculation is the payload (not including start byte and end byte)
    // pub fn calculate_crc(host_protocol: &mut HostProtocol) {
    //     let mut crc: u16 = 0xFFFF; // initial value of the CRC
    
    //     // iterate over the bytes of the payload and update the CRC
    //     let payload_bytes = unsafe {
    //         core::slice::from_raw_parts(
    //             &host_protocol.mode as *const _ as *const u8,
    //             core::mem::size_of_val(&host_protocol)
    //                 - core::mem::size_of_val(&host_protocol.start_flag)
    //                 - core::mem::size_of_val(&host_protocol.end_flag)
    //                 - core::mem::size_of_val(&host_protocol.crc)
    //         )
    //     };
    
    //     for &byte in payload_bytes {
    //         crc ^= u16::from(byte);
    
    //         for _ in 0..8 {
    //             if (crc & 0x0001) != 0 {
    //                 crc >>= 1;
    //                 crc ^= 0xA001;
    //             } else {
    //                 crc >>= 1;
    //             }
    //         }
    //     }
    //     host_protocol.crc = crc;
    // }

    pub fn calculate_crc16(&self) -> u16 {
        let mut state = State::<XMODEM>::new();
        state.update(&[self.mode]);
        state.update(&[self.joystick_lift]);
        state.update(&[self.joystick_yaw]);
        state.update(&[self.joystick_pitch]);
        state.update(&[self.joystick_roll]);
        state.get()
    }

    pub fn calculate_crc8(&self) -> u8 {
        let mut crc = CRCu8::create_crc(0x07, 8, 0, 0, false); // specify the CRC-8 polynomial
        crc.digest(&[self.mode]);
        crc.digest(&[self.joystick_lift]);
        crc.digest(&[self.joystick_yaw]);
        crc.digest(&[self.joystick_pitch]);
        crc.digest(&[self.joystick_roll]);
        crc.get_crc()
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

    pub fn set_crc(&mut self, crc: u8) {
        self.crc = crc;
    }

    pub fn get_mode(&self) -> u8 {
        self.mode
    }

    pub fn get_joystick_lift(&self) -> u8 {
        self.joystick_lift
    }

    pub fn get_joystick_yaw(&self) -> u8 {
        self.joystick_yaw
    }

    pub fn get_joystick_pitch(&self) -> u8 {
        self.joystick_pitch
    }

    pub fn get_joystick_roll(&self) -> u8 {
        self.joystick_roll
    }

    pub fn get_crc(&self) -> u8 {
        self.crc
    }
}

impl DeviceProtocol {
    // Construct a new DroneProtocol from its fields
    pub fn new(
        mode: u8,
        duration: u128,
        motor: [u16; 4],
        ypr: [u32; 3],
        acc: [i16; 3],
        bat: u16,
        pres: u32,
    ) -> Self {
        Self {
            start_flag: 0x7b,
            duration,
            mode,
            motor,
            ypr,
            acc,
            bat,
            pres,
            crc: 0x0000,    // This is the default value of the CRC, it will be calculated later. If the CRC is 0x0000, it means that the CRC has not been calculated yet.
            end_flag: 0x7d,
        }
    }

    // Serializes this protocol and creates a Vec of bytes
    pub fn serialize(&self) -> Result<Vec<u8, 52>, postcard::Error> { // the struct we created is 53 bytes long when crc-16, and 52 bytes long when crc-8
        let payload = to_vec(self)?;
        Ok(payload)
    }

    // Deserializes a byte vector into this protocol
    pub fn deserialize(payload: &[u8]) -> Result<Self, postcard::Error> {
        let prot = from_bytes::<DeviceProtocol>(payload)?;
        Ok(prot)
    }

    // The function below calculates the CRC-16 value for the struct, the value used for calculation is the payload (not including start byte and end byte)
    // pub fn calculate_crc(device_protocol: &mut DeviceProtocol) {
    //     let mut crc: u16 = 0xFFFF; // initial value of the CRC
    
    //     // iterate over the bytes of the payload and update the CRC
    //     let payload_bytes = unsafe {
    //         core::slice::from_raw_parts(
    //             &device_protocol.mode as *const _ as *const u8,
    //             core::mem::size_of_val(&device_protocol)
    //                 - core::mem::size_of_val(&device_protocol.start_flag)
    //                 - core::mem::size_of_val(&device_protocol.end_flag)
    //                 - core::mem::size_of_val(&device_protocol.crc)
    //         )
    //     };
    
    //     for &byte in payload_bytes {
    //         crc ^= u16::from(byte);
    
    //         for _ in 0..8 {
    //             if (crc & 0x0001) != 0 {
    //                 crc >>= 1;
    //                 crc ^= 0xA001;
    //             } else {
    //                 crc >>= 1;
    //             }
    //         }
    //     }
    
    //     device_protocol.crc = crc;
    // }
    
    pub fn calculate_crc16(&self) -> u16 {
        let mut state = State::<XMODEM>::new();
        state.update(&[self.mode]);
        state.update(&self.duration.to_be_bytes());
        for motor in self.motor.iter() {
            state.update(&motor.to_be_bytes());
        }
        for ypr in self.ypr.iter() {
            state.update(&ypr.to_be_bytes());
        }
        for acc in self.acc.iter() {
            state.update(&acc.to_be_bytes());
        }
        state.update(&self.bat.to_be_bytes());
        state.update(&self.pres.to_be_bytes());
        state.get()
    }

    pub fn calculate_crc8(&self) -> u8 {
        let mut crc = CRCu8::create_crc(0x07, 8, 0, 0, false); // specify the CRC-8 polynomial
        crc.digest(&[self.mode]);
        crc.digest(&self.duration.to_be_bytes());
        for motor in self.motor.iter() {
            crc.digest(&motor.to_be_bytes());
        }
        for ypr in self.ypr.iter() {
            crc.digest(&ypr.to_be_bytes());
        }
        for acc in self.acc.iter() {
            crc.digest(&acc.to_be_bytes());
        }
        crc.digest(&self.bat.to_be_bytes());
        crc.digest(&self.pres.to_be_bytes());
        crc.get_crc()
    }

    pub fn set_mode(&mut self, mode: u8) {
        self.mode = mode;
    }

    pub fn set_duration(&mut self, duration: u128) {
        self.duration = duration;
    }

    pub fn set_motor(&mut self, motor: [u16; 4]) {
        self.motor = motor;
    }

    pub fn set_ypr(&mut self, ypr: [u32; 3]) {
        self.ypr = ypr;
    }

    pub fn set_acc(&mut self, acc: [i16; 3]) {
        self.acc = acc;
    }

    pub fn set_bat(&mut self, bat: u16) {
        self.bat = bat;
    }

    pub fn set_pres(&mut self, pres: u32) {
        self.pres = pres;
    }

    pub fn set_crc(&mut self, crc: u8) {
        self.crc = crc;
    }

    pub fn get_mode(&self) -> u8 {
        self.mode
    }

    pub fn get_duration(&self) -> u128 {
        self.duration
    }

    pub fn get_motor(&self) -> [u16; 4] {
        self.motor
    }

    pub fn get_ypr(&self) -> [u32; 3] {
        self.ypr
    }

    pub fn get_acc(&self) -> [i16; 3] {
        self.acc
    }

    pub fn get_bat(&self) -> u16 {
        self.bat
    }

    pub fn get_pres(&self) -> u32 {
        self.pres
    }

    pub fn get_crc(&self) -> u8 {
        self.crc
    }

}
