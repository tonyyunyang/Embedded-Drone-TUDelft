use crc16::{State, XMODEM};
use crc_any::CRCu8;

use fixed::{
    types::{I6F26, I16F16},
};
use heapless::Vec;

use alloc::vec::{Vec as OtherVec, self};
use postcard::{from_bytes, to_vec};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct HostProtocol {
    // this is the data format for the data sent from the PC to the drone
    start_flag: u8,     // Start of frame indicator
    mode: u8,           // Two bytes to represent 9 modes
    joystick_lift: u8,  // Lift up/down control
    joystick_yaw: u8,   // Yaw left/right control
    keyboard_yaw: u8,
    joystick_pitch: u8, // Pitch up/down control
    keyboard_pitch_roll_1: u8,
    keyboard_pitch_roll_2: u8,
    joystick_roll: u8,  // Roll left/right control
    crc: u16,            // Cyclic redundancy check
    end_flag: u8,       // End of frame indicator
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct DeviceProtocol {
    // This is the data format for the data sent from the drone to the PC

    // Header
    start_flag: u8, // By default, this would be set to 0b01111011 = 0x7b, in ASCII, it is "{"

    // Payload
    mode: u8,        // Two bytes to represent 9 modes
    duration: u16,  // This is the duration of the tramision, 16 bytes
    motor: [u16; 4], // This is the data of the 4 motors on the drone, each motor has 2 bytes
    ypr: [I6F26; 3], // This is the data of the yaw, pitch and roll (Keep in mind that this is originally f32, but we are using u32), each has 4 bytes
    acc: [i16; 3], // This is the data of the acceleration of the drone (x, y and z), each has 2 bytes
    bat: u16,      // This is the data of the battery of the drone, 2 bytes
    pres: u32,     // This is the data of the pressure of the drone, 4 bytes
    ack: u8,       // This is the data of the acknowledgement byte, 1 byte

    // Footer
    crc: u16,      // Cyclic redundancy check
    end_flag: u8, // By default, this would be set to 0b01111101 = 0x7d, in ASCII, it is "}"
}

impl HostProtocol {
    // Construct a new HostProtocol from its fields
    pub fn new(
        mode: u8,
        joystick_lift: u8,
        joystick_yaw: u8,
        keyboard_yaw: u8,
        joystick_pitch: u8,
        keyboard_pitch_roll_1: u8,
        keyboard_pitch_roll_2: u8,
        joystick_roll: u8,
    ) -> Self {
        Self {
            start_flag: 0x7b,
            mode,
            joystick_lift,
            joystick_yaw,
            keyboard_yaw,
            joystick_pitch,
            keyboard_pitch_roll_1,
            keyboard_pitch_roll_2,
            joystick_roll,
            crc: 0x0000, // This is the default value of the CRC, it will be calculated later. If the CRC is 0x0000, it means that the CRC has not been calculated yet.
            end_flag: 0x7d,
        }
    }

    pub fn clone(&self) -> Self {
        Self {
            start_flag: self.start_flag,
            mode: self.mode,
            joystick_lift: self.joystick_lift,
            joystick_yaw: self.joystick_yaw,
            keyboard_yaw: self.keyboard_yaw,
            joystick_pitch: self.joystick_pitch,
            keyboard_pitch_roll_1: self.keyboard_pitch_roll_1,
            keyboard_pitch_roll_2: self.keyboard_pitch_roll_2,
            joystick_roll: self.joystick_roll,
            crc: self.crc,
            end_flag: self.end_flag,
        }
    }

    // Serializes this protocol and creates a Vec of bytes
    pub fn serialize(&self) -> Result<Vec<u8, 60>, postcard::Error> {
        let payload = to_vec(self)?;
        Ok(payload)
    }

    // Deserializes a byte vector into this protocol
    pub fn deserialize(payload: &[u8]) -> Result<Self, postcard::Error> {
        let prot = from_bytes::<HostProtocol>(payload)?;
        Ok(prot)
    }

    // Form the message to be sent to the drone in bytes, namely form an array of bytes
    // the size of the message formed now is 12 bytes
    pub fn form_message(&self, message: &mut vec::Vec<u8>) {
        message.push(self.start_flag);
        message.push(self.mode);
        message.push(self.joystick_lift);
        message.push(self.joystick_yaw);
        message.push(self.keyboard_yaw);
        message.push(self.joystick_pitch);
        message.push(self.keyboard_pitch_roll_1);
        message.push(self.keyboard_pitch_roll_2);
        message.push(self.joystick_roll);
        let crc = self.calculate_crc16();
        message.extend_from_slice(&crc.to_be_bytes());
        message.push(self.end_flag);
    }

    pub fn format_message(message: &mut vec::Vec<u8>) -> HostProtocol {
        let mut format_message = HostProtocol::new(0, 0, 0, 0, 0, 0, 0, 0);
        format_message.set_start_flag(message[0]);
        format_message.set_mode(message[1]);
        format_message.set_joystick_lift(message[2]);
        format_message.set_joystick_yaw(message[3]);
        format_message.set_keyboard_yaw(message[4]);
        format_message.set_joystick_pitch(message[5]);
        format_message.set_keyboard_pitch_roll_1(message[6]);
        format_message.set_keyboard_pitch_roll_2(message[7]);
        format_message.set_joystick_roll(message[8]);
        format_message.set_crc(u16::from_be_bytes([message[9], message[10]]));
        format_message.set_end_flag(message[11]);
        format_message
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
        state.update(&[self.keyboard_yaw]);
        state.update(&[self.joystick_pitch]);
        state.update(&[self.keyboard_pitch_roll_1]);
        state.update(&[self.keyboard_pitch_roll_2]);
        state.update(&[self.joystick_roll]);
        state.get()
    }

    pub fn calculate_crc8(&self) -> u8 {
        let mut crc = CRCu8::create_crc(0x07, 8, 0, 0, false); // specify the CRC-8 polynomial
        crc.digest(&[self.mode]);
        crc.digest(&[self.joystick_lift]);
        crc.digest(&[self.joystick_yaw]);
        crc.digest(&[self.keyboard_yaw]);
        crc.digest(&[self.joystick_pitch]);
        crc.digest(&[self.keyboard_pitch_roll_1]);
        crc.digest(&[self.keyboard_pitch_roll_2]);
        crc.digest(&[self.joystick_roll]);
        crc.get_crc()
    }

    pub fn set_start_flag(&mut self, start_flag: u8) {
        self.start_flag = start_flag;
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

    pub fn set_keyboard_yaw(&mut self, keyboard_yaw: u8) {
        self.keyboard_yaw = keyboard_yaw;
    }

    pub fn set_joystick_pitch(&mut self, joystick_pitch: u8) {
        self.joystick_pitch = joystick_pitch;
    }

    pub fn set_keyboard_pitch_roll_1(&mut self, keyboard_pitch_roll_1: u8) {
        self.keyboard_pitch_roll_1 = keyboard_pitch_roll_1;
    }

    pub fn set_keyboard_pitch_roll_2(&mut self, keyboard_pitch_roll_2: u8) {
        self.keyboard_pitch_roll_2 = keyboard_pitch_roll_2;
    }

    pub fn set_joystick_roll(&mut self, joystick_roll: u8) {
        self.joystick_roll = joystick_roll;
    }

    pub fn set_crc(&mut self, crc: u16) {
        self.crc = crc;
    }

    pub fn set_end_flag(&mut self, end_flag: u8) {
        self.end_flag = end_flag;
    }

    pub fn get_start_flag(&self) -> u8 {
        self.start_flag
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

    pub fn get_keyboard_yaw(&self) -> u8 {
        self.keyboard_yaw
    }

    pub fn get_joystick_pitch(&self) -> u8 {
        self.joystick_pitch
    }

    pub fn get_keyboard_pitch_roll_1(&self) -> u8 {
        self.keyboard_pitch_roll_1
    }

    pub fn get_keyboard_pitch_roll_2(&self) -> u8 {
        self.keyboard_pitch_roll_2
    }

    pub fn get_joystick_roll(&self) -> u8 {
        self.joystick_roll
    }

    pub fn get_crc(&self) -> u16 {
        self.crc
    }

    pub fn get_end_flag(&self) -> u8 {
        self.end_flag
    }
}

impl DeviceProtocol {
    // Construct a new DroneProtocol from its fields
    pub fn new(
        mode: u8,
        duration: u16,
        motor: [u16; 4],
        ypr: [I6F26; 3],
        acc: [i16; 3],
        bat: u16,
        pres: u32,
        ack: u8
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
            ack,
            crc: 0x0000, // This is the default value of the CRC, it will be calculated later. If the CRC is 0x0000, it means that the CRC has not been calculated yet.
            end_flag: 0x7d,
        }
    }

    // Serializes this protocol and creates a Vec of bytes
    pub fn serialize(&self) -> Result<Vec<u8, 60>, postcard::Error> {
        let payload = to_vec(self)?;
        Ok(payload)
    }

    // Deserializes a byte vector into this protocol
    pub fn deserialize(payload: &[u8]) -> Result<Self, postcard::Error> {
        let prot = from_bytes::<DeviceProtocol>(payload)?;
        Ok(prot)
    }

    // Form the message to be sent to the drone in bytes, namely form an array of bytes
    pub fn form_message(&self, message: &mut vec::Vec<u8>) {
        message.push(self.start_flag);
        message.push(self.mode);
        message.extend_from_slice(&self.duration.to_be_bytes());
        message.extend_from_slice(&self.motor[0].to_be_bytes());
        message.extend_from_slice(&self.motor[1].to_be_bytes());
        message.extend_from_slice(&self.motor[2].to_be_bytes());
        message.extend_from_slice(&self.motor[3].to_be_bytes());
        message.extend_from_slice(&self.ypr[0].to_be_bytes());
        message.extend_from_slice(&self.ypr[1].to_be_bytes());
        message.extend_from_slice(&self.ypr[2].to_be_bytes());
        message.extend_from_slice(&self.acc[0].to_be_bytes());
        message.extend_from_slice(&self.acc[1].to_be_bytes());
        message.extend_from_slice(&self.acc[2].to_be_bytes());
        message.extend_from_slice(&self.bat.to_be_bytes());
        message.extend_from_slice(&self.pres.to_be_bytes());
        message.push(self.ack);
        let crc = self.calculate_crc16();
        message.extend_from_slice(&crc.to_be_bytes());
        message.push(self.end_flag);
    }

    pub fn format_message(message: &mut vec::Vec<u8>) -> DeviceProtocol {
        let mut format_message = DeviceProtocol::new(0, 0, [0; 4], [I6F26::from_num(0); 3], [0; 3], 0, 0, 0);
        format_message.set_start_flag(message[0]);
        format_message.set_mode(message[1]);
        format_message.set_duration(u16::from_be_bytes([message[2], message[3]]));
        format_message.set_motor([u16::from_be_bytes([message[4], message[5]]), u16::from_be_bytes([message[6], message[7]]), u16::from_be_bytes([message[8], message[9]]), u16::from_be_bytes([message[10], message[11]])]);
        format_message.set_ypr([I6F26::from_be_bytes([message[12], message[13], message[14], message[15]]), I6F26::from_be_bytes([message[16], message[17], message[18], message[19]]), I6F26::from_be_bytes([message[20], message[21], message[22], message[23]])]);
        format_message.set_acc([i16::from_be_bytes([message[24], message[25]]), i16::from_be_bytes([message[26], message[27]]), i16::from_be_bytes([message[28], message[29]])]);
        format_message.set_bat(u16::from_be_bytes([message[30], message[31]]));
        format_message.set_pres(u32::from_be_bytes([message[32], message[33], message[34], message[35]]));
        format_message.set_ack(message[36]);
        format_message.set_crc(u16::from_be_bytes([message[37], message[38]]));
        format_message.set_end_flag(message[39]);
        format_message
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
        state.update(&self.ack.to_be_bytes());
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
        crc.digest(&self.ack.to_be_bytes());
        crc.get_crc()
    }

    pub fn set_mode(&mut self, mode: u8) {
        self.mode = mode;
    }

    pub fn set_duration(&mut self, duration: u16) {
        self.duration = duration;
    }

    pub fn set_motor(&mut self, motor: [u16; 4]) {
        self.motor = motor;
    }

    pub fn set_ypr(&mut self, ypr: [I6F26; 3]) {
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

    pub fn set_ack(&mut self, ack: u8) {
        self.ack = ack;
    }

    pub fn set_crc(&mut self, crc: u16) {
        self.crc = crc;
    }

    pub fn set_start_flag(&mut self, start_flag: u8) {
        self.start_flag = start_flag;
    }

    pub fn set_end_flag(&mut self, end_flag: u8) {
        self.end_flag = end_flag;
    }

    pub fn get_start_flag(&self) -> u8 {
        self.start_flag
    }

    pub fn get_mode(&self) -> u8 {
        self.mode
    }

    pub fn get_duration(&self) -> u16 {
        self.duration
    }

    pub fn get_motor(&self) -> [u16; 4] {
        self.motor
    }

    pub fn get_ypr(&self) -> [I6F26; 3] {
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

    pub fn get_ack(&self) -> u8 {
        self.ack
    }

    pub fn get_crc(&self) -> u16 {
        self.crc
    }

    pub fn get_end_flag(&self) -> u8 {
        self.end_flag
    }
}
