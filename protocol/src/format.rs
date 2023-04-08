use crc16::{State, XMODEM};
use crc_any::CRCu8;

use alloc::vec::Vec as OtherVec;
use fixed::types::I16F16;

use alloc::vec::{self};
pub struct HostProtocol {
    // this is the data format for the data sent from the PC to the drone
    start_flag: u8, // Start of frame indicator
    mode: u8,       // Two bytes to represent 9 modes
    lift: u8,       // Lift up/down control
    yaw: u8,        // Yaw left/right control
    pitch: u8,      // Pitch up/down control
    roll: u8,       // Roll left/right control
    p: u8,          // P control
    p1: u8,         // P1 control
    p2: u8,         // P2 control
    crc: u16,       // Cyclic redundancy check
    end_flag: u8,   // End of frame indicator
}

pub struct DeviceProtocol {
    // This is the data format for the data sent from the drone to the PC

    // Header
    start_flag: u8, // By default, this would be set to 0b01111011 = 0x7b, in ASCII, it is "{"

    // Payload
    mode: u8,         // Two bytes to represent 9 modes
    duration: u16,    // This is the duration of the tramision, 16 bytes
    motor: [u16; 4],  // This is the data of the 4 motors on the drone, each motor has 2 bytes
    ypr: [I16F16; 3], // This is the data of the yaw, pitch and roll (Keep in mind that this is originally f32, but we are using u32), each has 4 bytes
    ypr_filter: [I16F16; 3], // This is the data of the yaw, pitch and roll (Keep in mind that this is originally f32, but we are using u32), each has 4 bytes
    acc: [i16; 3], // This is the data of the acceleration of the drone (x, y and z), each has 2 bytes
    bat: u16,      // This is the data of the battery of the drone, 2 bytes
    pres: i32,     // This is the data of the pressure of the drone, 4 bytes
    ack: u8,       // This is the data of the acknowledgement byte, 1 byte

    // Footer
    crc: u16,     // Cyclic redundancy check
    end_flag: u8, // By default, this would be set to 0b01111101 = 0x7d, in ASCII, it is "}"
}

impl HostProtocol {
    // Construct a new HostProtocol from its fields
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        mode: u8,  // Two bytes to represent 9 modes
        lift: u8,  // Lift up/down control
        yaw: u8,   // Yaw left/right control
        pitch: u8, // Pitch up/down control
        roll: u8,  // Roll left/right control
        p: u8,     // P control
        p1: u8,    // P1 control
        p2: u8,    // P2 control
    ) -> Self {
        Self {
            start_flag: 0x7b,
            mode,
            lift,
            yaw,
            pitch,
            roll,
            p,
            p1,
            p2,
            crc: 0x0000, // This is the default value of the CRC, it will be calculated later. If the CRC is 0x0000, it means that the CRC has not been calculated yet.
            end_flag: 0x7d,
        }
    }

    pub fn another_clone(&self) -> Self {
        Self {
            start_flag: self.start_flag,
            mode: self.mode,
            lift: self.lift,
            yaw: self.yaw,
            pitch: self.pitch,
            roll: self.roll,
            p: self.p,
            p1: self.p1,
            p2: self.p2,
            crc: self.crc,
            end_flag: self.end_flag,
        }
    }

    // Form the message to be sent to the drone in bytes, namely form an array of bytes
    // the size of the message formed now is 12 bytes
    pub fn form_message(&self, message: &mut vec::Vec<u8>) {
        message.push(self.start_flag);
        message.push(self.mode);
        message.push(self.lift);
        message.push(self.yaw);
        message.push(self.pitch);
        message.push(self.roll);
        message.push(self.p);
        message.push(self.p1);
        message.push(self.p2);
        let crc = self.calculate_crc16();
        message.extend_from_slice(&crc.to_be_bytes());
        message.push(self.end_flag);
    }

    pub fn format_message(message: &mut [u8]) -> HostProtocol {
        let mut format_message = HostProtocol::new(0, 0, 0, 0, 0, 0, 0, 0);
        format_message.set_start_flag(message[0]);
        format_message.set_mode(message[1]);
        format_message.set_lift(message[2]);
        format_message.set_yaw(message[3]);
        format_message.set_pitch(message[4]);
        format_message.set_roll(message[5]);
        format_message.set_p(message[6]);
        format_message.set_p1(message[7]);
        format_message.set_p2(message[8]);
        format_message.set_crc(u16::from_be_bytes([message[9], message[10]]));
        format_message.set_end_flag(message[11]);
        format_message
    }

    pub fn format_message_not_mut(message: &[u8]) -> HostProtocol {
        let mut format_message = HostProtocol::new(0, 0, 0, 0, 0, 0, 0, 0);
        format_message.set_start_flag(message[0]);
        format_message.set_mode(message[1]);
        format_message.set_lift(message[2]);
        format_message.set_yaw(message[3]);
        format_message.set_pitch(message[4]);
        format_message.set_roll(message[5]);
        format_message.set_p(message[6]);
        format_message.set_p1(message[7]);
        format_message.set_p2(message[8]);
        format_message.set_crc(u16::from_be_bytes([message[9], message[10]]));
        format_message.set_end_flag(message[11]);
        format_message
    }

    pub fn format_message_alloc(message: &mut OtherVec<u8>) -> HostProtocol {
        let mut format_message = HostProtocol::new(0, 0, 0, 0, 0, 0, 0, 0);
        format_message.set_start_flag(message[0]);
        format_message.set_mode(message[1]);
        format_message.set_lift(message[2]);
        format_message.set_yaw(message[3]);
        format_message.set_pitch(message[4]);
        format_message.set_roll(message[5]);
        format_message.set_p(message[6]);
        format_message.set_p1(message[7]);
        format_message.set_p2(message[8]);
        format_message.set_crc(u16::from_be_bytes([message[9], message[10]]));
        format_message.set_end_flag(message[11]);
        format_message
    }

    pub fn calculate_crc16(&self) -> u16 {
        let mut state = State::<XMODEM>::new();
        state.update(&[self.mode]);
        state.update(&[self.lift]);
        state.update(&[self.yaw]);
        state.update(&[self.pitch]);
        state.update(&[self.roll]);
        state.update(&[self.p]);
        state.update(&[self.p1]);
        state.update(&[self.p2]);
        state.get()
    }

    pub fn calculate_crc8(&self) -> u8 {
        let mut crc = CRCu8::create_crc(0x07, 8, 0, 0, false); // specify the CRC-8 polynomial
        crc.digest(&[self.mode]);
        crc.digest(&[self.lift]);
        crc.digest(&[self.yaw]);
        crc.digest(&[self.pitch]);
        crc.digest(&[self.roll]);
        crc.digest(&[self.p]);
        crc.digest(&[self.p1]);
        crc.digest(&[self.p2]);
        crc.get_crc()
    }

    pub fn set_start_flag(&mut self, start_flag: u8) {
        self.start_flag = start_flag;
    }

    pub fn set_mode(&mut self, mode: u8) {
        self.mode = mode;
    }

    pub fn set_lift(&mut self, lift: u8) {
        self.lift = lift;
    }

    pub fn set_yaw(&mut self, yaw: u8) {
        self.yaw = yaw;
    }

    pub fn set_pitch(&mut self, pitch: u8) {
        self.pitch = pitch;
    }

    pub fn set_roll(&mut self, roll: u8) {
        self.roll = roll;
    }

    pub fn set_p1(&mut self, p1: u8) {
        self.p1 = p1;
    }

    pub fn set_p2(&mut self, p2: u8) {
        self.p2 = p2;
    }

    pub fn set_p(&mut self, p: u8) {
        self.p = p;
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

    pub fn get_lift(&self) -> u8 {
        self.lift
    }

    pub fn get_yaw(&self) -> u8 {
        self.yaw
    }

    pub fn get_pitch(&self) -> u8 {
        self.pitch
    }

    pub fn get_roll(&self) -> u8 {
        self.roll
    }

    pub fn get_p1(&self) -> u8 {
        self.p1
    }

    pub fn get_p2(&self) -> u8 {
        self.p2
    }

    pub fn get_p(&self) -> u8 {
        self.p
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
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        mode: u8,
        duration: u16,
        motor: [u16; 4],
        ypr: [I16F16; 3],
        ypr_filter: [I16F16; 3],
        acc: [i16; 3],
        bat: u16,
        pres: i32,
        ack: u8,
    ) -> Self {
        Self {
            start_flag: 0x7b,
            duration,
            mode,
            motor,
            ypr,
            ypr_filter,
            acc,
            bat,
            pres,
            ack,
            crc: 0x0000, // This is the default value of the CRC, it will be calculated later. If the CRC is 0x0000, it means that the CRC has not been calculated yet.
            end_flag: 0x7d,
        }
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
        message.extend_from_slice(&self.ypr_filter[0].to_be_bytes());
        message.extend_from_slice(&self.ypr_filter[1].to_be_bytes());
        message.extend_from_slice(&self.ypr_filter[2].to_be_bytes());
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

    pub fn format_message(message: &mut [u8]) -> DeviceProtocol {
        let mut format_message =
            DeviceProtocol::new(0, 0, [0; 4], [I16F16::from_num(0); 3], [I16F16::from_num(0); 3], [0; 3], 0, 0, 0);
        format_message.set_start_flag(message[0]);
        format_message.set_mode(message[1]);
        format_message.set_duration(u16::from_be_bytes([message[2], message[3]]));
        format_message.set_motor([
            u16::from_be_bytes([message[4], message[5]]),
            u16::from_be_bytes([message[6], message[7]]),
            u16::from_be_bytes([message[8], message[9]]),
            u16::from_be_bytes([message[10], message[11]]),
        ]);
        format_message.set_ypr([
            I16F16::from_be_bytes([message[12], message[13], message[14], message[15]]),
            I16F16::from_be_bytes([message[16], message[17], message[18], message[19]]),
            I16F16::from_be_bytes([message[20], message[21], message[22], message[23]]),
        ]);
        format_message.set_ypr_filter([
            I16F16::from_be_bytes([message[24], message[25], message[26], message[27]]),
            I16F16::from_be_bytes([message[28], message[29], message[30], message[31]]),
            I16F16::from_be_bytes([message[32], message[33], message[34], message[35]]),
        ]);
        format_message.set_acc([
            i16::from_be_bytes([message[36], message[37]]),
            i16::from_be_bytes([message[38], message[39]]),
            i16::from_be_bytes([message[40], message[41]]),
        ]);
        format_message.set_bat(u16::from_be_bytes([message[42], message[43]]));
        format_message.set_pres(i32::from_be_bytes([
            message[44],
            message[45],
            message[46],
            message[47],
        ]));
        format_message.set_ack(message[48]);
        format_message.set_crc(u16::from_be_bytes([message[49], message[50]]));
        format_message.set_end_flag(message[51]);
        format_message
    }

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
        for ypr_filter in self.ypr_filter.iter() {
            state.update(&ypr_filter.to_be_bytes());
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
        for ypr_filter in self.ypr_filter.iter() {
            crc.digest(&ypr_filter.to_be_bytes());
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

    pub fn set_ypr(&mut self, ypr: [I16F16; 3]) {
        self.ypr = ypr;
    }

    pub fn set_ypr_filter(&mut self, ypr: [I16F16; 3]) {
        self.ypr_filter = ypr;
    }

    pub fn set_acc(&mut self, acc: [i16; 3]) {
        self.acc = acc;
    }

    pub fn set_bat(&mut self, bat: u16) {
        self.bat = bat;
    }

    pub fn set_pres(&mut self, pres: i32) {
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

    pub fn get_ypr(&self) -> [I16F16; 3] {
        self.ypr
    }

    pub fn get_ypr_filter(&self) -> [I16F16; 3] {
        self.ypr_filter
    }

    pub fn get_acc(&self) -> [i16; 3] {
        self.acc
    }

    pub fn get_bat(&self) -> u16 {
        self.bat
    }

    pub fn get_pres(&self) -> i32 {
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
