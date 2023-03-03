use fixed_trigonometry::atan;
use tudelft_quadrupel::{
    fixed::{traits::Fixed, types::I16F16, types::I6F26},
    mpu::structs::Quaternion,
};

// A struct to hold yaw, pitch, and roll values
#[derive(Debug, Copy, Clone)]
pub struct YawPitchRoll {
    pub yaw: I6F26,
    pub pitch: I6F26,
    pub roll: I6F26,
}

// An implementation of the `From` trait for converting a `Quaternion` into a `YawPitchRoll`
impl From<Quaternion> for YawPitchRoll {
    fn from(q: Quaternion) -> Self {
        // Extract the components of the quaternion
        let Quaternion { w, x, y, z } = q;
        // 12.0000

        // Calculate the values for yaw, pitch, and roll using the quaternion components
        let two = 2;

        // // calculate values and format to FixedI32<types::extra::U29> format
        let gx = I6F26::from_num(two * x * z - w * y);
        let gy = I6F26::from_num(two * w * x + y * z);
        let gz = I6F26::from_num(w * w - x * x - y * y + z * z);

        // Calculate the values for yaw, pitch, and roll using the Cordic library
        let yaw = atan::atan2(gx, gz);
        let pitch = atan::atan2(gy, gz);
        let roll = atan::atan2(gx, gy);

        // Create a new `YawPitchRoll` struct with the calculated yaw, pitch, and roll values
        Self { yaw, pitch, roll }
    }
}
