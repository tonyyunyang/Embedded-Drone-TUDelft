use tudelft_quadrupel::{
    fixed::{types, FixedI32},
    mpu::structs::Quaternion,
};

// A struct to hold yaw, pitch, and roll values
#[derive(Debug, Copy, Clone)]
pub struct YawPitchRoll {
    pub yaw: FixedI32<types::extra::U29>,
    pub pitch: FixedI32<types::extra::U29>,
    pub roll: FixedI32<types::extra::U29>,
}

// An implementation of the `From` trait for converting a `Quaternion` into a `YawPitchRoll`
impl From<Quaternion> for YawPitchRoll {
    fn from(q: Quaternion) -> Self {
        // Extract the components of the quaternion
        let Quaternion { w, x, y, z } = q;

        // Calculate the values for yaw, pitch, and roll using the quaternion components
        let two = 2i32;

        // calculate values and format to FixedI32<types::extra::U29> format
        let gx = FixedI32::<types::extra::U29>::from_num(two * x * z - w * y);
        let gy = FixedI32::<types::extra::U29>::from_num(two * w * x + y * z);
        let gz = FixedI32::<types::extra::U29>::from_num(w * w - x * x - y * y + z * z);

        // Calculate the values for yaw, pitch, and roll using the Cordic library
        let yaw = cordic::atan2::<FixedI32<types::extra::U29>>(gx, gz);
        let pitch = cordic::atan2::<FixedI32<types::extra::U29>>(gy, gz);
        let roll = cordic::atan2::<FixedI32<types::extra::U29>>(gx, gy);

        // Create a new `YawPitchRoll` struct with the calculated yaw, pitch, and roll values
        Self { yaw, pitch, roll }
    }
}
