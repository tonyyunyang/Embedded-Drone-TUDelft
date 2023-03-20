use fixed_trigonometry::{atan, sqrt};
use tudelft_quadrupel::{fixed::types::I16F16, mpu::structs::Quaternion};

// A struct to hold yaw, pitch, and roll values
#[derive(Debug, Copy, Clone)]
pub struct YawPitchRoll {
    pub yaw: I16F16,
    pub pitch: I16F16,
    pub roll: I16F16,
}

// An implementation of the `From` trait for converting a `Quaternion` into a `YawPitchRoll`
impl From<Quaternion> for YawPitchRoll {
    fn from(q: Quaternion) -> Self {
        // Extract the components of the quaternion
        let Quaternion { w, x, y, z } = q;
        // 12.0000

        // Calculate the values for yaw, pitch, and roll using the quaternion components
        let w: I16F16 = w.to_num();
        let x: I16F16 = x.to_num();
        let y: I16F16 = y.to_num();
        let z: I16F16 = z.to_num();
        // let w: f32 = w.to_num();
        // let x: f32 = x.to_num();
        // let y: f32 = y.to_num();
        // let z: f32 = z.to_num();

        let two = I16F16::from_num(2.0);
        let one = I16F16::from_num(1.0);

        let gx = two * (x * z - w * y);
        let gy = two * (w * x + y * z);
        let gz = w * w - x * x - y * y + z * z;

        // Calculate the values for yaw, pitch, and roll using the Cordic library
        // let yaw = I16F16::from_num(micromath::F32Ext::atan2(2.0 * x * y - 2.0 * w * z, 2.0 * w * w + 2.0 * x * x - 1.0));
        // let pitch = I16F16::from_num(micromath::F32Ext::atan2(gx, micromath::F32Ext::sqrt(gy * gy + gz * gz)));
        // let roll = I16F16::from_num(micromath::F32Ext::atan2(gy, gz));
        let yaw = atan::atan2(two * x * y - two * w * z, two * w * w + two * x * x - one);
        let pitch = atan::atan2(gx, sqrt::niirf(gy * gy + gz * gz, 2));
        let roll = atan::atan2(gy, gz);

        // Create a new `YawPitchRoll` struct with the calculated yaw, pitch, and roll values
        Self { yaw, pitch, roll }
    }
}
