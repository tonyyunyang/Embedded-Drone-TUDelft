// This file serves as the implementation of a blackbox logging system for the dronecode project.
// This implementation takes into account the limited storage space (256kB flash) and the data communication protocol (UART) used by the drone.

// Things to consider for optimization:
// 1. data compression (i.e. use a compression algorithm to compress the data)
// 2. data sampling so that the data is not too dense (i.e. only log data every x seconds)

// The logger is implemented as a state machine with the following states:
// 1. IDLE: the logger is not logging data
// 2. LOGGING: the logger is logging data
// 3. ERROR: the logger has encountered an error and is not logging data

// The logger is implemented as a state machine with the following transitions:
// 1. IDLE -> LOGGING: the logger is started
// 2. LOGGING -> IDLE: the logger is stopped
// 3. LOGGING -> ERROR: the logger has encountered an error and is not logging data
// 4. ERROR -> IDLE: the logger is reset

// The following information is logged in the blackbox:
// 1. timestamp
// 2. accelerometer data
// 3. motor intensity data / power of the motors
// 4. yaw pitch roll data
// 5. battery data
// 6. pressure data
// 7. CPU usage data
// 8. RAM usage data
// 9. Flash usage data
// 10. mode data
// 11. gyro data

// The data will be stored as a postcard encoded byte array.

// Import necessary crates
use logger_storage::PageBasedLogger;
use postcard::to_slice_cobs;
use serde::{Deserialize, Serialize};

mod logger_storage;

const PAGE_SIZE: usize = 256;
const FLASH_START: u32 = 0x000000;
const FLASH_END: u32 = 0x01FFFE;

// Define the states of the logger
#[derive(Clone, Copy)]
enum LoggerState {
    Idle,
    Logging,
    Error,
}

// Define the struct that represents the logger
pub(crate) struct BlackBoxLogger {
    state: LoggerState,
    storer: PageBasedLogger,
    // Add any additional necessary fields
}

impl BlackBoxLogger {
    // Define a constructor for the logger
    pub fn new() -> Self {
        BlackBoxLogger {
            state: LoggerState::Idle,
            storer: PageBasedLogger::new(FLASH_START, FLASH_END, PAGE_SIZE),
            // Initialize any additional fields here
        }
    }

    // Define a function to start logging data
    pub fn start_logging(&mut self) {
        match self.state {
            LoggerState::Idle => {
                self.state = LoggerState::Logging;
            }
            LoggerState::Logging => {
                // Do nothing, already logging
            }
            LoggerState::Error => {
                // Do nothing, logger is in error state
            }
        }
    }

    // Define a function to stop logging data
    pub fn stop_logging(&mut self) {
        match self.state {
            LoggerState::Idle => {
                // Do nothing, already idle
            }
            LoggerState::Logging => {
                self.state = LoggerState::Idle;
            }
            LoggerState::Error => {
                // Do nothing, logger is in error state
            }
        }
    }

    // Define a function to handle errors in logging
    pub fn handle_error(&mut self) {
        match self.state {
            LoggerState::Idle => {
                // Do nothing, logger is already idle
            }
            LoggerState::Logging => {
                self.state = LoggerState::Error;
            }
            LoggerState::Error => {
                // Do nothing, already in error state
            }
        }
    }

    // Define a function to reset the logger
    pub fn reset(&mut self) {
        self.storer.reset();
        self.state = LoggerState::Idle;
    }

    // Define a function to log data
    pub fn log_data<T: Serialize>(&mut self, data: &DroneLogData) {
        let mut buffer = [0u8; 256];
        match self.state {
            LoggerState::Idle => {
                // Do nothing, logger is idle
            }
            LoggerState::Logging => {
                // Encode the data
                let result = to_slice_cobs(&data, &mut buffer);
                match result {
                    Ok(encoded_data) => {
                        // Write the data to the flash
                        self.storer.write(encoded_data);
                    }
                    Err(_) => {
                        // Do nothing, error encoding data
                    }
                }
            }
            LoggerState::Error => {
                // Do nothing, logger is in error state
            }
        }
    }

    // Optional when we find to have too much data on the flash and we have enough computational power/time to compress the data
    fn compress_data(&self, data: &[u8]) -> &[u8] {
        todo!()
    }
}

// Define the struct that represents the drone data to be logged
#[derive(Serialize, Deserialize)]
pub struct DroneLogData {
    timestamp: u64,
    accel: [i16; 3],
    motor: [u16; 4],
    ypr: [i32; 3],
    bat: u16,
    pres: u32,
    cpu_usage: u8,
    ram_usage: u8,
    flash_usage: u8,
    mode: u8,
    gyro: [i16; 3],
}

impl DroneLogData {
    pub fn new(
        timestamp: u64,
        accel: [i16; 3],
        motor: [u16; 4],
        ypr: [i32; 3],
        bat: u16,
        pres: u32,
        cpu_usage: u8,
        ram_usage: u8,
        flash_usage: u8,
        mode: u8,
        gyro: [i16; 3],
    ) -> Self {
        DroneLogData {
            timestamp,
            accel,
            motor,
            ypr,
            bat,
            pres,
            cpu_usage,
            ram_usage,
            flash_usage,
            mode,
            gyro,
        }
    }
}
