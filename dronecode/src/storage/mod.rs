use tudelft_quadrupel::flash::{flash_chip_erase, flash_read_bytes, flash_write_bytes, FlashError};

/// A storage structure that manages reading and writing data to flash memory.
///
/// # Fields
///
/// * `start_address` - A u32 representing the starting address of the flash memory
/// * `end_address` - A u32 representing the ending address of the flash memory
/// * `current_address` - A u32 representing the current write address in the flash memory
/// * `read_address` - A u32 representing the current read address in the flash memory

pub struct Storage {
    start_address: u32,
    end_address: u32,
    current_address: u32,
    read_address: u32,
}

impl Storage {
    /// Creates a new Storage instance with the specified start and end addresses.
    ///
    /// # Arguments
    ///
    /// * `start_address` - A u32 representing the starting address of the flash memory
    /// * `end_address` - A u32 representing the ending address of the flash memory
    ///
    /// # Returns
    ///
    /// * A Storage instance

    pub fn new(start_address: u32, end_address: u32) -> Self {
        Storage {
            start_address,
            end_address,
            current_address: start_address,
            read_address: start_address,
        }
    }

    /// Erases the entire flash memory.
    ///
    /// # Returns
    ///
    /// * A Result indicating success or failure (FlashError)

    pub fn erase_flash(&mut self) -> Result<(), FlashError> {
        flash_chip_erase()
    }

    /// Writes data to the flash memory at the current address.
    ///
    /// # Arguments
    ///
    /// * `data` - A byte slice containing the data to be written
    ///
    /// # Returns
    ///
    /// * A Result indicating success or failure (FlashError)

    pub fn write(&mut self, data: &[u8]) -> Result<(), FlashError> {
        let length = data.len();

        // Check if the new address will overflow the end address,
        // and erase the flash memory if necessary.
        let new_address = self.current_address + length as u32;
        if new_address > self.end_address {
            self.erase_flash()?;
            self.current_address = self.start_address;
        }

        flash_write_bytes(self.current_address, data)?;
        self.update_current_address(length)?;

        Ok(())
    }

    /// Reads data from the flash memory at the current read address into the provided buffer.
    ///
    /// # Arguments
    ///
    /// * `buffer` - A mutable byte slice to store the read data
    ///
    /// # Returns
    ///
    /// * A Result containing the number of bytes read, or a FlashError on failure

    pub fn read(&mut self, buffer: &mut [u8]) -> Result<usize, FlashError> {
        let length = buffer.len();
        flash_read_bytes(self.read_address, buffer)?;
        self.update_read_address(length)?;

        Ok(length)
    }

    // Update the current address
    fn update_current_address(&mut self, length: usize) -> Result<(), FlashError> {
        let new_address = self.current_address + length as u32;

        if new_address > self.end_address {
            self.current_address = self.start_address;
        } else {
            self.current_address = new_address;
        }

        Ok(())
    }

    // Update the read address
    fn update_read_address(&mut self, length: usize) -> Result<(), FlashError> {
        let new_address = self.read_address + length as u32;

        if new_address > self.end_address {
            self.read_address = self.start_address;
        } else {
            self.read_address = new_address;
        }

        Ok(())
    }
}
