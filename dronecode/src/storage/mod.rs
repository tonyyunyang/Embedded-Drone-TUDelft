use tudelft_quadrupel::flash::{flash_chip_erase, flash_read_bytes, flash_write_bytes, FlashError};
pub struct Storage {
    start_address: u32,
    end_address: u32,
    current_address: u32,
    read_address: u32,
}

impl Storage {
    pub fn new(start_address: u32, end_address: u32) -> Self {
        Storage {
            start_address,
            end_address,
            current_address: start_address,
            read_address: start_address,
        }
    }

    pub fn erase_flash(&mut self) -> Result<(), FlashError> {
        flash_chip_erase()
    }

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

    pub fn read(&mut self, buffer: &mut [u8]) -> Result<usize, FlashError> {
        let length = buffer.len();
        flash_read_bytes(self.read_address, buffer)?;
        self.update_read_address(length)?;

        Ok(length)
    }

    // pub fn start_address(&self) -> u32 {
    //     self.start_address
    // }

    // pub fn end_address(&self) -> u32 {
    //     self.end_address
    // }

    // pub fn current_address(&self) -> u32 {
    //     self.current_address
    // }

    fn update_current_address(&mut self, length: usize) -> Result<(), FlashError> {
        let new_address = self.current_address + length as u32;

        if new_address > self.end_address {
            self.current_address = self.start_address;
        } else {
            self.current_address = new_address;
        }

        Ok(())
    }

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
