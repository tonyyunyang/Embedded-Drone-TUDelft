use tudelft_quadrupel::flash::{flash_chip_erase, flash_write_bytes};
use tudelft_quadrupel::ringbuffer::{
    AllocRingBuffer, RingBuffer, RingBufferExt, RingBufferRead, RingBufferWrite,
};

const MAX_PAGE_SIZE: usize = 256;
const PAGE_MASK: usize = MAX_PAGE_SIZE - 1;

pub(crate) struct PageBasedLogger {
    // start_address is the first address of the flash memory
    start_address: u32,
    // end_address is the last address of the flash memory
    end_address: u32,
    // page_size is the number of bytes in a page with a maximum of 256 bytes
    page_size: usize,
    // buffer is the ring buffer that stores the data to be written to the flash memory
    buffer: AllocRingBuffer<u8>,
}

impl PageBasedLogger {
    /// Creates a new PageBasedLogger
    /// start_address is the first address of the flash memory
    /// end_address is the last address of the flash memory
    /// page_size is the number of bytes in a page
    pub fn new(start_address: u32, end_address: u32, page_size: usize) -> Self {
        // erase the flash memory fully
        match flash_chip_erase() {
            Ok(_) => {
                // This temporarily I am not sure if I want to keep this function here
            }
            Err(_) => {
                todo!("Handle error");
                // Handle error
            }
        }

        // size is the number of bytes in the flash memory
        let size = (end_address - start_address + 1) as usize;
        // buffer is the ring buffer that stores the data to be written to the flash memory
        let buffer = AllocRingBuffer::with_capacity(size);
        PageBasedLogger {
            start_address,
            end_address,
            page_size,
            buffer,
        }
    }

    /// Writes data to the flash memory
    /// data is the slice of data to be written to the flash memory
    /// The data is written to the flash memory in pages
    pub fn write(&mut self, data: &[u8]) {
        // remaining_data is the slice of the data that has not been written to the flash memory yet
        let mut remaining_data = data;

        // while there is still data to be written to the flash memory write it to the buffer`
        while !remaining_data.is_empty() {
            // write size tells how many bytes can be written to the buffer
            let write_size = self.page_size - (self.buffer.len() & PAGE_MASK);

            // write data is the slice of the remaining data with the calculated write size
            let write_data = &remaining_data[..write_size];

            // write the data to the buffer
            self.enqueue_slice(write_data);

            remaining_data = &remaining_data[write_size..];
            if self.buffer.len() & PAGE_MASK == 0 {
                self.flush();
            }
        }
    }

    /// Flushes the buffer to the flash memory by writing the data in the buffer to the flash memory
    /// The data is written to the flash memory in pages and the pages are erased before writing the data
    /// The buffer is emptied after the data is written to the flash memory
    fn flush(&mut self) {
        // address is the address of the first byte of the page to be written to the flash memory
        let mut address = self.start_address
            + (self.buffer.len() / self.page_size) as u32 * self.page_size as u32;

        // buffer_data is the array that stores the data to be written to the flash memory
        let mut buffer_data = [0; MAX_PAGE_SIZE];

        // num_bytes is the number of bytes that are written to the buffer_data array
        // The function obtains the data from the buffer and writes it to the buffer_data array
        let _num_bytes =
            self.dequeue_slice(self.buffer.len(), &mut buffer_data[..self.buffer.len()]);

        // write the data to the flash memory in pages
        for chunk in buffer_data.chunks_exact(self.page_size) {
            // self.flash_erase(address);

            // write the data to the flash memory
            self.flash_write(address, chunk);
            // increment the address by the page size
            address += self.page_size as u32;
        }
    }

    // fn flash_erase(&mut self, address: u32) {
    //     // For now we erase the whole flash memory and call it at creating a new logger
    //     // flash_chip_erase();
    // }

    pub fn reset(&mut self) {
        // erase the flash memory fully
        match flash_chip_erase() {
            Ok(_) => {
                // reset the buffer
                self.buffer.clear();
            }
            Err(_) => {
                todo!("Handle error");
                // TODO: handle error
            }
        }
    }

    fn flash_write(&mut self, address: u32, data: &[u8]) {
        match flash_write_bytes(address, data) {
            Ok(_) => {
                // Everything is fine
            }
            Err(_) => {
                // TODO: handle error
            }
        }
    }

    fn enqueue_slice(&mut self, data: &[u8]) {
        data.iter().for_each(|byte| {
            self.buffer.enqueue(*byte);
        });
    }

    fn dequeue_slice(&mut self, size: usize, output: &mut [u8]) -> usize {
        let mut i = 0;
        while i < size {
            if let Some(byte) = self.buffer.dequeue() {
                output[i] = byte;
                i += 1;
            } else {
                break;
            }
        }
        i
    }
}
