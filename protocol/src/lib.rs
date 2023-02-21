#![cfg_attr(not(test), no_std)]
extern crate alloc;
#[cfg(test)]
extern crate std;

mod format; // this is to load the data_format.rs file and the structs in it

#[cfg(test)]
mod tests {}
