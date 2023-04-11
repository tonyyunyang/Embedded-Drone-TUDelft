use csv::Writer;
use std::fs::OpenOptions;
use std::io::BufWriter;

/// `FileWriter` is a struct that simplifies the process of writing CSV records to a file.
///
/// It uses the `csv` crate and automatically handles file creation, appending, and buffering.
///
/// # Example
///
/// ```
/// use your_module::FileWriter;
///
/// let mut file_writer = FileWriter::new("output.csv").unwrap();
/// let record = vec!["field1", "field2", "field3"];
///
/// file_writer.write_record(&record).unwrap();
/// ```
pub struct FileWriter {
    file: BufWriter<std::fs::File>,
}

/// The implementation of methods for `FileWriter`.
impl FileWriter {
    /// Creates a new `FileWriter` instance for the specified file path.
    ///
    /// If the file does not exist, it will be created. If it exists, new records will be appended.
    ///
    /// # Arguments
    ///
    /// * `file_path` - The path to the file where the CSV records will be written.
    ///
    /// # Errors
    ///
    /// Returns an `std::io::Error` if there is an issue opening or creating the file.
    pub fn new(file_path: &str) -> Result<Self, std::io::Error> {
        let file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(file_path)?;

        let file = BufWriter::new(file);

        Ok(Self { file })
    }

    /// Writes a single CSV record to the file.
    ///
    /// # Arguments
    ///
    /// * `record` - An iterator over items that can be converted into byte slices (e.g., `&str` or `String`).
    ///
    /// # Errors
    ///
    /// Returns a `csv::Error` if there is an issue writing the record or flushing the buffer.
    pub fn write_record<I, T>(&mut self, record: I) -> Result<(), csv::Error>
    where
        I: IntoIterator<Item = T>,
        T: AsRef<[u8]>,
    {
        let mut writer = Writer::from_writer(&mut self.file);
        writer.write_record(record)?;
        writer.flush()?;

        Ok(())
    }
}
