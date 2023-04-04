use csv::Writer;
use std::fs::OpenOptions;
use std::io::BufWriter;

pub struct FileWriter {
    file: BufWriter<std::fs::File>,
}

impl FileWriter {
    pub fn new(file_path: &str) -> Result<Self, std::io::Error> {
        let file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(file_path)?;

        let file = BufWriter::new(file);

        Ok(Self { file })
    }

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
