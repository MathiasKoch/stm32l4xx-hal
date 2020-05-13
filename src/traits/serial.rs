/// Enable changing the baudrate after initiation of serial interface
pub trait ConfigureBaud {
    /// Baudrate type
    type BaudRate;
    /// Set baud error
    type Error;
    /// Change baudrate
    fn set_baud_rate(&mut self, baud_rate: Self::BaudRate) -> Result<(), Self::Error>;
}