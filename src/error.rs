//! Error definition for the crate

#[derive(Debug)]
pub enum BusError<SPIError, CSError> {
    Spi(SPIError),
    Pin(CSError),
}

#[derive(Debug)]
pub enum CalibrationError {
    Alpha,
    Beta,
    Delta,
}

#[derive(Debug)]
pub enum Error<E> {
    /// SPI bus error
    Bus(E),
    /// Error when calculating new calibration values
    Calibration(CalibrationError),
    /// Delay error
    Delay,
}

impl<SPIError, CSError> From<CSError> for Error<BusError<SPIError, CSError>> {
    fn from(e: CSError) -> Self {
        Self::Bus(BusError::Pin(e))
    }
}