//! Quad Serial Peripheral Interface (QSPI) bus

use crate::stm32::QUADSPI;
use crate::rcc::AHB3;
use crate::gpio::gpioe::{PE10, PE11, PE12, PE13, PE14, PE15};
use crate::gpio::{Alternate, Floating, Input, AF10};
use core::ptr;


#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum QspiMode{
    SingleChannel = 0b01,
    DualChannel = 0b10,
    QuadChannel = 0b11,
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum AddressSize{
    Addr8Bit = 0b00,
    Addr16Bit = 0b01,
    Addr24Bit = 0b10,
    Addr32Bit = 0b11,
}

#[derive(Copy, Clone, Debug, PartialEq)]
// #[repr(bool)]
pub enum SampleShift{
    None,
    HalfACycle,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum ClockMode {
    Mode0,
    Mode3,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct QspiConfig{
    /// This field defines the scaler factor for generating CLK based on the AHB clock
    /// (value+1).
    clock_prescaler : u8,
    /// Number of bytes in Flash memory = 2^[FSIZE+1]
    flash_size : u8,
    address_size : AddressSize,
    /// This bit indicates the level that CLK takes between commands Mode 0(low) / mode 3(high)
    clock_mode : ClockMode,
    /// FIFO threshold level (Activates FTF, QUADSPI_SR[2]) 0-15.
    fifo_threshold : u8,
    /// Single, dual og quad mode
    qspi_mode : QspiMode,
    sample_shift : SampleShift,
    /// CSHT+1 defines the minimum number of CLK cycles which the chip select (nCS) must
    /// remain high between commands issued to the Flash memory.
    chip_select_high_time: u8,
    double_data_rate : bool,
    qpi_mode : bool,
}

impl Default for QspiConfig{
    fn default() -> QspiConfig {
        QspiConfig {
            clock_prescaler : 0,
            flash_size : 22, //8MB //26 = 128MB
            address_size : AddressSize::Addr24Bit,
            clock_mode : ClockMode::Mode0,
            fifo_threshold : 1,
            qspi_mode : QspiMode::QuadChannel,
            sample_shift : SampleShift::HalfACycle,
            chip_select_high_time : 1,
            double_data_rate : false,
            qpi_mode : false,
        }
    }
}

impl QspiConfig {
    pub fn clock_prescaler(mut self, clk_pre: u8) -> Self {
        self.clock_prescaler = clk_pre;
        self
    }

    pub fn flash_size(mut self, fl_size: u8) -> Self {
        self.flash_size = fl_size;
        self
    }

    pub fn address_size(mut self, add_size: AddressSize) -> Self {
        self.address_size = add_size;
        self
    }

    pub fn clock_mode(mut self, clk_mode: ClockMode) -> Self {
        self.clock_mode = clk_mode;
        self 
    }

    pub fn fifo_threshold(mut self, fifo_thres: u8) -> Self {
        self.fifo_threshold = fifo_thres;
        self 
    }

    pub fn qspi_mode(mut self, qspi: QspiMode) -> Self {
        self.qspi_mode = qspi;
        self 
    }

    pub fn sample_shift(mut self, shift: SampleShift) -> Self {
        self.sample_shift = shift;
        self 
    }

    pub fn chip_select_high_time(mut self, csht: u8) -> Self {
        self.chip_select_high_time = csht;
        self 
    }

    pub fn double_data_rate(mut self, ddr: bool) -> Self {
        self.double_data_rate = ddr;
        self 
    }

    pub fn qpi_mode(mut self, qpi: bool) -> Self {
        self.qpi_mode = qpi;
        self 
    }
}

pub trait Pins<QSPI>{

}

impl Pins<QUADSPI> for (
    PE10<Alternate<AF10, Input<Floating>>>,
    PE11<Alternate<AF10, Input<Floating>>>,
    PE12<Alternate<AF10, Input<Floating>>>,
    PE13<Alternate<AF10, Input<Floating>>>,
    PE14<Alternate<AF10, Input<Floating>>>, 
    PE15<Alternate<AF10, Input<Floating>>>){}

pub struct QspiWriteCommand<'c>{
    pub instruction : Option<u8>,
    pub address : Option<u32>,
    pub alternative_bytes : Option<&'c[u8]>,
    pub dummy_cycles : u8,
    pub data: Option<&'c[u8]>,
}

pub struct QspiReadCommand<'c>{
    pub instruction : Option<u8>,
    pub address : Option<u32>,
    pub alternative_bytes : Option<&'c[u8]>,
    pub dummy_cycles : u8,
    pub recive_lenght : u32,
}


pub struct Qspi<PINS> {
    qspi: QUADSPI,
    pins : PINS,
    config: QspiConfig,
}

impl <PINS> Qspi <PINS> {
    pub fn new(qspi: QUADSPI, pins : PINS, ahb3 : &mut AHB3, config : QspiConfig) -> Self 
        where PINS :Pins<QUADSPI> {
        // Enable quad SPI in the clocks.
        ahb3.enr().modify(|_,w| w.qspien().bit(true));

        // Disable QUADSPI before configuring it.
        qspi.cr.modify(|_, w| {
            w.en().clear_bit()
        });

        // Clear all pending flags.
        qspi.fcr.write(|w| {
            w
            .ctof().set_bit()
            .csmf().set_bit()
            .ctcf().set_bit()
            .ctef().set_bit()
        });

        let mut unit = Qspi{qspi, pins, config};
        unit.apply_config(config);
        unit
    }

    pub fn is_busy(&self) -> bool {
        self.qspi.sr.read().busy().bit_is_set()
    }

    pub fn apply_config(&mut self, config : QspiConfig) {
        if self.qspi.sr.read().busy().bit_is_set() {
            //Todo: Handle error
            // return Err(QspiError::Busy);
            #[cfg(feature = "logging")]
            log::debug!{"Busy..."};
        }

        self.qspi.cr.modify(|_, w| unsafe {
                w.fthres().bits(config.fifo_threshold as u8)

        });

        while self.qspi.sr.read().busy().bit_is_set(){
            #[cfg(feature = "logging")]
            log::debug!{"Busy waiting"};
        }
        
        // modify the prescaler and select flash bank 2 - flash bank 1 is currently unsupported.
        self.qspi.cr.modify(|_, w| unsafe {
            w.prescaler().bits(config.clock_prescaler as u8)
                .sshift().bit(config.sample_shift == SampleShift::HalfACycle)
        });

        //Modify DCR with flash size, CSHT and clock mode
        self.qspi.dcr.modify(|_, w| unsafe{
            w.fsize().bits(config.flash_size as u8)
            .csht().bits(config.chip_select_high_time as u8)
            .ckmode().bit(config.clock_mode == ClockMode::Mode3)
        });

        //Enable SPI
        self.qspi.cr.modify(|_, w| unsafe {
            w.en().set_bit()
        });

        self.config = config;
    }

    pub fn transfer(& self, command : QspiReadCommand, buffer : &mut [u8]) {
        if self.is_busy() {
            //Todo handle error
            // return Err(QspiError::Busy);
            #[cfg(feature = "logging")]
            log::debug!{"Busy..."};
        }
        // Clear the transfer complete flag.
        self.qspi.fcr.modify(|_ ,w| w.ctcf().set_bit());

        let mut dmode : u8 = 0;
        let mut instruction : u8 = 0;
        let mut imode : u8 = 0;
        let mut admode : u8 = 0;
        let mut adsize : u8 = 0;
        let mut abmode : u8 = 0;
        let mut absize : u8 = 0;

        // Write the length and format of data  
        if command.recive_lenght > 0 {
            self.qspi.dlr.write(|w| unsafe {w.dl().bits(command.recive_lenght as u32 - 1)});
            if self.config.qpi_mode{
                dmode = self.config.qspi_mode as u8;
            } else {
                dmode = 0b01;
            }
        }
       
        #[cfg(feature = "logging")]
        log::debug!{"DLR bit :{:?}", self.qspi.dlr.read().dl().bits()};
        
        //Instruction mode
        if let Some(_) = command.instruction {
            if self.config.qpi_mode {
                imode = self.config.qspi_mode as u8;
                #[cfg(feature = "logging")]
                log::debug!("Instruction qpi mode");
            } else {
                imode = 0b01;
                #[cfg(feature = "logging")]
                log::debug!("Instruction single mode");
            }
        }

        // Note Address mode
        if let Some(_) = command.address {
            admode = self.config.qspi_mode as u8;
            adsize = self.config.address_size as u8;
        }
        
        // Write Alternative bytes
        if let Some(a_bytes) = command.alternative_bytes {
            abmode = self.config.qspi_mode as u8;
            absize = a_bytes.len() as u8 - 1;

            self.qspi.abr.write(|w| {
                let mut i = 0;
                let mut reg_byte: u32 = 0;
                for element in a_bytes.iter().rev(){
                    reg_byte = reg_byte | ((*element as u32) << i*8);
                    i += 1;
                }
                unsafe {
                    w.alternate().bits(reg_byte)
                }
            });
        }
        
        //Note instruction, triggers send if no address required
        if let Some(inst) = command.instruction {
            #[cfg(feature = "logging")]
            log::debug!("Instruction: {:x}", inst);
            instruction = inst;
        }
        #[cfg(feature = "logging")]
        log::debug!("0 - Tcf bit :{:?}\tCCR:{:x}\tCR:{:x}\tDCR:{:x}\tSR:{:x}", 
            self.qspi.sr.read().tcf().bit_is_set(), 
            self.qspi.ccr.read().bits(),
            self.qspi.cr.read().bits(),
            self.qspi.dcr.read().bits(),
            self.qspi.sr.read().bits()
        );
        
        //Write CCR register with instruction etc.
        self.qspi.ccr.modify(|_, w|
            unsafe {
                w.fmode().bits(0b01)
                    .admode().bits(admode)
                    .adsize().bits(adsize)
                    .abmode().bits(abmode)
                    .absize().bits(absize)
                    .ddrm().bit(self.config.double_data_rate)
                    .dcyc().bits(command.dummy_cycles)
                    .dmode().bits(dmode)
                    .imode().bits(imode)
                    .instruction().bits(instruction)

            });
 


        #[cfg(feature = "logging")]
        log::debug!("1 - Tcf bit :{:?}\tCCR:{:x}\tCR:{:x}\tDCR:{:x}\tSR:{:x}", 
            self.qspi.sr.read().tcf().bit_is_set(), 
            self.qspi.ccr.read().bits(),
            self.qspi.cr.read().bits(),
            self.qspi.dcr.read().bits(),
            self.qspi.sr.read().bits()
        );

        //Write address, triggers send
        if let Some(addr) = command.address {
            self.qspi.ar.write(|w| unsafe {w.address().bits(addr)});
        }

        #[cfg(feature = "logging")]
        log::debug!("2 - Tcf bit :{:?}\tCCR:{:x}\tCR:{:x}\tDCR:{:x}\tSR:{:x}", 
            self.qspi.sr.read().tcf().bit_is_set(), 
            self.qspi.ccr.read().bits(),
            self.qspi.cr.read().bits(),
            self.qspi.dcr.read().bits(),
            self.qspi.sr.read().bits()
        );

        //Read data from the buffer
        let mut b = buffer.iter_mut();
        while self.qspi.sr.read().tcf().bit_is_clear() {
            if self.qspi.sr.read().ftf().bit_is_set(){
                if let Some(v) = b.next() {
                    unsafe{
                        *v = ptr::read_volatile(&self.qspi.dr as *const _ as *const u8);
                    }
                    #[cfg(feature = "logging")]
                    log::debug!{"Recived: {:x}", *v};
                } else {
                    #[cfg(feature = "logging")]
                    log::debug!{"OVERFLOW!"};
                    // OVERFLOW
                }
                #[cfg(feature = "logging")]
                log::debug!{"ftf bit is set"};
                
            }
            #[cfg(feature = "logging")]
            log::debug!{"Tcf bit is clear"};
        }
        //When transfer complete, empty fifo buffer
        while self.qspi.sr.read().flevel().bits() > 0 {
            #[cfg(feature = "logging")]
            log::debug!{"flevel not zero: {:?}", self.qspi.sr.read().flevel().bits()};
            if let Some(v) = b.next() {
                #[cfg(feature = "logging")]
                log::debug!{"Recived: {:x}", self.qspi.dr.read().bits()};
                // unsafe{
                //     *v = ptr::read_volatile(&self.qspi.dr as *const _ as *const u8);
                // }
                // #[cfg(feature = "logging")]
                // log::debug!{"Recived: {:x}", *v};
            } else {
                #[cfg(feature = "logging")]
                log::debug!{"OVERFLOW!"};
                // OVERFLOW
            }
        }

        // unsafe {
        //     for location in buffer {
        //         *location = ptr::read_volatile(&self.qspi.dr as *const _ as *const u8);
        //     }
        // }

        self.qspi.fcr.write(|w| w.ctcf().set_bit());        
    }
/*
    pub fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), QspiError> {
        if self.is_busy() {
            return Err(QspiError::Busy);
        }

        // Clear the transfer complete flag.
        self.qspi.fcr.modify(|_ ,w| w.ctcf().set_bit());

        // Write the length
        self.qspi.dlr.write(|w| unsafe {w.dl().bits(data.len() as u32 - 1)});

        // Configure the mode to indirect write and configure the instruction byte.
        self.rb.ccr.write(|w| unsafe {
            w.fmode().bits(0b00)
             .instruction().bits(addr)
        });

        // Enable the transaction
        self.rb.cr.write(|w| {w.en().set_bit()});

        // Write data to the FIFO in a byte-wise manner.
        unsafe {
            for byte in data {
                ptr::write_volatile(&self.rb.dr as *const _ as *mut u8, *byte);
            }
        }

        // Wait for the transaction to complete
        while self.rb.sr.read().tcf().bit_is_clear() {}

        // Check that there is no more transaction pending.
        if self.is_busy() {
            return Err(QspiError::FifoData);
        }

        self.rb.cr.write(|w| {w.en().clear_bit()});

        // Clear the transfer complete flag.
        self.rb.fcr.write(|w| w.ctcf().set_bit());

        Ok(())
    }
*/
    pub fn read(&mut self, addr: u8, dest: &mut [u8]) {
        if self.is_busy() {
            // return Err(QspiError::Busy);
        }

        // Clear the transfer complete flag.
        self.qspi.fcr.modify(|_ ,w| w.ctcf().set_bit());

        // Write the length that should be read.
        self.qspi.dlr.write(|w| unsafe {
            w.dl().bits(dest.len() as u32 - 1)
        });

        self.qspi.ccr.modify(|_, w|
            unsafe {
                    w.admode().bits(0)
                    .adsize().bits(0)
                    .abmode().bits(0)
                    .absize().bits(0)
                    .ddrm().bit(false)
                    .dcyc().bits(0)
                    .dmode().bits(0b01)
                    // .dmode().bits(dmode)
                    .imode().bits(01)
            });

        // Configure the mode to indirect read and configure the instruction byte.
        self.qspi.ccr.modify(|_, w| unsafe {
            w.fmode().bits(0b01)
             .instruction().bits(addr)
        });

        // // Enable the transaction
        // self.qspi.cr.modify(|_, w| {w.en().set_bit()});

        // // Write the instruction bits to force the read to start. This has to be done after the
        // // transaction is enabled to indicate to the peripheral that we are ready to start the
        // // transaction, even though these bits should already be set.
        // self.qspi.ccr.modify(|_, w| unsafe {
        //     w.instruction().bits(addr)
        // });

        // Wait for the transaction to complete
        while self.qspi.sr.read().tcf().bit_is_clear() {}

        // Check for underflow on the FIFO.
        if (self.qspi.sr.read().flevel().bits() as usize) < dest.len() {
            // return Err(QspiError::Underflow);
        }

        // Read data from the FIFO in a byte-wise manner.
        unsafe {
            for location in dest {
                *location = ptr::read_volatile(&self.qspi.dr as *const _ as *const u8);
            }
        }

        // Check that there is no more transaction pending.
        if self.is_busy() {
            // return Err(QspiError::FifoData);
        }

        self.qspi.cr.modify(|_, w| {w.en().clear_bit()});

        // Clear the transfer complete flag.
        self.qspi.fcr.modify(|_ ,w| w.ctcf().set_bit());

        // Ok(());
    }

    
}