use crate::gpio::gpioa::{PA0, PA1};
use crate::gpio::{Analog, Floating, Input};
use crate::rcc::{AHB2, CCIPR};
use core::marker::PhantomData;
use embedded_hal::adc::{Channel, OneShot};
use crate::pac::{ADC1, ADC3, ADC2};


/// ADC Result Alignment
#[derive(PartialEq)]
// #[repr(bool)]
pub enum Align {
    /// Right aligned results (least significant bits)
    ///
    /// Results in all Resolutions returning values from 0-(2^bits-1) in
    /// steps of 1.
    Right,
    /// Left aligned results (most significant bits)
    ///
    /// Results in all Resolutions returning a value in the range 0-65535.
    /// Depending on the Resolution the result will step by larger or smaller
    /// amounts.
    Left,
}


#[derive(Copy, Clone, PartialEq)]
#[repr(u8)]
pub enum RegularOversampling{
    On = 0b1,
    Off = 0b0
}

#[derive(Copy, Clone, PartialEq)]
#[repr(u8)]
pub enum InjectedOversampling{
    On = 0b1,
    Off = 0b0
}

#[derive(Copy, Clone, PartialEq)]
#[repr(u16)]
pub enum OversamplingRatio{
   /// 2x ADC Oversampling ratio
   X2 = 0b000,
   /// 4x ADC Oversampling ratio
   X4 = 0b001,
   /// 8x ADC Oversampling ratio
   X8 = 0b010,
   /// 16x ADC Oversampling ratio
   X16 = 0b011,
   /// 32x ADC Oversampling ratio
   X32 = 0b100,
   /// 64x ADC Oversampling ratio
   X64 = 0b101,
   /// 128x ADC Oversampling ratio
   X128 = 0b110,
   /// 256x ADC Oversampling ratio
   X256 = 0b111,
}


/// 
#[derive(Copy, Clone, PartialEq)]
#[repr(u16)]
pub enum OversamplingShift{
    /// 0 bit Oversampling shift. Same as divide by 0
   S0 = 0b0000,
   /// 1 bits Oversampling shift. Same as divide by 2
   S1 = 0b0001,
   /// 2 bits Oversampling shift. Same as divide by 4
   S2 = 0b0010,
   /// 3 bits Oversampling shift. Same as divide by 8
   S3 = 0b0011,
   /// 4 bits Oversampling shift. Same as divide by 16
   S4 = 0b0100,
   /// 5 bits Oversampling shift. Same as divide by 32
   S5 = 0b0101,
   /// 6 bits Oversampling shift. Same as divide by 64
   S6 = 0b0110,
   /// 7 bits Oversampling shift. Same as divide by 128 
   S7 = 0b0111,
   /// 8 bits Oversampling shift. Same as divide by 256
   S8 = 0b1000,
}



/// ADC Sampling Resolution
#[derive(Copy, Clone, PartialEq)]
#[repr(u8)]
pub enum Resolution {
    /// 12 bit Resolution
    B12 = 0b00,
    /// 10 bit Resolution
    B10 = 0b01,
    /// 8 bit Resolution
    B8 = 0b10,
    /// 6 bit Resolution
    B6 = 0b11,
}

/// ADC Sampling time
#[derive(Copy, Clone, PartialEq)]
#[repr(u16)]
pub enum SampleTime {
    /// 2.5 ADC clock cycles
    T2_5 = 0b000,
    /// 6.5 ADC clock cycles
    T6_5 = 0b001,
    /// 12.5 ADC clock cycles
    T12_5 = 0b010,
    /// 24.5 ADC clock cycles
    T24_5 = 0b011,
    /// 47.5 ADC clock cycles
    T47_5 = 0b100,
    /// 92.5 ADC clock cycles
    T92_5 = 0b101,
    /// 247.5 ADC clock cycles
    T247_5 = 0b110,
    /// 640.5 ADC clock cycles
    T640_5 = 0b111,
}

pub struct Config {
    pub align: Align,
    pub resolution: Resolution,
    pub sample_time: SampleTime,
    pub reg_oversampl : RegularOversampling,
    pub inj_oversampl : InjectedOversampling,
    pub oversampl_ratio : OversamplingRatio,
    pub oversampl_shift : OversamplingShift,
}

impl Config {
    pub fn align(mut self, align: Align) -> Self {
        self.align = align;
        self
    }

    pub fn resolution(mut self, resolution : Resolution) -> Self {
        self.resolution = resolution;
        self
    }

    pub fn sample_time(mut self, sample_time: SampleTime) -> Self {
        self.sample_time = sample_time;
        self
    }
}

impl Default for Config {
    fn default() -> Config {
        Config {
            align : Align::Right,
            resolution: Resolution::B12,
            sample_time: SampleTime::T2_5,
            reg_oversampl : RegularOversampling::Off,
            inj_oversampl : InjectedOversampling::Off,
            oversampl_ratio : OversamplingRatio::X16,
            oversampl_shift : OversamplingShift::S4,
        }
    }
}




pub struct Adc<ADC1> {
    adc: ADC1,
    config : Config,
}

#[cfg(feature = "stm32l4x5")]
impl Adc<ADC1> {
    pub fn adc1(adc: ADC1, config : Config, ahb2 : &mut AHB2, ccipr : &mut CCIPR) -> Self {
        
        // Select system clock as clock for ADC        
        ccipr.ccipr().modify(|_, w|unsafe { w.adcsel().bits(0b11) } );
        // 00: No clock selected
        // 01: PLLSAI1 “R” clock (PLLADC1CLK) selected as ADCs clock
        // 10: PLLSAI2 “R” clock (PLLADC2CLK) selected as ADCs clock
        // 11: System clock selected as ADCs clock

        //        common.ccr.modify(|_, w| unsafe { w.ckmode().bits(0b11) });
        //        common.ccr.modify(|_, w| unsafe { w.presc().bits(0b1011) });
        ahb2.enr().modify(|r, w| w.adcen().set_bit());
        //        common.ccr.modify(|_, w| w.vrefen().set_bit());

        // Disable deep power down and start ADC voltage regulator
        adc.cr.modify(|_, w| w.deeppwd().clear_bit());
        adc.cr.modify(|_, w| w.advregen().set_bit());
        cortex_m::asm::delay(8_000_000);

        // Calibrate
        adc.cr.modify(|_, w| w.adcaldif().clear_bit());
        adc.cr.modify(|_, w| w.adcal().set_bit());

        while adc.cr.read().adcal().bit_is_set() {}
        
        let adc = Self { adc, config};
        // adc.applyConfig();
        adc
    }

    fn power_up(&mut self) {
        self.adc.isr.modify(|_, w| w.adrdy().set_bit());
        self.adc.cr.modify(|_, w| w.aden().set_bit());
        while self.adc.isr.read().adrdy().bit_is_clear() {}
    }

    fn power_down(&mut self) {
        self.adc.cr.modify(|_, w| w.addis().set_bit());
        self.adc.isr.modify(|_, w| w.adrdy().set_bit());
        while self.adc.cr.read().aden().bit_is_set() {}
    }

    pub fn set_align(&mut self, align: Align) {
        self.adc.cfgr.modify(|_, w| w.align().bit(align == Align::Left))
    }

    pub fn set_resolution(&mut self, resolution: Resolution) {
        self.adc.cfgr.modify(|_, w| unsafe {w.res().bits(resolution as u8)})
    }

    pub fn set_regular_oversampling(&mut self, reg_oversampl : RegularOversampling){
        self.adc.cfgr2.modify(|_, w| unsafe{ w.rovse().bit(reg_oversampl == RegularOversampling::On)})
    }

    pub fn set_injected_oversampling(&mut self, inj_oversampl : InjectedOversampling){
        self.adc.cfgr2.modify(|_, w| unsafe{ w.jovse().bit(inj_oversampl == InjectedOversampling::On)})
    }

    pub fn set_oversampling_ratio(&mut self, oversampl_ratio : OversamplingRatio){
        self.adc.cfgr2.modify(|_, w| unsafe{ w.ovsr().bits(oversampl_ratio as u8)})
    }

    pub fn set_oversampling_shift(&mut self, oversampl_shift : OversamplingShift){
        self.adc.cfgr2.modify(|_, w| unsafe{ w.ovss().bits(oversampl_shift as u8)})
    }

    //Todo: get for calfac
}

impl Channel<Adc<ADC1>> for PA0<Analog> {
    type ID = u8;

    fn channel() -> u8 {
        17
    }
}

impl Channel<Adc<ADC1>> for PA1<Analog> {
    type ID = u8;

    fn channel() -> u8 {
        6
    }
}

#[cfg(feature = "stm32l4x5")]
impl<WORD, PIN> OneShot<Adc<ADC1>, WORD, PIN> for Adc<ADC1>
where
    WORD: From<u16>,
    PIN: Channel<Adc<ADC1>, ID = u8>,
{
    type Error = ();

    fn read(&mut self, pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
        self.power_up();
        self.adc.cfgr.modify(|_, w| unsafe { w.exten().bits(0b00) });
        self.adc
            .cfgr
            .modify(|_, w| unsafe { w.align().clear_bit().res().bits(0b00).cont().clear_bit() });
        self.adc
            .sqr1
            .modify(|_, w| unsafe { w.sq1().bits(PIN::channel()) });
        self.adc
            .cfgr2
            .modify(|_, w| unsafe { w.rovse().set_bit().ovsr().bits(0b011) });
        
        self.adc.isr.modify(|_, w| w.eoc().set_bit());
        self.adc.cr.modify(|_, w| w.adstart().set_bit());

            //    cortex_m::asm::delay(80_000_000);
        while self.adc.isr.read().eoc().bit_is_clear() {}

        let val = self.adc.dr.read().regular_data().bits().into();
        self.power_down();
        Ok(val)
    }
}




macro_rules! int_adc {
    ($($Chan:ident: ($chan:expr, $en:ident)),+ $(,)*) => {
        $(
            pub struct $Chan;

            impl $Chan {
                pub fn new() -> Self {
                    Self {}
                }
                

                pub fn enable(&mut self) {
                    let adc_common = unsafe { &*crate::device::ADC123_COMMON::ptr() };
                    adc_common.ccr.modify(|_, w| w.$en().set_bit());
                }

                pub fn disable(&mut self) {
                    let adc_common = unsafe { &*crate::device::ADC123_COMMON::ptr() };
                    adc_common.ccr.modify(|_, w| w.$en().clear_bit());
                }
            }
        )+
    };
}

macro_rules! adc_pins {
    ($($Adc:ty: ($pin:ty, $chan:expr)),+ $(,)*) => {
        $(
            impl Channel<Adc<$Adc>> for $pin {
                type ID = u8;

                fn channel() -> u8 { $chan }
            }
        )+
    };
}

int_adc! {
    VTemp: (17, tsen),
    VRef: (0, vrefen),
    VBat: (18, vbaten)
}

adc_pins! {
    ADC1: (VTemp, 17),
    ADC3: (VTemp, 17),
    ADC1: (VRef, 0),
    ADC1: (VBat, 18)
}

// pub trait AdcChannel {
//     fn setup(&mut self, adc: &mut Adc);
// }

// macro_rules! adc_pins {
//     ($($Chan:ty: ($pin:ty, $bank_b:tt, $chan:expr, $smprx:ident)),+ $(,)*) => {
//         $(
//             impl Channel<Adc> for $pin {
//                 type ID = u8;

//                 fn channel() -> u8 { $chan }
//             }

//             impl AdcChannel for $pin {
//                 fn setup(&mut self, adc: &mut Adc) {
//                     adc.rb.$smprx.modify(|r, w| unsafe {
//                         const OFFSET: u8 = 3 * $chan % 10;
//                         let mut bits = r.smp().bits() as u32;
//                         bits &= !(0xfff << OFFSET);
//                         bits |= (adc.sample_time as u32) << OFFSET;
//                         w.bits(bits)
//                     });
//                     adc.rb.sqr5.write(|w| unsafe { w.sq1().bits($chan) });
//                 }
//             }
//         )+
//     };
// }

// adc_pins! {
//     Channel0: (gpioa::PA0<Analog>, false, 0_u8, smpr3),
//     Channel1: (gpioa::PA1<Analog>, false, 1_u8, smpr3),
//     Channel2: (gpioa::PA2<Analog>, false, 2_u8, smpr3),
//     Channel3: (gpioa::PA3<Analog>, false, 3_u8, smpr3),
//     Channel4: (gpioa::PA4<Analog>, false, 4_u8, smpr3),
//     Channel5: (gpioa::PA5<Analog>, false, 5_u8, smpr3),
//     Channel6: (gpioa::PA6<Analog>, false, 6_u8, smpr3),
//     Channel7: (gpioa::PA7<Analog>, false, 7_u8, smpr3),
//     Channel8: (gpiob::PB0<Analog>, false, 8_u8, smpr3),
//     Channel9: (gpiob::PB1<Analog>, false, 9_u8, smpr3),
//     Channel10: (gpioc::PC0<Analog>, false, 10_u8, smpr2),
//     Channel11: (gpioc::PC1<Analog>, false, 11_u8, smpr2),
//     Channel12: (gpioc::PC2<Analog>, false, 12_u8, smpr2),
//     Channel13: (gpioc::PC3<Analog>, false, 13_u8, smpr2),
//     Channel14: (gpioc::PC4<Analog>, false, 14_u8, smpr2),
//     Channel15: (gpioc::PC5<Analog>, false, 15_u8, smpr2),
//     Channel18: (gpiob::PB12<Analog>, false, 18_u8, smpr2),
//     Channel19: (gpiob::PB13<Analog>, false, 19_u8, smpr2),
//     Channel20: (gpiob::PB14<Analog>, false, 20_u8, smpr1),
//     Channel21: (gpiob::PB15<Analog>, false, 21_u8, smpr1),
// }


