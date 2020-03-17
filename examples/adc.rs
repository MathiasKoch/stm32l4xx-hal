//! Blinks an LED

#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_std]
#![no_main]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt as rt;
extern crate cortex_m_semihosting as sh;
extern crate panic_semihosting;
extern crate stm32l4xx_hal as hal;
// #[macro_use(block)]
// extern crate nb;

use crate::hal::prelude::*;
use crate::rt::ExceptionFrame;
use crate::hal::adc::*;

use core::fmt::Write;
use crate::sh::hio;

#[entry]
fn main() -> ! {

    let mut hstdout = hio::hstdout().unwrap();

    writeln!(hstdout, "Hello, world!").unwrap();

    let dp = hal::stm32::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();

    // Try a different clock configuration
    hal::adc::adc_global_setup(
        hal::adc::CommonConfig::default()
        .clock_mode(hal::adc::ClockMode::HclkDiv4),
        &mut rcc.ahb2, &mut rcc.ccipr);
    let mut adc1 = Adc::adc1(
        dp.ADC1, hal::adc::Config::default(),
        &mut rcc.ahb2, &mut rcc.ccipr
    );

   
    let mut t = VTemp::new();
    r.enable();
    let mut r = VRef::new();
    r.enable();
    //cortex_m::asm::delay(40);

    let temp_val: u16 =  match adc1.read(&mut t){
        Ok(v) => v,
        Err(e)=> {
            writeln!(hstdout, "Err: {:?}", e).unwrap();
            0
        }
    };
    let ref_val: u16 =  match adc1.read(&mut r){
        Ok(v) => v,
        Err(e)=> {
            writeln!(hstdout, "Err: {:?}", e).unwrap();
            0
        }
    };
    let vdda = VRef::read_vdda(&mut adc1);
    let temp = VTemp::read(&mut adc1);
    let t_cal = VTemp::get_cal();
    writeln!(hstdout, "RefVal: {:?}, Vdda: {:?}, TempVal: {:?}, Temp: {:?} C, Tcal: {:?}\r\n",
     ref_val, vdda, temp_val, temp, t_cal).unwrap();
    loop {}
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
