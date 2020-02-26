//! Timers

use cast::{u16, u32};
use ticklock::clock::{Frequency, U32Ext};
use crate::hal::timer::{CountDown, Periodic};
use nb;
use crate::stm32::{TIM2, TIM6, TIM7, TIM15, TIM16};
use void::Void;

use crate::rcc::{APB1R1, Clocks, APB2};

/// Hardware timers
pub struct Timer<TIM> {
    clocks: Clocks,
    tim: TIM,
    timeout: Frequency,
}

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

macro_rules! hal {
    ($($TIM:ident: ($tim:ident, $timXen:ident, $timXrst:ident, $apb:ident),)+) => {
        $(
            impl Periodic for Timer<$TIM> {}

            impl CountDown for Timer<$TIM> {

                type Time = Frequency;

                // NOTE(allow) `w.psc().bits()` is safe for TIM{6,7} but not for TIM{2,3,4} due to
                // some SVD omission
                #[allow(unused_unsafe)]
                fn start<T>(&mut self, timeout: T)
                    where T: Into<Frequency>
                {
                    // pause
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());

                    self.timeout = timeout.into();
                    let ticks = self.clocks.pclk1() / self.timeout; // TODO check pclk that timer is on
                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();

                    self.tim.psc.write(|w| unsafe { w.psc().bits(psc) });

                    let arr = u16(ticks / u32(psc + 1)).unwrap();

                    self.tim.arr.write(|w| unsafe { w.bits(u32(arr)) });

                    // Trigger an update event to load the prescaler value to the clock
                    self.tim.egr.write(|w| w.ug().set_bit());
                    // The above line raises an update event which will indicate
                    // that the timer is already finished. Since this is not the case,
                    // it should be cleared
                    self.clear_update_interrupt_flag();

                    // start counter
                    self.tim.cr1.modify(|_, w| w.cen().set_bit());
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.sr.read().uif().bit_is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.clear_update_interrupt_flag();
                        Ok(())
                    }
                }
            }

            impl Timer<$TIM> {
                // XXX(why not name this `new`?) bummer: constructors need to have different names
                // even if the `$TIM` are non overlapping (compare to the `free` function below
                // which just works)
                /// Configures a TIM peripheral as a periodic count down timer
                pub fn $tim<T>(tim: $TIM, timeout: Frequency, clocks: Clocks, apb: &mut $apb) -> Self
                {
                    // enable and reset peripheral to a clean slate state
                    apb.enr().modify(|_, w| w.$timXen().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

                    let mut timer = Timer {
                        clocks,
                        tim,
                        timeout: 0.hz(),
                    };
                    timer.start(timeout);

                    timer
                }

                /// Starts listening for an `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().set_bit());
                        }
                    }
                }

                /// Stops listening for an `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().clear_bit());
                        }
                    }
                }

                /// Clears Update Interrupt Flag
                pub fn clear_update_interrupt_flag(&mut self) {
                    self.tim.sr.modify(|_, w| w.uif().clear_bit());
                }
                
                /// Releases the TIM peripheral
                pub fn free(self) -> $TIM {
                    // pause counter
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    self.tim
                }
            }
        )+
    }
}

hal! {
    TIM2: (tim2, tim2en, tim2rst, APB1R1),
    TIM6: (tim6, tim6en, tim6rst, APB1R1),
    TIM7: (tim7, tim7en, tim7rst, APB1R1),
    TIM15: (tim15, tim15en, tim15rst, APB2),
    TIM16: (tim16, tim16en, tim16rst, APB2),
}
