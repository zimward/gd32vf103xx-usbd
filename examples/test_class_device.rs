#![no_std]
#![no_main]

use gd32vf103xx_hal::pac::USBFS_GLOBAL;
use gd32vf103xx_hal::pac::{self, USBFS_DEVICE};
use gd32vf103xx_hal::prelude::*;

use gd32vf103xx_hal::gpio::gpioa::*;
use usb_device::prelude::*;

use gd32vf103xx_usbd::bus::UsbBus;
use gd32vf103xx_usbd::UsbPeripheral;

extern crate panic_halt;

#[riscv_rt::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut rcu = dp
        .RCU
        .configure()
        .ext_hf_clock(8.mhz())
        .sysclk(108.mhz())
        .freeze();

    let gpioa = dp.GPIOA.split(&mut rcu);
    let per = UsbPeripheral {
        pin_dm: gpioa.pa11,
        pin_dp: gpioa.pa12,
        ignore_vbus: true,
    };

    let bus = UsbBus::new(per);
    let mut test = usb_device::test_class::TestClass::new(&bus);
    let mut device = test.make_device(&bus);
    loop {
        if device.poll(&mut [&mut test]) {
            test.poll();
        }
    }
}
