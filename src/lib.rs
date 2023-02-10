#![no_std]
pub mod bus;
mod endpoint;
mod usb;

pub use bus::UsbBus;

use gd32vf103xx_hal::gpio::{
    gpioa::{PA11, PA12},
    Floating, Input,
};
pub struct UsbPeripheral {
    pub pin_dm: PA11<Input<Floating>>,
    pub pin_dp: PA12<Input<Floating>>,
    pub ignore_vbus: bool,
}

unsafe impl Sync for UsbPeripheral {}
unsafe impl Send for UsbPeripheral {}
