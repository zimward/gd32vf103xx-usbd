use core::ptr;

use crate::UsbPeripheral;
use byteorder::{ByteOrder, LittleEndian};
use gd32vf103_pac as pac;
use usb_device::{
    bus::PollResult,
    class_prelude::UsbBusAllocator,
    endpoint::{EndpointAddress, EndpointDirection, EndpointType},
    UsbDirection, UsbError,
};

#[derive(Clone, Copy, PartialEq)]
enum EndpointNumber {
    EP0,
    EP1,
    EP2,
    EP3,
    NoEPavail,
}

impl EndpointNumber {
    fn advance(&mut self) -> Self {
        let old = *self;
        match self {
            EndpointNumber::EP0 => *self = EndpointNumber::EP1,
            EndpointNumber::EP1 => *self = EndpointNumber::EP2,
            EndpointNumber::EP2 => *self = EndpointNumber::EP3,
            EndpointNumber::EP3 => *self = EndpointNumber::NoEPavail, //no more endpoints available
            //after this
            EndpointNumber::NoEPavail => {}
        }
        old
    }
}

impl TryFrom<usize> for EndpointNumber {
    type Error = ();

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(EndpointNumber::EP0),
            1 => Ok(EndpointNumber::EP1),
            2 => Ok(EndpointNumber::EP2),
            3 => Ok(EndpointNumber::EP3),
            _ => Err(()),
        }
    }
}

pub struct UsbBus {
    peripheral: UsbPeripheral,
    ep_in: EndpointNumber,
    ep_out: EndpointNumber,
}

impl UsbBus {
    pub fn new(peripheral: UsbPeripheral) -> UsbBusAllocator<UsbBus> {
        let bus = UsbBus {
            peripheral,
            ep_in: EndpointNumber::EP0,
            ep_out: EndpointNumber::EP0,
        };

        UsbBusAllocator::new(bus)
    }

    pub fn free(self) -> UsbPeripheral {
        self.peripheral
    }
}

macro_rules! rep_register {
    ($index:expr,$var:expr,$reg0:ident,$reg1:ident,$reg2:ident,$reg3:ident,$func:expr) => {
        match $index {
            EndpointNumber::EP0 => $var.$reg0.modify($func),
            EndpointNumber::EP1 => $var.$reg1.modify($func),
            EndpointNumber::EP2 => $var.$reg2.modify($func),
            EndpointNumber::EP3 => $var.$reg3.modify($func),
            EndpointNumber::NoEPavail => {}
        }
    };
    ($index:expr,$var:expr,$reg1:ident,$reg2:ident,$reg3:ident,$func:expr) => {
        match $index {
            EndpointNumber::EP1 => $var.$reg1.modify($func),
            EndpointNumber::EP2 => $var.$reg2.modify($func),
            EndpointNumber::EP3 => $var.$reg3.modify($func),
            _ => {}
        }
    };
}

impl usb_device::bus::UsbBus for UsbBus {
    fn enable(&mut self) {
        let rcu_regs = unsafe { &*pac::RCU::ptr() };
        // Enable USB peripheral
        rcu_regs.ahben.modify(|_, w| w.usbfsen().set_bit());
        //reset
        rcu_regs.ahbrst.modify(|_, w| w.usbfsrst().set_bit());
        rcu_regs.ahbrst.modify(|_, w| w.usbfsrst().clear_bit());
        let usbfs_global = unsafe { &*pac::USBFS_GLOBAL::ptr() };
        let usbfs_device = unsafe { &*pac::USBFS_DEVICE::ptr() };
        //set device mode disable OTG
        usbfs_global.gusbcs.modify(|_, w| {
            w.fdm()
                .set_bit()
                .fhm()
                .clear_bit()
                .hnpcen()
                .clear_bit()
                .srpcen()
                .clear_bit()
        });
        usbfs_global
            .gahbcs
            .modify(|_, w| w.txfth().set_bit().ginten().set_bit());
        // set device speed
        usbfs_device.dcfg.modify(|_, w| {
            unsafe {
                w.ds().bits(0b11) // full speed
            }
        });
        // enable interrupts
        usbfs_global.ginten.modify(|_, w| {
            w.rstie()
                .set_bit() // usb reset
                .enumfie()
                .set_bit() // enumeration done
                .sofie()
                .set_bit() // usb sof
                .spie()
                .set_bit() // usb suspend
                .espie()
                .set_bit() // early usb suspend
                .rxfneie()
                .set_bit()
        });
        // power on phy etc
        usbfs_global.gccfg.modify(|_, w| {
            if self.peripheral.ignore_vbus {
                w.vbusig().set_bit();
            }
            w.sofoen()
                .set_bit() // enable SOF output pin
                .vbusacen()
                .set_bit() // VBUS A-device Comparer enable
                .vbusbcen()
                .set_bit() // VBUS B-device Comparer enable
                .pwron()
                .set_bit() // USB PHY power on
        });
        // wait for usb reset interrupt
        while usbfs_global.gintf.read().rst().bit_is_clear() {}
        while usbfs_global.gintf.read().enumf().bit_is_clear() {}
    }
    fn suspend(&self) {}
    fn reset(&self) {
        let rcu_regs = unsafe { &*pac::RCU::ptr() };
        rcu_regs.ahbrst.modify(|_, w| w.usbfsrst().set_bit());
        rcu_regs.ahbrst.modify(|_, w| w.usbfsrst().clear_bit());
    }
    fn force_reset(&self) -> usb_device::Result<()> {
        Ok(())
    }
    fn resume(&self) {}
    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        todo!();
    }
    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        todo!();
    }
    fn set_device_address(&self, addr: u8) {
        let usbfs_device = unsafe { &*pac::USBFS_DEVICE::ptr() };
        usbfs_device
            .dcfg
            .modify(|_, w| unsafe { w.dar().bits(addr) });
    }
    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> usb_device::Result<usize> {
        if !ep_addr.is_out() {
            return Err(UsbError::InvalidEndpoint);
        }
        todo!();
    }
    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> usb_device::Result<usize> {
        if !ep_addr.is_in() {
            return Err(UsbError::InvalidEndpoint);
        }
        let fifo = (0x50001000 + ep_addr.index() * 0x1000) as *mut u32;
        for chunk in buf.rchunks(4) {
            let mut w = LittleEndian::read_u32(chunk);
            if chunk.len() != 4 {
                w <<= 4 - chunk.len();
            }
            unsafe {
                ptr::write_volatile(fifo, w);
            }
        }
        Ok(buf.len())
    }
    fn poll(&self) -> PollResult {
        todo!();
    }
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        _interval: u8,
    ) -> usb_device::Result<EndpointAddress> {
        let device = unsafe { &*pac::USBFS_DEVICE::ptr() };

        //This is somewhat ugly. should be improved later
        let index: EndpointNumber;
        if let Some(addr) = ep_addr {
            if let Ok(ep_nr) = EndpointNumber::try_from(addr.index()) {
                index = ep_nr;
            } else {
                return Err(UsbError::InvalidEndpoint);
            }
        } else {
            index = match ep_dir {
                UsbDirection::Out => self.ep_out.advance(),
                UsbDirection::In => self.ep_in.advance(),
            };
        }
        if index == EndpointNumber::NoEPavail {
            return Err(UsbError::EndpointOverflow);
        }
        match ep_dir {
            UsbDirection::In => {
                if index == EndpointNumber::EP0 {
                    if ep_type != EndpointType::Control {
                        return Err(UsbError::Unsupported);
                    }
                    let mpl: u8 = match max_packet_size {
                        8 => 0x03,
                        16 => 0x02,
                        32 => 0x01,
                        64 => 0x00,
                        _ => {
                            return Err(UsbError::Unsupported);
                        }
                    };
                    device
                        .diep0ctl
                        .modify(|_, w| unsafe { w.epen().set_bit().mpl().bits(mpl) })
                } else if max_packet_size > 2047 {
                    return Err(UsbError::Unsupported);
                }
                let type_bits: u8 = match ep_type {
                    EndpointType::Control => 0x00,
                    EndpointType::Isochronous => 0x01,
                    EndpointType::Bulk => 0x02,
                    EndpointType::Interrupt => 0x03,
                };
                rep_register!(index, device, diep1ctl, diep2ctl, diep3ctl, |_, w| {
                    unsafe {
                        w.epen()
                            .set_bit() //endpoint enable
                            .mpl()
                            .bits(max_packet_size)
                            .eptype()
                            .bits(type_bits)
                            .txfnum()
                            .bits(index as u8)
                    }
                });
                Ok(EndpointAddress::from_parts(index as usize, ep_dir))
            }
            UsbDirection::Out => {
                rep_register!(
                    index,
                    device,
                    doep0ctl,
                    doep1ctl,
                    doep2ctl,
                    doep3ctl,
                    |_, w| {
                        w.epen().set_bit() //endpoint enable
                    }
                );
                Ok(ep_addr.unwrap())
            }
        }
    }
}