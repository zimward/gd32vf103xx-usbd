use core::ptr;

use crate::UsbPeripheral;
use gd32vf103_pac as pac;
use pac::usbfs_device::{diep0len, doep3ctl, doep3len};
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
            ep_in: EndpointNumber::EP1,
            ep_out: EndpointNumber::EP1,
        };

        let rcu_regs = unsafe { &*pac::RCU::ptr() };
        // Enable USB peripheral
        rcu_regs.ahben.modify(|_, w| w.usbfsen().set_bit());
        //reset
        rcu_regs.ahbrst.modify(|_, w| w.usbfsrst().set_bit());
        rcu_regs.ahbrst.modify(|_, w| w.usbfsrst().clear_bit());
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
        // Enable USB peripheral
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
                .rstie()
                .set_bit()
                .wkupie()
                .set_bit()
                .oepie()
                .set_bit()
                .iepie()
                .set_bit()
        });
        // power on phy etc
        usbfs_global.gccfg.modify(|_, w| {
            w.sofoen()
                .set_bit() // enable SOF output pin
                .pwron()
                .set_bit() // USB PHY power on
        });
        if self.peripheral.ignore_vbus {
            usbfs_global.gccfg.modify(|_, w| w.vbusig().set_bit());
        }
        usbfs_global.grstctl.modify(|_, w| w.csrst().set_bit()); //soft reset
        while usbfs_global.grstctl.read().csrst().bit_is_set() {} //wait for soft reset
                                                                  //FIFO len
        usbfs_global
            .grflen
            .modify(|_, w| unsafe { w.rxfd().bits(256) });
        //EP interrupts
        usbfs_device.diepinten.modify(|_, w| w.tfen().set_bit());
        usbfs_device
            .doepinten
            .modify(|_, w| w.tfen().set_bit().stpfen().set_bit());
    }

    fn suspend(&self) {}

    fn reset(&self) {
        let usbfs_device = unsafe { &*pac::USBFS_DEVICE::ptr() };
        //usbfs_device.dcfg.modify(|_, w| unsafe { w.dar().bits(0) });
        /*
        //flush fifos
        let usbfs_global = unsafe { &*pac::USBFS_GLOBAL::ptr() };
        //mark all fifos
        usbfs_global
            .grstctl
            .modify(|_, w| unsafe { w.txfnum().bits(0xFF) });
        usbfs_global
            .grstctl
            .modify(|_, w| w.rxff().set_bit().txff().set_bit());
        //wait till flush is complete
        while usbfs_global.grstctl.read().txff().bit_is_set()
            || usbfs_global.grstctl.read().rxff().bit_is_set()
        {}*/
    }

    fn force_reset(&self) -> usb_device::Result<()> {
        Ok(())
        //todo!();
    }

    fn resume(&self) {}
    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        let usbfs_device = unsafe { &*pac::USBFS_DEVICE::ptr() };
        if let Ok(index) = EndpointNumber::try_from(ep_addr.index()) {
            rep_register!(
                index,
                usbfs_device,
                doep0ctl,
                doep1ctl,
                doep2ctl,
                doep3ctl,
                |_, w| {
                    if stalled {
                        w.stall().set_bit()
                    } else {
                        w.stall().clear_bit()
                    }
                }
            );
            if index == EndpointNumber::EP0 {
                usbfs_device
                    .doep0len
                    .modify(|_, w| unsafe { w.pcnt().set_bit().stpcnt().bits(1) });
            }
            rep_register!(
                index,
                usbfs_device,
                doep1len,
                doep2len,
                doep3len,
                |_, w| unsafe { w.pcnt().bits(1).stpcnt_rxdpid().bits(1) }
            );
            rep_register!(
                index,
                usbfs_device,
                doep0ctl,
                doep1ctl,
                doep2ctl,
                doep3ctl,
                |_, w| w.cnak().set_bit()
            );
        }
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        let usbfs_device = unsafe { &*pac::USBFS_DEVICE::ptr() };
        if let Ok(index) = EndpointNumber::try_from(ep_addr.index()) {
            return match index {
                EndpointNumber::EP0 => usbfs_device.doep0ctl.read().stall().bit_is_set(),
                EndpointNumber::EP1 => usbfs_device.doep1ctl.read().stall().bit_is_set(),
                EndpointNumber::EP2 => usbfs_device.doep2ctl.read().stall().bit_is_set(),
                EndpointNumber::EP3 => usbfs_device.doep3ctl.read().stall().bit_is_set(),
                EndpointNumber::NoEPavail => false,
            };
        }
        false
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
        let usbfs_global = unsafe { &*pac::USBFS_GLOBAL::ptr() };
        if usbfs_global.grstatr_device().read().epnum().bits() as usize != ep_addr.index() {
            return Err(UsbError::WouldBlock);
        }
        let count: usize = usbfs_global.grstatr_device().read().bcount().bits().into();
        if count > buf.len() {
            return Err(UsbError::EndpointMemoryOverflow);
        }
        //pop TF finished
        let _ = usbfs_global.grstatp_device().read();
        let reads = (count + 3) / 4;
        for i in 0..reads {
            let data = usbfs_global.grstatp_device().read().bits().to_ne_bytes();
            buf[i * 4..i * 4 + data.len()].copy_from_slice(&data);
        }
        let usbfs_device = unsafe { &*pac::USBFS_DEVICE::ptr() };
        let index = EndpointNumber::try_from(ep_addr.index()).unwrap_or(EndpointNumber::NoEPavail);
        if index == EndpointNumber::EP0 {
            usbfs_device
                .doep0len
                .modify(|_, w| unsafe { w.pcnt().set_bit().stpcnt().bits(3).tlen().bits(32) });
        }
        rep_register!(
            index,
            usbfs_device,
            doep1len,
            doep2len,
            doep3len,
            |_, w| unsafe { w.pcnt().bits(1).stpcnt_rxdpid().bits(1) }
        );
        //clear NAK
        rep_register!(
            index,
            usbfs_device,
            doep0ctl,
            doep1ctl,
            doep2ctl,
            doep3ctl,
            |_, w| { w.cnak().set_bit() }
        );
        Ok(count)
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> usb_device::Result<usize> {
        if !ep_addr.is_in() {
            return Err(UsbError::InvalidEndpoint);
        }
        let usbfs_device = unsafe { &*pac::USBFS_DEVICE::ptr() };
        //setup pcnt  and tlen for transmission
        let index = EndpointNumber::try_from(ep_addr.index()).unwrap();
        if index == EndpointNumber::EP0 {
            usbfs_device
                .diep0len
                .modify(|_, w| unsafe { w.pcnt().bits(1).tlen().bits(buf.len() as u8) });
            //clear nak and enable EP
            usbfs_device
                .diep0ctl
                .modify(|_, w| w.cnak().set_bit().epen().set_bit());
        }
        rep_register!(index, usbfs_device, diep1len, diep2len, diep3len, |_, w| {
            unsafe { w.pcnt().bits(1).tlen().bits(buf.len() as u32) }
        });
        rep_register!(index, usbfs_device, diep1ctl, diep2ctl, diep3ctl, |_, w| {
            w.cnak().set_bit().epen().set_bit()
        });
        // FIFO base addr = 0x50000000 + 0x1000 * (nFIFO+1 )
        let fifo = (0x50001000 + ep_addr.index() * 0x1000) as *mut u32;
        for chunk in buf.chunks(4) {
            let mut w: u32 = 0;
            //read chunk as little endian u32
            for c in chunk.iter().rev() {
                w <<= 8;
                w |= *c as u32;
            }
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
        let usbfs_global = unsafe { &*pac::USBFS_GLOBAL::ptr() };
        let usbfs_device = unsafe { &*pac::USBFS_DEVICE::ptr() };
        let gintf = usbfs_global.gintf.read();
        let mut ep_in: u16 = 0;
        let mut ep_out: u16 = 0;
        let mut ep_setup: u16 = 0;
        //EP in
        ep_in |= usbfs_device
            .diep0intf
            .read()
            .tf()
            .bit_is_set()
            .then(|| {
                usbfs_device.diep0intf.modify(|_, w| w.tf().clear_bit());
                0x1
            })
            .unwrap_or(0);
        ep_in |= usbfs_device
            .diep1intf
            .read()
            .tf()
            .bit_is_set()
            .then(|| {
                usbfs_device.diep1intf.modify(|_, w| w.tf().clear_bit());
                0x2
            })
            .unwrap_or(0);
        ep_in |= usbfs_device
            .diep2intf
            .read()
            .tf()
            .bit_is_set()
            .then(|| {
                usbfs_device.diep2intf.modify(|_, w| w.tf().clear_bit());
                0x4
            })
            .unwrap_or(0);
        ep_in |= usbfs_device
            .diep3intf
            .read()
            .tf()
            .bit_is_set()
            .then(|| {
                usbfs_device.diep3intf.modify(|_, w| w.tf().clear_bit());
                0x8
            })
            .unwrap_or(0);

        if gintf.rxfneif().bit_is_set() {
            let ep = usbfs_global.grstatr_device().read().epnum().bits();
            let pty = usbfs_global.grstatr_device().read().rpckst().bits();
            if pty == 0b0110 {
                ep_setup |= 1 << ep;
            } else if pty == 0b0010 {
                ep_out |= 1 << ep;
            }
        }
        if ep_in != 0 || ep_out != 0 || ep_setup != 0 {
            PollResult::Data {
                ep_out,
                ep_in_complete: ep_in,
                ep_setup,
            }
        } else if gintf.sp().bit_is_set() {
            usbfs_global.gintf.modify(|_, w| w.sp().clear_bit());
            PollResult::None
        } else if gintf.rst().bit_is_set() {
            usbfs_global.gintf.modify(|_, w| w.rst().clear_bit());
            //usbfs_global.ginten.modify(|_, w| w.rstie().clear_bit()); //mask reset interrupt
            PollResult::Reset
        } else if gintf.wkupif().bit_is_set() {
            usbfs_global.gintf.modify(|_, w| w.wkupif().clear_bit());
            PollResult::Resume
        } else {
            PollResult::None
        }
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
        let usbfs_global = unsafe { &*pac::USBFS_GLOBAL::ptr() };

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
                let words = (max_packet_size + 31) / 32;
                if index == EndpointNumber::EP0 {
                    if ep_type != EndpointType::Control {
                        return Err(UsbError::Unsupported);
                    }
                    let mpl: u8 = match max_packet_size {
                        8 => 0b11,
                        16 => 0b10,
                        32 => 0b01,
                        64 => 0b00,
                        _ => {
                            return Err(UsbError::Unsupported);
                        }
                    };
                    device.diep0ctl.modify(|_, w| unsafe {
                        w.epen()
                            .set_bit()
                            .mpl()
                            .bits(mpl)
                            .txfnum()
                            .bits(0)
                            .cnak()
                            .set_bit()
                    });
                    usbfs_global
                        .diep0tflen_mut()
                        .modify(|_, w| unsafe { w.iep0txfd().bits(words) });
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
                            .cnak()
                            .set_bit()
                    }
                });
                rep_register!(
                    index,
                    usbfs_global,
                    diep1tflen,
                    diep2tflen,
                    diep3tflen,
                    |_, w| { unsafe { w.ieptxfd().bits(words) } }
                );
                Ok(EndpointAddress::from_parts(index as usize, ep_dir))
            }
            UsbDirection::Out => {
                if index == EndpointNumber::EP0 {
                    device.doep0len.modify(|_, w| unsafe {
                        w.tlen()
                            .bits(max_packet_size as u8)
                            .stpcnt()
                            .bits(1) //allow 1 b2b setup packets
                            .pcnt() //allow packets to be received
                            .set_bit()
                    })
                }
                rep_register!(index, device, doep1len, doep2len, doep3len, |_, w| {
                    unsafe {
                        w.tlen()
                            .bits(max_packet_size as u32)
                            .pcnt()
                            .bits(1)
                            .stpcnt_rxdpid()
                            .bits(1)
                    }
                });
                rep_register!(
                    index,
                    device,
                    doep0ctl,
                    doep1ctl,
                    doep2ctl,
                    doep3ctl,
                    |_, w| {
                        w.epen()
                            .set_bit() //endpoint enable
                            .cnak()
                            .set_bit()
                    }
                );
                Ok(EndpointAddress::from_parts(index as usize, ep_dir))
            }
        }
    }
}
