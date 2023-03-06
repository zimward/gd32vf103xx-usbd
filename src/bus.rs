use core::{ptr, usize};

use crate::UsbPeripheral;
use gd32vf103_pac as pac;
use usb_device::{
    bus::PollResult,
    class_prelude::UsbBusAllocator,
    endpoint::{EndpointAddress, EndpointType},
    UsbDirection, UsbError,
};

struct EpStorage {
    value: u32,
}

impl EpStorage {
    fn new() -> EpStorage {
        Self { value: 0 }
    }
    fn advance(&mut self, dir: UsbDirection) -> usb_device::Result<usize> {
        let offset = match dir {
            UsbDirection::In => 0,
            UsbDirection::Out => 4,
        };
        for i in 1..3 {
            if self.value & (1 << (i + offset)) != 0 {
                self.value |= 1 << i;
                return Ok(i);
            }
        }
        Err(UsbError::EndpointOverflow)
    }
    fn allocate(&mut self, index: usize, dir: UsbDirection) -> usb_device::Result<usize> {
        let pos = match dir {
            UsbDirection::In => 0,
            UsbDirection::Out => 4,
        } + index;
        if pos > 7 {
            Err(UsbError::EndpointOverflow)
        } else {
            self.value |= 1 << pos;
            Ok(index)
        }
    }
    fn is_allocated(&self, index: usize, dir: UsbDirection) -> bool {
        let pos = match dir {
            UsbDirection::In => 0,
            UsbDirection::Out => 4,
        } + index;
        self.value & (1 << pos) != 0
    }
    fn set_cntl_max_tlen(&mut self, max: u8) {
        self.value |= (max as u32) << 8;
    }
    fn get_cntl_max_tlen(&self) -> u8 {
        ((self.value >> 8) & 0xFF) as u8
    }
}

pub struct UsbBus {
    peripheral: UsbPeripheral,
    endpoints: EpStorage,
}

impl UsbBus {
    pub fn new(peripheral: UsbPeripheral, ignore_vbus: bool) -> UsbBusAllocator<UsbBus> {
        let bus = UsbBus {
            peripheral,
            endpoints: EpStorage::new(),
        };

        let rcu_regs = unsafe { &*pac::RCU::ptr() };
        // Enable USB peripheral
        rcu_regs.ahben.modify(|_, w| w.usbfsen().set_bit());
        //reset
        rcu_regs.ahbrst.modify(|_, w| w.usbfsrst().set_bit());
        rcu_regs.ahbrst.modify(|_, w| w.usbfsrst().clear_bit());

        let usbfs_global = unsafe { &*pac::USBFS_GLOBAL::ptr() };
        usbfs_global.gahbcs.modify(|_, w| w.txfth().set_bit());
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
        // power on phy
        usbfs_global.gccfg.modify(|_, w| {
            w.sofoen()
                .set_bit() // enable SOF output pin
                .pwron()
                .set_bit() // USB PHY power on
        });
        if ignore_vbus {
            usbfs_global.gccfg.modify(|_, w| w.vbusig().set_bit());
        }
        UsbBusAllocator::new(bus)
    }

    pub fn free(self) -> UsbPeripheral {
        self.peripheral
    }
}

macro_rules! rep_register {
    ($index:expr,$var:expr,$reg0:ident,$reg1:ident,$reg2:ident,$reg3:ident,$func:expr) => {
        match $index {
            0 => $var.$reg0.modify($func),
            1 => $var.$reg1.modify($func),
            2 => $var.$reg2.modify($func),
            3 => $var.$reg3.modify($func),
            _ => {}
        }
    };
    ($index:expr,$var:expr,$reg1:ident,$reg2:ident,$reg3:ident,$func:expr) => {
        match $index {
            1 => $var.$reg1.modify($func),
            2 => $var.$reg2.modify($func),
            3 => $var.$reg3.modify($func),
            _ => {}
        }
    };
}

impl usb_device::bus::UsbBus for UsbBus {
    fn enable(&mut self) {
        // Enable USB peripheral
        let usbfs_global = unsafe { &*pac::USBFS_GLOBAL::ptr() };
        let usbfs_device = unsafe { &*pac::USBFS_DEVICE::ptr() };

        // enable interrupts
        usbfs_global.ginten.modify(|_, w| {
            w.rstie()
                .set_bit()
                .enumfie()
                .set_bit()
                .spie()
                .set_bit()
                .espie()
                .set_bit()
                .rxfneie()
                .set_bit()
                .rstie()
                .set_bit()
                .wkupie()
                .set_bit()
        });
        usbfs_global.gahbcs.modify(|_, w| w.ginten().set_bit());

        // set device speed
        usbfs_device.dcfg.modify(|_, w| {
            unsafe {
                w.ds().bits(0b11) // full speed
            }
        });
        usbfs_global.grstctl.modify(|_, w| w.csrst().set_bit()); //soft reset
        while usbfs_global.grstctl.read().csrst().bit_is_set() {} //wait for soft reset
                                                                  //EP interrupts
        usbfs_device.diepinten.modify(|_, w| w.tfen().set_bit());
        usbfs_device
            .doepinten
            .modify(|_, w| w.tfen().set_bit().stpfen().set_bit());
    }

    fn suspend(&self) {}

    fn reset(&self) {
        let usbfs_global = unsafe { &*pac::USBFS_GLOBAL::ptr() };
        let usbfs_device = unsafe { &*pac::USBFS_DEVICE::ptr() };

        //enable OUT endpoints to receive data from host
        usbfs_device.doep0len.modify(|_, w| unsafe {
            w.tlen()
                .bits(self.endpoints.get_cntl_max_tlen())
                .pcnt()
                .set_bit()
                .stpcnt()
                .bits(1)
        });
        usbfs_device
            .doep0ctl
            .modify(|_, w| w.epen().set_bit().cnak().set_bit());
        let max_rx_bytes = usbfs_global.grflen.read().rxfd().bits() as u32 * 4;
        for i in 1..3 {
            if self.endpoints.is_allocated(i, UsbDirection::Out) {
                rep_register!(
                    i,
                    usbfs_device,
                    doep1len,
                    doep2len,
                    doep3len,
                    |_, w| unsafe { w.tlen().bits(max_rx_bytes).pcnt().bits(1) }
                );
                rep_register!(i, usbfs_device, doep1ctl, doep2ctl, doep3ctl, |_, w| w
                    .epen()
                    .set_bit()
                    .cnak()
                    .set_bit());
            }
        }
    }

    fn force_reset(&self) -> usb_device::Result<()> {
        Ok(())
    }

    fn resume(&self) {}
    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        let usbfs_device = unsafe { &*pac::USBFS_DEVICE::ptr() };
        let index = ep_addr.index();

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
        if index == 0 {
            usbfs_device.doep0len.modify(|_, w| unsafe {
                w.pcnt()
                    .set_bit()
                    .stpcnt()
                    .bits(1)
                    .tlen()
                    .bits(self.endpoints.get_cntl_max_tlen())
            });
        }
        let usbfs_global = unsafe { &*pac::USBFS_GLOBAL::ptr() };
        let max_rx_bytes = usbfs_global.grflen.read().rxfd().bits() as u32 * 4;
        rep_register!(
            index,
            usbfs_device,
            doep1len,
            doep2len,
            doep3len,
            |_, w| unsafe { w.pcnt().bits(1).tlen().bits(max_rx_bytes) }
        );
        rep_register!(
            index,
            usbfs_device,
            doep0ctl,
            doep1ctl,
            doep2ctl,
            doep3ctl,
            |_, w| w.epen().set_bit().cnak().set_bit()
        );
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        let usbfs_device = unsafe { &*pac::USBFS_DEVICE::ptr() };
        let index = ep_addr.index();
        match index {
            0 => usbfs_device.doep0ctl.read().stall().bit_is_set(),
            1 => usbfs_device.doep1ctl.read().stall().bit_is_set(),
            2 => usbfs_device.doep2ctl.read().stall().bit_is_set(),
            3 => usbfs_device.doep3ctl.read().stall().bit_is_set(),
            _ => false,
        }
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
        let index = ep_addr.index();
        if index == 0 {
            usbfs_device.doep0len.modify(|_, w| unsafe {
                w.pcnt()
                    .set_bit()
                    .stpcnt()
                    .bits(1)
                    .tlen()
                    .bits(self.endpoints.get_cntl_max_tlen())
            });
        }
        rep_register!(
            index,
            usbfs_device,
            doep1len,
            doep2len,
            doep3len,
            |_, w| unsafe { w.pcnt().bits(1) }
        );
        //clear NAK
        rep_register!(
            index,
            usbfs_device,
            doep0ctl,
            doep1ctl,
            doep2ctl,
            doep3ctl,
            |_, w| { w.epen().set_bit().cnak().set_bit() }
        );
        Ok(count)
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> usb_device::Result<usize> {
        if !ep_addr.is_in() {
            return Err(UsbError::InvalidEndpoint);
        }
        let usbfs_device = unsafe { &*pac::USBFS_DEVICE::ptr() };
        //setup pcnt  and tlen for transmission
        let index = ep_addr.index();
        if index == 0 {
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

        let index = if let Some(addr) = ep_addr {
            if self.endpoints.is_allocated(addr.index(), ep_dir) {
                Err(UsbError::InvalidEndpoint)
            } else {
                self.endpoints.allocate(addr.index(), ep_dir)
            }
        } else {
            self.endpoints.advance(ep_dir)
        }?;

        let words = (max_packet_size + 31) / 32;
        match ep_dir {
            UsbDirection::In => {
                if index == 0 {
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
                    device
                        .diep0ctl
                        .modify(|_, w| unsafe { w.mpl().bits(mpl).txfnum().bits(0) });
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
                        w.mpl()
                            .bits(max_packet_size)
                            .eptype()
                            .bits(type_bits)
                            .txfnum()
                            .bits(index as u8)
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
                Ok(EndpointAddress::from_parts(index, ep_dir))
            }
            UsbDirection::Out => {
                if index == 0 {
                    self.endpoints.set_cntl_max_tlen(max_packet_size as u8);
                }
                //adjust RX FIFO depth to be long enough for the larget endpoint
                let rxfd = usbfs_global.grflen.read().rxfd().bits();
                usbfs_global.grflen.modify(|_, w| unsafe {
                    if words > rxfd {
                        w.rxfd().bits(words)
                    } else {
                        w
                    }
                });
                Ok(EndpointAddress::from_parts(index as usize, ep_dir))
            }
        }
    }
}
