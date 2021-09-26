//! Universal Serial Bus (USB)
//!
//! See [Chapter 4 Section 1](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details
//!
//! ## Usage
//!
//! ```no_run
//! use rp2040_hal::{pac, watchdog, clocks, usb};
//!
//! static mut USB: *mut usb::UsbDevice<pac::USBCTRL_REGS, pac::USBCTRL_DPRAM> = core::ptr::null_mut();
//! #[allow(non_snake_case)]
//! unsafe extern "C" fn USBCTRL_IRQ() {
//!     (*USB).poll();
//! }
//!
//! let mut pac = pac::Peripherals::take().unwrap();
//! let mut watchdog = watchdog::Watchdog::new(pac.WATCHDOG);
//! const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
//! let _clocks = clocks::init_clocks_and_plls(
//!     XOSC_CRYSTAL_FREQ, pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB, &mut pac.RESETS, &mut watchdog,
//! ).ok().unwrap();
//! let mut usb = usb::UsbDevice::new(pac.USBCTRL_REGS, pac.USBCTRL_DPRAM, &mut pac.RESETS);
//! unsafe {
//!     USB = &mut usb;
//!     cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ);
//! }
//! usb.enable();
//! ```

use crate::resets::SubsystemReset;
use core::ops::Deref;
use pac::{generic::Reg, usbctrl_dpram, usbctrl_regs, RESETS};

#[macro_use]
mod macros;
pub mod protocol;

const NUMBER_OF_ENDPOINTS: u8 = 16;
const BUFFERS_START_OFFSET: u16 = 0x180;
const BUFFERS_END_OFFSET: u16 = 0x1000;

/// Pac USB control registers
pub trait UsbController: Deref<Target = usbctrl_regs::RegisterBlock> + SubsystemReset {}
impl UsbController for pac::USBCTRL_REGS {}

/// Pac USB dpram
pub trait UsbMemory: Deref<Target = usbctrl_dpram::RegisterBlock> {}
impl UsbMemory for pac::USBCTRL_DPRAM {}

/// Used to allocate and initialize endpoints
pub struct EndpointConfiguration {
    /// The transfer type of this endpoint
    pub kind: usbctrl_dpram::ep_control::ENDPOINT_TYPE_A,
    /// Total bytes allocated for this endpoint
    ///
    /// This must be 64 bytes except if `kind` is ISOCHRONOUS,
    /// then it can be 128, 256, 512 or 1024.
    /// If `is_double_buffered` is `true` the value needs to be doubled.
    /// A value of `0` indicates that this endpoint is disabled / skipped.
    pub buffer_length: u16,
    /// Use double buffering
    pub is_double_buffered: bool,
    /// Inverse frequency (wavelength) of interrupt transfer polling
    ///
    /// For devices this is measured in frames times 4 (shifted 2 bits up).
    /// For the host it is instead measured in milliseconds minus one.
    pub interval: u16,
}

fn endpoint_index_to_address(endpoint_index: u8) -> u8 {
    (endpoint_index >> 1) | ((endpoint_index ^ 0x01) << 7)
}

/*fn endpoint_address_to_index(endpoint_address: u8) -> u8 {
    (endpoint_address << 1) | ((endpoint_address >> 7) ^ 0x01)
}*/

/// Endpoint data handler
///
/// Device send: number of bytes sent last time, buffer to write data in, returns number of bytes to send or 0 to stop
/// Device receive: number of bytes received, buffer to read from, returns number of bytes to receive next or 0 to stop
pub type EndpointHandler<'a> = &'a mut dyn FnMut(usize, &mut [u8]) -> usize;

struct UsbCommon<'a, C: UsbController, M: UsbMemory> {
    controller: C,
    memory: M,
    handlers: &'a mut [EndpointHandler<'a>],
}

impl<'a, C: UsbController, M: UsbMemory> UsbCommon<'a, C, M> {
    fn endpoint_control_register(
        &self,
        endpoint_index: u8,
    ) -> &Reg<usbctrl_dpram::ep_control::EP_CONTROL_SPEC> {
        &self.memory.ep_control[endpoint_index as usize - 2]
    }

    fn reset(&mut self, resets: &mut RESETS) {
        self.controller.reset_bring_down(resets);
        self.controller.reset_bring_up(resets);
        for word in unsafe {
            core::slice::from_raw_parts_mut(
                core::mem::transmute::<_, *mut u32>(pac::USBCTRL_DPRAM::ptr()),
                0x1000 / core::mem::size_of::<u32>(),
            )
        } {
            *word = 0;
        }
        self.controller
            .usb_muxing
            .write(|w| w.to_phy().set_bit().softcon().set_bit());
        self.controller.sie_ctrl.write(|w| {
            // if is_double_buffered { w.ep0_double_buf().set_bit(); }
            // if int_stall { w.ep0_int_stall().set_bit(); }
            // if int_nak { w.ep0_int_nak().set_bit(); }
            w.ep0_int_1buf().set_bit()
        });
        self.memory.ep_buffer_control[0].modify(|_, w| w.pid_0().set_bit().pid_1().set_bit());
    }

    fn reset_endpoint_pids(&mut self) {
        for endpoint_index in 2..NUMBER_OF_ENDPOINTS * 2 {
            self.memory.ep_buffer_control[endpoint_index as usize]
                .modify(|_, w| w.pid_0().clear_bit().pid_1().clear_bit());
        }
    }

    fn allocate_endpoints(&mut self, endpoints: &[EndpointConfiguration]) {
        assert!(2 + endpoints.len() <= NUMBER_OF_ENDPOINTS as usize);
        let mut buffer_start = BUFFERS_START_OFFSET;
        for endpoint_index in 2..2 + endpoints.len() as u8 {
            let endpoint = &endpoints[endpoint_index as usize - 2];
            if endpoint.buffer_length == 0 {
                continue;
            }
            self.endpoint_control_register(endpoint_index)
                .write(|w| unsafe {
                    w.bits((endpoint.interval as u32 & 0x3FF) << 16);
                    if endpoint.is_double_buffered {
                        w.double_buffered().set_bit();
                    }
                    // if int_stall { w.interrupt_on_stall().set_bit(); }
                    // if int_nak { w.interrupt_on_nak().set_bit(); }
                    w.enable()
                        .set_bit()
                        .interrupt_per_buff()
                        .set_bit()
                        .endpoint_type()
                        .bits(endpoint.kind as u8)
                        .buffer_address()
                        .bits(buffer_start)
                });
            if endpoint.kind == usbctrl_dpram::ep_control::ENDPOINT_TYPE_A::ISOCHRONOUS {
                if endpoint.is_double_buffered {
                    let log2 = endpoint.buffer_length.trailing_zeros() as u8;
                    assert!(endpoint.buffer_length == 1 << log2 && log2 >= 8 && log2 < 12);
                    self.memory.ep_buffer_control[endpoint_index as usize]
                        .write(|w| w.double_buffer_iso_offset().bits(log2 - 8));
                } else {
                    assert!(
                        endpoint.buffer_length & 63 == 0
                            && endpoint.buffer_length >= 64
                            && endpoint.buffer_length <= 1024
                    );
                }
            } else {
                assert_eq!(endpoint.buffer_length, 64);
            }
            buffer_start += endpoint.buffer_length;
        }
        self.memory.ep_control[endpoints.len()]
            .write(|w| unsafe { w.buffer_address().bits(buffer_start) });
    }

    fn get_endpoint_double_buffering(&self, endpoint_index: u8) -> (bool, usize) {
        let is_double_buffered = if endpoint_index < 2 {
            self.controller
                .sie_ctrl
                .read()
                .ep0_double_buf()
                .bit_is_set()
        } else {
            self.endpoint_control_register(endpoint_index)
                .read()
                .double_buffered()
                .bit_is_set()
        };
        let double_buffer_index = if is_double_buffered {
            (self.controller.buff_cpu_should_handle.read().bits() >> endpoint_index) as usize & 1
        } else {
            0
        };
        (is_double_buffered, double_buffer_index)
    }

    fn get_endpoint_buffer(&mut self, endpoint_index: u8) -> &'static mut [u8] {
        let (is_double_buffered, double_buffer_index) =
            self.get_endpoint_double_buffering(endpoint_index);
        let (buffer_start, buffer_length) = if endpoint_index < 2 {
            (BUFFERS_START_OFFSET - 128, 64)
        } else {
            let buffer_start = if endpoint_index == 2 {
                BUFFERS_START_OFFSET
            } else {
                self.endpoint_control_register(endpoint_index)
                    .read()
                    .buffer_address()
                    .bits()
            };
            (
                buffer_start,
                (if endpoint_index + 1 >= NUMBER_OF_ENDPOINTS {
                    BUFFERS_END_OFFSET
                } else {
                    self.endpoint_control_register(endpoint_index + 1)
                        .read()
                        .buffer_address()
                        .bits()
                } - buffer_start) as usize
                    >> if is_double_buffered { 1 } else { 0 },
            )
        };
        unsafe {
            core::slice::from_raw_parts_mut(
                core::mem::transmute::<_, *mut u8>(pac::USBCTRL_DPRAM::ptr())
                    .add(buffer_start as usize + double_buffer_index as usize * buffer_length),
                // core::mem::transmute::<_, *mut u8>(&self.memory).add(buffer_start as usize),
                buffer_length,
            )
        }
    }

    fn endpoint_transfer(&mut self, endpoint_index: u8, send: bool, length: usize) {
        let (_, double_buffer_index) = self.get_endpoint_double_buffering(endpoint_index);
        self.memory.ep_buffer_control[endpoint_index as usize].modify(|_, w| unsafe {
            if double_buffer_index == 0 {
                w.available_0()
                    .set_bit()
                    .full_0()
                    .bit(send)
                    .length_0()
                    .bits(length as u16)
                // .last_0()
                // .bit(end_of_transfer)
            } else {
                w.available_1()
                    .set_bit()
                    .full_1()
                    .bit(send)
                    .length_1()
                    .bits(length as u16)
                // .last_1()
                // .bit(end_of_transfer)
            }
        });
    }

    fn trigger_endpoint(&mut self, endpoint_index: u8) -> bool {
        let (_, double_buffer_index) = self.get_endpoint_double_buffering(endpoint_index);
        let sending = endpoint_index & 1 == 0;
        let buffer = self.get_endpoint_buffer(endpoint_index);
        let register = &self.memory.ep_buffer_control[endpoint_index as usize];
        let (in_use, transferred_length) = if double_buffer_index == 0 {
            (
                register.read().available_0().bit(),
                register.read().length_0().bits() as usize,
            )
        } else {
            (
                register.read().available_1().bit(),
                register.read().length_1().bits() as usize,
            )
        };
        if in_use {
            return false;
        }
        let length_to_transfer = if sending || transferred_length > 0 {
            (self.handlers[endpoint_index as usize - 2])(transferred_length, buffer)
        } else {
            buffer.len()
        };
        if length_to_transfer > 0 {
            register.modify(|_, w| w.stall().clear_bit());
            self.endpoint_transfer(endpoint_index, sending, length_to_transfer);
        } else {
            register.modify(|_, w| w.stall().set_bit());
        }
        true
    }

    fn handle_endpoint_buffers(&mut self) {
        let needs_handling = self.controller.buff_status.read().bits();
        for endpoint_index in 2..NUMBER_OF_ENDPOINTS * 2 {
            if needs_handling & (1 << endpoint_index) != 0 {
                self.trigger_endpoint(endpoint_index);
                self.memory.ep_buffer_control[endpoint_index as usize].modify(|r, w| {
                    w.pid_0()
                        .bit(!r.pid_0().bit_is_set())
                        .pid_1()
                        .bit(!r.pid_1().bit_is_set())
                });
            }
        }
        self.controller
            .buff_status
            .write(|w| unsafe { w.bits(0xFFFFFFFF) });
    }
}

/// Localized strings with USB language code as key
pub type StringTable<'a> = [(u16, &'a [&'static str])];

/// Represents the USB bus configured as a device
pub struct UsbDevice<'a, C: UsbController, M: UsbMemory> {
    common: UsbCommon<'a, C, M>,
    string_table: &'a StringTable<'a>,
    device_descriptor: &'a protocol::DeviceDescriptor,
    configuration_descriptors: &'a [protocol::ConfigurationDescriptor],
    interface_descriptors: &'a [protocol::InterfaceDescriptor],
    interface_endpoint_map: &'a [&'a [u8]],
    address: u8,
    configuration: u8,
}

impl<'a, C: UsbController, M: UsbMemory> UsbDevice<'a, C, M> {
    /// Resets the bus and becomes an USB device
    pub fn new(
        controller: C,
        memory: M,
        resets: &mut RESETS,
        string_table: &'a StringTable,
        device_descriptor: &'a protocol::DeviceDescriptor,
        configuration_descriptors: &'a [protocol::ConfigurationDescriptor],
        interface_descriptors: &'a [protocol::InterfaceDescriptor],
        interface_endpoint_map: &'a [&'a [u8]],
        endpoints: &[EndpointConfiguration],
        handlers: &'a mut [EndpointHandler<'a>],
    ) -> Self {
        let mut common = UsbCommon::<'a, C, M> {
            controller,
            memory,
            handlers,
        };
        common.reset(resets);

        common.controller.usb_pwr.write(|w| {
            w.vbus_detect()
                .set_bit()
                .vbus_detect_override_en()
                .set_bit()
        });

        // Enable the USB controller in device mode.
        common
            .controller
            .main_ctrl
            .write(|w| w.controller_en().set_bit());

        common.controller.inte.write(|w| {
            w.setup_req()
                .set_bit()
                .buff_status()
                .set_bit()
                .bus_reset()
                .set_bit()
        });

        common.allocate_endpoints(endpoints);

        Self {
            common,
            string_table,
            device_descriptor,
            configuration_descriptors,
            interface_descriptors,
            interface_endpoint_map,
            address: 0,
            configuration: 0,
        }
    }

    /// Releases the underlying controller and memory
    pub fn free(self) -> (C, M) {
        (self.common.controller, self.common.memory)
    }

    /// Is connected and not suspended
    pub fn is_running(&self) -> bool {
        let status = self.common.controller.sie_status.read();
        status.connected().bit_is_set() && !status.suspended().bit_is_set()
    }

    /// Enable bus
    pub fn enable(&mut self) {
        self.common
            .controller
            .sie_ctrl
            .modify(|_, w| w.pullup_en().set_bit());
    }

    /// Disable bus
    pub fn disable(&mut self) {
        self.common
            .controller
            .sie_ctrl
            .modify(|_, w| w.pullup_en().clear_bit());
    }

    /// Prepare for host to read from / write to this endpoint
    pub fn trigger_endpoint(&mut self, endpoint_index: u8) {
        self.common.trigger_endpoint(endpoint_index);
    }

    /// Call this in USBCTRL_IRQ
    pub fn poll(&mut self) {
        if self.common.controller.ints.read().setup_req().bit_is_set() {
            self.handle_setup_packet();
            self.common
                .controller
                .sie_status
                .modify(|_, w| w.setup_rec().set_bit());
        }
        if self
            .common
            .controller
            .ints
            .read()
            .buff_status()
            .bit_is_set()
        {
            if self.common.controller.buff_status.read().bits() & 1 != 0 {
                let length = self.common.get_endpoint_buffer(1).len();
                self.common.endpoint_transfer(1, false, length);
                self.common
                    .controller
                    .addr_endp
                    .write(|w| unsafe { w.address().bits(self.address) });
            }
            self.common.handle_endpoint_buffers();
        }
        if self.common.controller.ints.read().bus_reset().bit_is_set() {
            self.address = 0;
            self.configuration = 0;
            self.common.controller.addr_endp.write(|w| w);
            self.common
                .controller
                .sie_status
                .modify(|_, w| w.bus_reset().set_bit());
        }
    }

    fn endpoint_descriptor(&mut self, endpoint_index: u8) -> protocol::EndpointDescriptor {
        let register_read = self.common.endpoint_control_register(endpoint_index).read();
        protocol::EndpointDescriptor {
            bLength: core::mem::size_of::<protocol::EndpointDescriptor>() as u8,
            bDescriptorType: protocol::DescriptorType::Endpoint,
            bEndpointAddress: endpoint_index_to_address(endpoint_index),
            bmAttributes: register_read.endpoint_type().bits(),
            wMaxPacketSize: self.common.get_endpoint_buffer(endpoint_index).len() as u16,
            bInterval: (register_read.bits() >> 18) as u8,
        }
    }

    fn handle_get_descriptor_request(&mut self, buffer: &mut [u8]) -> usize {
        let wvalue = self.common.memory.setup_packet_low.read().wvalue().bits();
        let descriptor_type = deserialize_value!(protocol::DescriptorType, [(wvalue >> 8) as u8]);
        let descriptor_index = (wvalue & 0xFF) as usize;
        match descriptor_type {
            protocol::DescriptorType::Device => {
                buffer[0..core::mem::size_of::<protocol::DeviceDescriptor>()].copy_from_slice(
                    serialize_value!(protocol::DeviceDescriptor, *self.device_descriptor),
                );
                core::mem::size_of::<protocol::DeviceDescriptor>()
            }
            protocol::DescriptorType::Configuration => {
                let mut offset = core::mem::size_of::<protocol::ConfigurationDescriptor>();
                let configuration_descriptor = &self.configuration_descriptors[descriptor_index];
                buffer[0..offset].copy_from_slice(serialize_value!(
                    protocol::ConfigurationDescriptor,
                    *configuration_descriptor
                ));
                let start_index = self
                    .configuration_descriptors
                    .iter()
                    .take(descriptor_index)
                    .fold(0, |accu, conf| accu + conf.bNumInterfaces);
                for i in start_index..start_index + configuration_descriptor.bNumInterfaces {
                    let interface_descriptor = &self.interface_descriptors[i as usize];
                    buffer[offset..offset + core::mem::size_of::<protocol::InterfaceDescriptor>()]
                        .copy_from_slice(serialize_value!(
                            protocol::InterfaceDescriptor,
                            *interface_descriptor
                        ));
                    offset += core::mem::size_of::<protocol::InterfaceDescriptor>();
                    for j in 0..interface_descriptor.bNumEndpoints {
                        let endpoint_index = self.interface_endpoint_map[i as usize][j as usize];
                        if self
                            .common
                            .endpoint_control_register(endpoint_index)
                            .read()
                            .enable()
                            .bit_is_set()
                        {
                            buffer[offset
                                ..offset + core::mem::size_of::<protocol::EndpointDescriptor>()]
                                .copy_from_slice(serialize_value!(
                                    protocol::EndpointDescriptor,
                                    self.endpoint_descriptor(endpoint_index)
                                ));
                            offset += core::mem::size_of::<protocol::EndpointDescriptor>();
                        }
                    }
                }
                offset
            }
            protocol::DescriptorType::String => {
                let mut offset = 2;
                if descriptor_index == 0 {
                    for (lang_id, _strings) in self.string_table.iter() {
                        buffer[offset] = *lang_id as u8;
                        offset += 1;
                        buffer[offset] = (*lang_id >> 8) as u8;
                        offset += 1;
                    }
                } else {
                    let search_lang_id =
                        self.common.memory.setup_packet_high.read().windex().bits();
                    let lang_index = self
                        .string_table
                        .iter()
                        .position(|(lang_id, _strings)| *lang_id == search_lang_id)
                        .unwrap();
                    let string = self.string_table[lang_index].1[descriptor_index - 1];
                    for word in string.encode_utf16() {
                        buffer[offset] = word as u8;
                        offset += 1;
                        buffer[offset] = (word >> 8) as u8;
                        offset += 1;
                    }
                }
                buffer[0] = offset as u8;
                buffer[1] = protocol::DescriptorType::String as u8;
                offset
            }
            _ => 0,
        }
    }

    fn handle_setup_packet(&mut self) {
        let request = deserialize_value!(
            protocol::Request,
            [self.common.memory.setup_packet_low.read().brequest().bits()]
        );

        match self
            .common
            .memory
            .setup_packet_low
            .read()
            .bmrequesttype()
            .bits()
            & 0x9F
        {
            0x00 => {
                // Host to Device
                let wvalue = self.common.memory.setup_packet_low.read().wvalue().bits();
                match request {
                    protocol::Request::ClearFeature => {
                        let _feature = wvalue;
                    }
                    protocol::Request::SetFeature => {
                        let _feature = wvalue;
                    }
                    protocol::Request::SetAddress => {
                        self.address = wvalue as u8;
                    }
                    protocol::Request::SetConfiguration => {
                        self.configuration = wvalue as u8;
                        self.common.reset_endpoint_pids();
                    }
                    protocol::Request::SetDescriptor => {
                        let _descriptor_type =
                            deserialize_value!(protocol::DescriptorType, [(wvalue >> 8) as u8]);
                        let _descriptor_index = wvalue as u8;
                    }
                    _ => {}
                }
                self.common.endpoint_transfer(0, true, 0);
            }
            0x01 => {
                // Host to Interface
                match request {
                    protocol::Request::ClearFeature => {
                        let _feature = self.common.memory.setup_packet_low.read().wvalue().bits();
                    }
                    protocol::Request::SetFeature => {
                        let _feature = self.common.memory.setup_packet_low.read().wvalue().bits();
                    }
                    protocol::Request::SetInterface => {
                        let _setting = self.common.memory.setup_packet_low.read().wvalue().bits();
                        self.common.reset_endpoint_pids();
                    }
                    _ => {}
                }
                self.common.endpoint_transfer(0, true, 0);
            }
            0x02 => {
                // Host to Endpoint
                match request {
                    protocol::Request::ClearFeature => {
                        let _feature = self.common.memory.setup_packet_low.read().wvalue().bits();
                    }
                    protocol::Request::SetFeature => {
                        let _feature = self.common.memory.setup_packet_low.read().wvalue().bits();
                    }
                    _ => {}
                }
                self.common.endpoint_transfer(0, true, 0);
            }
            0x80 => {
                // Device to Host
                let buffer = self.common.get_endpoint_buffer(0);
                match request {
                    protocol::Request::GetStatus => {
                        buffer[0] = 0;
                        buffer[1] = 0;
                        self.common.endpoint_transfer(0, true, 2);
                    }
                    protocol::Request::GetDescriptor => {
                        let wlength =
                            self.common.memory.setup_packet_high.read().wlength().bits() as usize;
                        let length = self.handle_get_descriptor_request(buffer);
                        self.common.endpoint_transfer(0, true, wlength.min(length));
                    }
                    protocol::Request::GetConfiguration => {
                        buffer[0] = self.configuration;
                        self.common.endpoint_transfer(0, true, 1);
                    }
                    _ => {}
                }
            }
            0x81 => {
                // Interface to Host
                let buffer = self.common.get_endpoint_buffer(0);
                match request {
                    protocol::Request::GetStatus => {
                        self.common.endpoint_transfer(0, true, 0);
                    }
                    protocol::Request::GetInterface => {
                        buffer[0] = 0;
                        self.common.endpoint_transfer(0, true, 1);
                    }
                    _ => {}
                }
            }
            0x82 => {
                // Endpoint to Host
                let buffer = self.common.get_endpoint_buffer(0);
                match request {
                    protocol::Request::GetStatus => {
                        buffer[0] = 0;
                        self.common.endpoint_transfer(0, true, 1);
                    }
                    protocol::Request::SynchFrame => {}
                    _ => {}
                }
            }
            _ => {}
        }
    }
}
