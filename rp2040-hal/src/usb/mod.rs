//! Universal Serial Bus (USB)
//!
//! See [Chapter 4 Section 1](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details
//!
//! ## Usage
//!
//! ```no_run
//! use rp2040_hal::{pac, watchdog, clocks, usb};
//!
//! static mut USB: *mut usb::Usb<pac::USBCTRL_REGS, pac::USBCTRL_DPRAM> = core::ptr::null_mut();
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

static USB_DEVICE_DESCRIPTOR: protocol::DeviceDescriptor = protocol::DeviceDescriptor {
    bLength: core::mem::size_of::<protocol::DeviceDescriptor>() as u8,
    bDescriptorType: protocol::DescriptorType::Device,
    bcdUSB: 0x0110,        // USB version
    bDeviceClass: 0xEF,    // USB_CLASS_MISC
    bDeviceSubClass: 2,    // MISC_SUBCLASS_COMMON
    bDeviceProtocol: 1,    // MISC_PROTOCOL_IAD
    bMaxPacketSize0: 64,   // Max packet size for ep0
    idVendor: 0x2E8A,      // Raspberry Pi
    idProduct: 0x000A,     // Pico SDK CDC
    bcdDevice: 0x0100,     // No device revision number
    iManufacturer: 1,      // String index
    iProduct: 2,           // String index
    iSerialNumber: 3,      // String index
    bNumConfigurations: 1, // One configuration
};

static USB_CONFIGURATION_DESCRIPTOR: protocol::ConfigurationDescriptor =
    protocol::ConfigurationDescriptor {
        bLength: core::mem::size_of::<protocol::ConfigurationDescriptor>() as u8,
        bDescriptorType: protocol::DescriptorType::Configuration,
        wTotalLength: (core::mem::size_of::<protocol::ConfigurationDescriptor>()
            + core::mem::size_of::<protocol::InterfaceDescriptor>()
            + core::mem::size_of::<protocol::EndpointDescriptor>() * 2)
            as u16,
        bNumInterfaces: 1,
        bConfigurationValue: 1, // Configuration 1
        iConfiguration: 4,      // String index
        bmAttributes: 0x80,     // attributes: Bus powered
        bMaxPower: 250,         // 500ma
    };

static USB_INTERFACE_DESCRIPTOR: protocol::InterfaceDescriptor = protocol::InterfaceDescriptor {
    bLength: core::mem::size_of::<protocol::InterfaceDescriptor>() as u8,
    bDescriptorType: protocol::DescriptorType::Interface,
    bInterfaceNumber: 0,
    bAlternateSetting: 0,
    bNumEndpoints: 2,
    bInterfaceClass: 0xFF, // USB_CLASS_VENDOR_SPECIFIC
    bInterfaceSubClass: 0x00,
    bInterfaceProtocol: 0x00,
    iInterface: 5, // String index
};

const USB_STRING_TABLE: &[(u16, &[&'static str])] = &[(
    0x0409,
    &["Raspberry Pi", "Rust Pi Pico", "3.14159", "Testing", "Echo"],
)];

/// Pac USB control registers
pub trait UsbController: Deref<Target = usbctrl_regs::RegisterBlock> + SubsystemReset {}
impl UsbController for pac::USBCTRL_REGS {}

/// Pac USB dpram
pub trait UsbMemory: Deref<Target = usbctrl_dpram::RegisterBlock> {}
impl UsbMemory for pac::USBCTRL_DPRAM {}

enum BusState<C: UsbController, M: UsbMemory> {
    Device(UsbDevice<C, M>),
    // Host(UsbHost<C, M>),
}

fn endpoint_index_to_address(endpoint_index: u8) -> u8 {
    (endpoint_index >> 1) | ((endpoint_index ^ 0x01) << 7)
}

/*fn endpoint_address_to_index(endpoint_address: u8) -> u8 {
    (endpoint_address << 1) | ((endpoint_address >> 7) ^ 0x01)
}*/

struct UsbCommon<C: UsbController, M: UsbMemory> {
    controller: C,
    memory: M,
}

impl<C: UsbController, M: UsbMemory> UsbCommon<C, M> {
    fn endpoint_control_register(
        &self,
        endpoint_index: u8,
    ) -> &'static Reg<usbctrl_dpram::ep1_in_control::EP1_IN_CONTROL_SPEC> {
        unsafe {
            &*(&self.memory.ep1_in_control
                as *const Reg<usbctrl_dpram::ep1_in_control::EP1_IN_CONTROL_SPEC>)
                .add(endpoint_index as usize - 2)
        }
    }

    fn get_endpoint_buffer_control_register(
        &self,
        endpoint_index: u8,
    ) -> &'static Reg<usbctrl_dpram::ep0_in_buffer_control::EP0_IN_BUFFER_CONTROL_SPEC> {
        unsafe {
            &*(&self.memory.ep0_in_buffer_control
                as *const Reg<usbctrl_dpram::ep0_in_buffer_control::EP0_IN_BUFFER_CONTROL_SPEC>)
                .add(endpoint_index as usize)
        }
    }

    /*fn endpoint_address_control_register(
        &self,
        endpoint_index: u8,
    ) -> &'static Reg<usbctrl_regs::addr_endp1::ADDR_ENDP1_SPEC> {
        unsafe {
            &*(&self.controller.addr_endp1 as *const Reg<usbctrl_regs::addr_endp1::ADDR_ENDP1_SPEC>)
                .add((endpoint_index as usize >> 1) - 1)
        }
    }*/

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
    }

    fn reset_endpoint_pids(&mut self) {
        for endpoint_index in 2..NUMBER_OF_ENDPOINTS * 2 {
            self.get_endpoint_buffer_control_register(endpoint_index)
                .modify(|_, w| w.pid_0().clear_bit().pid_1().clear_bit());
        }
    }

    fn handle_endpoint_buffers(&mut self) {
        let needs_handling = self.controller.buff_status.read().bits();
        for endpoint_index in 0..NUMBER_OF_ENDPOINTS * 2 {
            if (needs_handling & (1 << endpoint_index)) != 0 {
                // TODO: Trigger endpoint handler
                self.get_endpoint_buffer_control_register(endpoint_index)
                    .modify(|r, w| {
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

    fn allocate_endpoint(
        &mut self,
        endpoint_index: u8,
        endpoint_type: usbctrl_dpram::ep1_in_control::ENDPOINT_TYPE_A,
        buffer_length: u16,
        is_double_buffered: bool,
        interval: u16,
    ) {
        assert!(endpoint_index >= 2);
        assert_eq!(buffer_length & 63, 0);
        let register = self.endpoint_control_register(endpoint_index);
        let buffer_start = if endpoint_index == 2 {
            BUFFERS_START_OFFSET
        } else {
            register.read().buffer_address().bits()
        };
        register.write(|w| unsafe {
            w.bits((interval as u32 & 0x3FF) << 16);
            if buffer_length > 0 {
                w.enable().set_bit();
            }
            if is_double_buffered {
                w.double_buffered().set_bit();
            }
            // if int_stall { w.interrupt_on_stall().set_bit(); }
            // if int_nak { w.interrupt_on_nak().set_bit(); }
            w.interrupt_per_buff()
                .set_bit()
                .endpoint_type()
                .bits(endpoint_type as u8)
                .buffer_address()
                .bits(buffer_start)
        });
        if endpoint_index + 1 < NUMBER_OF_ENDPOINTS {
            let register = self.endpoint_control_register(endpoint_index + 1);
            register.write(|w| unsafe { w.buffer_address().bits(buffer_start + buffer_length) });
        }
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

    fn get_endpoint_buffer(
        &mut self,
        endpoint_index: u8,
        use_received_length: bool,
    ) -> &'static mut [u8] {
        let (buffer_start, buffer_end) = if endpoint_index < 2 {
            (BUFFERS_START_OFFSET - 64 * 2, BUFFERS_START_OFFSET - 64)
        } else {
            (
                if endpoint_index == 2 {
                    BUFFERS_START_OFFSET
                } else {
                    self.endpoint_control_register(endpoint_index)
                        .read()
                        .buffer_address()
                        .bits()
                },
                if endpoint_index + 1 >= NUMBER_OF_ENDPOINTS {
                    BUFFERS_END_OFFSET
                } else {
                    self.endpoint_control_register(endpoint_index + 1)
                        .read()
                        .buffer_address()
                        .bits()
                },
            )
        };
        let double_buffer_length = (buffer_end - buffer_start) as usize;
        let (is_double_buffered, double_buffer_index) =
            self.get_endpoint_double_buffering(endpoint_index);
        let buffer_length = double_buffer_length >> if is_double_buffered { 1 } else { 0 };
        unsafe {
            core::slice::from_raw_parts_mut(
                core::mem::transmute::<_, *mut u8>(pac::USBCTRL_DPRAM::ptr())
                    .add(buffer_start as usize + double_buffer_index as usize * buffer_length),
                // core::mem::transmute::<_, *mut u8>(&self.memory).add(buffer_start as usize),
                if use_received_length {
                    let register_read = self
                        .get_endpoint_buffer_control_register(endpoint_index)
                        .read();
                    if double_buffer_index == 0 {
                        register_read.length_0().bits() as usize
                    } else {
                        register_read.length_1().bits() as usize
                    }
                } else {
                    buffer_length
                },
            )
        }
    }

    fn endpoint_send(&mut self, endpoint_index: u8, length: usize) {
        let (_, double_buffer_index) = self.get_endpoint_double_buffering(endpoint_index);
        self.get_endpoint_buffer_control_register(endpoint_index)
            .modify(|_, w| unsafe {
                if double_buffer_index == 0 {
                    w.available_0()
                        .set_bit()
                        .length_0()
                        .bits(length as u16)
                        .full_0()
                        .set_bit()
                } else {
                    w.available_1()
                        .set_bit()
                        .length_1()
                        .bits(length as u16)
                        .full_1()
                        .set_bit()
                }
            });
    }

    fn endpoint_receive(&mut self, endpoint_index: u8) {
        let (_, double_buffer_index) = self.get_endpoint_double_buffering(endpoint_index);
        self.get_endpoint_buffer_control_register(endpoint_index)
            .modify(|_, w| unsafe {
                if double_buffer_index == 0 {
                    w.available_0()
                        .set_bit()
                        .length_0()
                        .bits(self.get_endpoint_buffer(endpoint_index, false).len() as u16)
                        .full_0()
                        .clear_bit()
                } else {
                    w.available_1()
                        .set_bit()
                        .length_1()
                        .bits(self.get_endpoint_buffer(endpoint_index, false).len() as u16)
                        .full_1()
                        .clear_bit()
                }
            });
    }

    /*fn set_endpoint_stalled(&mut self, endpoint_index: u8, is_stalled: bool) {
        self.get_endpoint_buffer_control_register(endpoint_index)
            .modify(|_, w| w.stall().bit(is_stalled));
    }

    fn is_endpoint_stalled(&mut self, endpoint_index: u8) -> bool {
        self.get_endpoint_buffer_control_register(endpoint_index)
            .read()
            .stall()
            .bit_is_set()
    }*/
}

/// Represents the USB bus, which can be device or host
pub struct Usb<C: UsbController, M: UsbMemory> {
    state: BusState<C, M>,
}

impl<C: UsbController, M: UsbMemory> Usb<C, M> {
    /// Releases the underlying controller and memory
    pub fn free(self) -> (C, M) {
        match self.state {
            BusState::Device(device) => (device.common.controller, device.common.memory),
        }
    }

    /// Is connected and not suspended
    pub fn is_running(&self) -> bool {
        match &self.state {
            BusState::Device(device) => {
                let status = device.common.controller.sie_status.read();
                status.connected().bit_is_set() && !status.suspended().bit_is_set()
            }
        }
    }

    /// Enable bus
    pub fn enable(&mut self) {
        match &self.state {
            BusState::Device(device) => {
                device
                    .common
                    .controller
                    .sie_ctrl
                    .modify(|_, w| w.pullup_en().set_bit());
            }
        }
    }

    /// Disable bus
    pub fn disable(&mut self) {
        match &self.state {
            BusState::Device(device) => {
                device
                    .common
                    .controller
                    .sie_ctrl
                    .modify(|_, w| w.pullup_en().clear_bit());
            }
        }
    }

    /// Call this in USBCTRL_IRQ
    pub fn poll(&mut self) {
        match &mut self.state {
            BusState::Device(usb_device) => usb_device.poll(),
        }
    }
}

/// Represents the USB bus configured as a device
pub struct UsbDevice<C: UsbController, M: UsbMemory> {
    common: UsbCommon<C, M>,
    address: u8,
    configuration: u8,
}

impl<C: UsbController, M: UsbMemory> UsbDevice<C, M> {
    /// Resets the bus and becomes an USB device
    pub fn new(controller: C, memory: M, resets: &mut RESETS) -> Usb<C, M> {
        let mut common = UsbCommon { controller, memory };
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

        common.allocate_endpoint(
            2,
            usbctrl_dpram::ep1_in_control::ENDPOINT_TYPE_A::BULK,
            0,
            false,
            0,
        );
        common.allocate_endpoint(
            3,
            usbctrl_dpram::ep1_in_control::ENDPOINT_TYPE_A::BULK,
            64,
            false,
            0,
        );
        common.allocate_endpoint(
            4,
            usbctrl_dpram::ep1_in_control::ENDPOINT_TYPE_A::BULK,
            64,
            false,
            0,
        );
        common.endpoint_receive(3);

        Usb {
            state: BusState::<C, M>::Device(UsbDevice::<C, M> {
                common,
                address: 0,
                configuration: 0,
            }),
        }
    }

    fn endpoint_descriptor(&mut self, endpoint_index: u8) -> protocol::EndpointDescriptor {
        let register_read = self.common.endpoint_control_register(endpoint_index).read();
        protocol::EndpointDescriptor {
            bLength: core::mem::size_of::<protocol::EndpointDescriptor>() as u8,
            bDescriptorType: protocol::DescriptorType::Endpoint,
            bEndpointAddress: endpoint_index_to_address(endpoint_index),
            bmAttributes: register_read.endpoint_type().bits(),
            wMaxPacketSize: self.common.get_endpoint_buffer(endpoint_index, false).len() as u16,
            bInterval: (register_read.bits() >> 18) as u8,
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
                self.common.endpoint_send(0, 0);
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
                self.common.endpoint_send(0, 0);
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
                self.common.endpoint_send(0, 0);
            }
            0x80 => {
                // Device to Host
                let buffer = self.common.get_endpoint_buffer(0, false);
                match request {
                    protocol::Request::GetStatus => {
                        buffer[0] = 0;
                        buffer[1] = 0;
                        self.common.endpoint_send(0, 2);
                    }
                    protocol::Request::GetDescriptor => {
                        let wlength =
                            self.common.memory.setup_packet_high.read().wlength().bits() as usize;
                        let wvalue = self.common.memory.setup_packet_low.read().wvalue().bits();
                        let descriptor_type =
                            deserialize_value!(protocol::DescriptorType, [(wvalue >> 8) as u8]);
                        let descriptor_index = wvalue as u8;
                        match descriptor_type {
                            protocol::DescriptorType::Device => {
                                buffer[0..18].copy_from_slice(serialize_value!(
                                    protocol::DeviceDescriptor,
                                    USB_DEVICE_DESCRIPTOR
                                ));
                                self.common.endpoint_send(0, wlength.min(18));
                            }
                            protocol::DescriptorType::Configuration => {
                                buffer[0..9].copy_from_slice(serialize_value!(
                                    protocol::ConfigurationDescriptor,
                                    USB_CONFIGURATION_DESCRIPTOR
                                ));
                                buffer[9..18].copy_from_slice(serialize_value!(
                                    protocol::InterfaceDescriptor,
                                    USB_INTERFACE_DESCRIPTOR
                                ));
                                buffer[18..25].copy_from_slice(serialize_value!(
                                    protocol::EndpointDescriptor,
                                    self.endpoint_descriptor(3)
                                ));
                                buffer[25..32].copy_from_slice(serialize_value!(
                                    protocol::EndpointDescriptor,
                                    self.endpoint_descriptor(4)
                                ));
                                self.common.endpoint_send(0, wlength.min(32));
                            }
                            protocol::DescriptorType::String => {
                                let mut length = 2;
                                if descriptor_index == 0 {
                                    for (lang_id, _strings) in USB_STRING_TABLE.iter() {
                                        buffer[length] = *lang_id as u8;
                                        length += 1;
                                        buffer[length] = (*lang_id >> 8) as u8;
                                        length += 1;
                                    }
                                } else {
                                    let search_lang_id =
                                        self.common.memory.setup_packet_high.read().windex().bits();
                                    let lang_index = USB_STRING_TABLE
                                        .iter()
                                        .position(|(lang_id, _strings)| *lang_id == search_lang_id)
                                        .unwrap();
                                    let string = USB_STRING_TABLE[lang_index].1
                                        [descriptor_index as usize - 1];
                                    for word in string.encode_utf16() {
                                        buffer[length] = word as u8;
                                        length += 1;
                                        buffer[length] = (word >> 8) as u8;
                                        length += 1;
                                    }
                                }
                                buffer[0] = length as u8;
                                buffer[1] = protocol::DescriptorType::String as u8;
                                self.common.endpoint_send(0, wlength.min(length));
                            }
                            _ => {}
                        }
                    }
                    protocol::Request::GetConfiguration => {
                        buffer[0] = self.configuration;
                        self.common.endpoint_send(0, 1);
                    }
                    _ => {}
                }
            }
            0x81 => {
                // Interface to Host
                let buffer = self.common.get_endpoint_buffer(0, false);
                match request {
                    protocol::Request::GetStatus => {
                        self.common.endpoint_send(0, 0);
                    }
                    protocol::Request::GetInterface => {
                        buffer[0] = 0;
                        self.common.endpoint_send(0, 1);
                    }
                    _ => {}
                }
            }
            0x82 => {
                // Endpoint to Host
                let buffer = self.common.get_endpoint_buffer(0, false);
                match request {
                    protocol::Request::GetStatus => {
                        buffer[0] = 0;
                        self.common.endpoint_send(0, 1);
                    }
                    protocol::Request::SynchFrame => {}
                    _ => {}
                }
            }
            _ => {}
        }
    }

    fn poll(&mut self) {
        if self.common.controller.ints.read().setup_req().bit_is_set() {
            self.common
                .get_endpoint_buffer_control_register(0)
                .modify(|_, w| w.pid_0().set_bit());
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
            if (self.common.controller.buff_status.read().bits() & (1 << 0)) != 0 {
                self.common.endpoint_receive(1);
                self.common
                    .controller
                    .addr_endp
                    .write(|w| unsafe { w.address().bits(self.address) });
            }
            /*if (self.common.controller.buff_status.read().bits() & (1 << 1)) != 0 {

            }*/
            if (self.common.controller.buff_status.read().bits() & (1 << 3)) != 0 {
                let in_buffer = self.common.get_endpoint_buffer(3, true);
                let out_buffer = self.common.get_endpoint_buffer(4, false);
                for (input, output) in in_buffer.iter().zip(out_buffer.iter_mut()) {
                    *output = if *input >= 97 && *input <= 122 {
                        *input - 32
                    } else {
                        *input
                    };
                }
                self.common.endpoint_send(4, in_buffer.len());
            }
            if (self.common.controller.buff_status.read().bits() & (1 << 4)) != 0 {
                self.common.endpoint_receive(3);
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
}
