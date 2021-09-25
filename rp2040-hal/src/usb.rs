//! Universal Serial Bus (USB)
//!
//! See [Chapter 4 Section 1](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details
//!
//! ## Usage
//!
//! ```no_run
//! use rp2040_hal::{pac, watchdog, clocks, usb};
//!
//! let mut pac = pac::Peripherals::take().unwrap();
//! let mut watchdog = watchdog::Watchdog::new(pac.WATCHDOG);
//! const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
//! let _clocks = clocks::init_clocks_and_plls(
//!     XOSC_CRYSTAL_FREQ, pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB, &mut pac.RESETS, &mut watchdog,
//! ).ok().unwrap();
//! let mut usb = usb::UsbDevice::new(pac.USBCTRL_REGS, pac.USBCTRL_DPRAM, &mut pac.RESETS);
//! ```

use crate::resets::SubsystemReset;
use core::ops::Deref;
use pac::{generic::Reg, usbctrl_dpram, usbctrl_regs, RESETS};

#[allow(dead_code)]
#[derive(Clone, Copy)]
enum RequestRecipient {
    Device = 0,
    Interface = 1,
    Endpoint = 2,
    Other = 3,
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
enum Request {
    GetStatus = 0x00,
    ClearFeature = 0x01,
    SetFeature = 0x03,
    SetAddress = 0x05,
    GetDescriptor = 0x06,
    SetDescriptor = 0x07,
    GetConfiguration = 0x08,
    SetConfiguration = 0x09,
    GetInterface = 0x0A,
    SetInterface = 0x11,
    SynchFrame = 0x12,
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
enum DescriptorType {
    Device = 1,
    Configuration = 2,
    String = 3,
    Interface = 4,
    Endpoint = 5,
    DeviceQualifier = 6,
    OtherSpeedConfiguration = 7,
    InterfacePower = 8,
    OnTheGo = 9,
    Debug = 10,
    InterfaceAssociation = 11,
}

#[repr(C)]
#[repr(packed)]
#[allow(non_snake_case)]
#[derive(Clone, Copy)]
pub struct DeviceDescriptor {
    bLength: u8,
    bDescriptorType: DescriptorType,
    bcdUSB: u16,
    bDeviceClass: u8,
    bDeviceSubClass: u8,
    bDeviceProtocol: u8,
    bMaxPacketSize0: u8,
    idVendor: u16,
    idProduct: u16,
    bcdDevice: u16,
    iManufacturer: u8,
    iProduct: u8,
    iSerialNumber: u8,
    bNumConfigurations: u8,
}

#[repr(C)]
#[repr(packed)]
#[allow(non_snake_case)]
#[derive(Clone, Copy)]
pub struct ConfigurationDescriptor {
    bLength: u8,
    bDescriptorType: DescriptorType,
    wTotalLength: u16,
    bNumInterfaces: u8,
    bConfigurationValue: u8,
    iConfiguration: u8,
    bmAttributes: u8,
    bMaxPower: u8,
}

#[repr(C)]
#[repr(packed)]
#[allow(non_snake_case)]
#[derive(Clone, Copy)]
pub struct InterfaceDescriptor {
    bLength: u8,
    bDescriptorType: DescriptorType,
    bInterfaceNumber: u8,
    bAlternateSetting: u8,
    bNumEndpoints: u8,
    bInterfaceClass: u8,
    bInterfaceSubClass: u8,
    bInterfaceProtocol: u8,
    iInterface: u8,
}

#[repr(C)]
#[repr(packed)]
#[allow(non_snake_case)]
#[derive(Clone, Copy)]
pub struct EndpointDescriptor {
    bLength: u8,
    bDescriptorType: DescriptorType,
    bEndpointAddress: u8,
    bmAttributes: u8,
    wMaxPacketSize: u16,
    bInterval: u8,
}

const NUMBER_OF_ENDPOINTS: u8 = 16;
const BUFFERS_START_OFFSET: u16 = 0x180;
const BUFFERS_END_OFFSET: u16 = 0x1000;

static USB_DEVICE_DESCRIPTOR: DeviceDescriptor = DeviceDescriptor {
    bLength: core::mem::size_of::<DeviceDescriptor>() as u8,
    bDescriptorType: DescriptorType::Device,
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

static USB_CONFIGURATION_DESCRIPTOR: ConfigurationDescriptor = ConfigurationDescriptor {
    bLength: core::mem::size_of::<ConfigurationDescriptor>() as u8,
    bDescriptorType: DescriptorType::Configuration,
    wTotalLength: (core::mem::size_of::<ConfigurationDescriptor>()
        + core::mem::size_of::<InterfaceDescriptor>()
        + core::mem::size_of::<EndpointDescriptor>() * 2) as u16,
    bNumInterfaces: 1,
    bConfigurationValue: 1, // Configuration 1
    iConfiguration: 4,      // String index
    bmAttributes: 0x80,     // attributes: Bus powered
    bMaxPower: 250,         // 500ma
};

static USB_INTERFACE_DESCRIPTOR: InterfaceDescriptor = InterfaceDescriptor {
    bLength: core::mem::size_of::<InterfaceDescriptor>() as u8,
    bDescriptorType: DescriptorType::Interface,
    bInterfaceNumber: 0,
    bAlternateSetting: 0,
    bNumEndpoints: 2,
    bInterfaceClass: 0xFF, // USB_CLASS_VENDOR_SPECIFIC
    bInterfaceSubClass: 0x00,
    bInterfaceProtocol: 0x00,
    iInterface: 5, // String index
};

const USB_STRING_TABLE: &[&'static str] =
    &["Raspberry Pi", "Rust Pi Pico", "3.14159", "Testing", "Echo"];

/// Pac USB control registers
pub trait UsbController: Deref<Target = usbctrl_regs::RegisterBlock> + SubsystemReset {}
impl UsbController for pac::USBCTRL_REGS {}

/// Pac USB dpram
pub trait UsbMemory: Deref<Target = usbctrl_dpram::RegisterBlock> {}
impl UsbMemory for pac::USBCTRL_DPRAM {}

enum BusState {
    Device(UsbDevice),
    // Host(UsbHost),
}

struct UsbCommon<C: UsbController, M: UsbMemory> {
    controller: C,
    memory: M,
}

impl<C: UsbController, M: UsbMemory> UsbCommon<C, M> {
    fn endpoint_address_to_index(endpoint_address: u8) -> u8 {
        (endpoint_address << 1) | ((endpoint_address >> 7) ^ 0x01)
    }

    fn endpoint_index_to_address(endpoint_index: u8) -> u8 {
        (endpoint_index >> 1) | ((endpoint_index ^ 0x01) << 7)
    }

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

    fn endpoint_buffer_control_register(
        &self,
        endpoint_index: u8,
    ) -> &'static Reg<usbctrl_dpram::ep0_in_buffer_control::EP0_IN_BUFFER_CONTROL_SPEC> {
        unsafe {
            &*(&self.memory.ep0_in_buffer_control
                as *const Reg<usbctrl_dpram::ep0_in_buffer_control::EP0_IN_BUFFER_CONTROL_SPEC>)
                .add(endpoint_index as usize)
        }
    }

    fn endpoint_address_control_register(
        &self,
        endpoint_index: u8,
    ) -> &'static Reg<usbctrl_regs::addr_endp1::ADDR_ENDP1_SPEC> {
        unsafe {
            &*(&self.controller.addr_endp1 as *const Reg<usbctrl_regs::addr_endp1::ADDR_ENDP1_SPEC>)
                .add((endpoint_index as usize >> 1) - 1)
        }
    }

    fn reset_endpoint_pids(&mut self) {
        for endpoint_index in 2..NUMBER_OF_ENDPOINTS * 2 {
            self.endpoint_buffer_control_register(endpoint_index)
                .modify(|_, w| w.pid_0().clear_bit().pid_1().clear_bit());
        }
    }

    fn handle_endpoint_buffers(&mut self) {
        let needs_handling = self.controller.buff_status.read().bits();
        for endpoint_index in 0..NUMBER_OF_ENDPOINTS * 2 {
            if (needs_handling & (1 << endpoint_index)) != 0 {
                // TODO: Trigger endpoint handler
                self.endpoint_buffer_control_register(endpoint_index)
                    .modify(|r, w| unsafe {
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

    fn configure_endpoint(
        &mut self,
        endpoint_index: u8,
        endpoint_type: usbctrl_dpram::ep1_in_control::ENDPOINT_TYPE_A,
        buffer_length: u16,
        double_buffered: bool,
    ) {
        assert_eq!(buffer_length & 63, 0);
        if endpoint_index < 2 {
            assert_eq!(endpoint_index & 1, 0);
            assert_eq!(
                endpoint_type,
                usbctrl_dpram::ep1_in_control::ENDPOINT_TYPE_A::CONTROL
            );
            assert_eq!(buffer_length, if double_buffered { 64 * 2 } else { 64 });
            self.controller.sie_ctrl.write(|w| {
                if double_buffered {
                    w.ep0_double_buf().set_bit();
                }
                // if int_stall { w.ep0_int_stall().set_bit(); }
                // if int_nak { w.ep0_int_nak().set_bit(); }
                w.ep0_int_1buf().set_bit()
            });
        } else {
            let register = self.endpoint_control_register(endpoint_index);
            let buffer_start = if endpoint_index == 2 {
                BUFFERS_START_OFFSET
            } else {
                register.read().buffer_address().bits()
            };
            register.write(|w| unsafe {
                if buffer_length > 0 {
                    w.enable().set_bit();
                }
                if double_buffered {
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
                register
                    .write(|w| unsafe { w.buffer_address().bits(buffer_start + buffer_length) });
            }
        }
    }

    fn endpoint_buffer(
        &mut self,
        endpoint_index: u8,
        use_received_length: bool,
    ) -> &'static mut [u8] {
        // TODO: Double buffering
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
        let double_buffer_index = (self.controller.buff_cpu_should_handle.read().bits()
            >> Self::endpoint_address_to_index(endpoint_index))
            & 1;
        let offset = double_buffer_index as usize * 64;
        let received_length = self
            .endpoint_buffer_control_register(endpoint_index)
            .read()
            .length_0()
            .bits() as usize;
        unsafe {
            core::slice::from_raw_parts_mut(
                core::mem::transmute::<_, *mut u8>(pac::USBCTRL_DPRAM::ptr())
                    .add(buffer_start as usize + offset),
                // core::mem::transmute::<_, *mut u8>(&self.memory).add(buffer_start as usize),
                if use_received_length {
                    received_length
                } else {
                    (buffer_end - buffer_start) as usize
                },
            )
        }
    }

    fn endpoint_descriptor(&mut self, endpoint_index: u8) -> EndpointDescriptor {
        let register_read = self.endpoint_control_register(endpoint_index).read();
        EndpointDescriptor {
            bLength: core::mem::size_of::<EndpointDescriptor>() as u8,
            bDescriptorType: DescriptorType::Endpoint,
            bEndpointAddress: Self::endpoint_index_to_address(endpoint_index),
            bmAttributes: register_read.endpoint_type().bits(),
            wMaxPacketSize: self.endpoint_buffer(endpoint_index, false).len() as u16,
            bInterval: 1,
        }
    }

    fn send(&mut self, endpoint_index: u8, data: &[u8]) {
        let buffer = self.endpoint_buffer(endpoint_index, false);
        assert!(data.len() <= buffer.len());
        buffer[0..data.len()].copy_from_slice(data);
        self.endpoint_buffer_control_register(endpoint_index)
            .modify(|r, w| unsafe {
                w.available_0()
                    .set_bit()
                    .length_0()
                    .bits(data.len() as u16)
                    .full_0()
                    .set_bit()
                    .stall()
                    .clear_bit()
            });
    }

    fn receive(&mut self, endpoint_index: u8) {
        self.endpoint_buffer_control_register(endpoint_index)
            .modify(|r, w| unsafe {
                w.available_0()
                    .set_bit()
                    .length_0()
                    .bits(self.endpoint_buffer(endpoint_index, false).len() as u16)
                    .full_0()
                    .clear_bit()
                    .stall()
                    .clear_bit()
            });
    }
}

pub struct Usb<C: UsbController, M: UsbMemory> {
    common: UsbCommon<C, M>,
    state: BusState,
}

static mut USB: *mut Usb<pac::USBCTRL_REGS, pac::USBCTRL_DPRAM> = core::ptr::null_mut();

impl<C: UsbController, M: UsbMemory> Usb<C, M> {
    /// Releases the underlying controller and memory.
    pub fn free(self) -> (C, M) {
        (self.common.controller, self.common.memory)
    }

    /// Is connected and not suspended
    pub fn is_running(&self) -> bool {
        self.common
            .controller
            .sie_status
            .read()
            .connected()
            .bit_is_set()
            && !self
                .common
                .controller
                .sie_status
                .read()
                .suspended()
                .bit_is_set()
    }

    /// Enable bus
    pub fn connect(&mut self) {
        unsafe {
            USB = core::mem::transmute(&*self);
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ);
        }
        match &self.state {
            BusState::Device(_device) => {
                self.common
                    .controller
                    .sie_ctrl
                    .modify(|_, w| w.pullup_en().set_bit());
            }
        }
    }

    /// Disable bus
    pub fn disconnect(&mut self) {
        unsafe {
            cortex_m::peripheral::NVIC::mask(pac::Interrupt::USBCTRL_IRQ);
            USB = core::ptr::null_mut();
        }
        match &self.state {
            BusState::Device(_device) => {
                self.common
                    .controller
                    .sie_ctrl
                    .modify(|_, w| w.pullup_en().clear_bit());
            }
        }
    }
}

pub struct UsbDevice {
    address: u8,
    configuration: u8,
}

impl UsbDevice {
    /// Resets the bus and becomes an USB device
    pub fn new<C: UsbController, M: UsbMemory>(
        controller: C,
        memory: M,
        resets: &mut RESETS,
    ) -> Usb<C, M> {
        let mut common = UsbCommon { controller, memory };
        common.controller.reset_bring_down(resets);
        common.controller.reset_bring_up(resets);

        for byte in unsafe {
            core::slice::from_raw_parts_mut(core::mem::transmute(pac::USBCTRL_DPRAM::ptr()), 0x1000)
        } {
            *byte = 0;
        }

        common
            .controller
            .usb_muxing
            .write(|w| w.to_phy().set_bit().softcon().set_bit());

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

        common.configure_endpoint(
            0,
            usbctrl_dpram::ep1_in_control::ENDPOINT_TYPE_A::CONTROL,
            64,
            false,
        );
        common.configure_endpoint(
            2,
            usbctrl_dpram::ep1_in_control::ENDPOINT_TYPE_A::BULK,
            0,
            false,
        );
        common.configure_endpoint(
            3,
            usbctrl_dpram::ep1_in_control::ENDPOINT_TYPE_A::BULK,
            64,
            false,
        );
        common.configure_endpoint(
            4,
            usbctrl_dpram::ep1_in_control::ENDPOINT_TYPE_A::BULK,
            64,
            false,
        );
        common.receive(3);

        Usb {
            common,
            state: BusState::Device(UsbDevice {
                address: 0,
                configuration: 0,
            }),
        }
    }

    fn handle_setup_packet<C: UsbController, M: UsbMemory>(
        &mut self,
        common: &mut UsbCommon<C, M>,
    ) {
        let request = unsafe {
            core::mem::transmute::<_, Request>(
                common.memory.setup_packet_low.read().brequest().bits(),
            )
        };

        match common.memory.setup_packet_low.read().bmrequesttype().bits() & 0x9F {
            0x00 => {
                // Host to Device
                let wvalue = common.memory.setup_packet_low.read().wvalue().bits();
                match request {
                    Request::ClearFeature => {
                        let _feature = wvalue;
                    }
                    Request::SetFeature => {
                        let _feature = wvalue;
                    }
                    Request::SetAddress => {
                        self.address = wvalue as u8;
                    }
                    Request::SetConfiguration => {
                        self.configuration = wvalue as u8;
                        common.reset_endpoint_pids();
                    }
                    Request::SetDescriptor => {
                        let _descriptor_type = unsafe {
                            core::mem::transmute::<_, DescriptorType>((wvalue >> 8) as u8)
                        };
                        let _descriptor_index = wvalue as u8;
                    }
                    _ => {}
                }
                common.send(0, &[]);
            }
            0x01 => {
                // Host to Interface
                match request {
                    Request::ClearFeature => {
                        let _feature = common.memory.setup_packet_low.read().wvalue().bits();
                    }
                    Request::SetFeature => {
                        let _feature = common.memory.setup_packet_low.read().wvalue().bits();
                    }
                    Request::SetInterface => {
                        let _setting = common.memory.setup_packet_low.read().wvalue().bits();
                        common.reset_endpoint_pids();
                    }
                    _ => {}
                }
                common.send(0, &[]);
            }
            0x02 => {
                // Host to Endpoint
                match request {
                    Request::ClearFeature => {
                        let _feature = common.memory.setup_packet_low.read().wvalue().bits();
                    }
                    Request::SetFeature => {
                        let _feature = common.memory.setup_packet_low.read().wvalue().bits();
                    }
                    _ => {}
                }
                common.send(0, &[]);
            }
            0x80 => {
                // Device to Host
                match request {
                    Request::GetStatus => {
                        common.send(0, &[0, 0]);
                    }
                    Request::GetDescriptor => {
                        let wlength =
                            common.memory.setup_packet_high.read().wlength().bits() as usize;
                        let wvalue = common.memory.setup_packet_low.read().wvalue().bits();
                        let descriptor_type = unsafe {
                            core::mem::transmute::<_, DescriptorType>((wvalue >> 8) as u8)
                        };
                        let descriptor_index = wvalue as u8;
                        match descriptor_type {
                            DescriptorType::Device => {
                                let buffer = unsafe {
                                    &core::mem::transmute::<_, [u8; 18]>(USB_DEVICE_DESCRIPTOR)
                                };
                                common.send(0, &buffer[0..wlength.min(buffer.len())]);
                            }
                            DescriptorType::Configuration => {
                                let mut buffer = [0; 32];
                                buffer[0..9].copy_from_slice(unsafe {
                                    &core::mem::transmute::<_, [u8; 9]>(
                                        USB_CONFIGURATION_DESCRIPTOR,
                                    )
                                });
                                buffer[9..18].copy_from_slice(unsafe {
                                    &core::mem::transmute::<_, [u8; 9]>(USB_INTERFACE_DESCRIPTOR)
                                });
                                buffer[18..25].copy_from_slice(unsafe {
                                    &core::mem::transmute::<_, [u8; 7]>(
                                        common.endpoint_descriptor(3),
                                    )
                                });
                                buffer[25..32].copy_from_slice(unsafe {
                                    &core::mem::transmute::<_, [u8; 7]>(
                                        common.endpoint_descriptor(4),
                                    )
                                });
                                common.send(0, &buffer[0..wlength.min(buffer.len())]);
                            }
                            DescriptorType::String => {
                                let _lang_id =
                                    common.memory.setup_packet_high.read().windex().bits();
                                let mut buffer = [0; 64];
                                let length = if descriptor_index == 0 {
                                    buffer[2] = 0x09;
                                    buffer[3] = 0x04;
                                    4
                                } else {
                                    let string = USB_STRING_TABLE[descriptor_index as usize - 1];
                                    let mut length = 2;
                                    for word in string.encode_utf16() {
                                        buffer[length] = word as u8;
                                        length += 1;
                                        buffer[length] = (word >> 8) as u8;
                                        length += 1;
                                    }
                                    length
                                };
                                buffer[0] = length as u8;
                                buffer[1] = DescriptorType::String as u8;
                                common.send(0, &buffer[0..wlength.min(length)]);
                            }
                            _ => {}
                        }
                    }
                    Request::GetConfiguration => {
                        common.send(0, &[self.configuration]);
                    }
                    _ => {}
                }
            }
            0x81 => {
                // Interface to Host
                match request {
                    Request::GetStatus => {
                        common.send(0, &[]);
                    }
                    Request::GetInterface => {
                        common.send(0, &[0]);
                    }
                    _ => {}
                }
            }
            0x82 => {
                // Endpoint to Host
                match request {
                    Request::GetStatus => {
                        common.send(0, &[0]);
                    }
                    Request::SynchFrame => {}
                    _ => {}
                }
            }
            _ => {}
        }
    }

    fn isr<C: UsbController, M: UsbMemory>(&mut self, common: &mut UsbCommon<C, M>) {
        if common.controller.ints.read().setup_req().bit_is_set() {
            common
                .endpoint_buffer_control_register(0)
                .modify(|_, w| w.pid_0().set_bit());
            self.handle_setup_packet(common);
            common
                .controller
                .sie_status
                .modify(|_, w| w.setup_rec().set_bit());
        }
        if common.controller.ints.read().buff_status().bit_is_set() {
            if (common.controller.buff_status.read().bits() & (1 << 0)) != 0 {
                common.receive(1);
                common
                    .controller
                    .addr_endp
                    .write(|w| unsafe { w.address().bits(self.address) });
            }
            /*if (common.controller.buff_status.read().bits() & (1 << 1)) != 0 {

            }*/
            if (common.controller.buff_status.read().bits() & (1 << 3)) != 0 {
                let buffer = common.endpoint_buffer(3, true);
                for character in buffer.iter_mut() {
                    if *character >= 97 && *character <= 122 {
                        *character -= 32;
                    }
                }
                common.send(4, buffer);
            }
            if (common.controller.buff_status.read().bits() & (1 << 4)) != 0 {
                common.receive(3);
            }
            common.handle_endpoint_buffers();
        }
        if common.controller.ints.read().bus_reset().bit_is_set() {
            self.address = 0;
            self.configuration = 0;
            common.controller.addr_endp.write(|w| w);
            common
                .controller
                .sie_status
                .modify(|_, w| w.bus_reset().set_bit());
        }
    }
}

/// USB interrupt service routine
#[allow(non_snake_case)]
pub unsafe extern "C" fn USBCTRL_IRQ() {
    match &mut (*USB).state {
        BusState::Device(usb_device) => {
            usb_device.isr(&mut (*USB).common);
        }
        /*BusState::Host(usb_host) => {
            usb_host.isr(&mut (*USB).common);
        }*/
    }
}
