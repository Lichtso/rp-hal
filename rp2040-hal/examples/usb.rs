//! Echo capitalized characters over USB
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use hal::clocks::init_clocks_and_plls;
use hal::pac;
use hal::usb::{protocol, EndpointConfiguration, EndpointHandler, StringTable, UsbDevice};
use hal::watchdog::Watchdog;
use panic_halt as _;
use rp2040_hal as hal;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

static USB_STRING_TABLE: &StringTable = &[(
    0x0409,
    &[
        "Raspberry Pi",
        "Rustberry Pi Pico",
        "3.14159",
        "Testing",
        "Echo",
    ],
)];

static USB_DEVICE_DESCRIPTOR: &protocol::DeviceDescriptor = &protocol::DeviceDescriptor {
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

static USB_CONFIGURATION_DESCRIPTORS: &[protocol::ConfigurationDescriptor] =
    &[protocol::ConfigurationDescriptor {
        bLength: core::mem::size_of::<protocol::ConfigurationDescriptor>() as u8,
        bDescriptorType: protocol::DescriptorType::Configuration,
        wTotalLength: (core::mem::size_of::<protocol::ConfigurationDescriptor>()
            + core::mem::size_of::<protocol::InterfaceDescriptor>() * 1
            + core::mem::size_of::<protocol::EndpointDescriptor>() * 3)
            as u16,
        bNumInterfaces: 1,
        bConfigurationValue: 1, // Configuration 1
        iConfiguration: 4,      // String index
        bmAttributes: 0x80,     // attributes: Bus powered
        bMaxPower: 250,         // 500ma
    }];

static USB_INTERFACE_DESCRIPTORS: &[protocol::InterfaceDescriptor] =
    &[protocol::InterfaceDescriptor {
        bLength: core::mem::size_of::<protocol::InterfaceDescriptor>() as u8,
        bDescriptorType: protocol::DescriptorType::Interface,
        bInterfaceNumber: 0,
        bAlternateSetting: 0,
        bNumEndpoints: 3,
        bInterfaceClass: 0xFF, // USB_CLASS_VENDOR_SPECIFIC
        bInterfaceSubClass: 0x00,
        bInterfaceProtocol: 0x00,
        iInterface: 5, // String index
    }];

static USB_INTERFACE_ENDPOINT_MAP: &[&[u8]] = &[&[2, 3, 4]];

static mut USB: *mut UsbDevice<pac::USBCTRL_REGS, pac::USBCTRL_DPRAM> = core::ptr::null_mut();

#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    (*USB).poll();
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let _clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let ring_buffer = core::cell::RefCell::new(RingBuffer {
        buffer: [0; 512],
        in_offset: 0,
        out_offset: 0,
    });
    let endpoints = [
        // 2: IN Endpoint 1, Device to Host
        EndpointConfiguration {
            kind: pac::usbctrl_dpram::ep_control::ENDPOINT_TYPE_A::BULK,
            buffer_length: 0, // Skip this endpoint / leave it disabled
            is_double_buffered: false,
            interval: 0,
        },
        // 3: OUT Endpoint 1, Host to Device
        EndpointConfiguration {
            kind: pac::usbctrl_dpram::ep_control::ENDPOINT_TYPE_A::BULK,
            buffer_length: 64,
            is_double_buffered: false,
            interval: 1 << 2,
        },
        // 4: IN Endpoint 2, Device to Host
        EndpointConfiguration {
            kind: pac::usbctrl_dpram::ep_control::ENDPOINT_TYPE_A::BULK,
            buffer_length: 64,
            is_double_buffered: false,
            interval: 1 << 2,
        },
    ];
    let mut handlers = [
        // 2: IN Endpoint 1, Device to Host
        (&mut |_: usize, _buffer: &mut [u8]| 0usize) as EndpointHandler,
        // 3: OUT Endpoint 1, Host to Device
        (&mut |in_avail: usize, buffer: &mut [u8]| {
            {
                let mut ring_buffer = ring_buffer.borrow_mut();
                let length = in_avail.min(ring_buffer.free());
                for input in buffer[0..length].iter() {
                    ring_buffer.push(if *input >= 97 && *input <= 122 {
                        *input - 32
                    } else {
                        *input
                    });
                }
            }
            unsafe {
                (*USB).trigger_endpoint(4); // 4: IN Endpoint 2, Device to Host
            }
            buffer.len()
        }) as EndpointHandler,
        // 4: IN Endpoint 2, Device to Host
        (&mut |_: usize, buffer: &mut [u8]| {
            let mut ring_buffer = ring_buffer.borrow_mut();
            let length = buffer.len().min(ring_buffer.filled());
            for output in buffer[0..length].iter_mut() {
                *output = ring_buffer.pop();
            }
            length
        }) as EndpointHandler,
    ];

    let mut usb = UsbDevice::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        &mut pac.RESETS,
        USB_STRING_TABLE,
        USB_DEVICE_DESCRIPTOR,
        USB_CONFIGURATION_DESCRIPTORS,
        USB_INTERFACE_DESCRIPTORS,
        USB_INTERFACE_ENDPOINT_MAP,
        &endpoints,
        &mut handlers,
    );
    unsafe {
        USB = core::mem::transmute(&mut usb);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ);
    }
    usb.enable();
    usb.trigger_endpoint(3); // 3: OUT Endpoint 1, Host to Device

    loop {
        unsafe {
            (*USB).poll();
        }
    }
}

struct RingBuffer {
    buffer: [u8; 512],
    in_offset: usize,
    out_offset: usize,
}

impl RingBuffer {
    pub fn push(&mut self, byte: u8) {
        self.buffer[self.in_offset] = byte;
        self.in_offset = (self.in_offset + 1) % self.buffer.len();
    }
    pub fn pop(&mut self) -> u8 {
        let byte = self.buffer[self.out_offset];
        self.out_offset = (self.out_offset + 1) % self.buffer.len();
        byte
    }
    pub fn free(&self) -> usize {
        (if self.in_offset < self.out_offset {
            0
        } else {
            self.buffer.len()
        }) + self.out_offset
            - self.in_offset
    }
    pub fn filled(&self) -> usize {
        (if self.in_offset >= self.out_offset {
            0
        } else {
            self.buffer.len()
        }) + self.in_offset
            - self.out_offset
    }
}
