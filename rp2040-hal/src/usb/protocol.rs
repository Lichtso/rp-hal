//! The USB Protocol
#![allow(missing_docs)]

#[derive(Clone, Copy)]
pub enum RequestRecipient {
    Device = 0,
    Interface = 1,
    Endpoint = 2,
    Other = 3,
}

#[derive(Clone, Copy)]
pub enum Request {
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

#[derive(Clone, Copy)]
pub enum DescriptorType {
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
    pub bLength: u8,
    pub bDescriptorType: DescriptorType,
    pub bcdUSB: u16,
    pub bDeviceClass: u8,
    pub bDeviceSubClass: u8,
    pub bDeviceProtocol: u8,
    pub bMaxPacketSize0: u8,
    pub idVendor: u16,
    pub idProduct: u16,
    pub bcdDevice: u16,
    pub iManufacturer: u8,
    pub iProduct: u8,
    pub iSerialNumber: u8,
    pub bNumConfigurations: u8,
}

#[repr(C)]
#[repr(packed)]
#[allow(non_snake_case)]
#[derive(Clone, Copy)]
pub struct ConfigurationDescriptor {
    pub bLength: u8,
    pub bDescriptorType: DescriptorType,
    pub wTotalLength: u16,
    pub bNumInterfaces: u8,
    pub bConfigurationValue: u8,
    pub iConfiguration: u8,
    pub bmAttributes: u8,
    pub bMaxPower: u8,
}

#[repr(C)]
#[repr(packed)]
#[allow(non_snake_case)]
#[derive(Clone, Copy)]
pub struct InterfaceDescriptor {
    pub bLength: u8,
    pub bDescriptorType: DescriptorType,
    pub bInterfaceNumber: u8,
    pub bAlternateSetting: u8,
    pub bNumEndpoints: u8,
    pub bInterfaceClass: u8,
    pub bInterfaceSubClass: u8,
    pub bInterfaceProtocol: u8,
    pub iInterface: u8,
}

#[repr(C)]
#[repr(packed)]
#[allow(non_snake_case)]
#[derive(Clone, Copy)]
pub struct EndpointDescriptor {
    pub bLength: u8,
    pub bDescriptorType: DescriptorType,
    pub bEndpointAddress: u8,
    pub bmAttributes: u8,
    pub wMaxPacketSize: u16,
    pub bInterval: u8,
}
