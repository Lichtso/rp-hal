macro_rules! serialize_value {
    ($value_type:ty, $value:expr) => {
        unsafe {
            &core::mem::transmute::<$value_type, [u8; core::mem::size_of::<$value_type>()]>($value)
        }
    };
}

macro_rules! deserialize_value {
    ($value_type:ty, $slice:expr) => {
        unsafe {
            &core::mem::transmute::<[u8; core::mem::size_of::<$value_type>()], $value_type>($slice)
        }
    };
}
