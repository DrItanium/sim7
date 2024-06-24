pub const Operand = u5;

pub const ByteInteger = i8;
pub const ByteOrdinal = u8;
pub const ShortInteger = i16;
pub const ShortOrdinal = u16;
pub const Integer = i32;
pub const Ordinal = u32;
pub const LongOrdinal = u64;
pub const LongInteger = i64;
pub const TripleOrdinal = u96;
pub const QuadOrdinal = u128;
pub const Address = u32;

pub const Real = f32;
pub const LongReal = f64;
pub const ExtendedReal = f80;
pub fn StorageFrame(
    comptime T: type,
    comptime count: comptime_int,
) type {
    return [count]T;
}

pub const RegisterFrame = StorageFrame(Ordinal, 16);
