// sim7
// Copyright (c) 2024, Joshua Scoggins
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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

pub const ArchitectureLevel = enum {
    Core,
    Numerics,
    Protected,
    Extended,
};

pub const InstructionClass = enum(u2) {
    CTRL,
    COBR,
    REG,
    MEM,
    pub fn determine(opcode: u8) InstructionClass {
        return switch (comptime opcode) {
            0x00...0x1F => InstructionClass.CTRL,
            0x20...0x3F => InstructionClass.COBR,
            0x40...0x7F => InstructionClass.REG,
            0x80...0xFF => InstructionClass.MEM,
        };
    }
};

pub fn StorageFrame(
    comptime T: type,
    comptime count: comptime_int,
) type {
    return [count]T;
}

pub const RegisterFrame = StorageFrame(Ordinal, 16);
pub const MemoryPool = StorageFrame(ByteOrdinal, 4 * 1024 * 1024 * 1024);

pub const SegmentSelector = packed struct {
    _: u6 = 0b111_111,
    index: u26 = 0,

    const None = SegmentSelector{};
    pub fn toWholeValue(self: *const SegmentSelector) Ordinal {
        return @as(*const Ordinal, @ptrCast(self)).*;
    }
    pub fn make(value: u26) SegmentSelector {
        return SegmentSelector{ .index = value };
    }
    pub fn valid(self: *const SegmentSelector) bool {
        return self._ == 0b111_111;
    }
};
test "SegmentSelector tests" {
    try expectEqual(@sizeOf(Ordinal), @sizeOf(SegmentSelector));
    const t0 = SegmentSelector.make(0);
    try expectEqual(t0.toWholeValue(), 0b111_111);
    try expect(t0.valid());
    var t1 = SegmentSelector.make(0x10);
    try expectEqual(t1.toWholeValue(), 0b10000_111111);
    try expect(t1.valid());
    t1._ = 0b010101;
    try expect(!t1.valid());
}

const std = @import("std");
const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;
