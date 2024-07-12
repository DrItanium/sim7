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

const std = @import("std");
const math = @import("std").math;
const coreTypes = @import("types.zig");
const faults = @import("Faults.zig");
const opcodes = @import("Opcode.zig");
const main = @import("main.zig");
const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;
const ByteOrdinal = coreTypes.ByteOrdinal;
const ByteInteger = coreTypes.ByteInteger;
const ShortOrdinal = coreTypes.ShortOrdinal;
const ShortInteger = coreTypes.ShortInteger;
const Ordinal = coreTypes.Ordinal;
const Integer = coreTypes.Integer;
const LongOrdinal = coreTypes.LongOrdinal;
const LongInteger = coreTypes.LongInteger;
const TripleOrdinal = coreTypes.TripleOrdinal;
const QuadOrdinal = coreTypes.QuadOrdinal;
const Operand = coreTypes.Operand;
const ExtendedReal = coreTypes.ExtendedReal;
const Address = coreTypes.Address;
const FaultRecord = faults.FaultRecord;
const Faults = faults.Faults;
const FaultKind = faults.FaultKind;
const FaultTable = faults.FaultTable;
const RegisterFrame = coreTypes.RegisterFrame;
const MemoryPool = coreTypes.MemoryPool;
const SegmentSelector = coreTypes.SegmentSelector;
const FaultTableEntry = faults.FaultTableEntry;
const GenericSegmentDescriptor = main.GenericSegmentDescriptor;

const io = std.io;
const stdin = io.getStdIn().reader();
const stdout = io.getStdOut().writer();

const allocator = std.heap.page_allocator;
var pool: *MemoryPool = undefined;
var setup: bool = false;
pub fn begin() !void {
    if (!setup) {
        setup = true;
        pool = try allocator.create(MemoryPool);
        for (pool) |*cell| {
            cell.* = 0;
        }
    }
}
pub fn end() void {
    allocator.destroy(pool);
    setup = false;
}
// we only will ever have a single instance of this type
pub fn setBackingStore(mem: *MemoryPool) void {
    pool = mem;
}
pub fn getBackingStore() *MemoryPool {
    return pool;
}
// need to access byte by byte so we can reconstruct in a platform independent
// way
fn memoryLoad(
    comptime T: type,
    address: Address,
) T {
    // todo, at some point we need to take platform endianness into account!
    return switch (T) {
        ByteOrdinal, ByteInteger => @bitCast(pool[address]),
        ShortOrdinal, ShortInteger => {
            if ((address & 0b1) == 0) {
                const properView: *[]ShortOrdinal = @ptrFromInt(@intFromPtr(&pool));
                return @bitCast(properView.*[address >> 1]);
            } else {
                const a: ShortOrdinal = load(ByteOrdinal, address);
                const b: ShortOrdinal = load(ByteOrdinal, address +% 1);
                return @bitCast(a | (b << 8));
            }
        },
        Ordinal, Integer => {
            if ((address & 0b11) == 0) {
                const properView: *[]Ordinal = @ptrFromInt(@intFromPtr(&pool));
                return @bitCast(properView.*[address >> 2]);
            } else {
                const a: Ordinal = load(ShortOrdinal, address);
                const b: Ordinal = load(ShortOrdinal, address +% 2);
                return @bitCast(a | (b << 16));
            }
        },
        LongOrdinal, LongInteger => {
            if ((address & 0b111) == 0) {
                const properView: *[]LongOrdinal = @ptrFromInt(@intFromPtr(&pool));
                return @bitCast(properView.*[address >> 3]);
            } else {
                const a: LongOrdinal = load(Ordinal, address);
                const b: LongOrdinal = load(Ordinal, address +% 4);
                return @bitCast(a | (b << 32));
            }
        },
        TripleOrdinal => {
            if ((address & 0b1111) == 0) {
                // triple ordinals are strange as they are 96 bits in size.
                // However, the i960 treats them as aligned only on 16 byte
                // boundaries (same as quad ordinals) so we can use the
                // optimization only when aligned
                const properView: *[]TripleOrdinal = @ptrFromInt(@intFromPtr(&pool));
                return properView.*[address >> 4];
            } else if ((address & 0b11) == 0) {
                // cool beans, the address is actually aligned to 32-bit
                // boundaries so we can actually do three separate optimized
                // ordinal loads
                const adjustedAddress = address >> 2;
                const properView: *[]Ordinal = @ptrFromInt(@intFromPtr(&pool));
                const a: TripleOrdinal = properView.*[adjustedAddress +% 0];
                const b: TripleOrdinal = properView.*[adjustedAddress +% 1];
                const c: TripleOrdinal = properView.*[adjustedAddress +% 2];
                return a | (b << 32) | (c << 64);
            } else {
                // load three separate ordinals, it may turn out that some of
                // them are aligned actually!
                const a: TripleOrdinal = load(Ordinal, address);
                const b: TripleOrdinal = load(Ordinal, address +% 4);
                const c: TripleOrdinal = load(Ordinal, address +% 8);
                return a | (b << 32) | (c << 64);
            }
        },
        QuadOrdinal => {
            if ((address & 0b1111) == 0) {
                // triple ordinals are strange as they are 96 bits in size.
                // However, the i960 treats them as aligned only on 16 byte
                // boundaries (same as quad ordinals) so we can use the
                // optimization only when aligned
                const properView: *[]QuadOrdinal = @ptrFromInt(@intFromPtr(&pool));
                return properView.*[address >> 4];
            } else if ((address & 0b111) == 0) {
                const adjustedAddress = address >> 3;
                const properView: *[]LongOrdinal = @ptrFromInt(@intFromPtr(&pool));
                const a: QuadOrdinal = properView.*[adjustedAddress +% 0];
                const b: QuadOrdinal = properView.*[adjustedAddress +% 1];
                return a | (b << 64);
            } else if ((address & 0b11) == 0) {
                // cool beans, the address is actually aligned to 32-bit
                // boundaries so we can actually do three separate optimized
                // ordinal loads
                const adjustedAddress = address >> 2;
                const properView: *[]Ordinal = @ptrFromInt(@intFromPtr(&pool));
                const a: QuadOrdinal = properView.*[adjustedAddress +% 0];
                const b: QuadOrdinal = properView.*[adjustedAddress +% 1];
                const c: QuadOrdinal = properView.*[adjustedAddress +% 2];
                const d: QuadOrdinal = properView.*[adjustedAddress +% 3];
                return a | (b << 32) | (c << 64) | (d << 96);
            } else {
                // load three separate ordinals, it may turn out that some of
                // them are aligned actually!
                const a: QuadOrdinal = load(Ordinal, address);
                const b: QuadOrdinal = load(Ordinal, address +% 4);
                const c: QuadOrdinal = load(Ordinal, address +% 8);
                const d: QuadOrdinal = load(Ordinal, address +% 12);
                return a | (b << 32) | (c << 64) | (d << 96);
            }
        },
        FaultTableEntry => FaultTableEntry.make(load(LongOrdinal, address)),
        GenericSegmentDescriptor => {
            var descriptor = GenericSegmentDescriptor{};
            descriptor.setWholeValue(load(QuadOrdinal, address));
            return descriptor;
        },
        else => @compileError("Requested type not allowed!"),
    };
}
fn memoryStore(
    comptime T: type,
    address: Address,
    value: T,
) void {
    switch (T) {
        ByteOrdinal, ByteInteger => pool[address] = @bitCast(value),
        ShortOrdinal, ShortInteger => {
            const view: ShortOrdinal = @bitCast(value);
            if ((address & 0b1) == 0) {
                const properView: *[]ShortOrdinal = @ptrFromInt(@intFromPtr(&pool));
                properView.*[address >> 1] = view;
            } else {
                pool[address] = @truncate(view);
                pool[address +% 1] = @truncate(view >> 8);
            }
        },
        Ordinal, Integer => {
            const view: Ordinal = @bitCast(value);
            if ((address & 0b11) == 0) {
                const properView: *[]Ordinal = @ptrFromInt(@intFromPtr(&pool));
                properView.*[address >> 2] = view;
            } else {
                store(ShortOrdinal, address, @truncate(view));
                store(ShortOrdinal, address +% 2, @truncate(view >> 16));
            }
        },
        LongOrdinal, LongInteger => {
            const view: LongOrdinal = @bitCast(value);
            if ((address & 0b111) == 0) {
                const properView: *[]LongOrdinal = @ptrFromInt(@intFromPtr(&pool));
                properView.*[address >> 3] = view;
            } else {
                store(Ordinal, address, @truncate(view));
                store(Ordinal, address +% 4, @truncate(view >> 32));
            }
        },
        TripleOrdinal => {
            if ((address & 0b1111) == 0) {
                const properView: *[]TripleOrdinal = @ptrFromInt(@intFromPtr(&pool));
                properView.*[address >> 4] = value;
            } else {
                store(Ordinal, address, @truncate(value));
                store(Ordinal, address +% 4, @truncate(value >> 32));
                store(Ordinal, address +% 8, @truncate(value >> 64));
            }
        },
        QuadOrdinal => {
            if ((address & 0b1111) == 0) {
                const properView: *[]QuadOrdinal = @ptrFromInt(@intFromPtr(&pool));
                properView.*[address >> 4] = value;
            } else {
                store(LongOrdinal, address, @truncate(value));
                store(LongOrdinal, address +% 8, @truncate(value >> 64));
            }
        },
        *FaultRecord, *const FaultRecord, FaultRecord => value.storeToMemory(address),
        else => @compileError("Requested type not supported!"),
    }
}

pub const CPUClockRateAddress = 0xFE00_0000;
pub const SystemClockRateAddress = 0xFE00_0004;
pub const SerialIOAddress = 0xFE00_0008;
pub const SerialFlushAddress = 0xFE00_000C;
pub const MillisecondsTimestampAddress = 0xFE00_0040;
pub const MicrosecondsTimestampAddress = 0xFE00_0044;
pub const NanosecondsTimestampAddress = 0xFE00_0048;
pub const SystemClockRate: Ordinal = 20 * 1000 * 1000;
pub const CPUClockRate: Ordinal = SystemClockRate / 2;
pub const IOSpaceMemoryUnderlayStart = 0xFE10_0000;
pub const IOSpaceMemoryUnderlayEnd = 0xFEFF_FFFF;
pub const IOSpaceStart = 0xFE00_0000;
pub const IOSpaceEnd = 0xFEFF_FFFF;

fn loadFromIOMemory(
    comptime T: type,
    addr: Address,
) T {
    return switch (addr) {
        IOSpaceMemoryUnderlayStart...IOSpaceMemoryUnderlayEnd => memoryLoad(T, addr),
        CPUClockRateAddress => switch (T) {
            Ordinal, Integer => CPUClockRate,
            FaultTableEntry => FaultTableEntry.None,
            GenericSegmentDescriptor => GenericSegmentDescriptor{},
            else => 0,
        },
        SystemClockRateAddress => switch (T) {
            Ordinal, Integer => SystemClockRate,
            FaultTableEntry => FaultTableEntry.None,
            GenericSegmentDescriptor => GenericSegmentDescriptor{},
            else => 0,
        },
        SerialIOAddress => switch (T) {
            // read from standard input
            ByteOrdinal,
            ShortOrdinal,
            Ordinal,
            LongOrdinal,
            TripleOrdinal,
            QuadOrdinal,
            => stdin.readByte() catch @as(T, math.maxInt(T)),
            ByteInteger,
            ShortInteger,
            Integer,
            LongInteger,
            => stdin.readByteSigned() catch @as(T, math.minInt(T)),
            FaultTableEntry => FaultTableEntry.None,
            GenericSegmentDescriptor => GenericSegmentDescriptor{},
            else => @compileError("Unsupported load types provided"),
        },
        MicrosecondsTimestampAddress, MillisecondsTimestampAddress => scope: {
            // strange packed alignment work happens here so you can get a
            // 64-bit value out based on the base alignment
            switch (T) {
                FaultTableEntry => break :scope FaultTableEntry.None,
                GenericSegmentDescriptor => break :scope GenericSegmentDescriptor{},
                else => switch (@typeInfo(T)) {
                    .Int => |info| {
                        const tval = switch (comptime addr) {
                            MillisecondsTimestampAddress => std.time.milliTimestamp(),
                            MicrosecondsTimestampAddress => std.time.microTimestamp(),
                            else => unreachable,
                        };
                        if (info.signedness == std.builtin.Signedness.signed) {
                            break :scope if (info.bits >= 64) tval else @truncate(tval);
                        } else {
                            const val: u64 = @bitCast(tval);
                            break :scope if (info.bits >= 64) val else @truncate(val);
                        }
                    },
                    else => @compileError("unsupported load type provided"),
                },
            }
        },
        else => switch (T) {
            FaultTableEntry => FaultTableEntry.None,
            GenericSegmentDescriptor => GenericSegmentDescriptor{},
            else => 0,
        },
    };
}
fn storeToIOMemory(
    comptime T: type,
    addr: Address,
    value: T,
) void {
    switch (addr) {
        IOSpaceMemoryUnderlayStart...IOSpaceMemoryUnderlayEnd => memoryStore(T, addr, value),
        SerialIOAddress => {
            // putc
            switch (T) {
                // read from standard input
                ByteOrdinal,
                ByteInteger,
                ShortOrdinal,
                ShortInteger,
                Ordinal,
                Integer,
                LongOrdinal,
                LongInteger,
                TripleOrdinal,
                QuadOrdinal,
                => _ = stdout.writeByte(@truncate(value)) catch {},
                FaultRecord, *FaultRecord, *const FaultRecord => {},
                else => @compileError("Unsupported load types provided"),
            }
        },
        //SerialFlushAddress => stdout.sync() catch {},
        else => {},
    }
}
pub fn load(
    comptime T: type,
    address: Address,
) T {
    return switch (address) {
        IOSpaceStart...IOSpaceEnd => loadFromIOMemory(T, address),
        else => memoryLoad(T, address),
    };
}
pub fn store(
    comptime T: type,
    address: Address,
    value: T,
) void {
    switch (address) {
        // io space detection
        IOSpaceStart...IOSpaceEnd => storeToIOMemory(T, address, value),
        else => memoryStore(T, address, value),
    }
}

test "memory pool load/store test" {
    defer end();
    try begin();
    pool[0] = 0xED;
    pool[1] = 0xFD;
    pool[2] = 0xFF;
    pool[3] = 0xFF;
    pool[4] = 0xab;
    pool[5] = 0xcd;
    pool[6] = 0xef;
    pool[7] = 0x01;
    pool[8] = 0x23;
    pool[9] = 0x45;
    pool[10] = 0x67;
    pool[11] = 0x89;
    store(Ordinal, 12, 0x44332211);
    store(LongOrdinal, 16, 0xccbbaa99_88776655);
    store(ShortOrdinal, 24, 0xeedd);
    store(ByteOrdinal, 26, 0xff);
    try expectEqual(load(ByteOrdinal, 16), 0x55);
    try expectEqual(load(ByteOrdinal, 0), 0xED);
    try expectEqual(load(ByteOrdinal, 1), 0xFD);
    try expectEqual(load(ByteInteger, 2), -1);
    try expectEqual(load(ShortOrdinal, 0), 0xFDED);
    try expectEqual(load(ShortInteger, 2), -1);
    try expectEqual(load(ShortOrdinal, 2), 0xFFFF);
    try expectEqual(load(Ordinal, 0), 0xFFFFFDED);
    try expectEqual(load(TripleOrdinal, 0), 0x89674523_01efcdab_ffffFDED);
    try expectEqual(load(TripleOrdinal, 1), 0x1189674523_01efcdab_ffffFD);
    try expectEqual(load(TripleOrdinal, 2), 0x221189674523_01efcdab_ffff);
    try expectEqual(load(QuadOrdinal, 0), 0x44332211_89674523_01efcdab_ffffFDED);
    try expectEqual(load(QuadOrdinal, 1), 0x55443322_11896745_2301efcd_abffffFD);
    try expectEqual(load(ShortOrdinal, 24), 0xeedd);
}
