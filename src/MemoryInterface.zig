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
const coreTypes = @import("types.zig");
const faults = @import("Faults.zig");
const opcodes = @import("Opcode.zig");
const main = @import("main.zig");
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

// need to access byte by byte so we can reconstruct in a platform independent
// way
pub fn load(
    comptime T: type,
    pool: *MemoryPool,
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
                const a: ShortOrdinal = load(ByteOrdinal, pool, address);
                const b: ShortOrdinal = load(ByteOrdinal, pool, address +% 1);
                return @bitCast(a | (b << 8));
            }
        },
        Ordinal, Integer => {
            if ((address & 0b11) == 0) {
                const properView: *[]Ordinal = @ptrFromInt(@intFromPtr(&pool));
                return @bitCast(properView.*[address >> 2]);
            } else {
                const a: Ordinal = load(ShortOrdinal, pool, address);
                const b: Ordinal = load(ShortOrdinal, pool, address +% 2);
                return @bitCast(a | (b << 16));
            }
        },
        LongOrdinal, LongInteger => {
            if ((address & 0b111) == 0) {
                const properView: *[]LongOrdinal = @ptrFromInt(@intFromPtr(&pool));
                return @bitCast(properView.*[address >> 3]);
            } else {
                const a: LongOrdinal = load(Ordinal, pool, address);
                const b: LongOrdinal = load(Ordinal, pool, address +% 4);
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
                const a: TripleOrdinal = load(Ordinal, pool, address);
                const b: TripleOrdinal = load(Ordinal, pool, address +% 4);
                const c: TripleOrdinal = load(Ordinal, pool, address +% 8);
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
                const a: QuadOrdinal = load(Ordinal, pool, address);
                const b: QuadOrdinal = load(Ordinal, pool, address +% 4);
                const c: QuadOrdinal = load(Ordinal, pool, address +% 8);
                const d: QuadOrdinal = load(Ordinal, pool, address +% 12);
                return a | (b << 32) | (c << 64) | (d << 96);
            }
        },
        FaultTableEntry => FaultTableEntry.make(load(LongOrdinal, pool, address)),
        GenericSegmentDescriptor => {
            var descriptor = GenericSegmentDescriptor{};
            descriptor.setWholeValue(load(QuadOrdinal, pool, address));
            return descriptor;
        },
        else => @compileError("Requested type not allowed!"),
    };
}
pub fn store(
    comptime T: type,
    pool: *MemoryPool,
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
                store(ShortOrdinal, pool, address, @truncate(view));
                store(ShortOrdinal, pool, address +% 2, @truncate(view >> 16));
            }
        },
        LongOrdinal, LongInteger => {
            const view: LongOrdinal = @bitCast(value);
            if ((address & 0b111) == 0) {
                const properView: *[]LongOrdinal = @ptrFromInt(@intFromPtr(&pool));
                properView.*[address >> 3] = view;
            } else {
                store(Ordinal, pool, address, @truncate(view));
                store(Ordinal, pool, address +% 4, @truncate(view >> 32));
            }
        },
        TripleOrdinal => {
            if ((address & 0b1111) == 0) {
                const properView: *[]TripleOrdinal = @ptrFromInt(@intFromPtr(&pool));
                properView.*[address >> 4] = value;
            } else {
                store(Ordinal, pool, address, @truncate(value));
                store(Ordinal, pool, address +% 4, @truncate(value >> 32));
                store(Ordinal, pool, address +% 8, @truncate(value >> 64));
            }
        },
        QuadOrdinal => {
            if ((address & 0b1111) == 0) {
                const properView: *[]QuadOrdinal = @ptrFromInt(@intFromPtr(&pool));
                properView.*[address >> 4] = value;
            } else {
                store(LongOrdinal, pool, address, @truncate(value));
                store(LongOrdinal, pool, address +% 8, @truncate(value >> 64));
            }
        },
        *FaultRecord, *const FaultRecord, FaultRecord => value.storeToMemory(pool, address),
        else => @compileError("Requested type not supported!"),
    }
}
