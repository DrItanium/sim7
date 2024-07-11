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

pub const FaultProcedureEntry = packed struct {
    @"procedure address": u32 = 0,
    @"segment selector": u32 = 0,
    pub fn getProcedureIndex(self: *const FaultProcedureEntry) Address {
        return self.@"procedure address" & 0xFFFF_FFFC;
    }
    pub fn wholeValue(self: *const FaultProcedureEntry) u64 {
        return @as(*u64, @ptrCast(self)).*;
    }
};

pub const FaultTable = packed struct {
    @"override entry": FaultProcedureEntry,
    @"trace fault entry": FaultProcedureEntry,
    @"operation fault entry": FaultProcedureEntry,
    @"arithmetic fault entry": FaultProcedureEntry,
    @"floating-point fault entry": FaultProcedureEntry,
    @"constraint fault entry": FaultProcedureEntry,
    @"virtual-memory fault entry": FaultProcedureEntry,
    @"protection fault entry": FaultProcedureEntry,
    @"machine fault entry": FaultProcedureEntry,
    @"structure fault entry": FaultProcedureEntry,
    @"type fault entry": FaultProcedureEntry,
    reserved0: u64 = 0,
    @"process fault entry": FaultProcedureEntry,
    @"descriptor fault entry": FaultProcedureEntry,
    @"event fault entry": FaultProcedureEntry,
    reserved1: u1088 = 0,
    pub fn wholeValue(self: *const FaultProcedureEntry) u2048 {
        return @as(*u2048, @ptrCast(self)).*;
    }
};
pub const FaultKind = packed struct {
    subtype: u8 = 0,
    unused2: u8 = 0,
    type: u8 = 0,
    flags: u8 = 0,
    const None = FaultKind{};
    const Parallel = FaultKind{};
    const InstructionTrace = FaultKind{ .subtype = 0b10, .type = 0x1 };
    const BranchTrace = FaultKind{ .subtype = 0b100, .type = 0x1 };
    const CallTrace = FaultKind{ .subtype = 0b1000, .type = 0x1 };
    const ReturnTrace = FaultKind{ .subtype = 0b10000, .type = 0x1 };
    const PrereturnTrace = FaultKind{ .subtype = 0b100000, .type = 0x1 };
    const SupervisorTrace = FaultKind{ .subtype = 0b1000000, .type = 0x1 };
    const MarkTrace = FaultKind{ .subtype = 0b10000000, .type = 0x1 };

    const InvalidOpcode = FaultKind{ .type = 2, .subtype = 1 };
    const Unimplemented = FaultKind{ .type = 2, .subtype = 2 };
    const Unaligned = FaultKind{ .type = 2, .subtype = 3 };
    const InvalidOperand = FaultKind{ .type = 2, .subtype = 4 };
    const IntegerOverflow = FaultKind{ .type = 3, .subtype = 1 };
    const ZeroDivide = FaultKind{ .type = 3, .subtype = 2 };

    const FloatingPointOverflow = FaultKind{ .type = 0x4, .subtype = 0b1 };
    const FloatingPointUnderflow = FaultKind{ .type = 0x4, .subtype = 0b10 };
    const FloatingPointInvalidOperation = FaultKind{ .type = 0x4, .subtype = 0b100 };
    const FloatingPointZeroDivideOperation = FaultKind{ .type = 0x4, .subtype = 0b1000 };
    const FloatingPointInexact = FaultKind{ .type = 0x4, .subtype = 0b10000 };
    const FloatingPointReservedEncoding = FaultKind{ .type = 0x4, .subtype = 0b100000 };

    const ConstraintRange = FaultKind{ .type = 5, .subtype = 1 };
    const InvalidSS = FaultKind{ .type = 5, .subtype = 2 };
    const InvalidSegmentTableEntry = FaultKind{ .type = 6, .subtype = 1 };
    const InvalidPageTableDirectoryEntry = FaultKind{ .type = 6, .subtype = 2 };
    const InvalidPageTableEntry = FaultKind{ .type = 6, .subtype = 3 };

    const SegmentLength = FaultKind{ .type = 7, .subtype = 2 };
    const PageRights = FaultKind{ .type = 7, .subtype = 4 };
    const BadAccess = FaultKind{ .type = 7, .subtype = 0x20 };
    const MachineBadAccess = FaultKind{ .type = 8, .subtype = 1 };
    const MachineParityError = FaultKind{ .type = 8, .subtype = 2 };

    const Control = FaultKind{ .type = 9, .subtype = 1 };
    const Dispatch = FaultKind{ .type = 9, .subtype = 2 };
    const IAC = FaultKind{ .type = 9, .subtype = 3 };

    const TypeMismatch = FaultKind{ .type = 0xa, .subtype = 1 };
    const Contents = FaultKind{ .type = 0xa, .subtype = 2 };
    const TypeContents = Contents;
    const TimeSlice = FaultKind{ .type = 0xc, .subtype = 1 };
    const InvalidDescriptor = FaultKind{ .type = 0xd, .subtype = 1 };
    const EventNotice = FaultKind{ .type = 0xe, .subtype = 1 };
    const Override = FaultKind{ .type = 0x10 };

    pub fn wholeValue(self: *const FaultKind) Ordinal {
        return @as(*const Ordinal, @ptrCast(self)).*;
    }

    pub fn translate(kind: Faults) FaultKind {
        return switch (kind) {
            Faults.Parallel => FaultKind.Parallel,
            Faults.InstructionTrace => FaultKind.InstructionTrace,
            Faults.BranchTrace => FaultKind.BranchTrace,
            Faults.CallTrace => FaultKind.CallTrace,
            Faults.ReturnTrace => FaultKind.ReturnTrace,
            Faults.PrereturnTrace => FaultKind.PrereturnTrace,
            Faults.SupervisorTrace => FaultKind.SupervisorTrace,
            Faults.MarkTrace => FaultKind.MarkTrace,

            Faults.InvalidOpcode,
            Faults.IllegalOpcode,
            => FaultKind.InvalidOpcode,
            Faults.Unimplemented,
            => FaultKind.Unimplemented,

            Faults.Unaligned => FaultKind.Unaligned,

            Faults.InvalidOperand,
            Faults.IllegalOperand,
            => FaultKind.InvalidOperand,

            Faults.IntegerOverflow,
            Faults.Overflow,
            => FaultKind.IntegerOverflow,

            Faults.DivisionByZero,
            Faults.ZeroDivide,
            => FaultKind.ZeroDivide,

            Faults.FloatingPointOverflow => FaultKind.FloatingPointOverflow,
            Faults.FloatingPointUnderflow => FaultKind.FloatingPointUnderflow,
            Faults.FloatingPointInvalidOperation => FaultKind.FloatingPointInvalidOperation,
            Faults.FloatingPointZeroDivideOperation => FaultKind.FloatingPointZeroDivideOperation,
            Faults.FloatingPointInexact => FaultKind.FloatingPointInexact,
            Faults.FloatingPointReservedEncoding => FaultKind.FloatingPointReservedEncoding,

            Faults.ConstraintRange => FaultKind.ConstraintRange,
            Faults.InvalidSS => FaultKind.InvalidSS,
            Faults.InvalidSegmentTableEntry => FaultKind.InvalidSegmentTableEntry,
            Faults.InvalidPageTableDirectoryEntry => FaultKind.InvalidPageTableDirectoryEntry,
            Faults.InvalidPageTableEntry => FaultKind.InvalidPageTableEntry,

            Faults.SegmentLength => FaultKind.SegmentLength,
            Faults.PageRights => FaultKind.PageRights,
            Faults.BadAccess => FaultKind.BadAccess,
            Faults.MachineBadAccess => FaultKind.MachineBadAccess,
            Faults.MachineParityError => FaultKind.MachineParityError,

            Faults.Control => FaultKind.Control,
            Faults.Dispatch => FaultKind.Dispatch,
            Faults.IAC => FaultKind.IAC,
            Faults.TypeMismatch => FaultKind.TypeMismatch,
            Faults.TypeContents => FaultKind.TypeContents,
            Faults.TimeSlice => FaultKind.TimeSlice,
            Faults.InvalidDescriptor => FaultKind.InvalidDescriptor,
            Faults.EventNotice => FaultKind.EventNotice,
            Faults.Override => FaultKind.Override,
        };
    }
};

pub const Faults = error{
    Parallel,
    InstructionTrace,
    BranchTrace,
    CallTrace,
    ReturnTrace,
    PrereturnTrace,
    SupervisorTrace,
    MarkTrace,
    InvalidOpcode,
    Unimplemented,
    Unaligned,
    InvalidOperand,
    IntegerOverflow,
    DivisionByZero,
    ZeroDivide,
    FloatingPointOverflow,
    FloatingPointUnderflow,
    FloatingPointInvalidOperation,
    FloatingPointZeroDivideOperation,
    FloatingPointInexact,
    FloatingPointReservedEncoding,

    ConstraintRange,
    InvalidSS,
    InvalidSegmentTableEntry,
    InvalidPageTableDirectoryEntry,
    InvalidPageTableEntry,
    SegmentLength,
    PageRights,
    BadAccess,
    MachineBadAccess,
    MachineParityError,

    Control,
    Dispatch,
    IAC,
    TypeMismatch,
    TypeContents,
    TimeSlice,
    InvalidDescriptor,
    EventNotice,
    Override,
    IllegalOpcode,
    Overflow,
    IllegalOperand,
};
pub fn shouldSaveReturnAddress(self: Faults) bool {
    return switch (self) {
        Faults.EventNotice,
        Faults.IntegerOverflow,
        Faults.Overflow,
        Faults.ZeroDivide,
        Faults.DivisionByZero,
        Faults.FloatingPointInvalidOperation,
        Faults.FloatingPointOverflow,
        Faults.FloatingPointZeroDivideOperation,
        Faults.FloatingPointInexact,
        Faults.FloatingPointReservedEncoding,
        => true,
        else => false,
    };
}
pub const FaultRecord = struct {
    unused: u32 = 0,
    @"override fault data": u96 = 0,
    @"fault data": u96 = 0,
    @"override type": FaultKind = FaultKind.None,
    @"process controls": u32,
    @"arithmetic controls": u32,
    @"fault type": FaultKind,
    @"address of faulting instruction": u32,
    @"save return address": bool,
    pub fn clearTraceEnableBit(self: *const FaultRecord) bool {
        return switch (self.@"fault type".type) {
            0, 1 => true, // override or parallel, trace
            else => false,
        };
    }
    pub fn storeToMemory(self: *const FaultRecord, pool: *MemoryPool, address: Address) void {
        store(Ordinal, pool, address, self.unused);
        store(TripleOrdinal, pool, address + 4, self.@"override fault data");
        store(TripleOrdinal, pool, address + 16, self.@"fault data");
        store(Ordinal, pool, address + 28, self.@"override type".wholeValue());
        store(Ordinal, pool, address + 32, self.@"process controls");
        store(Ordinal, pool, address + 36, self.@"arithmetic controls");
        store(Ordinal, pool, address + 40, self.@"fault type".wholeValue());
        store(Ordinal, pool, address + 44, self.@"address of faulting instruction");
    }
};

pub const FaultTableEntry = packed struct {
    handlerFunctionAddress: Ordinal = 0,
    selector: SegmentSelector = SegmentSelector{},
    pub const None = FaultTableEntry{};
    pub fn isSystemTableEntry(self: *const FaultTableEntry) bool {
        return ((self.handlerFunctionAddress & 0b11) == 0b10);
    }
    pub fn isLocalProcedureEntry(self: *const FaultTableEntry) bool {
        return ((self.handlerFunctionAddress & 0b11) == 0);
    }
    pub fn getFaultHandlerProcedureNumber(self: *const FaultTableEntry) Address {
        return ((self.handlerFunctionAddress & 0xFFFF_FFFC));
    }
    pub fn make(value: LongOrdinal) FaultTableEntry {
        return FaultTableEntry{
            .handlerFunctionAddress = @truncate(value),
            .selector = SegmentSelector.make(@truncate(value >> 32)),
        };
    }
};

test "FaultTableEntry tests" {
    try expectEqual(@sizeOf(LongOrdinal), @sizeOf(FaultTableEntry));
}

test "test faults saving return addresses" {
    try expect(shouldSaveReturnAddress(Faults.EventNotice));
    try expect(shouldSaveReturnAddress(Faults.IntegerOverflow));
    try expect(shouldSaveReturnAddress(Faults.Overflow));
    try expect(shouldSaveReturnAddress(Faults.ZeroDivide));
    try expect(shouldSaveReturnAddress(Faults.DivisionByZero));
    try expect(shouldSaveReturnAddress(Faults.FloatingPointInvalidOperation));
    try expect(shouldSaveReturnAddress(Faults.FloatingPointOverflow));
    try expect(shouldSaveReturnAddress(Faults.FloatingPointZeroDivideOperation));
    try expect(shouldSaveReturnAddress(Faults.FloatingPointInexact));
    try expect(shouldSaveReturnAddress(Faults.FloatingPointReservedEncoding));
}

// you can include stuff after use!

const std = @import("std");
const coreTypes = @import("types.zig");
const nativeInterface = @import("MemoryInterface.zig");
const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;
const Ordinal = coreTypes.Ordinal;
const TripleOrdinal = coreTypes.TripleOrdinal;
const Address = coreTypes.Address;
const MemoryPool = coreTypes.MemoryPool;
const store = nativeInterface.store;
const SegmentSelector = coreTypes.SegmentSelector;
const LongOrdinal = coreTypes.LongOrdinal;
