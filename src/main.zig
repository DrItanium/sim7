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
const meta = @import("std").meta;
const math = @import("std").math;
const io = std.io;
const stdin = io.getStdIn().reader();
const stdout = io.getStdOut().writer();

const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;

const coreTypes = @import("types.zig");
const faults = @import("Faults.zig");
const opcodes = @import("Opcode.zig");
const NativeInterface = @import("MemoryInterface.zig");
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

const InstructionClass = coreTypes.InstructionClass;
const ArchitectureLevel = coreTypes.ArchitectureLevel;
const DecodedOpcode = opcodes.DecodedOpcode;

const LocalRegisterFrame = struct {
    contents: RegisterFrame = RegisterFrame{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    targetFramePointer: Address = 0,
    valid: bool = false,
    fn trySaveRegisters(self: *LocalRegisterFrame, core: *Core) void {
        if (self.valid) {
            var index: Address = self.targetFramePointer;
            for (self.contents) |value| {
                core.storeToMemory(Ordinal, index, value);
                index += 4;
            }
        }
    }
    fn relinquishOwnership(self: *LocalRegisterFrame, core: *Core) void {
        self.trySaveRegisters(core);
        self.valid = false;
        self.synchronizeOwnership(0);
        // should we clear registers? it would be a state leak...
        // I don't think i960 does that
    }
    fn synchronizeOwnership(self: *LocalRegisterFrame, val: Address) void {
        self.targetFramePointer = val;
    }
    fn clear(self: *LocalRegisterFrame) void {
        for (&self.contents) |*cell| {
            cell.* = 0;
        }
    }
    fn isValid(self: *LocalRegisterFrame) bool {
        return self.valid;
    }
    pub fn takeOwnership(self: *LocalRegisterFrame, newFP: Address, core: *Core) void {
        self.trySaveRegisters(core);
        self.valid = true;
        self.synchronizeOwnership(newFP);
        // don't clear out the registers
    }
    pub fn restoreOwnership(self: *LocalRegisterFrame, newFP: Address, core: *Core) void {
        // we already have a match so just return!
        if (self.valid) {
            if (self.targetFramePointer == newFP) {
                return;
            } else {
                // this is technically expensive because the valid bit is being
                // checked twice, the upside is that I eliminate code
                // duplication
                self.trySaveRegisters(core);
            }
        }
        // okay, so no match or is invalid
        self.valid = true;
        self.synchronizeOwnership(newFP);
        var index: Address = self.targetFramePointer;
        for (&self.contents) |*value| {
            value.* = core.loadFromMemory(Ordinal, index);
            index += 4;
        }
    }
};

const CTRLInstruction = packed struct {
    displacement: i24,
    opcode: u8,

    pub fn getDisplacement(self: *const CTRLInstruction) i24 {
        const bits = @as(u24, @bitCast(self.displacement)) & 0xFFFFFC;
        return @as(i24, @bitCast(bits));
    }
    pub fn getOpcode(self: *const CTRLInstruction) Faults!DecodedOpcode {
        return meta.intToEnum(DecodedOpcode, self.opcode) catch Faults.InvalidOpcode;
    }
};

const COBRInstruction = packed struct {
    displacement: i13,
    m1: bool,
    src2: Operand,
    src1: Operand,
    opcode: u8,

    pub fn getDisplacement(self: *const COBRInstruction) i13 {
        const bits = @as(u13, @bitCast(self.displacement)) & 0b1_1111_1111_1100;
        return @as(i13, @bitCast(bits));
    }
    pub fn treatSrc1AsLiteral(self: *const COBRInstruction) bool {
        return self.m1;
    }
    pub fn getOpcode(self: *const COBRInstruction) Faults!DecodedOpcode {
        return meta.intToEnum(DecodedOpcode, self.opcode) catch Faults.InvalidOpcode;
    }
};

const REGInstruction = packed struct {
    src1: Operand,
    s1: bool = false,
    s2: bool = false,
    opcodeExt: u4,
    m1: bool,
    m2: bool,
    m3: bool,
    src2: Operand,
    srcDest: Operand,
    opcode: u8,
    pub fn treatSrc1AsLiteral(self: *const REGInstruction) bool {
        return self.m1;
    }
    pub fn treatSrc2AsLiteral(self: *const REGInstruction) bool {
        return self.m2;
    }
    pub fn treatSrcDestAsLiteral(self: *const REGInstruction) bool {
        return self.m3;
    }
    pub fn getOpcode(self: *const REGInstruction) !DecodedOpcode {
        var major: u12 = @as(u12, self.opcode);
        major <<= 4;
        const minor: u12 = self.opcodeExt;
        //return @enumFromInt(major | minor);
        return meta.intToEnum(DecodedOpcode, major | minor) catch Faults.InvalidOpcode;
    }
};
const MEMAAddressComputationKind = enum(u1) {
    offset = 0,
    @"(abase)+offset" = 1,
};
const MEMBAddressComputationKind = enum(u4) {
    @"(abase)" = 0b0100,
    @"(ip)+displacement+8" = 0b0101,
    @"(abase)+(index)*2^scale" = 0b0111,
    displacement = 0b1100,
    @"(abase)+displacement" = 0b1101,
    @"(index)*2^scale+displacement" = 0b1110,
    @"(abase)+(index)*2^scale+displacement" = 0b1111,
    pub fn usesOptionalDisplacement(self: MEMBAddressComputationKind) bool {
        return switch (self) {
            MEMBAddressComputationKind.@"(abase)+displacement",
            MEMBAddressComputationKind.@"(index)*2^scale+displacement",
            MEMBAddressComputationKind.@"(abase)+(index)*2^scale+displacement",
            MEMBAddressComputationKind.displacement,
            MEMBAddressComputationKind.@"(ip)+displacement+8",
            => true,
            else => false,
        };
    }
    pub fn usesScaleField(self: MEMBAddressComputationKind) bool {
        return switch (self) {
            MEMBAddressComputationKind.@"(abase)+(index)*2^scale",
            MEMBAddressComputationKind.@"(index)*2^scale+displacement",
            MEMBAddressComputationKind.@"(abase)+(index)*2^scale+displacement",
            => true,
            else => false,
        };
    }
};
const MEMAInstruction = packed struct {
    offset: u12,
    determinant: u1,
    mode: MEMAAddressComputationKind,
    abase: Operand,
    srcDest: Operand,
    opcode: u8,

    pub fn getOpcode(self: *const MEMAInstruction) !DecodedOpcode {
        return meta.intToEnum(DecodedOpcode, self.opcode) catch error.IllegalOpcode;
    }

    pub fn getOffset(self: *const MEMAInstruction) u12 {
        return self.offset;
    }

    pub fn getComputationMode(self: *const MEMAInstruction) MEMAAddressComputationKind {
        return self.mode;
    }
};

const MEMBInstruction = packed struct {
    index: Operand = 0,
    unused: u2 = 0,
    scale: u3 = 0,
    mode: u4 = 0,
    abase: Operand = 0,
    srcDest: Operand = 0,
    opcode: u8 = 0,

    pub fn getOpcode(self: *const MEMBInstruction) !DecodedOpcode {
        return meta.intToEnum(DecodedOpcode, self.opcode) catch error.IllegalOpcode;
    }
    pub fn usesOptionalDisplacement(self: *const MEMBInstruction) bool {
        return self.mode.usesOptionalDisplacement();
    }
    pub fn getComputationMode(self: *const MEMBInstruction) Faults!MEMBAddressComputationKind {
        return meta.intToEnum(MEMBAddressComputationKind, self.mode) catch Faults.IllegalOperand;
    }
};

test "Bad MEMB Instruction" {
    const tmp = MEMBInstruction{
        .mode = 0b0110,
    };
    try expect(tmp.getComputationMode() == Faults.IllegalOperand);
}

const InstructionDeterminant = packed struct {
    unused0: u12,
    memDeterminant: u1,
    unused1: u11,
    opcode: u8,

    pub fn isMEMInstruction(self: *const InstructionDeterminant) bool {
        return self.determineClass() == InstructionClass.MEM;
    }
    pub fn isMEMAInstruction(self: *const InstructionDeterminant) bool {
        return self.isMEMInstruction() and (self.memDeterminant == 0);
    }
    pub fn isMEMBInstruction(self: *const InstructionDeterminant) bool {
        return self.isMEMInstruction() and (self.memDeterminant == 1);
    }
    fn determineClass(self: *const InstructionDeterminant) InstructionClass {
        return InstructionClass.determine(self.opcode);
    }
};

const Instruction = union(enum) {
    ctrl: CTRLInstruction,
    cobr: COBRInstruction,
    reg: REGInstruction,
    mema: MEMAInstruction,
    memb: MEMBInstruction,
    pub fn getOpcode(self: *const Instruction) !DecodedOpcode {
        return switch (self.*) {
            .ctrl => |k| k.getOpcode(),
            .cobr => |k| k.getOpcode(),
            .reg => |k| k.getOpcode(),
            .mema => |k| k.getOpcode(),
            .memb => |k| k.getOpcode(),
        };
    }
    pub fn getSrcDest(self: *const Instruction) !Operand {
        return switch (self.*) {
            .reg => |k| k.srcDest,
            .mema => |k| k.srcDest,
            .memb => |k| k.srcDest,
            else => error.NoArgument,
        };
    }
    pub fn getSrc1(self: *const Instruction) !Operand {
        return switch (self.*) {
            .reg => |k| k.src1,
            .cobr => |k| k.src1,
            else => error.NoArgument,
        };
    }
    pub fn getSrc2(self: *const Instruction) !Operand {
        return switch (self.*) {
            .reg => |k| k.src2,
            .cobr => |k| k.src2,
            else => error.NoArgument,
        };
    }
};

fn decode(opcode: Ordinal) Instruction {
    const determinant: *const InstructionDeterminant = @ptrCast(&opcode);
    return switch (determinant.determineClass()) {
        InstructionClass.CTRL => Instruction{
            .ctrl = @as(*const CTRLInstruction, @ptrCast(&opcode)).*,
        },
        InstructionClass.COBR => Instruction{
            .cobr = @as(*const COBRInstruction, @ptrCast(&opcode)).*,
        },
        InstructionClass.REG => Instruction{
            .reg = @as(*const REGInstruction, @ptrCast(&opcode)).*,
        },
        InstructionClass.MEM => if (determinant.isMEMBInstruction()) Instruction{
            .memb = @as(*const MEMBInstruction, @ptrCast(&opcode)).*,
        } else Instruction{
            .mema = @as(*const MEMAInstruction, @ptrCast(&opcode)).*,
        },
    };
}

const PFP: Operand = 0;
const SP: Operand = 1;
const RIP: Operand = 2;
const LinkRegister: Operand = 30; // g14
const FramePointer: Operand = 31;
const FP = FramePointer;
pub const GenericSegmentDescriptor = packed struct {
    reserved: u64 = 0,
    @"base address": Ordinal = 0,
    valid: u1 = 0,
    @"paging method": u2 = 0,
    @"access status": u5 = 0,
    unused0: u10 = 0,
    size: u6 = 0,
    unused1: u4 = 0,
    @"segment type": u4 = 0,

    pub fn isValid(self: *const GenericSegmentDescriptor) bool {
        return self.valid;
    }
    pub fn wholeValue(self: *const GenericSegmentDescriptor) QuadOrdinal {
        return @as(*const QuadOrdinal, @ptrCast(self)).*;
    }

    pub fn setWholeValue(self: *GenericSegmentDescriptor, value: QuadOrdinal) void {
        @as(*QuadOrdinal, @ptrCast(self)).* = value;
    }
};
test "segment descriptor" {
    try expectEqual(@sizeOf(GenericSegmentDescriptor), @sizeOf(QuadOrdinal));
    const v0 = GenericSegmentDescriptor{};
    try expectEqual(v0.wholeValue(), 0);
    var v1 = GenericSegmentDescriptor{};
    try expectEqual(v1.wholeValue(), 0);
    v1.setWholeValue(0xFDED);
    try expectEqual(v1.wholeValue(), 0xFDED);
}
const ProcessorControls = packed struct {
    unused0: u1 = 0,
    @"multiprocessor preempt": u1,
    state: u2,
    unused1: u1 = 0,
    @"nonpreempt limit": u5,
    @"addressing mode": u1,
    @"check dispatch port": u1,
    unused2: u5 = 0,
    @"interim priority": u5,
    unused3: u10 = 0,
    @"write external priority": u1,

    pub fn toWholeValue(self: *const ProcessorControls) Ordinal {
        return @as(*Ordinal, @ptrCast(self)).*;
    }
};
const IACMessage = packed struct {
    field2: u16,
    field1: u8,
    messageType: u8,
    field3: u32,
    field4: u32,
    field5: u32,
};
const ArithmeticControls = packed struct {
    @"condition code": u3 = 0,
    @"arithmetic status": u4 = 0,
    unused0: u1 = 0,
    @"integer overflow flag": u1 = 0,
    unused1: u3 = 0,
    @"integer overflow mask": u1 = 0,
    unused2: u2 = 0,
    @"no imprecise faults": u1 = 0,
    @"floating overflow flag": u1 = 0,
    @"floating underflow flag": u1 = 0,
    @"floating invalid-op flag": u1 = 0,
    @"floating zero-divide flag": u1 = 0,
    @"floating inexact flag": u1 = 0,
    unused3: u3 = 0,
    @"floating overflow mask": u1 = 0,
    @"floating underflow mask": u1 = 0,
    @"floating invalid-op mask": u1 = 0,
    @"floating zero-divide mask": u1 = 0,
    @"floating inexact mask": u1 = 0,
    @"floating-point normalizing mode": u1 = 0,
    @"floating-point rounding control": u2 = 0,

    pub fn toWholeValue(self: *const ArithmeticControls) Ordinal {
        return @as(*const Ordinal, @ptrCast(self)).*;
    }
    pub fn setWholeValue(self: *ArithmeticControls, value: Ordinal) void {
        @as(*Ordinal, @ptrCast(self)).* = value;
    }
};
const ProcessControls = packed struct {
    @"trace enable": u1 = 0,
    @"execution mode": u1 = 0,
    unused0: u4 = 0,
    @"time-slice reschedule": u1 = 0,
    @"time-slice": u1 = 0,
    timing: u1 = 0,
    @"resume": u1 = 0,
    @"trace-fault pending": u1 = 0,
    preempt: u1 = 0,
    refault: u1 = 0,
    state: u2 = 0,
    unused1: u1 = 0,
    priority: u5 = 0,
    @"internal state": u11 = 0,
    pub fn toWholeValue(self: *const ProcessControls) Ordinal {
        return @as(*const Ordinal, @ptrCast(self)).*;
    }
    pub fn setWholeValue(self: *ProcessControls, value: Ordinal) void {
        @as(*Ordinal, @ptrCast(self)).* = value;
    }
    pub fn inSupervisorMode(self: *const ProcessControls) bool {
        return self.@"execution mode" != 0;
    }
};

const TraceControls = packed struct {
    unused0: u1 = 0,
    @"instruction trace mode": u1 = 0,
    @"branch trace mode": u1 = 0,
    @"call trace mode": u1 = 0,
    @"return trace mode": u1 = 0,
    @"prereturn trace mode": u1 = 0,
    @"supervisor trace mode": u1 = 0,
    @"breakpoint trace mode": u1 = 0,
    unused1: u9 = 0,
    @"instruction trace event": u1 = 0,
    @"branch trace event": u1 = 0,
    @"call trace event": u1 = 0,
    @"return trace event": u1 = 0,
    @"prereturn trace event": u1 = 0,
    @"supervisor trace event": u1 = 0,
    @"breakpoint trace event": u1 = 0,
    unused2: u8 = 0,

    pub fn toWholeValue(self: *const TraceControls) Ordinal {
        return @as(*Ordinal, @ptrCast(self)).*;
    }
    pub fn setWholeValue(self: *TraceControls, value: Ordinal) void {
        @as(*Ordinal, @ptrCast(self)).* = value;
    }
};

const PreviousFramePointer = packed struct {
    rt: u3 = 0,
    p: u1 = 0,
    unused: u2 = 0, // used by the Hx processors but not Kx/Sx/Mx processors
    address: u26 = 0,
    pub fn toWholeValue(self: *const PreviousFramePointer) Ordinal {
        return @as(*const Ordinal, @ptrCast(self)).*;
    }
    pub fn setWholeValue(self: *PreviousFramePointer, value: Ordinal) void {
        @as(*Ordinal, @ptrCast(self)).* = value;
    }
    pub fn make(value: u32) PreviousFramePointer {
        var result: PreviousFramePointer = PreviousFramePointer{};
        result.setWholeValue(value);
        return result;
    }
};
test "check PreviousFramePointer" {
    try expectEqual(@sizeOf(PreviousFramePointer), @sizeOf(Ordinal));
}

// there really isn't a point in keeping the instruction processing within the
// Core structure
const BootResult = error{
    ChecksumFail,
    SelfTestFailure,
};

const Core = struct {
    globals: RegisterFrame = RegisterFrame{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    locals: [4]LocalRegisterFrame = [_]LocalRegisterFrame{
        LocalRegisterFrame{},
        LocalRegisterFrame{},
        LocalRegisterFrame{},
        LocalRegisterFrame{},
    },
    // @todo link the size of this field to the number of local register frames
    currentLocalFrame: u2 = 0,
    fpr: [4]ExtendedReal = [_]ExtendedReal{ 0.0, 0.0, 0.0, 0.0 },
    ip: Ordinal = 0,
    advanceBy: u4 = 4,
    pc: ProcessControls = @bitCast(@as(u32, 0)),
    ac: ArithmeticControls = @bitCast(@as(u32, 0)),
    tc: TraceControls = @bitCast(@as(u32, 0)),
    continueExecuting: bool = true,
    systemAddressTableBase: Ordinal = 0,
    prcbAddress: Ordinal = 0,
    fn getFaultEntry(self: *Core, index: u8) FaultTableEntry {
        const faultTableBaseAddress = self.getFaultTableBaseAddress();
        const maskedIndex: Address = index & 0b0001_1111;
        const realOffset = maskedIndex * (@sizeOf(FaultTableEntry));
        return self.loadFromMemory(FaultTableEntry, faultTableBaseAddress + realOffset);
    }
    fn generateFault(self: *Core, record: *const FaultRecord) !void {
        if (record.@"save return address") {
            self.saveReturnAddress(RIP);
        }
        const faultType = record.@"fault type".type;
        const entry = self.getFaultEntry(faultType);
        if (entry.isLocalProcedureEntry()) {
            self.localProcedureEntry_FaultCall(record, entry.handlerFunctionAddress);
        } else if (entry.isSystemTableEntry()) {
            try self.procedureTableEntry_FaultCall(record, &entry);
        } else {
            return error.BadFault;
        }
    }
    fn getStackPointer(self: *Core) Ordinal {
        return self.getRegisterValue(SP);
    }
    fn faultCallGeneric(self: *Core, record: *const FaultRecord, address: Address, stackPointer: Address) void {
        // first allocate a new frame on the stack that the processor is
        // currently using.
        //
        // Set the frame-return status field to 0b001
        //
        // allocate enough space before the start of the frame for the fault
        // record (and optionally a resumption record if necessary). Be lazy
        // and just allocate two frames worth of information be safe! Three
        // frames worth are necessary to make sure we have enough padding.

        const nextFrame = Core.computeNextFrameFaultBase(stackPointer);
        const fp = self.getRegisterValue(FP);
        // save the current registers to the stack
        self.enterCall(fp);
        // manually setup the stack frame as needed
        var pfp = PreviousFramePointer.make(self.getRegisterValue(FP) & 0xFFFFFFF0);
        pfp.rt = 0b001;
        self.setRegisterValue(PFP, pfp.toWholeValue());
        self.setRegisterValue(FP, nextFrame);
        self.setRegisterValue(SP, nextFrame + 64);
        self.storeToMemory(@TypeOf(record), nextFrame - 48, record);
        // no need to push a resumption record right now and set the resume
        // flag in the saved process controls
        self.ip = address;
        self.advanceBy = 0;
    }
    fn localProcedureEntry_FaultCall(self: *Core, record: *const FaultRecord, address: Address) void {
        self.faultCallGeneric(record, address, self.getStackPointer());
    }
    fn supervisorProcedureTableEntry_FaultCall(
        self: *Core,
        record: *const FaultRecord,
        procedureAddress: Address,
        baseTableAddress: Address,
    ) void {
        var baseStackAddress: Ordinal = undefined;
        if (self.pc.inSupervisorMode()) {
            baseStackAddress = self.getStackPointer();
        } else {
            baseStackAddress = self.loadFromMemory(Ordinal, baseTableAddress + 12);
            self.pc.@"execution mode" = 1;
        }
        if (record.clearTraceEnableBit()) {
            self.pc.@"trace enable" = 0;
        } else {
            self.pc.@"trace enable" = @truncate(baseStackAddress & 0b1);
        }
        // now that we have the right area to be in, lets do teh FaultCall as
        // though it is a local one!
        const temp = baseStackAddress & 0xFFFF_FFFC;
        self.faultCallGeneric(record, procedureAddress, temp);
    }
    fn procedureTableEntry_FaultCall(self: *Core, record: *const FaultRecord, entry: *const FaultTableEntry) !void {

        // first get the segment descriptor
        const idx: Ordinal = entry.selector.index;
        var baseIndex = self.systemAddressTableBase;
        baseIndex += (idx * @sizeOf(GenericSegmentDescriptor));
        const descriptor = self.loadFromMemory(GenericSegmentDescriptor, baseIndex);
        // next find the procedure number
        const index = entry.getFaultHandlerProcedureNumber();
        // @todo implement override fault if the segment descriptor is invalid
        // or wrong for this target

        // now we can get the base offset table
        const tableAddress = descriptor.@"base address";

        // get the starting offset to the procedure-table structure entries
        const procedureEntry = self.loadFromMemory(Ordinal, tableAddress + 48 + index);
        const procedureAddress = procedureEntry & 0xFFFF_FFFC;
        switch (procedureEntry & 0b11) {
            0b00 => self.localProcedureEntry_FaultCall(record, procedureAddress),
            0b10 => self.supervisorProcedureTableEntry_FaultCall(record, procedureAddress, tableAddress),
            else => return error.IllegalProcedureEntry,
        }
    }
    fn constructFaultRecord(self: *Core, err: Faults) FaultRecord {
        return FaultRecord{
            .@"process controls" = self.pc.toWholeValue(),
            .@"arithmetic controls" = self.ac.toWholeValue(),
            .@"fault type" = FaultKind.translate(err),
            .@"address of faulting instruction" = self.ip,
            .@"save return address" = faults.shouldSaveReturnAddress(err),
        };
    }
    fn getSystemProcedureTableBase(self: *Core) Ordinal {
        return self.loadFromMemory(Ordinal, self.systemAddressTableBase + 120);
    }
    fn getSupervisorStackPointer(self: *Core) Ordinal {
        return self.loadFromMemory(Ordinal, self.systemAddressTableBase + 12);
    }
    fn loadFromPRCB(self: *Core, offset: u8) Ordinal {
        return self.loadFromMemory(Ordinal, self.prcbAddress + offset);
    }
    fn getProcessorControls(self: *Core) Ordinal {
        return self.loadFromPRCB(4);
    }
    fn getCurrentProcessSegmentSelector(self: *Core) Ordinal {
        return self.loadFromPRCB(12);
    }
    fn getDispatchPortSegmentSelector(self: *Core) Ordinal {
        return self.loadFromPRCB(16);
    }
    fn getInterruptTablePointer(self: *Core) Address {
        return self.loadFromPRCB(20);
    }
    fn getInterruptStackAddress(self: *Core) Address {
        return self.loadFromPRCB(24);
    }
    fn getSystemProcedureTableSegmentSelector(self: *Core) Address {
        return self.loadFromPRCB(36);
    }
    fn getFaultTableBaseAddress(self: *Core) Address {
        return self.loadFromPRCB(40);
    }
    fn getCurrentLocalFrame(self: *Core) *LocalRegisterFrame {
        return &self.locals[self.currentLocalFrame];
    }
    fn enterCall(self: *Core, fp: Ordinal) void {
        // first step, we want to make sure that the current frame is owned by
        // the current frame pointer, it will be holding onto the frame pointer
        // to compare to. So we need to do an update
        self.getCurrentLocalFrame().synchronizeOwnership(fp);
        // get the next frame
        const nextIdx = self.currentLocalFrame +% 1;
        self.locals[nextIdx].takeOwnership(fp, self);
        self.currentLocalFrame = nextIdx;
    }
    fn performSelfTest(self: *Core) !void {
        _ = self;
    }
    fn deassertFailureState(self: *Core) void {
        _ = self;
        // do nothing
    }
    fn assertFailureState(self: *Core) void {
        _ = self;
        // do nothing
    }
    pub fn start(self: *Core) BootResult!void {
        try self.performSelfTest();
        self.deassertFailureState();
        var storage: [8]Ordinal = undefined;
        var i: Address = 0;
        var j: Address = 0;
        while (i < 8) : ({
            i += 1;
            j += 4;
        }) {
            storage[i] = self.loadFromMemory(Ordinal, j);
        }
        var temp: Ordinal = 0xFFFF_FFFF;
        var carry: u1 = 0;
        for (storage) |value| {
            const intermediate = @addWithOverflow(temp, value);
            const finalOutput = @addWithOverflow(intermediate[0], carry);
            temp = finalOutput[0];
            carry = finalOutput[1] | intermediate[1];
        }
        if (temp != 0) {
            self.assertFailureState();
            //std.debug.print("Got Checksum: 0x{x}\n", .{temp});
            return BootResult.ChecksumFail;
        }
        self.boot0(storage[0], storage[1], storage[3]);
    }
    fn boot0(self: *Core, sat: Address, pcb: Address, startIP: Address) void {
        self.systemAddressTableBase = sat;
        self.prcbAddress = pcb;
        self.ip = startIP;

        // fetch IMI
        const theStackPointer = self.getInterruptStackAddress();
        // get the interrupt stack pointer base since we are starting in an
        // interrupt context
        //
        // set the frame pointer to the start of the interrupt stack
        self.setRegisterValue(FramePointer, theStackPointer);
        self.pc.priority = 31;
        // The next value is considered to be a reserved state in the i960 MC
        // manual but in the Sx/Kx manuals 0b01/0b1 is considered to be
        // interrupted...good job intel... high quality documentation
        self.pc.state = 1; // interrupted
        self.currentLocalFrame = 0;
        for (&self.locals) |*frame| {
            frame.relinquishOwnership(self);
            frame.clear();
        }
        // the current local register window needs to be owned on startup
        self.getCurrentLocalFrame().takeOwnership(theStackPointer, self);
        self.setRegisterValue(SP, theStackPointer + 64);
        self.setRegisterValue(PFP, theStackPointer);
        // todo: clear pending interrupts/
        // clear any latched external interrupt/IAC signals
        // begin execution
    }
    fn newCycle(self: *Core) void {
        self.advanceBy = 4;
    }
    fn nextInstruction(self: *Core) void {
        if (self.advanceBy > 0) {
            self.ip +%= self.advanceBy;
            self.advanceBy = 0;
        }
    }
    fn getRegister(self: *Core, index: Operand) *Ordinal {
        if (index > 16) {
            return &self.globals[index & 0b1111];
        } else {
            return &self.locals[self.currentLocalFrame].contents[index & 0b1111];
        }
    }

    fn getFloatingPointRegister(self: *Core, index: Operand) !*ExtendedReal {
        return switch (index) {
            0b00000...0b00011 => |val| &self.fpr[val],
            else => Faults.IllegalOperand,
        };
    }
    fn setRegisterValue(self: *Core, index: Operand, value: Ordinal) void {
        self.getRegister(index).* = value;
    }
    fn getRegisterValue(self: *Core, index: Operand) Ordinal {
        return self.getRegister(index).*;
    }
    fn getTripleRegisterValue(self: *Core, index: Operand) Faults!TripleOrdinal {
        if ((index & 0b11) != 0) {
            return error.InvalidOpcode;
        }
        const a: TripleOrdinal = self.getRegisterValue(index);
        const b: TripleOrdinal = self.getRegisterValue(index + 1);
        const c: TripleOrdinal = self.getRegisterValue(index + 2);
        return a | math.shl(TripleOrdinal, b, 32) | math.shl(TripleOrdinal, c, 64);
    }
    fn getQuadRegisterValue(self: *Core, index: Operand) Faults!QuadOrdinal {
        if ((index & 0b11) != 0) {
            return error.InvalidOpcode;
        }
        const a: QuadOrdinal = self.getRegisterValue(index);
        const b: QuadOrdinal = self.getRegisterValue(index + 1);
        const c: QuadOrdinal = self.getRegisterValue(index + 2);
        const d: QuadOrdinal = self.getRegisterValue(index + 3);
        return a | math.shl(QuadOrdinal, b, 32) | math.shl(QuadOrdinal, c, 64) | math.shl(QuadOrdinal, d, 96);
    }
    fn getLongRegisterValue(self: *Core, index: Operand) Faults!LongOrdinal {
        if ((index & 0b1) != 0) {
            return error.InvalidOpcode;
        }
        const lower: LongOrdinal = self.getRegisterValue(index);
        const upper: LongOrdinal = self.getRegisterValue(index + 1);
        return lower | math.shl(LongOrdinal, upper, 32);
    }
    fn setLongRegisterValue(self: *Core, index: Operand, value: LongOrdinal) Faults!void {
        if ((index & 0b1) != 0) {
            return Faults.InvalidOpcode;
        }
        self.setRegisterValue(index, @truncate(value));
        self.setRegisterValue(index + 1, @truncate(math.shr(LongOrdinal, value, 32)));
    }
    fn setTripleRegisterValue(self: *Core, index: Operand, value: TripleOrdinal) Faults!void {
        if ((index & 0b11) != 0) {
            return Faults.InvalidOpcode;
        }
        self.setRegisterValue(index, @truncate(value));
        self.setRegisterValue(index + 1, @truncate(math.shr(TripleOrdinal, value, 32)));
        self.setRegisterValue(index + 2, @truncate(math.shr(TripleOrdinal, value, 64)));
    }
    fn setQuadRegisterValue(self: *Core, index: Operand, value: QuadOrdinal) Faults!void {
        if ((index & 0b11) != 0) {
            return Faults.InvalidOpcode;
        }
        self.setRegisterValue(index, @truncate(value));
        self.setRegisterValue(index + 1, @truncate(math.shr(QuadOrdinal, value, 32)));
        self.setRegisterValue(index + 2, @truncate(math.shr(QuadOrdinal, value, 64)));
        self.setRegisterValue(index + 3, @truncate(math.shr(QuadOrdinal, value, 96)));
    }
    fn relativeBranch(
        self: *Core,
        comptime T: type,
        displacement: T,
    ) void {
        self.advanceBy = 0;
        var theip: Integer = @bitCast(self.ip);
        theip += displacement;
        self.ip = @bitCast(theip);
    }
    fn saveReturnAddress(self: *Core, dest: Operand) void {
        self.setRegisterValue(dest, self.ip + self.advanceBy);
    }
    fn moveRegisterValue(self: *Core, dest: Operand, src: Operand) void {
        self.setRegisterValue(dest, self.getRegisterValue(src));
    }
    fn loadFromMemory(
        self: *Core,
        comptime T: type,
        addr: Address,
    ) T {
        _ = self;
        return NativeInterface.load(T, addr);
    }
    fn storeToMemory(
        self: *Core,
        comptime T: type,
        addr: Address,
        value: T,
    ) void {
        _ = self;
        NativeInterface.store(T, addr, value);
    }
    fn atomicLoad(
        self: *Core,
        addr: Address,
    ) Ordinal {
        // @todo implement atomic locking support
        return self.loadFromMemory(Ordinal, addr);
    }
    fn atomicStore(
        self: *Core,
        addr: Address,
        value: Ordinal,
    ) void {
        // @todo implement atomic locking support
        self.storeToMemory(Ordinal, addr, value);
    }
    fn computeEffectiveAddress(
        self: *Core,
        instruction: Instruction,
    ) Faults!Address {
        return switch (instruction) {
            .mema => |inst| {
                return switch (inst.getComputationMode()) {
                    MEMAAddressComputationKind.offset => inst.getOffset(),
                    MEMAAddressComputationKind.@"(abase)+offset" => @bitCast(self.getRegisterValue(inst.abase) + inst.getOffset()),
                };
            },
            .memb => |inst| {
                return switch (try inst.getComputationMode()) {
                    MEMBAddressComputationKind.@"(abase)" => @bitCast(self.getRegisterValue(inst.abase)),
                    MEMBAddressComputationKind.@"(ip)+displacement+8" => {
                        // okay so this is the first instruction that needs to
                        // read the next word
                        const nextWord: Integer = self.loadFromMemory(Integer, self.ip + 4);
                        const theIP: Integer = @bitCast(self.ip);
                        const outcome: Integer = theIP + nextWord + 8;
                        self.advanceBy = 8;
                        return @bitCast(outcome);
                    },
                    MEMBAddressComputationKind.@"(abase)+(index)*2^scale" => @bitCast(self.getRegisterValue(inst.abase) +% (self.getRegisterValue(inst.index) << inst.scale)),
                    MEMBAddressComputationKind.displacement => {
                        self.advanceBy = 8;
                        const outcome: Integer = self.loadFromMemory(Integer, self.ip + 4);
                        return @bitCast(outcome);
                    },
                    MEMBAddressComputationKind.@"(abase)+displacement" => {
                        self.advanceBy = 8;
                        const outcome: Integer = self.loadFromMemory(Integer, self.ip + 4);
                        const abase: Integer = @bitCast(self.getRegisterValue(inst.abase));
                        return @bitCast(outcome +% abase);
                    },
                    MEMBAddressComputationKind.@"(index)*2^scale+displacement" => {
                        self.advanceBy = 8;
                        const displacement: Integer = self.loadFromMemory(Integer, self.ip + 4);
                        const idx: Integer = @bitCast(self.getRegisterValue(inst.index));
                        const scale = inst.scale;
                        return @bitCast((idx << scale) + displacement);
                    },
                    MEMBAddressComputationKind.@"(abase)+(index)*2^scale+displacement" => {
                        self.advanceBy = 8;
                        const displacement: Integer = self.loadFromMemory(Integer, self.ip + 4);
                        const idx: Integer = @bitCast(self.getRegisterValue(inst.index));
                        const scale = inst.scale;
                        const abase: Integer = @bitCast(self.getRegisterValue(inst.abase));
                        return @bitCast(abase + (idx << scale) + displacement);
                    },
                };
            },
            else => Faults.InvalidOperand,
        };
    }
    fn syncf(self: *Core) !void {
        // right now, there is no need for synchronizing faults but if I ever
        // implement more advanced actions then a synchronization may be
        // necessary
        _ = self;
    }
    fn leaveCall(self: *Core) void {
        var pfpAddr = self.getRegisterValue(PFP);
        pfpAddr &= 0xFFFFFF_C0; // mask the value to yield the proper return value
        self.setRegisterValue(FP, pfpAddr);
        self.getCurrentLocalFrame().relinquishOwnership(self);
        var previousPack = self.currentLocalFrame;
        previousPack = previousPack -% 1;
        self.locals[previousPack].restoreOwnership(self.getRegisterValue(FP), self);
        self.currentLocalFrame = previousPack;
    }
    fn restoreStandardFrame(self: *Core) void {
        self.leaveCall();
        self.restoreRIPToIP();
        self.advanceBy = 0;
    }
    fn restoreRIPToIP(self: *Core) void {
        self.ip = self.getRegisterValue(RIP);
    }
    fn localReturn(self: *Core) void {
        self.restoreStandardFrame();
    }
    fn restoreACFromStack(self: *Core, fp: Address) Ordinal {
        return self.loadFromMemory(Ordinal, fp - 12);
    }
    fn restorePCFromStack(self: *Core, fp: Address) Ordinal {
        return self.loadFromMemory(Ordinal, fp - 16);
    }
    fn inSupervisorMode(self: *const Core) bool {
        return self.pc.inSupervisorMode();
    }
    fn faultReturn(self: *Core) void {
        const fp = self.getRegisterValue(FP);
        const oldPC = self.restorePCFromStack(fp);
        const oldAC = self.restoreACFromStack(fp);
        self.restoreStandardFrame();
        self.ac.setWholeValue(oldAC);
        if (self.inSupervisorMode()) {
            self.pc.setWholeValue(oldPC);
        }
    }
    fn supervisorReturn(self: *Core, traceModeSetting: bool) void {
        if (self.inSupervisorMode()) {
            self.pc.@"trace enable" = if (traceModeSetting) 1 else 0;
            self.pc.@"execution mode" = 0;
        }
        self.restoreStandardFrame();
    }
    fn checkForPendingInterrupts(self: *Core) void {
        // @todo implement
        _ = self;
    }
    fn interruptReturn(self: *Core) void {
        const fp = self.getRegisterValue(FP);
        const oldPC = self.restorePCFromStack(fp);
        const oldAC = self.restoreACFromStack(fp);
        self.restoreStandardFrame();
        self.ac.setWholeValue(oldAC);
        if (self.inSupervisorMode()) {
            self.pc.setWholeValue(oldPC);
            self.checkForPendingInterrupts();
        }
    }
    fn ret(self: *Core) !void {
        try self.syncf();
        const pfpFull = self.getRegisterValue(PFP);
        const rt: u3 = @truncate(pfpFull & 0b111);
        switch (rt) {
            0b000 => self.localReturn(),
            0b001 => self.faultReturn(),
            0b010 => self.supervisorReturn(false),
            0b011 => self.supervisorReturn(true),
            0b111 => self.interruptReturn(),
            else => return error.Unimplemented,
        }
    }
    fn balx(self: *Core, targ: Ordinal, dest: Operand) void {

        // compute the effective address first since the link register
        // could be included in the computation
        self.setRegisterValue(dest, self.ip + self.advanceBy);
        self.ip = targ;
        self.advanceBy = 0;
    }
    fn computeBitpos(position: u5) Ordinal {
        return @as(Ordinal, 1) << position;
    }
    fn setupNewFrameInternals(self: *Core, fp: Ordinal, temp: Ordinal) void {
        self.setRegisterValue(PFP, fp);
        self.setRegisterValue(FP, temp);
        self.setRegisterValue(SP, temp + 64);
    }
    const SALIGN: Ordinal = 4; // this can be changed
    const C: Ordinal = (SALIGN * 16) - 1;
    const NotC = ~C;
    fn computeNextFrameBase(self: *Core) Ordinal {
        return (self.getRegisterValue(SP) + C) & NotC;
    }
    fn computeNextFrameFaultBase(stackPointer: Ordinal) Ordinal {
        return (stackPointer + (C * 3)) & NotC;
    }
};
fn processInstruction(core: *Core, instruction: Instruction) Faults!void {
    switch (try instruction.getOpcode()) {
        DecodedOpcode.b => core.relativeBranch(i24, instruction.ctrl.getDisplacement()),
        DecodedOpcode.bx => core.relativeBranch(i32, @bitCast(try core.computeEffectiveAddress(instruction))),
        DecodedOpcode.call,
        DecodedOpcode.callx,
        => |op| {
            // wait for any uncompleted instructions to finish
            const temp = (core.getRegisterValue(SP) + 63) & (~@as(Ordinal, 63)); // round to next boundary
            const fp = core.getRegisterValue(FP);
            // save the return address to RIP in the _current_ frame
            core.saveReturnAddress(RIP);
            core.enterCall(fp);
            // at this point, all local register references are to the new
            // frame
            // now update everything to be correct
            core.setupNewFrameInternals(fp, temp);
            // now we need to do the branching operation itself
            switch (op) {
                DecodedOpcode.call => core.relativeBranch(i24, instruction.ctrl.getDisplacement()),
                DecodedOpcode.callx => core.ip = try core.computeEffectiveAddress(instruction),
                else => unreachable,
            }
        },
        DecodedOpcode.bal => {
            core.setRegisterValue(LinkRegister, core.ip + core.advanceBy);
            core.relativeBranch(i24, instruction.ctrl.getDisplacement());
        },
        DecodedOpcode.balx => {
            const targ = try core.computeEffectiveAddress(instruction);
            const dest = instruction.getSrcDest() catch unreachable;
            core.balx(targ, dest);
        },
        DecodedOpcode.calls => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const targ: Ordinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            if (targ > 259) {
                return Faults.SegmentLength;
            }
            try core.syncf();
            const tempPE = core.loadFromMemory(Ordinal, core.getSystemProcedureTableBase() + 48 + (4 * targ));
            const typ: u2 = @truncate(tempPE & 0b11);
            const procedureAddress = tempPE & 0xFFFFFFFC;
            core.balx(procedureAddress, RIP);
            var tmp: Ordinal = 0;
            const fp = core.getRegisterValue(FP);
            var copy: PreviousFramePointer = PreviousFramePointer{};
            copy.setWholeValue(fp);
            if ((typ == 0b00) or (core.inSupervisorMode())) {
                tmp = core.computeNextFrameBase();
                copy.rt = 0;
            } else {
                tmp = core.getSupervisorStackPointer();

                const secondary: u3 = if (core.pc.@"trace enable" != 0) 0b001 else 0b000;
                copy.rt = 0b010 | secondary;
                core.pc.@"execution mode" = 1;
                core.pc.@"trace enable" = @truncate(tmp & 0b1);
                tmp &= 0xFFFF_FFFC; // clear the lowest two bits after being done here.
                // if trace is active then that could cause problems overall
                // with the address offset
            }
            core.enterCall(fp);
            core.setupNewFrameInternals(copy.toWholeValue(), tmp);
        },
        DecodedOpcode.bno => {
            if (core.ac.@"condition code" == 0b000) {
                core.relativeBranch(i24, instruction.ctrl.getDisplacement());
            }
        },
        DecodedOpcode.bg,
        DecodedOpcode.be,
        DecodedOpcode.bge,
        DecodedOpcode.bl,
        DecodedOpcode.bne,
        DecodedOpcode.ble,
        DecodedOpcode.bo,
        => |opcode| {
            if ((opcode.getConditionCode() & core.ac.@"condition code") != 0) {
                core.relativeBranch(i24, instruction.ctrl.getDisplacement());
            }
        },

        DecodedOpcode.faultno => {
            if (core.ac.@"condition code" == 0) {
                return Faults.ConstraintRange;
            }
        },
        DecodedOpcode.faultg,
        DecodedOpcode.faulte,
        DecodedOpcode.faultge,
        DecodedOpcode.faultl,
        DecodedOpcode.faultne,
        DecodedOpcode.faultle,
        DecodedOpcode.faulto,
        => |opcode| {
            if ((opcode.getConditionCode() & core.ac.@"condition code") != 0) {
                return Faults.ConstraintRange;
            }
        },
        DecodedOpcode.testno => core.setRegisterValue(instruction.getSrc1() catch unreachable, if (core.ac.@"condition code" == 0b000) 1 else 0),
        DecodedOpcode.teste,
        DecodedOpcode.testne,
        DecodedOpcode.testl,
        DecodedOpcode.testle,
        DecodedOpcode.testg,
        DecodedOpcode.testge,
        DecodedOpcode.testo,
        => |opcode| core.setRegisterValue(instruction.getSrc1() catch unreachable, if ((core.ac.@"condition code" & opcode.getConditionCode()) != 0) 1 else 0),
        DecodedOpcode.bbc => {
            // branch if bit clear
            const displacement = instruction.cobr.getDisplacement();
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const bitpos: Ordinal = Core.computeBitpos(@truncate((if (instruction.cobr.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index)) & 0b11111)); // bitpos mod 32
            const src = core.getRegisterValue(src2Index);
            if ((src & bitpos) == 0) {
                core.ac.@"condition code" = 0b010;
                core.relativeBranch(@TypeOf(displacement), displacement);
                // resume execution at the new ip
            } else {
                core.ac.@"condition code" = 0b000;
            }
        },

        DecodedOpcode.bbs => {
            // branch if bit set
            const displacement = instruction.cobr.getDisplacement();
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const bitpos: Ordinal = Core.computeBitpos(@truncate((if (instruction.cobr.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index)) & 0b11111)); // bitpos mod 32
            const src = core.getRegisterValue(src2Index);
            if ((src & bitpos) != 0) {
                core.ac.@"condition code" = 0b010;
                core.relativeBranch(@TypeOf(displacement), displacement);
                // resume execution at the new ip
            } else {
                core.ac.@"condition code" = 0b000;
            }
        },
        DecodedOpcode.emul => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const src1: LongOrdinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const src2: LongOrdinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            try core.setLongRegisterValue(srcDestIndex, src2 *% src1);
        },
        DecodedOpcode.ediv => {
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            if ((srcDestIndex & 0b1) != 0) {
                return Faults.InvalidOpcode;
            }
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const src1: Ordinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const src2: LongOrdinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else try core.getLongRegisterValue(src2Index);
            core.setRegisterValue(srcDestIndex, @truncate(try math.rem(LongOrdinal, src2, src1)));
            core.setRegisterValue(srcDestIndex + 1, @truncate(try math.divTrunc(LongOrdinal, src2, src1)));
        },
        DecodedOpcode.rotate => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const len: Ordinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const src: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            core.setRegisterValue(srcDestIndex, math.rotl(Ordinal, src, len & 0b11111));
        },
        DecodedOpcode.scanbyte => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const src1: Ordinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const src2: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            core.ac.@"condition code" = if (((src1 & 0x000000FF) == (src2 & 0x000000FF)) or
                ((src1 & 0x0000FF00) == (src2 & 0x0000FF00)) or
                ((src1 & 0x00FF0000) == (src2 & 0x00FF0000)) or
                ((src1 & 0xFF000000) == (src2 & 0xFF000000))) 0b010 else 0b000;
        },
        DecodedOpcode.setbit => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const bitpos: Ordinal = Core.computeBitpos(@truncate((if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index)) & 0b11111)); // bitpos mod 32
            const src: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            core.setRegisterValue(srcDestIndex, src | bitpos);
        },
        DecodedOpcode.clrbit => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const bitpos: Ordinal = Core.computeBitpos(@truncate((if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index)) & 0b11111)); // bitpos mod 32
            const src: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            core.setRegisterValue(srcDestIndex, src & (~bitpos));
        },
        DecodedOpcode.notbit => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const bitpos: Ordinal = Core.computeBitpos(@truncate((if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index)) & 0b11111)); // bitpos mod 32
            const src: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            core.setRegisterValue(srcDestIndex, src ^ bitpos);
        },
        DecodedOpcode.chkbit => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const bitpos: Ordinal = Core.computeBitpos(@truncate((if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index)) & 0b11111)); // bitpos mod 32
            const src: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            core.ac.@"condition code" = if ((src & bitpos) == 0) 0b000 else 0b010;
        },
        DecodedOpcode.cmpo,
        DecodedOpcode.cmpdeco,
        DecodedOpcode.cmpinco,
        => |op| {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const src1: Ordinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const src2: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            core.ac.@"condition code" = if (src1 < src2) 0b100 else if (src1 == src2) 0b010 else 0b001;
            switch (op) {
                DecodedOpcode.cmpdeco => core.setRegisterValue(srcDestIndex, src2 -% 1),
                DecodedOpcode.cmpinco => core.setRegisterValue(srcDestIndex, src2 +% 1),
                else => {},
            }
        },
        DecodedOpcode.cmpi,
        DecodedOpcode.cmpdeci,
        DecodedOpcode.cmpinci,
        => |op| {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const src1: Integer = @bitCast(if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index));
            const src2: Integer = @bitCast(if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index));
            core.ac.@"condition code" = if (src1 < src2) 0b100 else if (src1 == src2) 0b010 else 0b001;
            switch (op) {
                DecodedOpcode.cmpdeci => core.setRegisterValue(srcDestIndex, @bitCast(src2 -% 1)),
                DecodedOpcode.cmpinci => core.setRegisterValue(srcDestIndex, @bitCast(src2 +% 1)),
                else => {},
            }
        },
        DecodedOpcode.@"and",
        DecodedOpcode.@"or",
        DecodedOpcode.andnot,
        DecodedOpcode.notand,
        DecodedOpcode.ornot,
        DecodedOpcode.notor,
        DecodedOpcode.nand,
        DecodedOpcode.nor,
        DecodedOpcode.xor,
        DecodedOpcode.xnor,
        DecodedOpcode.mulo,
        DecodedOpcode.shro,
        DecodedOpcode.shlo,
        DecodedOpcode.not,
        DecodedOpcode.addo,
        DecodedOpcode.mov,
        => |operation| {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const src1: Ordinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const src2: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            // aliases
            const len = src1;
            const src = src2;
            core.setRegisterValue(srcDestIndex, switch (comptime operation) {
                DecodedOpcode.@"and" => src2 & src1,
                DecodedOpcode.andnot => src2 & (~src1),
                DecodedOpcode.notand => (~src2) & src1,
                DecodedOpcode.@"or" => src2 | src1,
                DecodedOpcode.ornot => src2 | (~src1),
                DecodedOpcode.notor => (~src2) | src1,
                DecodedOpcode.nand => (~src2) | (~src1),
                DecodedOpcode.nor => (~src2) & (~src1),
                DecodedOpcode.xor => src2 ^ src1,
                DecodedOpcode.xnor => ~(src2 ^ src1),
                DecodedOpcode.mulo => src2 *% src1,
                DecodedOpcode.not => ~src1,
                DecodedOpcode.shlo => if (len < 32) src << @truncate(len) else 0,
                DecodedOpcode.shro => if (len < 32) src >> @truncate(len) else 0,
                DecodedOpcode.addo => src2 +% src1,
                DecodedOpcode.mov => src1,
                else => unreachable,
            });
        },
        DecodedOpcode.addi,
        DecodedOpcode.muli,
        => |operation| {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const src1: Integer = @bitCast(if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index));
            const src2: Integer = @bitCast(if (instruction.reg.treatSrc2AsLiteral()) src1Index else core.getRegisterValue(src2Index));
            // support detecting integer overflow here
            core.setRegisterValue(srcDestIndex, @bitCast(switch (comptime operation) {
                DecodedOpcode.addi => try math.add(Integer, src2, src1),
                DecodedOpcode.muli => try math.mul(Integer, src2, src1),
                else => unreachable,
            }));
        },
        DecodedOpcode.modify => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const mask: Ordinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const src: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            const srcDest = core.getRegisterValue(srcDestIndex);
            core.setRegisterValue(srcDestIndex, modify(mask, src, srcDest));
        },
        DecodedOpcode.alterbit => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const bitpos: Ordinal = Core.computeBitpos(@truncate((if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index)) & 0b11111)); // bitpos mod 32
            const src: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            core.setRegisterValue(srcDestIndex, alterbit(src, bitpos, (core.ac.@"condition code" & 0b010) == 0));
        },
        DecodedOpcode.syncf => try core.syncf(),
        DecodedOpcode.movl => {
            // be as simple as possible
            const src1Index = instruction.getSrc1() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const srcValue: LongOrdinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else try core.getLongRegisterValue(src1Index);
            try core.setLongRegisterValue(srcDestIndex, srcValue);
        },
        DecodedOpcode.movt => {
            // be as simple as possible
            const src1Index = instruction.getSrc1() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const srcValue: TripleOrdinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else try core.getTripleRegisterValue(src1Index);
            try core.setTripleRegisterValue(srcDestIndex, srcValue);
        },
        DecodedOpcode.movq => {
            // be as simple as possible
            const src1Index = instruction.getSrc1() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const srcValue: QuadOrdinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else try core.getQuadRegisterValue(src1Index);
            try core.setQuadRegisterValue(srcDestIndex, srcValue);
        },
        DecodedOpcode.divo => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const denominator: Ordinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const numerator: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            core.setRegisterValue(srcDestIndex, math.divTrunc(Ordinal, numerator, denominator) catch |err| {
                core.nextInstruction();
                return err;
            });
        },
        DecodedOpcode.divi => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const denominator: Integer = @bitCast(if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index));
            const numerator: Integer = @bitCast(if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index));
            core.setRegisterValue(srcDestIndex, @bitCast(math.divTrunc(Integer, numerator, denominator) catch |err| switch (err) {
                error.Overflow => {
                    core.setRegisterValue(srcDestIndex, @bitCast(@as(Integer, math.minInt(Integer))));
                    if (core.ac.@"integer overflow mask" == 1) {
                        core.ac.@"integer overflow flag" = 1;
                        return;
                    } else {
                        core.nextInstruction();
                        return err;
                    }
                },
                else => {
                    core.nextInstruction();
                    return err;
                },
            }));
        },
        DecodedOpcode.lda => {
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            core.setRegisterValue(srcDestIndex, try core.computeEffectiveAddress(instruction));
        },
        DecodedOpcode.ldob => {
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            core.setRegisterValue(srcDestIndex, core.loadFromMemory(ByteOrdinal, try core.computeEffectiveAddress(instruction)));
        },
        DecodedOpcode.ldib => {
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const efa = try core.computeEffectiveAddress(instruction);
            const upgradedValue: Integer = core.loadFromMemory(ByteInteger, efa);
            core.setRegisterValue(srcDestIndex, @bitCast(upgradedValue));
        },
        DecodedOpcode.ldos => {
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            core.setRegisterValue(srcDestIndex, core.loadFromMemory(ShortOrdinal, try core.computeEffectiveAddress(instruction)));
        },
        DecodedOpcode.ldis => {
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const efa = try core.computeEffectiveAddress(instruction);
            const upgradedValue: Integer = core.loadFromMemory(ShortInteger, efa);
            core.setRegisterValue(srcDestIndex, @bitCast(upgradedValue));
        },
        DecodedOpcode.ld => {
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            core.setRegisterValue(srcDestIndex, core.loadFromMemory(Ordinal, try core.computeEffectiveAddress(instruction)));
        },
        DecodedOpcode.ldl => {
            const effectiveAddress = try core.computeEffectiveAddress(instruction);
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            try core.setLongRegisterValue(srcDestIndex, core.loadFromMemory(LongOrdinal, effectiveAddress));
        },
        DecodedOpcode.ldt => {
            const effectiveAddress = try core.computeEffectiveAddress(instruction);
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            try core.setTripleRegisterValue(srcDestIndex, core.loadFromMemory(TripleOrdinal, effectiveAddress));
        },
        DecodedOpcode.ldq => {
            const effectiveAddress = try core.computeEffectiveAddress(instruction);
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            try core.setQuadRegisterValue(srcDestIndex, core.loadFromMemory(QuadOrdinal, effectiveAddress));
        },
        DecodedOpcode.modi => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const denominator: Integer = @bitCast(if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index));
            const numerator: Integer = @bitCast(if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index));
            if (denominator == 0) {
                return error.DivisionByZero;
            }
            var newDest: Integer = numerator - (@divExact(numerator, denominator) * denominator);
            if ((numerator * denominator) < 0) {
                newDest += denominator;
            }
            core.setRegisterValue(srcDestIndex, @bitCast(newDest));
        },
        DecodedOpcode.selno,
        DecodedOpcode.selg,
        DecodedOpcode.sele,
        DecodedOpcode.selge,
        DecodedOpcode.sell,
        DecodedOpcode.selne,
        DecodedOpcode.selle,
        DecodedOpcode.selo,
        => |op| {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const src1: Ordinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const src2: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            const cc = core.ac.@"condition code";
            const mask = op.getConditionCode();
            core.setRegisterValue(srcDestIndex, if (((mask & cc) != 0) or (mask == cc)) src2 else src1);
        },
        DecodedOpcode.addono,
        DecodedOpcode.addog,
        DecodedOpcode.addoe,
        DecodedOpcode.addoge,
        DecodedOpcode.addol,
        DecodedOpcode.addone,
        DecodedOpcode.addole,
        DecodedOpcode.addoo,
        => |op| {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const src1: Ordinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const src2: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            const cc = core.ac.@"condition code";
            const mask = op.getConditionCode();
            if (((mask & cc) != 0) or (mask == cc)) {
                core.setRegisterValue(srcDestIndex, src2 +% src1);
            }
        },
        DecodedOpcode.addino,
        DecodedOpcode.addig,
        DecodedOpcode.addie,
        DecodedOpcode.addige,
        DecodedOpcode.addil,
        DecodedOpcode.addine,
        DecodedOpcode.addile,
        DecodedOpcode.addio,
        => |op| {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const src1: Integer = @bitCast(if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index));
            const src2: Integer = @bitCast(if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index));
            const cc = core.ac.@"condition code";
            const mask = op.getConditionCode();
            if (((mask & cc) != 0) or (mask == cc)) {
                // always set the destination register
                const trueResult = @addWithOverflow(src2, src1);
                core.setRegisterValue(srcDestIndex, @bitCast(trueResult[0]));
                if (trueResult[1] != 0) {
                    if (core.ac.@"integer overflow mask" == 1) {
                        core.ac.@"integer overflow flag" = 1;
                    } else {
                        return error.Overflow;
                    }
                }
            }
        },
        DecodedOpcode.addc => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const src1: Ordinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const src2: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            const carryBit: Ordinal = if ((core.ac.@"condition code" & 0b010) != 0) 1 else 0;
            const partialCombination = @addWithOverflow(src2, src1);
            const finalOutput = @addWithOverflow(partialCombination[0], carryBit);
            const resultantCarry: u1 = partialCombination[1] | finalOutput[1];
            const src2TopBit = src2 & 0x8000_0000;
            const src1TopBit = src1 & 0x8000_0000;
            const destTopBit = finalOutput[0] & 0x8000_0000;
            core.setRegisterValue(srcDestIndex, finalOutput[0]);
            var cc: u3 = 0b000;
            // compute if an integer overflow would have taken place
            cc |= if ((src2TopBit == src1TopBit) and (src2TopBit != destTopBit)) 0b001 else 0b000;
            // translate the overflow bit to the carry out position
            cc |= if (resultantCarry != 0) 0b010 else 0b000;
            core.ac.@"condition code" = cc;
        },
        DecodedOpcode.atadd => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const destIndex = instruction.getSrcDest() catch unreachable;
            const addr: Ordinal = core.getRegisterValue(src1Index);
            const src: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            // implicit syncf is performed but who cares for the simulator's
            // purposes
            try core.syncf();
            const tempa = addr & (~(@as(Ordinal, 3)));
            const temp = core.atomicLoad(tempa);
            core.atomicStore(tempa, temp +% src);
            core.setRegisterValue(destIndex, temp);
        },
        DecodedOpcode.atmod => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const destIndex = instruction.getSrcDest() catch unreachable;
            const addr: Ordinal = core.getRegisterValue(src1Index);
            const mask: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            // implicit syncf is performed but who cares for the simulator's
            // purposes
            try core.syncf();
            const tempa = addr & (~(@as(Ordinal, 3)));
            const temp = core.atomicLoad(tempa);

            core.atomicStore(tempa, (addr & mask) | (temp & ~mask));
            core.setRegisterValue(destIndex, temp);
        },
        DecodedOpcode.cmpstr => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const src1: Ordinal = core.getRegisterValue(src1Index);
            const src2: Ordinal = core.getRegisterValue(src2Index);
            const len: Ordinal = if (instruction.reg.treatSrcDestAsLiteral()) srcDestIndex else core.getRegisterValue(srcDestIndex);
            core.ac.@"condition code" = 0b010;
            for (0..(len - 1)) |i| {
                const offset: Address = @truncate(i);
                const bs1 = core.loadFromMemory(ByteOrdinal, src1 +% offset);
                const bs2 = core.loadFromMemory(ByteOrdinal, src2 +% offset);
                if (bs1 > bs2) {
                    core.ac.@"condition code" = 0b001;
                    break;
                } else if (bs1 < bs2) {
                    core.ac.@"condition code" = 0b100;
                    break;
                }
            }
        },
        DecodedOpcode.fill => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const dst: Ordinal = core.getRegisterValue(src1Index);
            const value: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            const len: Ordinal = if (instruction.reg.treatSrcDestAsLiteral()) srcDestIndex else core.getRegisterValue(srcDestIndex);
            core.ac.@"condition code" = 0b010;
            for (0..((len / 4) - 1)) |i| {
                const offset: Address = @truncate(i);
                core.storeToMemory(Ordinal, dst +% offset, value);
            }
            switch (@mod(len, 4)) {
                1 => core.storeToMemory(ByteOrdinal, dst + len - 1, @truncate(value)),
                2 => core.storeToMemory(ShortOrdinal, dst + len - 2, @truncate(value)),
                3 => {
                    core.storeToMemory(ShortOrdinal, dst + len - 3, @truncate(value));
                    core.storeToMemory(ByteOrdinal, dst + len - 1, @truncate(value >> 16));
                },
                else => {},
            }
        },
        DecodedOpcode.movqstr => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const dst: Ordinal = core.getRegisterValue(src1Index);
            const src: Ordinal = core.getRegisterValue(src2Index);
            const len: Ordinal = if (instruction.reg.treatSrcDestAsLiteral()) srcDestIndex else core.getRegisterValue(srcDestIndex);
            for (0..(len - 1)) |i| {
                const offset: Address = @truncate(i);
                core.storeToMemory(ByteOrdinal, dst +% offset, core.loadFromMemory(ByteOrdinal, src +% offset));
            }
        },
        DecodedOpcode.movstr => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const dst: Ordinal = core.getRegisterValue(src1Index);
            const src: Ordinal = core.getRegisterValue(src2Index);
            const len: Ordinal = if (instruction.reg.treatSrcDestAsLiteral()) srcDestIndex else core.getRegisterValue(srcDestIndex);
            if (src <= dst) {
                for (1..len) |i| {
                    const offset: Address = @truncate(i);
                    core.storeToMemory(ByteOrdinal, dst + len - offset, core.loadFromMemory(ByteOrdinal, src + len - offset));
                }
            } else {
                for (0..(len - 1)) |i| {
                    const offset: Address = @truncate(i);
                    core.storeToMemory(ByteOrdinal, dst +% offset, core.loadFromMemory(ByteOrdinal, src +% offset));
                }
            }
        },
        DecodedOpcode.remo => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const denominator: Ordinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const numerator: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);

            core.setRegisterValue(srcDestIndex, try math.rem(Ordinal, numerator, denominator));
        },
        DecodedOpcode.remi => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const denominator: Integer = @bitCast(if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index));
            const numerator: Integer = @bitCast(if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index));
            // @todo implement support for Integer Overflow (which the Hx docs
            // state does not generate an overflow fault)
            //
            // According to the Hx documentation, "remi produces the correct
            // result (0) even when computing -2^31 remi -1, which would
            // normally cause the corresponding division to overflow, although
            // no fault is generated.
            //
            // So, no integer overflow despite the MC and KB manuals stating
            // that an integer overflow is generated...
            const dest: Integer = numerator - (math.divTrunc(Integer, numerator, denominator) catch |x| switch (x) {
                error.Overflow => 0,
                else => return x,
            }) * denominator;
            core.setRegisterValue(srcDestIndex, @bitCast(dest));
        },
        DecodedOpcode.extract => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const bitpos: Ordinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const len: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            const srcDest: Ordinal = core.getRegisterValue(srcDestIndex);
            if (bitpos >= 32) {
                // anding anything with zero will result in zero so just bypass
                core.setRegisterValue(srcDestIndex, 0);
            } else if (len >= 32) {
                // so if len is greater than or equal to 32 then it means zeros
                // are shifted in and then inverted so just do the shifting
                // operation
                core.setRegisterValue(srcDestIndex, srcDest >> @truncate(bitpos));
            } else {
                // they are both in range :)
                core.setRegisterValue(srcDestIndex, (srcDest >> @truncate(bitpos)) & (~(@as(Ordinal, 0xFFFF_FFFF) << @truncate(len))));
            }
        },
        DecodedOpcode.flushreg => {
            for (&core.locals) |*x| {
                x.relinquishOwnership(core);
            }
        },
        DecodedOpcode.ret => try core.ret(),

        else => return Faults.Unimplemented,
    }
}
fn modify(mask: Ordinal, src: Ordinal, srcDest: Ordinal) Ordinal {
    return (src & mask) | (srcDest & (~mask));
}
fn alterbit(src: Ordinal, bitpos: Ordinal, clearBit: bool) Ordinal {
    return if (clearBit) (src & (~bitpos)) else (src | bitpos);
}
pub fn main() !void {
    //std.debug.print("i960 Simulator\n", .{});
    // allocate all of the memory at once
    NativeInterface.begin();
    defer NativeInterface.end();
    var core = Core{};
    // part of the system tests
    const message = "i960 Simulator\n";
    for (message) |x| {
        core.storeToMemory(@TypeOf(x), NativeInterface.SerialIOAddress, x);
    }
    try core.start();
    while (core.continueExecuting) {
        core.newCycle();
        const result = decode(core.loadFromMemory(Ordinal, core.ip));
        processInstruction(&core, result) catch |x| {
            const record: FaultRecord = core.constructFaultRecord(x);
            try core.generateFault(&record);
        };

        core.nextInstruction();
    }
}

// test cases
test "io system test" {
    var core = Core{};
    const compareMilli = std.time.milliTimestamp();
    const compareMicro = std.time.microTimestamp();
    try expectEqual(core.loadFromMemory(Ordinal, NativeInterface.CPUClockRateAddress), NativeInterface.CPUClockRate);
    try expectEqual(core.loadFromMemory(Ordinal, NativeInterface.SystemClockRateAddress), NativeInterface.SystemClockRate);
    try expect(core.loadFromMemory(LongInteger, NativeInterface.MillisecondsTimestampAddress) >= compareMilli);
    try expect(core.loadFromMemory(LongInteger, NativeInterface.MicrosecondsTimestampAddress) >= compareMicro);
    std.debug.print("\n\n0x{x} vs 0x{x}\n0x{x} vs 0x{x}\n", .{
        core.loadFromMemory(LongInteger, NativeInterface.MillisecondsTimestampAddress),
        compareMilli,
        core.loadFromMemory(LongInteger, NativeInterface.MicrosecondsTimestampAddress),
        compareMicro,
    });
    // store operations
    core.storeToMemory(ByteOrdinal, NativeInterface.SerialIOAddress, 'A');
    core.storeToMemory(Ordinal, NativeInterface.SerialIOAddress, 'A');
    //core.storeToMemory(ByteOrdinal, 0xFE00_000C, 0);
    // only activate this one once I figure out a way to input data to standard in
    //try expect(core.loadFromMemory(Ordinal, 0xFE00_0008) != 0xFFFF_FFFF);

}

test "Opcodes Sanity Checks 2" {
    const originalValue: u32 = 0x3f00_0000;
    const decodedInstruction = decode(originalValue);
    const value = decodedInstruction.getOpcode() catch {
        try expect(false);
        return;
    };
    try expect(value == DecodedOpcode.cmpibo);
}

test "sizeof sanity check" {
    try expect(@sizeOf(ArithmeticControls) == 4);
    try expect(@sizeOf(ProcessControls) == 4);
    try expect(@sizeOf(TraceControls) == 4);
    try expectEqual(@sizeOf(FaultTable), 256);
}

test "instruction size tests" {
    try expect(@sizeOf(Ordinal) == 4);
    try expect(@sizeOf(CTRLInstruction) == @sizeOf(Ordinal));
    try expect(@sizeOf(COBRInstruction) == @sizeOf(Ordinal));
    try expect(@sizeOf(REGInstruction) == @sizeOf(Ordinal));
    try expect(@sizeOf(MEMAInstruction) == @sizeOf(Ordinal));
    try expect(@sizeOf(MEMBInstruction) == @sizeOf(Ordinal));
    try expect(@sizeOf(InstructionDeterminant) == @sizeOf(Ordinal));
}

test "simple reg test" {
    const x = REGInstruction{
        .src1 = 9,
        .opcodeExt = 0x2,
        .m1 = false,
        .m2 = true,
        .m3 = false,
        .src2 = 8,
        .srcDest = 7,
        .opcode = 0x58,
    };
    try expect(@intFromEnum(x.getOpcode() catch |err| {
        try expect(err == Faults.InvalidOpcode);
        return;
    }) == 0x582);
    try expect(x.getOpcode() catch |err| {
        try expect(err == Faults.InvalidOpcode);
        return;
    } == DecodedOpcode.andnot);
    try expect(x.srcDest == 7);
    try expect(x.src2 == 8);
    try expect(x.src1 == 9);
}

test "simple ctrl test" {
    const x = CTRLInstruction{
        .displacement = 0xFDEC,
        .opcode = 0x8,
    };
    try expect(@intFromEnum(x.getOpcode() catch |err| {
        try expect(err == Faults.InvalidOpcode);
        return;
    }) == 0x08);
    try expect(x.getOpcode() catch |err| {
        try expect(err == Faults.InvalidOpcode);
        return;
    } == DecodedOpcode.b);
    try expect(x.getDisplacement() == 0xFDEC);
}

test "simple ctrl test2" {
    var x: CTRLInstruction = .{
        .displacement = 0,
        .opcode = 0,
    };

    x.displacement = 0xFDEC;
    x.opcode = 0x08;
    try expect(@intFromEnum(x.getOpcode() catch |err| {
        try expect(err == Faults.InvalidOpcode);
        return;
    }) == 0x08);
    try expect(x.getOpcode() catch |err| {
        try expect(err == Faults.InvalidOpcode);
        return;
    } == DecodedOpcode.b);
    try expect(x.getDisplacement() == 0xFDEC);
}

test "simple cobr test" {
    const x = COBRInstruction{
        .displacement = 0xFF,
        .m1 = false,
        .src1 = 4,
        .src2 = 5,
        .opcode = 0x32,
    };
    try expect(x.getDisplacement() == 0xFC);
    try expect(!x.m1);
    try expect(x.src1 == 4);
    try expect(x.src2 == 5);
    const value: u32 = @bitCast(x);
    std.debug.print("{x}\n", .{value});
}

test "MEMBFormat tests" {
    try expect(MEMBAddressComputationKind.displacement.usesOptionalDisplacement());
}

test "instruction decoder test 0" {
    const originalValue: u32 = 0x322140ff;

    try expect(switch (decode(originalValue)) {
        .cobr => true,
        else => false,
    });
}

test "instruction decoder test 1" {
    const originalValue: u32 = 0x0800_0000;

    try expect(switch (decode(originalValue)) {
        .ctrl => true,
        else => false,
    });
}

test "instruction decoder test 2" {
    const originalValue: u32 = 0x8000_0000;

    try expect(switch (decode(originalValue)) {
        .mema => true,
        else => false,
    });
}

test "allocation test3" {
    const allocator = std.heap.page_allocator;
    const buffer = try allocator.alloc(u64, 4);
    defer allocator.free(buffer);
    try expect(buffer.len == 4);
    for (buffer, 0..) |*val, ind| {
        val.* = @truncate(ind + 0x0706050403020100);
    }
    for (buffer, 0..) |val, ind| {
        try expectEqual(val, (ind + 0x0706050403020100));
    }
    // okay, so now we need to see if we can view the various components
    //
    // it is trivial to go from larger to smaller!
    const buffer2: *const [@sizeOf(u64)]u8 = @ptrCast(&buffer[0]);
    for (buffer2, 0..) |value, idx| {
        try expectEqual(value, idx);
    }
    // view as two ordinals instead
    const buffer3: *const [@sizeOf(u64) / @sizeOf(Ordinal)]Ordinal = @ptrCast(&buffer[0]);
    try expect(buffer3[0] == 0x03020100 or buffer3[0] == 0x07060504);
    try expect(buffer3[1] == 0x03020100 or buffer3[1] == 0x07060504);
}

test "allocation test4" {
    const AllocationType = u128;
    const allocator = std.heap.page_allocator;
    const buffer = try allocator.alloc(AllocationType, 4);
    defer allocator.free(buffer);
    try expect(buffer.len == 4);
    const combinatorial: AllocationType = 0x0f0e0d0c_0b0a0908_07060504_03020100;
    for (buffer, 0..) |*val, ind| {
        val.* = @truncate(ind + combinatorial);
    }
    for (buffer, 0..) |val, ind| {
        try expectEqual(val, (ind + combinatorial));
    }
    // okay, so now we need to see if we can view the various components
    //
    // it is trivial to go from larger to smaller!
    const buffer2: *const [@sizeOf(AllocationType)]u8 = @ptrCast(&buffer[0]);
    for (buffer2, 0..) |value, idx| {
        try expectEqual(value, idx);
    }
}

test "Structure Size Check 2" {
    try expectEqual(@sizeOf(MemoryPool), (4 * 1024 * 1024 * 1024));
}

test "alterbit logic" {
    try expectEqual(alterbit(0, (1 << 24), false), 0x0100_0000);
}
test "modify logic" {
    try expectEqual(modify(0xFF, 0xFFFF, 0), 0xFF);
}
test "core startup test" {
    try NativeInterface.begin();
    defer NativeInterface.end();
    var core = Core{};
    // make a fake PRCB
    var bootWords = [8]Ordinal{
        0x600,
        0x6c0,
        0,
        0x20,
        0xFFFF_F320,
        0,
        0,
        0xFFFF_FFFF,
    };
    for (bootWords, 0..) |word, idx| {
        const adr: Address = @truncate(idx * @sizeOf(Ordinal));
        core.storeToMemory(Ordinal, adr, word);
    }
    try core.start();
    // hack bootwords and update
    bootWords[6] = 128;
    for (bootWords, 0..) |word, idx| {
        const adr: Address = @truncate(idx * @sizeOf(Ordinal));
        core.storeToMemory(Ordinal, adr, word);
    }
    if (core.start()) {
        // there should be a checksum failure
        return error.@"Expected Checksum to fail!";
    } else |err| {
        try expectEqual(err, BootResult.ChecksumFail);
    }
}
