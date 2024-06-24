const std = @import("std");
const meta = @import("std").meta;
const expect = std.testing.expect;
const expect_eq = std.testing.expectEqual;
const types960 = @import("types960.zig");
const opcodes960 = @import("opcodes.zig");
const instruction960 = @import("Instruction.zig");

const Operand = types960.Operand;
const Ordinal = types960.Ordinal;
const Address = types960.Address;
const Instruction = instruction960.Instruction;
const DecodedOpcode = opcodes960.DecodedOpcode;
const PFP: Operand = 0;
const SP: Operand = 1;
const RIP: Operand = 2;
const LinkRegister: Operand = 30; // g14
const FramePointer: Operand = 31;
fn getPFPAddress(value: Ordinal) Ordinal {
    return value & 0xFFFFFFF0;
}
const GenericSegmentDescriptor = packed struct {
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
};
const FaultProcedureEntry = packed struct {
    @"procedure address": u32 = 0,
    @"segment selector": u32 = 0,
    pub fn getProcedureIndex(self: *const FaultProcedureEntry) Address {
        return self.@"procedure address" & 0xFFFF_FFFC;
    }
    pub fn wholeValue(self: *const FaultProcedureEntry) u64 {
        return @as(*u64, @ptrCast(self)).*;
    }
};

const FaultTable = packed struct {
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
const FaultRecord = packed struct {
    unused: u32 = 0,
    @"override fault data": u96 = 0,
    @"fault data": u96 = 0,
    @"override subtype": u8 = 0,
    unused1: u8 = 0,
    @"override type": u8 = 0,
    @"override flags": u8 = 0,
    @"process controls": u32,
    @"arithmetic controls": u32,
    @"fault subtype": u8 = 0,
    unused2: u8 = 0,
    @"fault type": u8 = 0,
    @"fault flags": u8 = 0,
    @"address of faulting instruction": u32,
};
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
    @"floating-point rounding control": u1 = 0,

    pub fn toWholeValue(self: *const ArithmeticControls) Ordinal {
        return @as(*Ordinal, @ptrCast(self)).*;
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
        return @as(*Ordinal, @ptrCast(self)).*;
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
};

const Core = struct {
    globals: types960.RegisterFrame,
    locals: [4]types960.RegisterFrame,
    fpr: [4]types960.ExtendedReal,
    ip: Ordinal = 0,
    currentLocalFrame: u2 = 0,
    advanceBy: u3 = 4,
    pc: ProcessControls,
    ac: ArithmeticControls,
    tc: TraceControls,
    continueExecuting: bool = true,
    fn getRegister(self: *Core, index: Operand) *u32 {
        if (index > 16) {
            return &self.globals[index and 0b1111];
        } else {
            return &self.locals[self.currentLocalFrame][index and 0b1111];
        }
    }
    fn getRegisterValue(self: *Core, index: Operand) u32 {
        return self.getRegister(index).*;
    }
    fn processInstruction(self: *Core, instruction: Instruction) !void {
        switch (instruction.getOpcode() catch |x| {
            return x;
        }) {
            DecodedOpcode.b => {
                self.advanceBy = 0;
                self.ip += instruction.ctrl.getDisplacement();
            },
            DecodedOpcode.bal => {
                self.advanceBy = 0;
                self.getRegister(LinkRegister).* = (self.ip + 4);
                self.ip += instruction.ctrl.getDisplacement();
            },
            DecodedOpcode.addo => {
                const dest = instruction.getSrcDest() catch |x| {
                    return x;
                };
                const src1 = instruction.getSrc1() catch |x| {
                    return x;
                };
                const src2 = instruction.getSrc2() catch |x| {
                    return x;
                };
                self.getRegister(dest).* = self.getRegisterValue(src2) + self.getRegisterValue(src1);
            },
            else => return error.Unimplemented,
        }
    }
    fn cycle(self: *Core) void {
        while (self.continueExecuting) {}
    }
};

pub fn main() void {
    std.debug.print("i960 Simulator\n", .{});
    //var c = Core{};
}

// test cases

test "Opcodes Sanity Checks" {
    try expect(@intFromEnum(DecodedOpcode.cmpibo) == 0x3f);
}

test "Opcodes Sanity Checks 2" {
    const originalValue: u32 = 0x3f00_0000;
    const decodedInstruction = instruction960.decode(originalValue);
    const value = decodedInstruction.getOpcode() catch {
        try expect(false);
        return;
    };
    try expect(value == DecodedOpcode.cmpibo);
}

test "FaultRecord sanity check" {
    try expect(@sizeOf(FaultRecord) == 48);
}

test "sizeof sanity check" {
    try expect(@sizeOf(ArithmeticControls) == 4);
    try expect(@sizeOf(ProcessControls) == 4);
    try expect(@sizeOf(TraceControls) == 4);
    try expect_eq(@sizeOf(FaultTable), 256);
}
