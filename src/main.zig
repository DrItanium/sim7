const std = @import("std");
const meta = @import("std").meta;
const expect = std.testing.expect;
const expect_eq = std.testing.expectEqual;
const types960 = @import("types960.zig");
const opcodes960 = @import("opcodes.zig");

const Ordinal = types960.Ordinal;
const Operand = types960.Operand;
const Address = types960.Address;
const DecodedOpcode = opcodes960.DecodedOpcode;
const InstructionClass = opcodes960.InstructionClass;

const CTRLInstruction = packed struct {
    displacement: i24,
    opcode: u8,

    pub fn getDisplacement(self: *const CTRLInstruction) i24 {
        const bits = @as(u24, @bitCast(self.displacement)) & 0xFFFFFC;
        return @as(i24, @bitCast(bits));
    }
    pub fn getOpcode(self: *const CTRLInstruction) !DecodedOpcode {
        return meta.intToEnum(DecodedOpcode, self.opcode) catch error.IllegalOpcode;
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
    pub fn getOpcode(self: *const COBRInstruction) !DecodedOpcode {
        return meta.intToEnum(DecodedOpcode, self.opcode) catch error.IllegalOpcode;
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
    pub fn getOpcode(self: *const REGInstruction) !DecodedOpcode {
        var major: u12 = @as(u12, self.opcode);
        major <<= 4;
        const minor: u12 = self.opcodeExt;
        //return @enumFromInt(major | minor);
        return meta.intToEnum(DecodedOpcode, major | minor) catch error.IllegalOpcode;
    }
};
const MEMAAddressComputationKind = enum(u1) {
    Offset = 0,
    @"(abase)+offset" = 1,
};
const MEMBAddressComputationKind = enum(u4) {
    @"(abase)" = 0b0100,
    @"(IP)+displacement+8" = 0b0101,
    @"(abase)+(index)*2^scale" = 0b0111,
    displacement = 0b1100,
    @"(abase)+displacement" = 0b1101,
    @"(index)*2^scale+displacement" = 0b1110,
    @"(abase)+(index)*2^scale+displacement" = 0b1111,
    pub fn usesOptionalDisplacement(self: MEMBAddressComputationKind) bool {
        return switch (self) {
            MEMBAddressComputationKind.@"(abase)+displacement", MEMBAddressComputationKind.@"(index)*2^scale+displacement", MEMBAddressComputationKind.@"(abase)+(index)*2^scale+displacement", MEMBAddressComputationKind.displacement, MEMBAddressComputationKind.@"(IP)+displacement+8" => true,
            else => false,
        };
    }
    pub fn usesScaleField(self: MEMBAddressComputationKind) bool {
        return switch (self) {
            MEMBAddressComputationKind.@"(abase)+(index)*2^scale", MEMBAddressComputationKind.@"(index)*2^scale+displacement", MEMBAddressComputationKind.@"(abase)+(index)*2^scale+displacement" => true,
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
    index: Operand,
    unused: u2,
    scale: u3,
    mode: MEMBAddressComputationKind,
    abase: Operand,
    srcDest: Operand,
    opcode: u8,

    pub fn getOpcode(self: *const MEMBInstruction) !DecodedOpcode {
        return meta.intToEnum(DecodedOpcode, self.opcode) catch error.IllegalOpcode;
    }
    pub fn usesOptionalDisplacement(self: *const MEMBInstruction) bool {
        return self.mode.usesOptionalDisplacement();
    }
};

const InstructionDeterminant = packed struct {
    unused0: u12,
    memDeterminant: u1,
    unused1: u11,
    opcode: u8,

    pub fn isCTRLInstruction(self: *const InstructionDeterminant) bool {
        return opcodes960.determineInstructionClass(self.opcode) == InstructionClass.CTRL;
    }
    pub fn isCOBRInstruction(self: *const InstructionDeterminant) bool {
        return opcodes960.determineInstructionClass(self.opcode) == InstructionClass.COBR;
    }
    pub fn isREGInstruction(self: *const InstructionDeterminant) bool {
        return opcodes960.determineInstructionClass(self.opcode) == InstructionClass.REG;
    }
    pub fn isMEMInstruction(self: *const InstructionDeterminant) bool {
        return opcodes960.determineInstructionClass(self.opcode) == InstructionClass.MEM;
    }
    pub fn isMEMAInstruction(self: *const InstructionDeterminant) bool {
        return self.isMEMInstruction() and (self.memDeterminant == 0);
    }
    pub fn isMEMBInstruction(self: *const InstructionDeterminant) bool {
        return self.isMEMInstruction() and (self.memDeterminant == 1);
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
            .mema => |k| k.src1,
            .memb => |k| k.src1,
            .cobr => |k| k.src1,
            else => error.NoArgument,
        };
    }
    pub fn getSrc2(self: *const Instruction) !Operand {
        return switch (self.*) {
            .reg => |k| k.src2,
            .mema => |k| k.src2,
            .memb => |k| k.src2,
            .cobr => |k| k.src2,
            else => error.NoArgument,
        };
    }
};

fn decode(opcode: Ordinal) Instruction {
    const determinant: *const InstructionDeterminant = @ptrCast(&opcode);
    return switch (opcodes960.determineInstructionClass(determinant.opcode)) {
        InstructionClass.CTRL => Instruction{
            .ctrl = @as(*const CTRLInstruction, @ptrCast(&opcode)).*,
        },
        InstructionClass.COBR => Instruction{
            .cobr = @as(*const COBRInstruction, @ptrCast(&opcode)).*,
        },
        InstructionClass.REG => Instruction{
            .reg = @as(*const REGInstruction, @ptrCast(&opcode)).*,
        },
        InstructionClass.MEM => if (determinant.memDeterminant == 1) Instruction{
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
        try expect(err == error.IllegalOpcode);
        return;
    }) == 0x582);
    try expect(x.getOpcode() catch |err| {
        try expect(err == error.IllegalOpcode);
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
        try expect(err == error.IllegalOpcode);
        return;
    }) == 0x08);
    try expect(x.getOpcode() catch |err| {
        try expect(err == error.IllegalOpcode);
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
        try expect(err == error.IllegalOpcode);
        return;
    }) == 0x08);
    try expect(x.getOpcode() catch |err| {
        try expect(err == error.IllegalOpcode);
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
    //const value: u32 = @bitCast(x);
    //std.debug.print("{x}\n", .{value});
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

test "Opcodes Sanity Checks" {
    try expect(@intFromEnum(DecodedOpcode.cmpibo) == 0x3f);
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

test "FaultRecord sanity check" {
    try expect(@sizeOf(FaultRecord) == 48);
}

test "sizeof sanity check" {
    try expect(@sizeOf(ArithmeticControls) == 4);
    try expect(@sizeOf(ProcessControls) == 4);
    try expect(@sizeOf(TraceControls) == 4);
    try expect_eq(@sizeOf(FaultTable), 256);
}
