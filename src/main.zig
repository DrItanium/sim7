const std = @import("std");
const expect = std.testing.expect;

const CTRLInstruction = packed struct {
    displacement: i24,
    opcode: u8,

    pub fn getDisplacement(self: *const CTRLInstruction) i24 {
        const bits = @as(u24, @bitCast(self.displacement)) & 0xFFFFFC;
        return @as(i24, @bitCast(bits));
    }
};

const COBRInstruction = packed struct {
    displacement: i13,
    m1: bool,
    src2: u5,
    src1: u5,
    opcode: u8,

    pub fn getDisplacement(self: *const COBRInstruction) i13 {
        const bits = @as(u13, @bitCast(self.displacement)) & 0b1_1111_1111_1100;
        return @as(i13, @bitCast(bits));
    }
    pub fn treatSrc1AsLiteral(self: *const COBRInstruction) bool {
        return self.m1;
    }
};

// test cases
test "instruction size tests" {
    try expect(@sizeOf(CTRLInstruction) == @sizeOf(u32));
    try expect(@sizeOf(COBRInstruction) == @size
}
test "simple ctrl test" {
    const x = CTRLInstruction{
        .displacement = 0xFDEC,
        .opcode = 0x8,
    };
    try expect(x.opcode == 0x8);
    try expect(x.getDisplacement() == 0xFDEC);
}

test "simple ctrl test2" {
    var x: CTRLInstruction = .{
        .displacement = 0,
        .opcode = 0,
    };

    x.displacement = 0xFDEC;
    x.opcode = 0x08;
    try expect(x.opcode == 0x8);
    try expect(x.getDisplacement() == 0xFDEC);
}

pub fn main() void {
    std.debug.print("i960 Simulator\n", .{});
}
