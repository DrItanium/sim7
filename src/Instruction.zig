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

test "simple ctrl test" {
    const x = CTRLInstruction{
        .displacement = 0xFDEC,
        .opcode = 0x8,
    };
    try expect(x.opcode == 0x8);
    try expect(x.getDisplacement() == 0xFDEC);
}
