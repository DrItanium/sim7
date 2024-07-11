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

test "overflow test" {
    const a: i32 = 0x7FFF_FFFF;
    const b: i32 = 1;
    try expect((a +% b) < 0);
    const c: u32 = 0xFFFF_FFFF;
    const d: u32 = 1;
    const e = math.add(u32, c, d) catch 33;
    try expect(e == 33);
}
test "allocation test" {
    const allocator = std.heap.page_allocator;
    const buffer = try allocator.alloc(u8, 4);
    defer allocator.free(buffer);
    try expect(buffer.len == 4);
    for (buffer, 0..) |*val, ind| {
        val.* = @truncate(ind);
    }
    for (buffer, 0..) |val, ind| {
        try expectEqual(val, ind);
    }
}
test "allocation test2" {
    const allocator = std.heap.page_allocator;
    const buffer = try allocator.alloc(u32, 4);
    defer allocator.free(buffer);
    try expect(buffer.len == 4);
    for (buffer, 0..) |*val, ind| {
        val.* = @truncate(ind + 0x03020100);
    }
    for (buffer, 0..) |val, ind| {
        try expectEqual(val, (ind + 0x03020100));
    }
    // okay, so now we need to see if we can view the various components
    //
    // it is trivial to go from larger to smaller!
    const buffer2: *const [4]u8 = @ptrCast(&buffer[0]);
    try expectEqual(buffer2[0], 0x00);
    try expectEqual(buffer2[1], 0x01);
    try expectEqual(buffer2[2], 0x02);
    try expectEqual(buffer2[3], 0x03);
}
test "add/subtract index test" {
    // this is a sanity check to make sure that my understanding of zig
    // overflow semantics is correct
    var ind: u2 = 0;
    ind = ind -% 1;
    try expectEqual(ind, 0b11);
    ind = ind -% 1;
    try expectEqual(ind, 0b10);
    ind = ind -% 1;
    try expectEqual(ind, 0b01);
    ind = ind -% 1;
    try expectEqual(ind, 0b00);
    ind = ind +% 1;
    try expectEqual(ind, 0b01);
    ind = ind +% 1;
    try expectEqual(ind, 0b10);
    ind = ind +% 1;
    try expectEqual(ind, 0b11);
    ind = ind +% 1;
    try expectEqual(ind, 0b00);
}
