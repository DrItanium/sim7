const std = @import("std");

pub fn build(b: *std.Build) void {
    const exe = b.addExecutable(.{
        .name = "sim960",
        .root_source_file = .{ .path = "src/main.zig" },
        .target = b.starndardTargetOptions(.{}),
        .optimize = b.standardOptimizeOptions(.{}),
    });
    b.installArtifact(exe);
}
