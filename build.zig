const std = @import("std");

const Model = struct {
    target: std.zig.CrossTarget,
    machine: []const u8,
};

pub fn build(b: *std.build.Builder) void {
    const mode = b.standardReleaseOptions();

    const exe = b.addExecutable("kernel8.img", "src/main.zig");

    const model: Model = .{
        .target = std.zig.CrossTarget.parse(.{ .arch_os_abi = "aarch64-freestanding", .cpu_features = "cortex_a53-neon" }) catch unreachable,
        .machine = "raspi3",
    };

    // zig build-exe -target aarch64-freestanding src/boot.s src/main.zig --script src/linkerscript.ld -femit-bin=zig-cache/bin/app
    // qemu-system-aarch64 -M raspi3 -serial stdio -kernel zig-cache/bin/kernel8.img

    exe.addAssemblyFile("src/boot.s");
    exe.addCSourceFile("src/deps/sd.c", &.{});

    exe.setTarget(model.target);
    exe.setBuildMode(mode);
    exe.setLinkerScriptPath("src/linkerscript.ld");
    exe.install();

    const fs_cmd = b.addSystemCommand(&.{ "sh", "tools/genfs.sh" });
    fs_cmd.step.dependOn(&exe.install_step.?.step);

    const qemu_path: []const u8 = b.option([]const u8, "qemu", "qemu-system-aarch64") orelse "qemu-system-aarch64";
    const run_cmd = b.addSystemCommand(&.{
        qemu_path,
        "-M",
        model.machine,
        "-serial",
        "stdio",
        "-drive",
        "file=fs/images/sdcard.img,if=sd,format=raw",
        "-kernel",
        b.getInstallPath(exe.install_step.?.dest_dir, exe.out_filename),
        // -s -S to enable localhost:1234 gdb debugging
    });
    run_cmd.step.dependOn(&exe.install_step.?.step);
    run_cmd.step.dependOn(&fs_cmd.step);
    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);
}
