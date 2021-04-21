const std = @import("std");

pub const raspi: enum {
    raspi3,
} = .raspi3;

pub fn spinHint() void {
    asm volatile ("YIELD");
}

pub fn delay(count: u32) void {
    for (range(count)) |_| asm volatile ("NOP");
}

pub fn range(max: usize) []const void {
    return @as([]const void, &[_]void{}).ptr[0..max];
}

const mmio = @import("mmio.zig");
const mbox = @import("mbox.zig");
const uart = @import("uart.zig");
const power = @import("power.zig");
const framebuffer = @import("framebuffer.zig");
const sd = @import("sd.zig");
const boot_fs = @import("boot_fs.zig");

var log_location: union(enum) {
    discard,
    uart,
} = .discard;

pub fn log(
    comptime message_level: std.log.Level,
    comptime scope: @Type(.EnumLiteral),
    comptime format: []const u8,
    args: anytype,
) void {
    switch (log_location) {
        .discard => {},
        .uart => {
            const level_txt = switch (message_level) {
                .emerg => "emergency",
                .alert => "alert",
                .crit => "critical",
                .err => "error",
                .warn => "warning",
                .notice => "notice",
                .info => "info",
                .debug => "debug",
            };
            const prefix2 = if (scope == .default) ": " else "(" ++ @tagName(scope) ++ "): ";
            // const stderr = std.io.getStdErr().writer();
            // const held = std.debug.getStderrMutex().acquire();
            // defer held.release();
            uart.writer().print(level_txt ++ prefix2 ++ format ++ "\n", args) catch return;
        },
    }
}
// alternatively:
//     root.os.io.getStdErrHandle
//     that returns a root.os.bits.fd_t
// nah I think log is a better choice

// // arguments for AArch64
// void kernel_main(uint64_t dtb_ptr32, uint64_t x1, uint64_t x2, uint64_t x3)
// #else
// // arguments for AArch32
// void kernel_main(uint32_t r0, uint32_t r1, uint32_t atags)

fn hexdump(data: []const u8) void {
    const out = uart.writer();
    var i: usize = 0;
    while (i < data.len) {
        out.print("0x{x:0>8}: ", .{i}) catch unreachable;
        for (range(16)) |_, offset| {
            if (i + offset < data.len) {
                out.print("{x:0>2} ", .{data[i + offset]}) catch unreachable;
            } else {
                out.print(".. ", .{}) catch unreachable;
            }
            if (offset % 4 == 3) uart.putc(' ');
        }
        for (range(16)) |_, offset| {
            if (i + offset >= data.len) break;
            const char = data[i + offset];

            if (std.ascii.isPrint(char)) uart.putc(char) //
            else uart.puts("·");
        }
        i += 16;
        uart.putc('\n');
    }
}

extern var _end: opaque {};
var root_allocator_fba: std.heap.FixedBufferAllocator = undefined;
var root_allocator: *std.mem.Allocator = undefined;

/// bitextract(0bFEDCBA, 2, u3) → CDE
///
/// bitextract(value: u{N}, start: Log2Int(u{N}), Size: type(u{N})) Size
fn bitextract(value: anytype, comptime start: comptime_int, comptime Size: type) Size {
    comptime const len = std.meta.bitCount(Size);
    return @intCast(Size, (value >> start) & comptime (0b1 << len) - 1);
}

test "bitextract" {
    std.testing.expectEqual(bitextract(0b000011100, 2, u3), 0b111);
    std.testing.expectEqual(bitextract(0b000011100, 3, u3), 0b011);
    std.testing.expectEqual(bitextract(0b000011100, 1, u3), 0b110);
    std.testing.expectEqual(bitextract(0b000011100, 0, u3), 0b100);
}

fn userCode() noreturn {
    for ("Hello, World!") |char| _ = asm volatile ("svc #0"
        : [ret] "={x0}" (-> usize)
        : [number] "{x8}" (0),
          [arg1] "{x0}" (char)
        : "memory", "cc"
    );
    _ = asm volatile ("svc #0"
        : [ret] "={x0}" (-> usize)
        : [number] "{x8}" (1)
    );
    unreachable;
}

fn callUserCode() void {
    // set up page table
    // also stack I guess for now
    // eret to el0 (&userCode)
    // on exception: switch(exception)
    std.log.info("not implemented", .{});
}

fn main() !void {
    uart.init();
    log_location = .uart;

    uart.puts("Starting…\n");
    std.log.info("It works! This is the default web page for this web server!", .{});

    try framebuffer.init();
    framebuffer.draw_image();

    hexdump("Testing hexdump. It appears to work, great!");

    {
        var b = mbox.Builder{};
        const memory_tag = b.add(mbox.get_arm_memory, {});
        const vcmem_tag = b.add(mbox.get_vc_memory, {});
        try b.exec();
        const memory = try memory_tag.get();
        const vcmem = try vcmem_tag.get();
        std.log.info("Available memory: {*}[0..0x{X}]", .{ memory.ptr, memory.len });
        std.log.info("VC memory: {*}[0..0x{X}]", .{ vcmem.ptr, vcmem.len });

        root_allocator_fba = blk: {
            const end_of_kernel = @ptrCast([*]u8, &_end);
            const total_mem = @ptrToInt(memory.ptr) + memory.len - @ptrToInt(end_of_kernel);
            break :blk std.heap.FixedBufferAllocator.init(end_of_kernel[0..memory.len]);
        };
        root_allocator = &root_allocator_fba.allocator;
    }

    try sd.init();
    const read_block = try root_allocator.alloc([512]u8, 1);
    try sd.readblock(0, read_block);

    hexdump(@ptrCast([*]const u8, read_block.ptr)[0 .. read_block.len * 512]);

    const root_partition = boot_fs.getPartition(root_allocator);
    std.log.info("partition: {}", .{root_partition});

    while (true) {
        switch (uart.getc()) {
            'h' => uart.puts("Help menu:\n- [h]elp\n- [r]estart\n- [s]hutdown\n- [p]anic\n- [u]ser code\n"),
            's' => {
                std.log.info("Shutting down…", .{});
                power.power_off();
                std.log.info("uuh?", .{});
            },
            'r' => {
                std.log.info("Resetting…", .{});
                power.reset();
                std.log.info("uuh?", .{});
            },
            'p' => {
                @panic("crash");
            },
            'u' => {
                callUserCode();
            },
            else => |c| uart.putc(c),
        }
    }
}

export fn zigMain(dtb_ptr32: u64, x1: u64, x2: u64, x3: u64) noreturn {
    const current_el = bitextract(asm volatile ("mrs %[ret], CurrentEL"
        : [ret] "=r" (-> usize)
    ), 2, u2); // bits[2..4]
    // bss will be cleared multiple times because it's done before
    // this code runs. TODO clear bss in zig.
    switch (current_el) {
        0b11 => {
            uart.init();
            uart.puts("hello! I'm in l3?\n");
            @panic("oops");
        },
        0b10 => {},
        0b01 => {},
        0b00 => {},
    }
    uart.init();
    uart.puts("hello! I'm a different level\n");

    main() catch |e| {
        std.log.err("Main error: {}", .{e});
        @panic("Main exited with error.");
    };
    @panic("Main exited without error.");
}

pub fn panic(message: []const u8, trace: ?*std.builtin.StackTrace) noreturn {
    @setCold(true);
    std.log.emerg("Panicked! {s}", .{message});
    @import("panic.zig").panicLog(trace);
    @breakpoint();
    while (true) asm volatile ("wfi");
}

// var debug_info_allocator: ?*mem.Allocator = null;
// var debug_info_arena_allocator: std.heap.ArenaAllocator = undefined;
// fn getDebugInfoAllocator() *mem.Allocator {
//     if (debug_info_allocator) |a| return a;

//     debug_info_arena_allocator = std.heap.ArenaAllocator.init(root_allocator);
//     debug_info_allocator = &debug_info_arena_allocator.allocator;
//     return &debug_info_arena_allocator.allocator;
// }
// // copied from std.debug.dumpStackTrace because it does not let you pass a custom writer
// pub fn dumpStackTrace(stack_trace: std.builtin.StackTrace, stderr: anytype) void {
//     if (std.builtin.strip_debug_info) {
//         stderr.print("Unable to dump stack trace: debug info stripped\n", .{}) catch return;
//         return;
//     }
//     const debug_info = std.debug.getSelfDebugInfo() catch |err| {
//         stderr.print("Unable to dump stack trace: Unable to open debug info: {s}\n", .{@errorName(err)}) catch return;
//         return;
//     };
//     std.debug.writeStackTrace(stack_trace, stderr, getDebugInfoAllocator(), debug_info, detectTTYConfig()) catch |err| {
//         stderr.print("Unable to dump stack trace: {s}\n", .{@errorName(err)}) catch return;
//         return;
//     };
// }
// pub fn dumpCurrentStackTrace(start_addr: ?usize, stderr: anytype) void {
//     if (std.builtin.strip_debug_info) {
//         stderr.print("Unable to dump stack trace: debug info stripped\n", .{}) catch return;
//         return;
//     }
//     const debug_info = std.debug.getSelfDebugInfo() catch |err| {
//         stderr.print("Unable to dump stack trace: Unable to open debug info: {s}\n", .{@errorName(err)}) catch return;
//         return;
//     };
//     std.debug.writeCurrentStackTrace(stderr, debug_info, std.debug.detectTTYConfig(), start_addr) catch |err| {
//         stderr.print("Unable to dump stack trace: {s}\n", .{@errorName(err)}) catch return;
//         return;
//     };
// }
