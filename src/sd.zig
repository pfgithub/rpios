const std = @import("std");
const uart = @import("uart.zig");
const delay = @import("main.zig").delay;

extern fn sd_init() c_int;
extern fn sd_readblock(lba: c_uint, buffer: [*][512]u8, num: c_uint) c_int;
export fn uart_puts(text: [*:0]u8) void {
    uart.puts(std.mem.spanZ(text));
}
export fn uart_hex(value: c_uint) void {
    uart.writer().print("{x:0<16}", .{value}) catch unreachable;
}
export fn wait_cycles(value: c_uint) void {
    delay(value);
}
pub fn init() !void {
    switch (sd_init()) {
        0 => {},
        -1 => return error.SdTimeout,
        -2 => return error.SdError,
        else => unreachable,
    }
}
pub fn readblock(sector: u32, buffer: [][512]u8) !void {
    if (sd_readblock(sector, buffer.ptr, @intCast(c_uint, buffer.len)) == 0) return error.ReadblockError;
}
