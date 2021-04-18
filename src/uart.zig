const std = @import("std");
const mmio = @import("mmio.zig");
const mbox = @import("mbox.zig");
const delay = @import("main.zig").delay;
const raspi = @import("main.zig").raspi;
const spinHint = @import("main.zig").spinHint;

pub fn init() void {
    // Disable UART0.
    mmio.uart0_cr.* = 0x00000000;
    // Setup the GPIO pin 14 && 15.

    // Disable pull up/down for all GPIO pins
    // Disable pull up/down for pin 14,15
    // Write
    mmio.gppud.* = 0x00000000;
    delay(150);
    mmio.gppudclk0.* = (1 << 14) | (1 << 15);
    delay(150);
    mmio.gppudclk0.* = 0x00000000;

    // Clear pending interrupts.
    mmio.uart0_icr.* = 0x7FF;

    // Set integer & fractional part of baud rate.
    // Divider = UART_CLOCK/(16 * Baud)
    // Fraction part register = (Fractional part * 64) + 0.5
    // Baud = 115200.

    // For Raspi3 and 4 the UART_CLOCK is system-clock dependent by default.
    // Set it to 3Mhz so that we can consistently set the baud rate
    switch (raspi) {
        .raspi3 => {
            var b = mbox.Builder{};
            _ = b.add(mbox.set_clock_rate, .{
                .clock_id = .uart, // UART clock
                .rate_hz = 3000000, // 3Mhz
                .skip_setting_turbo = false, // clear turbo
            });
            b.exec() catch {
                const out = writer();
                for (b.message[0..b.index]) |value, i| {
                    out.print("msg[{}] = {} (0x{x})", .{ i, value, value }) catch unreachable;
                }
                out.print("ptr is {*}", .{mbox.mbox_ptr}) catch unreachable;
                @panic("failed to set clock rate");
            };
        },
    }

    // Divider = 3000000 / (16 * 115200) = 1.627 = ~1.
    mmio.uart0_ibrd.* = 1;
    // Fractional part register = (.627 * 64) + 0.5 = 40.6 = ~40.
    mmio.uart0_fbrd.* = 40;

    // Enable FIFO & 8 bit data transmission (1 stop bit, no parity).
    mmio.uart0_lcrh.* = (1 << 4) | (1 << 5) | (1 << 6);

    // Mask all interrupts.
    mmio.uart0_imsc.* = (1 << 1) | (1 << 4) | (1 << 5) | (1 << 6) |
        (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10) //
    ;

    // Enable UART0, receive & transfer part of UART.
    mmio.uart0_cr.* = (1 << 0) | (1 << 8) | (1 << 9);
}
pub fn putc(c: u8) void {
    // Wait for UART to become ready to transmit.
    while (mmio.uart0_fr.* & (1 << 5) != 0) spinHint();
    mmio.uart0_dr.* = c;
}
pub fn getc() u8 {
    // Wait for UART to have received something.
    while (mmio.uart0_fr.* & (1 << 4) != 0) spinHint();
    return @intCast(u8, mmio.uart0_dr.*);
}
pub fn puts(str: []const u8) void {
    for (str) |char| putc(char);
}
const WriteError = error{Never};
const Writer = std.io.Writer(void, WriteError, write);
fn write(self: void, bytes: []const u8) WriteError!usize {
    puts(bytes);
    return bytes.len;
}
pub fn writer() Writer {
    return .{ .context = {} };
}
