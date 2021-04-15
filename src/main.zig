const std = @import("std");

const raspi: enum {
    raspi3,
} = .raspi3;

pub fn spinHint() void {
    asm volatile ("YIELD");
}

fn delay(count: i32) void {
    asm volatile ("__delay_%=: subs %[count], %[count], #1; bne __delay_%=\n"
        : [ret] "=r" (count)
        : [count] "0" (count)
        : "cc"
    );
}

const mmio = struct {
    const base = @intToPtr([*]volatile u32, switch (raspi) {
        .raspi3 => 0x3F000000,
    });

    // The offsets for reach register.
    const gpio_base = base + 0x200000 / 4;

    // Controls actuation of pull up/down to ALL GPIO pins.
    const gppud = @ptrCast(*volatile u32, gpio_base + 0x94 / 4);

    // Controls actuation of pull up/down for specific GPIO pin.
    const gppudclk0 = @ptrCast(*volatile u32, gpio_base + 0x98 / 4);

    // The base address for UART.
    const uart0_base = gpio_base + 0x1000 / 4; // for raspi4 0xFE201000, raspi2 & 3 0x3F201000, and 0x20201000 for raspi1

    // The offsets for reach register for the UART.
    const uart0_dr = @ptrCast(*volatile u32, uart0_base + 0x00 / 4);
    const uart0_rsrecr = @ptrCast(*volatile u32, uart0_base + 0x04 / 4);
    const uart0_fr = @ptrCast(*volatile u32, uart0_base + 0x18 / 4);
    const uart0_ilpr = @ptrCast(*volatile u32, uart0_base + 0x20 / 4);
    const uart0_ibrd = @ptrCast(*volatile u32, uart0_base + 0x24 / 4);
    const uart0_fbrd = @ptrCast(*volatile u32, uart0_base + 0x28 / 4);
    const uart0_lcrh = @ptrCast(*volatile u32, uart0_base + 0x2C / 4);
    const uart0_cr = @ptrCast(*volatile u32, uart0_base + 0x30 / 4);
    const uart0_ifls = @ptrCast(*volatile u32, uart0_base + 0x34 / 4);
    const uart0_imsc = @ptrCast(*volatile u32, uart0_base + 0x38 / 4);
    const uart0_ris = @ptrCast(*volatile u32, uart0_base + 0x3C / 4);
    const uart0_mis = @ptrCast(*volatile u32, uart0_base + 0x40 / 4);
    const uart0_icr = @ptrCast(*volatile u32, uart0_base + 0x44 / 4);
    const uart0_dmacr = @ptrCast(*volatile u32, uart0_base + 0x48 / 4);
    const uart0_itcr = @ptrCast(*volatile u32, uart0_base + 0x80 / 4);
    const uart0_itip = @ptrCast(*volatile u32, uart0_base + 0x84 / 4);
    const uart0_itop = @ptrCast(*volatile u32, uart0_base + 0x88 / 4);
    const uart0_tdr = @ptrCast(*volatile u32, uart0_base + 0x8C / 4);

    // The offsets for Mailbox registers
    const mbox_base = base + 0xB880 / 4;
    const mbox_read = @ptrCast(*volatile u32, mbox_base + 0x00 / 4);
    const mbox_status = @ptrCast(*volatile u32, mbox_base + 0x18 / 4);
    const mbox_write = @ptrCast(*volatile u32, mbox_base + 0x20 / 4);
};

const mbox = struct {
    const MBOX_MAX_SIZE = 36;
    const mbox_ptr: *align(0b1000) volatile [MBOX_MAX_SIZE]u32 = &(struct {
        var mbox_raw: [MBOX_MAX_SIZE]u32 align(0b1000) = [_]u32{undefined} ** MBOX_MAX_SIZE;
    }).mbox_raw;
    const MboxChannel = enum(u4) {
        ch_power = 0,
        ch_fb = 1,
        ch_vuart = 2,
        ch_vchiq = 3,
        ch_leds = 4,
        ch_btns = 5,
        ch_touch = 6,
        ch_count = 7,
        ch_prop = 8,
    };
    const MBOX_FULL: u32 = 0x80000000;
    const MBOX_RESPONSE: u32 = 0x80000000;
    const MBOX_EMPTY: u32 = 0x40000000;

    // b.add(.set_phy_wh, .{.w = 1024, .h = 768})
    // const real_wh = b.add(.set_virt_wh, .{.w = 1024, .h = 768})
    // b.add(.set_virt_offset, .{.x = 0, .y = 0})
    // b.add(.set_depth, 32);
    // const px_order = b.add(.set_pixel_order, .rgb)
    // const fb = b.add(.get_framebuffer, .{.ptr = 4096, .size = 0}) // fb.get() should return the actual value & 0x3FFFFFFF; "convert GPU address to ARM address"
    // const pitch = b.add(.get_pitch, {})
    // try b.exec();
    // real_wh.get()
    // px_order.get()

    const Builder = struct {
        // call : copies msg to mbox_ptr : copies mbox_ptr back to msg
        message: [MBOX_MAX_SIZE]u32 = [_]u32{ undefined, 0 } ++ [_]u32{undefined} ** (MBOX_MAX_SIZE - 2),
        index: u32 = 1,

        fn ArrayLen(comptime Type: type) usize {
            return @typeInfo(Type).Array.len;
        }
        fn Arg0(comptime Type: type) type {
            return @typeInfo(Type).Fn.args[0].arg_type.?;
        }
        fn ReturnType(comptime Type: type) type {
            return @typeInfo(Type).Fn.return_type.?;
        }
        fn LaterResult(comptime Tag: type) type {
            return struct {
                // could support runtime known lengths without too much difficulty
                data: *const [ArrayLen(Arg0(@TypeOf(Tag.unpack)))]u32,
                pub fn get(this: @This()) ReturnType(@TypeOf(Tag.unpack)) {
                    return Tag.unpack(this.data.*);
                }
            };
        }
        fn appendSlice(b: *Builder, arr: []const u32) void {
            std.mem.copy(u32, b.message[b.index..], arr);
            b.index += @intCast(u32, arr.len);
        }
        /// request: std.meta.ArgsTuple(@TypeOf(Tag.pack))
        pub fn add(b: *Builder, comptime Tag: type, request: anytype) LaterResult(Tag) {
            const req_data = @call(.{}, Tag.pack, request);
            comptime const req_len = ArrayLen(ReturnType(@TypeOf(Tag.pack)));
            comptime const res_len = ArrayLen(Arg0(@TypeOf(Tag.unpack)));
            comptime const buffer_size = @intCast(u32, std.math.max(req_len, res_len));

            b.appendSlice(&.{ Tag.tag, buffer_size, 0 });
            std.mem.copy(u32, b.message[b.index..], &req_data);
            const res_data = b.message[b.index..][0..res_len];
            b.index += buffer_size;

            return .{ .data = res_data };
        }
        pub fn exec(b: *Builder) !void {
            b.appendSlice(&.{0});
            b.message[0] = b.index;

            const channel: MboxChannel = .ch_prop;

            for (b.message[0..b.index]) |v, i| mbox_ptr[i] = v;

            if (@ptrToInt(mbox_ptr) & ~@as(usize, 0b111) != @ptrToInt(mbox_ptr)) @panic("misaligned mbox pointer. must be 0b1000 aligned");
            const r = @intCast(u32, @ptrToInt(mbox_ptr)) | @enumToInt(channel);

            // wait until we can talk to the VC
            while (mmio.mbox_status.* & MBOX_FULL != 0) spinHint();
            // send our message to property channel and wait for the response
            mmio.mbox_write.* = r;

            while (true) {
                // wait for response
                while (mmio.mbox_status.* & MBOX_EMPTY != 0) spinHint();

                // if it is ours
                if (mmio.mbox_read.* == r) {
                    for (mbox_ptr[0..b.message.len]) |v, i| b.message[i] = v;
                    if (b.message[1] == MBOX_RESPONSE) return;
                    return error.Failure;
                }
            }
        }
        // fn exec:
        // append(0)
        // index++
        // set message[0] = index
    };

    // const MboxTag = enum(u32) {
    //     get_board_serial: 0x00010004, // () → (serial: u64)
    // };
    pub const get_board_serial = struct {
        const channel: MboxChannel = .ch_prop;
        const tag: u32 = 0x00010004;
        fn pack() [0]u32 {
            return .{};
        }
        fn unpack(response: [2]u32) Response {
            // std.mem.writeIntNative // readIntNative
            return .{ .serial_0 = response[0], .serial_1 = response[1] };
        }
        const Response = struct {
            serial_0: u32,
            serial_1: u32,
            // TODO serial: u64
        };
    };
    pub const set_clock_rate = struct {
        const channel: MboxChannel = .ch_prop;
        const tag: u32 = 0x00038002;
        fn pack(request: Request) [3]u32 {
            return .{ request.clock_id, request.rate_hz, @boolToInt(request.skip_setting_turbo) };
        }
        fn unpack(response: [2]u32) Response {
            return .{ .clock_id = response[0], .rate = response[1] };
        }
        const Request = struct {
            clock_id: u32,
            rate_hz: u32,
            skip_setting_turbo: bool,
        };
        const Response = struct {
            clock_id: u32,
            rate: u32,
        };
    };
    // // callTag(Tag.get_board_serial, .{})
    // // callTag(Tag.set_clock_rate, .{.clock_id = 2, .rate_hz = 3000000, .skip_setting_turbo = 0})
    // pub fn callTag(comptime Tag: type, request: Tag.Request) ?Tag.Response {
    //     const req_value = request.pack();
    //     const resp_w = @typeInfo(@typeInfo(Tag.Response).Fn.args[0].arg_type.?).Array.len;
    //     const total_w = std.math.max(req_value.len, resp_w.len);
    //     var total_data = []u32{0} ** (3 + total_w);

    //     total_data[0] = Tag.tag;
    //     total_data[1] = total_w * 4;
    //     total_data[2] = 0; // I have no idea what this is?
    //     std.mem.copy(u32, &total_data[3..], &req_value);

    //     call();
    // }
};

const uart = struct {
    pub fn init() void {
        // Disable UART0.
        mmio.uart0_cr.* = 0x00000000;
        // Setup the GPIO pin 14 && 15.

        // Disable pull up/down for all GPIO pins & delay for 150 cycles.
        mmio.gppud.* = 0x00000000;
        delay(150);

        // Disable pull up/down for pin 14,15 & delay for 150 cycles.
        mmio.gppudclk0.* = (1 << 14) | (1 << 15);
        delay(150);

        // Write 0 to GPPUDCLK0 to make it take effect.
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
                _ = b.add(mbox.set_clock_rate, .{.{
                    .clock_id = 2, // UART clock
                    .rate_hz = 3000000, // 3Mhz
                    .skip_setting_turbo = false, // clear turbo
                }});
                b.exec() catch @panic("failed to set clock rate");
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
};

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
export fn zigMain(dtb_ptr32: u64, x1: u64, x2: u64, x3: u64) noreturn {
    uart.init();
    uart.puts("Hello, kernel World!\r\n");

    var b = mbox.Builder{};
    const board_serial = b.add(mbox.get_board_serial, .{});

    if (b.exec()) {
        var hex_fmt = [_]u8{undefined} ** 50;
        uart.puts("Serial number: ");
        const serial = board_serial.get();
        uart.puts(std.fmt.bufPrint(&hex_fmt, "{x},{x}", .{ serial.serial_0, serial.serial_1 }) catch @panic("too long"));
        uart.puts("\r\n");
        for (b.message[0..b.index]) |itm, i| {
            if (i != 0) uart.puts(", ");
            uart.puts(std.fmt.bufPrint(&hex_fmt, "{x}", .{itm}) catch @panic("too long"));
        }
        uart.puts("\r\n");
    } else |e| {
        uart.puts("Failed to fetch serial!\r\n");
    }

    log_location = .uart;
    std.log.info("It works! This is the default web page for this web server!", .{});

    while (true) {
        switch (uart.getc()) {
            'h' => uart.puts("Help menu:\r\n- [h]elp\r\n"),
            else => |c| uart.putc(c),
        }
    }
}
