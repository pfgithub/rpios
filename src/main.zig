const std = @import("std");

const raspi: enum {
    raspi3,
} = .raspi3;

pub fn spinHint() void {
    asm volatile ("YIELD");
}

fn delay(count: u32) void {
    // asm volatile ("__delay_%=: subs %[count], %[count], #1; bne __delay_%=\n"
    //     : [ret] "=r" (count)
    //     : [count] "0" (count)
    //     : "cc"
    // );
    for (range(count)) |_| spinHint(); // should be good enough
}

const mmio = struct {
    const base_mmio = @intToPtr([*]volatile u32, switch (raspi) {
        .raspi3 => 0x3F000000,
    });

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
    const mbox_base = base_mmio + 0xB880 / 4;
    const mbox_read = @ptrCast(*volatile u32, mbox_base + 0x00 / 4);
    const mbox_status = @ptrCast(*volatile u32, mbox_base + 0x18 / 4);
    const mbox_write = @ptrCast(*volatile u32, mbox_base + 0x20 / 4);

    // Random numbers
    const rng_base = base_mmio + 0x104000 / 4;
    const rng_ctrl = @ptrCast(*volatile u32, gpio_base + 0x00 / 4);
    const rng_status = @ptrCast(*volatile u32, gpio_base + 0x04 / 4);
    const rng_data = @ptrCast(*volatile u32, gpio_base + 0x08 / 4);
    const rng_int_mask = @ptrCast(*volatile u32, gpio_base + 0x10 / 4);

    // Power management
    const pm_base = base_mmio + 0x100000 / 4;
    const pm_rstc = @ptrCast(*volatile u32, pm_base + 0x1c / 4);
    const pm_rsts = @ptrCast(*volatile u32, pm_base + 0x20 / 4);
    const pm_wdog = @ptrCast(*volatile u32, pm_base + 0x24 / 4);

    // The offsets for reach register.
    const gpio_base = base_mmio + 0x200000 / 4;
    const gpfsel0 = @ptrCast(*volatile u32, gpio_base + 0x0 / 4);
    const gpfsel1 = @ptrCast(*volatile u32, gpio_base + 0x4 / 4);
    const gpfsel2 = @ptrCast(*volatile u32, gpio_base + 0x8 / 4);
    const gpfsel3 = @ptrCast(*volatile u32, gpio_base + 0xC / 4);
    const gpfsel4 = @ptrCast(*volatile u32, gpio_base + 0x10 / 4);
    const gpfsel5 = @ptrCast(*volatile u32, gpio_base + 0x14 / 4);
    const gpset0 = @ptrCast(*volatile u32, gpio_base + 0x1C / 4);
    const gpset1 = @ptrCast(*volatile u32, gpio_base + 0x20 / 4);
    const gpclr0 = @ptrCast(*volatile u32, gpio_base + 0x28 / 4);
    const gplev0 = @ptrCast(*volatile u32, gpio_base + 0x34 / 4);
    const gplev1 = @ptrCast(*volatile u32, gpio_base + 0x38 / 4);
    const gpeds0 = @ptrCast(*volatile u32, gpio_base + 0x40 / 4);
    const gpeds1 = @ptrCast(*volatile u32, gpio_base + 0x44 / 4);
    const gphen0 = @ptrCast(*volatile u32, gpio_base + 0x64 / 4);
    const gphen1 = @ptrCast(*volatile u32, gpio_base + 0x68 / 4);
    // Controls actuation of pull up/down to ALL GPIO pins.
    const gppud = @ptrCast(*volatile u32, gpio_base + 0x94 / 4);
    // Controls actuation of pull up/down for specific GPIO pin.
    const gppudclk0 = @ptrCast(*volatile u32, gpio_base + 0x98 / 4);
    const gppudclk1 = @ptrCast(*volatile u32, gpio_base + 0x9C / 4);
};

const mbox = struct {
    const MBOX_MAX_SIZE = 36;
    const mbox_ptr: *align(0b1000) volatile [MBOX_MAX_SIZE]u32 = &(struct {
        var mbox_raw: [MBOX_MAX_SIZE]u32 align(0b10000) = [_]u32{undefined} ** MBOX_MAX_SIZE;
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
        message: [MBOX_MAX_SIZE]u32 = [_]u32{ 0xAAAAAAAA, 0 } ++ [_]u32{0xAAAAAAAA} ** (MBOX_MAX_SIZE - 2),
        index: u32 = 2,

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
                data: *const [ArrayLen(Arg0(@TypeOf(Tag.unpack))) + 1]u32,
                pub fn get(this: @This()) !ReturnType(@TypeOf(Tag.unpack)) {
                    if (this.data[0] & (0b1 << 31) == 0) return error.ResponseNotSet;
                    return Tag.unpack(this.data[1..].*);
                }
            };
        }
        fn appendSlice(b: *Builder, arr: []const u32) void {
            std.mem.copy(u32, b.message[b.index..], arr);
            b.index += @intCast(u32, arr.len);
        }
        /// request: std.meta.ArgsTuple(@TypeOf(Tag.pack))
        pub fn add(b: *Builder, comptime Tag: type, request: Arg0(@TypeOf(Tag.pack))) LaterResult(Tag) {
            const req_data = Tag.pack(request);
            comptime const req_len = ArrayLen(ReturnType(@TypeOf(Tag.pack)));
            comptime const res_len = ArrayLen(Arg0(@TypeOf(Tag.unpack)));
            comptime const buffer_size = @intCast(u32, std.math.max(req_len, res_len));

            b.appendSlice(&.{ Tag.tag, buffer_size * 4 });
            const res_data = b.message[b.index..][0 .. res_len + 1];
            b.appendSlice(&.{0});
            std.mem.copy(u32, b.message[b.index..], &req_data);
            b.index += buffer_size;

            return .{ .data = res_data };
        }
        pub fn exec(b: *Builder) !void {
            b.appendSlice(&.{0});
            b.message[0] = b.index * 4;

            const channel: MboxChannel = .ch_prop;

            for (b.message[0..b.index]) |v, i| mbox_ptr[i] = v;

            if (@ptrToInt(mbox_ptr) & ~@as(usize, 0b1111) != @ptrToInt(mbox_ptr)) @panic("misaligned mbox pointer. must be 0b10000 aligned");
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
        fn pack(_: void) [0]u32 {
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
            return .{ @enumToInt(request.clock_id), request.rate_hz, @boolToInt(request.skip_setting_turbo) };
        }
        fn unpack(response: [2]u32) Response {
            return .{ .clock_id = @intToEnum(Clock, response[0]), .rate = response[1] };
        }
        const Clock = enum(u32) {
            emmc = 0x000000001,
            uart = 0x000000002,
            arm = 0x000000003,
            core = 0x000000004,
            v3d = 0x000000005,
            h264 = 0x000000006,
            isp = 0x000000007,
            sdram = 0x000000008,
            pixel = 0x000000009,
            pwm = 0x00000000a,
            hevc = 0x00000000b,
            emmc2 = 0x00000000c,
            m2mc = 0x00000000d,
            pixel_bvb = 0x00000000e,
        };
        const Request = struct {
            clock_id: Clock,
            rate_hz: u32,
            skip_setting_turbo: bool,
        };
        const Response = struct {
            clock_id: Clock,
            rate: u32,
        };
    };
    pub const set_power_state = struct {
        const channel: MboxChannel = .ch_prop;
        const tag: u32 = 0x00028001;
        fn pack(request: Request) [2]u32 {
            return .{ request.device_id, (@as(u32, @boolToInt(request.power)) << 1) & @boolToInt(request.wait) };
        }
        fn unpack(response: [2]u32) Response {
            return .{
                .device_id = response[0],
                .power = response[1] & 0b1 != 0,
                .device_exists = response[1] & 0b10 != 0,
            };
        }
        const Request = struct {
            device_id: u32,
            power: bool,
            wait: bool,
        };
        const Response = struct {
            device_id: u32,
            power: bool,
            device_exists: bool,
        };
    };
    /// Set physical (display) width/height
    ///
    /// Note that the "physical (display)" size is the size of the allocated buffer in memory, not
    /// the resolution of the video signal sent to the display device.
    ///
    /// The response may not be the same as the request so it must be checked. May be the previous
    /// width/height or 0 for unsupported.
    pub const set_physical_w_h = struct {
        const channel: MboxChannel = .ch_prop;
        const tag: u32 = 0x00048003;
        fn pack(request: WH) [2]u32 {
            return .{ request.w, request.h };
        }
        fn unpack(response: [2]u32) WH {
            return .{ .w = response[0], .h = response[1] };
        }
        const WH = struct {
            w: u32,
            h: u32,
        };
    };
    /// Set virtual (buffer) width/height
    ///
    /// Note that the "virtual (buffer)" size is the portion of buffer that is sent to the display
    /// device, not the resolution the buffer itself. This may be smaller than the allocated buffer
    /// size in order to implement panning.
    ///
    /// Response is the same as the request (or modified), to indicate if this configuration is
    /// supported (in combination with all the other settings). Does not modify the current
    /// hardware or frame buffer state.
    pub const set_virtual_w_h = struct {
        const channel: MboxChannel = .ch_prop;
        const tag: u32 = 0x00048004;
        fn pack(request: WH) [2]u32 {
            return .{ request.w, request.h };
        }
        fn unpack(response: [2]u32) WH {
            return .{ .w = response[0], .h = response[1] };
        }
        const WH = struct {
            w: u32,
            h: u32,
        };
    };
    /// Set virtual offset
    ///
    /// The response may not be the same as the request so it must be checked. May be the previous
    /// offset or 0 for unsupported.
    pub const set_virtual_offset = struct {
        const channel: MboxChannel = .ch_prop;
        const tag: u32 = 0x00048009;
        fn pack(request: XY) [2]u32 {
            return .{ request.x, request.y };
        }
        fn unpack(response: [2]u32) XY {
            return .{ .x = response[0], .y = response[1] };
        }
        const XY = struct {
            x: u32,
            y: u32,
        };
    };
    /// The response may not be the same as the request so it must be checked. May be the previous
    /// depth or 0 for unsupported.
    pub const set_depth = struct {
        const channel: MboxChannel = .ch_prop;
        const tag: u32 = 0x00048005;
        fn pack(request: Request) [1]u32 {
            return .{request.bits_per_px};
        }
        fn unpack(response: [1]u32) Response {
            return .{ .bits_per_px = response[0] };
        }
        const Request = struct {
            bits_per_px: u32,
        };
        const Response = struct {
            bits_per_px: u32,
        };
    };
    /// The response may not be the same as the request so it must be checked
    pub const set_pixel_order = struct {
        const channel: MboxChannel = .ch_prop;
        const tag: u32 = 0x00048006;
        fn pack(request: Order) [1]u32 {
            return .{@enumToInt(request)};
        }
        fn unpack(response: [1]u32) Order {
            return std.meta.intToEnum(Order, response[0]) catch @panic("bad pixel order response");
        }
        const Order = enum(u1) {
            bgr = 0,
            rgb = 1,
        };
    };
    /// If the requested alignment is unsupported then the current base and size (which may be 0 if
    /// not allocated) is returned and no change occurs.
    pub const allocate_framebuffer = struct {
        const channel: MboxChannel = .ch_prop;
        const tag: u32 = 0x00040001;
        fn pack(request: Request) [1]u32 {
            return .{request.alignment};
        }
        fn unpack(response: [2]u32) Response {
            if (response[0] == 0) return @as([*]u8, undefined)[0..0];
            // & 0x3FFFFFFF? that just cuts off the value if it goes out of memory, why would you do that?
            // better to catch the error it causes
            return @intToPtr([*]u8, response[0])[0..response[1]];
        }
        const Request = struct {
            alignment: u32,
        };
        const Response = []u8;
    };
    pub const get_pitch = struct {
        const channel: MboxChannel = .ch_prop;
        const tag: u32 = 0x00040008;
        fn pack(request: void) [0]u32 {
            return .{};
        }
        fn unpack(response: [1]u32) Response {
            return .{ .pitch = response[0] };
        }
        const Response = struct {
            pitch: u32,
        };
    };
    /// Future formats may specify multiple base+size combinations.
    pub const get_arm_memory = struct {
        const channel: MboxChannel = .ch_prop;
        const tag: u32 = 0x00010005;
        fn pack(request: void) [0]u32 {
            return .{};
        }
        fn unpack(response: [2]u32) []allowzero u8 {
            return @intToPtr([*]allowzero u8, response[0])[0..response[1]];
        }
    };
    /// Future formats may specify multiple base+size combinations.
    pub const get_vc_memory = struct {
        const channel: MboxChannel = .ch_prop;
        const tag: u32 = 0x00010006;
        fn pack(request: void) [0]u32 {
            return .{};
        }
        fn unpack(response: [2]u32) []allowzero u8 {
            return @intToPtr([*]allowzero u8, response[0])[0..response[1]];
        }
    };
};

pub const uart = struct {
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
                    log_location = .uart;
                    for (b.message[0..b.index]) |value, i| {
                        std.log.info("msg[{}] = {} (0x{x})", .{ i, value, value });
                    }
                    std.log.info("ptr is {*}", .{mbox.mbox_ptr});
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
};

pub fn range(max: usize) []const void {
    return @as([]const void, &[_]void{}).ptr[0..max];
}

const power = struct {
    const PM_WDOG_MAGIC: u32 = 0x5a000000;
    const PM_RSTC_FULLRST: u32 = 0x00000020;
    pub fn power_off() void {
        // power off devices one by one
        for (range(16)) |_, r| {
            var b = mbox.Builder{};
            _ = b.add(mbox.set_power_state, .{ .device_id = @intCast(u32, r), .power = false, .wait = false });
            b.exec() catch {};
        }

        // power off gpio pins (but not VCC pins)
        mmio.gpfsel0.* = 0;
        mmio.gpfsel1.* = 0;
        mmio.gpfsel2.* = 0;
        mmio.gpfsel3.* = 0;
        mmio.gpfsel4.* = 0;
        mmio.gpfsel5.* = 0;
        mmio.gppud.* = 0;
        delay(150);
        mmio.gppudclk0.* = 0xffffffff;
        mmio.gppudclk1.* = 0xffffffff;
        delay(150);
        mmio.gppudclk0.* = 0;
        mmio.gppudclk1.* = 0;

        // power off the SoC (GPU + CPU)
        const r: u32 = (mmio.pm_rsts.* & 0x555) | 0x555; // partition 63 used to indicate halt
        mmio.pm_rsts.* = PM_WDOG_MAGIC | r;
        mmio.pm_wdog.* = PM_WDOG_MAGIC | 10;
        mmio.pm_rstc.* = PM_WDOG_MAGIC | PM_RSTC_FULLRST;
    }
    pub fn reset() void {
        const r: u32 = mmio.pm_rsts.* & 0x555;
        // trigger a restart by instructing the GPU to boot from partition 0
        mmio.pm_rsts.* = PM_WDOG_MAGIC | r; // boot from partition 0
        mmio.pm_wdog.* = PM_WDOG_MAGIC | 10;
        mmio.pm_rstc.* = PM_WDOG_MAGIC | PM_RSTC_FULLRST;
    }
};

const framebuffer = struct {
    var width: u32 = undefined;
    var height: u32 = undefined;
    var pitch: u32 = undefined;
    var isrgb: mbox.set_pixel_order.Order = undefined;
    var lfb: []u8 = undefined;
    pub fn init() !void {
        var b = mbox.Builder{};
        const phys_wh_key = b.add(mbox.set_physical_w_h, .{ .w = 1024, .h = 768 });
        const virt_wh_key = b.add(mbox.set_virtual_w_h, .{ .w = 1024, .h = 768 });
        const virt_offset_key = b.add(mbox.set_virtual_offset, .{ .x = 0, .y = 0 });
        const depth_key = b.add(mbox.set_depth, .{ .bits_per_px = 32 });
        const pixel_order_key = b.add(mbox.set_pixel_order, .bgr);
        const fb_key = b.add(mbox.allocate_framebuffer, .{ .alignment = 4096 });
        const bytes_per_line_key = b.add(mbox.get_pitch, {});
        for (b.message) |value, i| {
            std.log.info("msg[{}] = {} (0x{x})", .{ i, value, value });
        }
        try b.exec();
        for (b.message) |value, i| {
            std.log.info("msg[{}] = {} (0x{x})", .{ i, value, value });
        }

        const phys_wh = try phys_wh_key.get();
        const virt_wh = try virt_wh_key.get();
        const virt_offset = try virt_offset_key.get();
        const depth = try depth_key.get();
        if (depth.bits_per_px != 32) return error.UnsupportedDepth;
        const pixel_order = try pixel_order_key.get();
        const fb = try fb_key.get();
        if (fb.len == 0) return error.NoFrameBuffer;
        const bytes_per_line = try bytes_per_line_key.get();

        if (fb.len == 0) return error.PtrLen0;
        std.log.info("Got framebuffer {}x{}:{} {}. {*}[0..{}]", .{
            phys_wh.w,
            phys_wh.h,
            bytes_per_line.pitch,
            pixel_order,
            fb.ptr,
            fb.len,
        });

        lfb = fb;
        isrgb = pixel_order;
        // https://github.com/bztsrc/raspi3-tutorial/blob/master/09_framebuffer/lfb.c
    }
    const fb_image_example = @embedFile("deps/raspbian.rgba");
    pub fn draw_image() void {
        const H = 256;
        const W = 256;
        const C = 4;
        for (range(H)) |_, y| {
            for (range(W)) |_, x| {
                switch (isrgb) {
                    .rgb => {
                        lfb[0 + (x * C) + (y * W * C)] = fb_image_example[0 + (x * C) + (y * W * C)];
                        lfb[2 + (x * C) + (y * W * C)] = fb_image_example[2 + (x * C) + (y * W * C)];
                    },
                    .bgr => {
                        lfb[0 + (x * C) + (y * W * C)] = fb_image_example[2 + (x * C) + (y * W * C)];
                        lfb[2 + (x * C) + (y * W * C)] = fb_image_example[0 + (x * C) + (y * W * C)];
                    },
                }
                lfb[1 + (x * C) + (y * W * C)] = fb_image_example[1 + (x * C) + (y * W * C)];
                lfb[3 + (x * C) + (y * W * C)] = fb_image_example[3 + (x * C) + (y * W * C)];
            }
        }
    }
};

const sd = struct {
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
};

const boot_fs = struct {
    const BiosParameterBlock = packed struct {
        jmp: [3]u8,
        oem: [8]u8,
        bps0: u8,
        bps1: u8,
        spc: u8,
        rsc: u16,
        nf: u8,
        nr0: u8,
        nr1: u8,
        ts16: u16,
        media: u8,
        spf16: u16,
        spt: u16,
        nh: u16,
        hs: u32,
        ts32: u32,
        spf32: u32,
        flg: u32,
        rc: u32,
        vol: [6]u8,
        fst: [8]u8,
        dmy: [20]u8,
        fst2: [8]u8,
    };
    const FatDir = packed struct {
        name: [8]u8,
        ext: [3]u8,
        /// [0]: packed struct {readonly: bool, hidden: bool, system: bool, volume_label: bool, directory: bool, archive: bool}
        attr: [9]u8,
        ch: u16,
        attr2: u32,
        c1: u16,
        size: u32,
    };
    const Partition = struct {
        lba: u32,
    };
    const log_ = std.log.scoped(.boot_fs);
    pub fn getPartition(alloc: *std.mem.Allocator) !Partition {
        const partition: Partition = blk: {
            const mbr_blocks = try root_allocator.alloc([512]u8, 1);
            defer alloc.free(mbr_blocks);
            try sd.readblock(0, mbr_blocks);
            const mbr = mbr_blocks[0];

            // check magic
            if (mbr[0x1FE] != 0x55 or mbr[0x1FF] != 0xAA) return error.BadMBR;

            // check partition type
            // - fat16 lba | fat32 lba
            if (mbr[0x1C2] != 0xE and mbr[0x1C2] != 0xC) return error.BadBootPartition;

            log_.info("disk identifier: {}\n", .{std.mem.readIntLittle(u32, mbr[0x1B8..][0..4])});

            const partition_lba = std.mem.readIntLittle(u32, mbr[0x1C6..][0..4]);
            log_.info("disk starts at {} (0x{X})", .{ partition_lba, partition_lba });

            break :blk Partition{ .lba = partition_lba };
        };
        return partition;
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

extern var _end: opaque {};

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

var root_allocator_fba: std.heap.FixedBufferAllocator = undefined;
var root_allocator: *std.mem.Allocator = undefined;

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
            'h' => uart.puts("Help menu:\n- [h]elp\n- [r]estart\n- [s]hutdown\n- [p]anic\n"),
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
            else => |c| uart.putc(c),
        }
    }
}

export fn zigMain(dtb_ptr32: u64, x1: u64, x2: u64, x3: u64) noreturn {
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
