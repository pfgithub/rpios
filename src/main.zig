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
    const mbox_ptr: *align(0b1000) volatile [36]u32 = &(struct {
        var mbox_raw: [36]u32 align(0b1000) = [_]u32{0} ** 36;
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

    /// https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface#set-clock-rate
    pub fn call(channel: MboxChannel, args: []const u32) bool {
        mbox_ptr[0] = @intCast(u32, (args.len + 1) * 4);
        for (args) |arg, i| mbox_ptr[i + 1] = arg;

        if (@ptrToInt(mbox_ptr) & ~@as(usize, 0b111) != @ptrToInt(mbox_ptr)) @panic("misaligned mbox pointer. must be 0b1000 aligned");
        const r = @intCast(u32, @ptrToInt(mbox_ptr)) | @enumToInt(channel); // if it's align(16), what's the point of & ~0xF?

        // wait until we can talk to the VC
        while (mmio.mbox_status.* & MBOX_FULL != 0) spinHint();
        // send our message to property channel and wait for the response
        mmio.mbox_write.* = r;

        while (true) {
            // wait for response
            while (mmio.mbox_status.* & MBOX_EMPTY != 0) spinHint();

            // if it is ours
            if (mmio.mbox_read.* == r) return mbox_ptr[1] == MBOX_RESPONSE;
        }
    }
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
                if (!mbox.call(.ch_prop, &[_]u32{
                    0, // MBOX_REQUEST
                    0x38002, // MBOX_TAG_SETCLKRATE
                    12,
                    8,
                    2, // UART clock
                    3000000, // 3Mhz
                    0, // clear turbo
                    0, // MBOX_TAG_LAST
                })) {
                    @panic("failed to set something");
                }
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
};

// // arguments for AArch64
// void kernel_main(uint64_t dtb_ptr32, uint64_t x1, uint64_t x2, uint64_t x3)
// #else
// // arguments for AArch32
// void kernel_main(uint32_t r0, uint32_t r1, uint32_t atags)
export fn zigMain(dtb_ptr32: u64, x1: u64, x2: u64, x3: u64) noreturn {
    uart.init();
    uart.puts("Hello, kernel World!\r\n");
    while (true) {
        switch (uart.getc()) {
            'h' => uart.puts("Help menu:\r\n- [h]elp\r\n"),
            else => |c| uart.putc(c),
        }
    }
}
