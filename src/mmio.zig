const raspi = @import("main.zig").raspi;

const base_mmio = @intToPtr([*]volatile u32, switch (raspi) {
    .raspi3 => 0x3F000000,
});

// The base address for UART.
pub const uart0_base = gpio_base + 0x1000 / 4; // for raspi4 0xFE201000, raspi2 & 3 0x3F201000, and 0x20201000 for raspi1

// The offsets for reach register for the UART.
pub const uart0_dr = @ptrCast(*volatile u32, uart0_base + 0x00 / 4);
pub const uart0_rsrecr = @ptrCast(*volatile u32, uart0_base + 0x04 / 4);
pub const uart0_fr = @ptrCast(*volatile u32, uart0_base + 0x18 / 4);
pub const uart0_ilpr = @ptrCast(*volatile u32, uart0_base + 0x20 / 4);
pub const uart0_ibrd = @ptrCast(*volatile u32, uart0_base + 0x24 / 4);
pub const uart0_fbrd = @ptrCast(*volatile u32, uart0_base + 0x28 / 4);
pub const uart0_lcrh = @ptrCast(*volatile u32, uart0_base + 0x2C / 4);
pub const uart0_cr = @ptrCast(*volatile u32, uart0_base + 0x30 / 4);
pub const uart0_ifls = @ptrCast(*volatile u32, uart0_base + 0x34 / 4);
pub const uart0_imsc = @ptrCast(*volatile u32, uart0_base + 0x38 / 4);
pub const uart0_ris = @ptrCast(*volatile u32, uart0_base + 0x3C / 4);
pub const uart0_mis = @ptrCast(*volatile u32, uart0_base + 0x40 / 4);
pub const uart0_icr = @ptrCast(*volatile u32, uart0_base + 0x44 / 4);
pub const uart0_dmacr = @ptrCast(*volatile u32, uart0_base + 0x48 / 4);
pub const uart0_itcr = @ptrCast(*volatile u32, uart0_base + 0x80 / 4);
pub const uart0_itip = @ptrCast(*volatile u32, uart0_base + 0x84 / 4);
pub const uart0_itop = @ptrCast(*volatile u32, uart0_base + 0x88 / 4);
pub const uart0_tdr = @ptrCast(*volatile u32, uart0_base + 0x8C / 4);

// The offsets for Mailbox registers
pub const mbox_base = base_mmio + 0xB880 / 4;
pub const mbox_read = @ptrCast(*volatile u32, mbox_base + 0x00 / 4);
pub const mbox_status = @ptrCast(*volatile u32, mbox_base + 0x18 / 4);
pub const mbox_write = @ptrCast(*volatile u32, mbox_base + 0x20 / 4);

// Random numbers
pub const rng_base = base_mmio + 0x104000 / 4;
pub const rng_ctrl = @ptrCast(*volatile u32, gpio_base + 0x00 / 4);
pub const rng_status = @ptrCast(*volatile u32, gpio_base + 0x04 / 4);
pub const rng_data = @ptrCast(*volatile u32, gpio_base + 0x08 / 4);
pub const rng_int_mask = @ptrCast(*volatile u32, gpio_base + 0x10 / 4);

// Power management
pub const pm_base = base_mmio + 0x100000 / 4;
pub const pm_rstc = @ptrCast(*volatile u32, pm_base + 0x1c / 4);
pub const pm_rsts = @ptrCast(*volatile u32, pm_base + 0x20 / 4);
pub const pm_wdog = @ptrCast(*volatile u32, pm_base + 0x24 / 4);

// The offsets for reach register.
pub const gpio_base = base_mmio + 0x200000 / 4;
pub const gpfsel0 = @ptrCast(*volatile u32, gpio_base + 0x0 / 4);
pub const gpfsel1 = @ptrCast(*volatile u32, gpio_base + 0x4 / 4);
pub const gpfsel2 = @ptrCast(*volatile u32, gpio_base + 0x8 / 4);
pub const gpfsel3 = @ptrCast(*volatile u32, gpio_base + 0xC / 4);
pub const gpfsel4 = @ptrCast(*volatile u32, gpio_base + 0x10 / 4);
pub const gpfsel5 = @ptrCast(*volatile u32, gpio_base + 0x14 / 4);
pub const gpset0 = @ptrCast(*volatile u32, gpio_base + 0x1C / 4);
pub const gpset1 = @ptrCast(*volatile u32, gpio_base + 0x20 / 4);
pub const gpclr0 = @ptrCast(*volatile u32, gpio_base + 0x28 / 4);
pub const gplev0 = @ptrCast(*volatile u32, gpio_base + 0x34 / 4);
pub const gplev1 = @ptrCast(*volatile u32, gpio_base + 0x38 / 4);
pub const gpeds0 = @ptrCast(*volatile u32, gpio_base + 0x40 / 4);
pub const gpeds1 = @ptrCast(*volatile u32, gpio_base + 0x44 / 4);
pub const gphen0 = @ptrCast(*volatile u32, gpio_base + 0x64 / 4);
pub const gphen1 = @ptrCast(*volatile u32, gpio_base + 0x68 / 4);
/// Controls actuation of pull up/down to ALL GPIO pins.
pub const gppud = @ptrCast(*volatile u32, gpio_base + 0x94 / 4);
/// Controls actuation of pull up/down for specific GPIO pin.
pub const gppudclk0 = @ptrCast(*volatile u32, gpio_base + 0x98 / 4);
pub const gppudclk1 = @ptrCast(*volatile u32, gpio_base + 0x9C / 4);
