// doesn't work
// TODO: try the c version and see what's wrong with mine

const mmio = @import("mmio.zig");
const mbox = @import("mbox.zig");
const range = @import("main.zig").range;
const delay = @import("main.zig").delay;

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
