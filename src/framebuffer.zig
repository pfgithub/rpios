const log = @import("std").log.scoped(.framebuffer);
const mbox = @import("mbox.zig");
const range = @import("main.zig").range;

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
        log.info("msg[{}] = {} (0x{x})", .{ i, value, value });
    }
    try b.exec();
    for (b.message) |value, i| {
        log.info("msg[{}] = {} (0x{x})", .{ i, value, value });
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
    log.info("Got framebuffer {}x{}:{} {}. {*}[0..{}]", .{
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
