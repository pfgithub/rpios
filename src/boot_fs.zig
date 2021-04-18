const std = @import("std");
const log = std.log.scoped(.boot_fs);
const sd = @import("sd.zig");

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
pub fn getPartition(alloc: *std.mem.Allocator) !Partition {
    const partition: Partition = blk: {
        const mbr_blocks = try alloc.alloc([512]u8, 1);
        defer alloc.free(mbr_blocks);
        try sd.readblock(0, mbr_blocks);
        const mbr = mbr_blocks[0];

        // check magic
        if (mbr[0x1FE] != 0x55 or mbr[0x1FF] != 0xAA) return error.BadMBR;

        // check partition type
        // - fat16 lba | fat32 lba
        if (mbr[0x1C2] != 0xE and mbr[0x1C2] != 0xC) return error.BadBootPartition;

        log.info("disk identifier: {}\n", .{std.mem.readIntLittle(u32, mbr[0x1B8..][0..4])});

        const partition_lba = std.mem.readIntLittle(u32, mbr[0x1C6..][0..4]);
        log.info("disk starts at {} (0x{X})", .{ partition_lba, partition_lba });

        break :blk Partition{ .lba = partition_lba };
    };
    return partition;
}
