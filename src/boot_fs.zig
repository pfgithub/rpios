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
    kind: PartitionType,
    start: u32,
    len: u32,
};
const PartitionType = enum(u8) {
    empty = 0x00, // there is no entry, return null
    fat16_lba = 0x0E,
    fat32_lba = 0x0C,
    linux = 0x83,
    _,
};
pub fn readPartitionEntry(entry: [0x10]u8) Partition {
    // sample:
    // 80  00 02 00  0c  14 11 04  01 00 00 00  00 00 01 00
    // ^a  ^------b  ^c  ^------d  ^---------e  ^---------f
    // - a: 0x80 = bootable, 0x00 = not bootable, 0x01...0x7F = invalid
    // - b: unused (chs start address)
    // - c: partition type (0xE = FAT16 LBA, 0xC = FAT32 LBA)
    // - d: unused (chs end address)
    // - e: start address
    // - f: length
    const partition_type = @intToEnum(PartitionType, entry[0x4]);
    if (partition_type == .empty) {
        return .{ .kind = .empty, .start = 0, .len = 0 };
    }
    const first_sector = std.mem.readIntLittle(u32, entry[0x8..][0..0x4]);
    const len = std.mem.readIntLittle(u32, entry[0xC..][0..0x4]);
    return .{
        .kind = partition_type,
        .start = first_sector,
        .len = len,
    };
}
pub fn readPartitionTable(mbr: [0x200]u8) ![4]Partition {
    if (mbr[0x1FE] != 0x55 or mbr[0x1FF] != 0xAA) return error.BadMBR;

    return [4]Partition{
        readPartitionEntry(mbr[0x1BE..][0..0x10].*),
        readPartitionEntry(mbr[0x1CE..][0..0x10].*),
        readPartitionEntry(mbr[0x1DE..][0..0x10].*),
        readPartitionEntry(mbr[0x1EE..][0..0x10].*),
    };
}
pub fn getPartition(alloc: *std.mem.Allocator) !void {
    const mbr_blocks = try alloc.alloc([512]u8, 1);
    defer alloc.free(mbr_blocks);
    try sd.readblock(0, mbr_blocks);
    const mbr = mbr_blocks[0];

    const partition_table = try readPartitionTable(mbr);
    for (partition_table) |partition| {
        log.info("partition: {}", .{partition});
    }

    // // check magic
    // if (mbr[0x1FE] != 0x55 or mbr[0x1FF] != 0xAA) return error.BadMBR;

    // // check partition type
    // // - fat16 lba | fat32 lba
    // if (mbr[0x1C2] != 0xE and mbr[0x1C2] != 0xC) return error.BadBootPartition;

    // log.info("disk identifier: {}\n", .{std.mem.readIntLittle(u32, mbr[0x1B8..][0..4])});

    // const partition_lba = std.mem.readIntLittle(u32, mbr[0x1C6..][0..4]);
    // log.info("disk starts at {} (0x{X})", .{ partition_lba, partition_lba });

    // break :blk Partition{ .lba = partition_lba };
}
