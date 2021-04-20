const builtin = @import("builtin");
const std = @import("std");
const assert = std.debug.assert;
const log = std.log.scoped(.debug);
const log_writer = @import("uart.zig").writer();
const c = @import("c.zig");

extern var __debug_info_start: u8;
extern var __debug_info_end: u8;
extern var __debug_abbrev_start: u8;
extern var __debug_abbrev_end: u8;
extern var __debug_str_start: u8;
extern var __debug_str_end: u8;
extern var __debug_line_start: u8;
extern var __debug_line_end: u8;
extern var __debug_ranges_start: u8;
extern var __debug_ranges_end: u8;

fn getSliceStartEnd(start: *const u8, end: *const u8) []const u8 {
    return @ptrCast([*]const u8, start)[0 .. @ptrToInt(end) - @ptrToInt(start)];
}

var kernel_panic_allocator_bytes: [32 * 1024]u8 = undefined;
var kernel_panic_allocator_state = std.heap.FixedBufferAllocator.init(kernel_panic_allocator_bytes[0..]);
const kernel_panic_allocator = &kernel_panic_allocator_state.allocator;

pub fn getSelfDebugInfo() !*std.dwarf.DwarfInfo {
    const S = struct {
        var self_debug_info: ?std.dwarf.DwarfInfo = null;
    };

    if (S.self_debug_info != null) return &S.self_debug_info.?;

    log.warn("__debug_info_start: {}", .{&__debug_info_start});
    log.warn("__debug_info_end: {}", .{&__debug_info_end});
    log.warn("__debug_abbrev_start: {}", .{&__debug_abbrev_start});
    log.warn("__debug_abbrev_end: {}", .{&__debug_abbrev_end});
    log.warn("__debug_line_start: {}", .{&__debug_line_start});
    log.warn("__debug_line_end: {}", .{&__debug_line_end});
    log.warn("__debug_ranges_start: {}", .{&__debug_ranges_start});
    log.warn("__debug_ranges_end: {}", .{&__debug_ranges_end});

    S.self_debug_info = std.dwarf.DwarfInfo{
        .endian = builtin.Endian.Little,
        .debug_info = getSliceStartEnd(&__debug_info_start, &__debug_info_end),
        .debug_abbrev = getSliceStartEnd(&__debug_abbrev_start, &__debug_abbrev_end),
        .debug_str = getSliceStartEnd(&__debug_str_start, &__debug_str_end),
        .debug_line = getSliceStartEnd(&__debug_line_start, &__debug_line_end),
        .debug_ranges = getSliceStartEnd(&__debug_ranges_start, &__debug_ranges_end),
        .abbrev_table_list = undefined,
        .compile_unit_list = undefined,
        .func_list = undefined,
    };
    try std.dwarf.openDwarfDebugInfo(&S.self_debug_info.?, kernel_panic_allocator);

    return &S.self_debug_info.?;
}

var already_panicking: bool = false;

pub fn panicLog(stack_trace: ?*builtin.StackTrace) void {
    @setCold(true);
    if (already_panicking) {
        log.emerg("\npanicked during kernel panic", .{});
        return;
    }
    already_panicking = true;

    // const first_trace_addr = @returnAddress();
    // if (stack_trace) |t| {
    //     dumpStackTrace(t);
    // }
    // dumpCurrentStackTrace(first_trace_addr);
}

fn panicHalt() noreturn {
    asm volatile ("cli; hlt");
    while (true) {}
}

fn dwarfSectionFromSymbolAbs(start: *u8, end: *u8) std.dwarf.DwarfInfo.Section {
    return std.dwarf.DwarfInfo.Section{
        .offset = 0,
        .size = @ptrToInt(end) - @ptrToInt(start),
    };
}

fn dwarfSectionFromSymbol(start: *u8, end: *u8) std.dwarf.DwarfInfo.Section {
    return std.dwarf.DwarfInfo.Section{
        .offset = @ptrToInt(start),
        .size = @ptrToInt(end) - @ptrToInt(start),
    };
}

pub fn dumpStackTrace(stack_trace: *const builtin.StackTrace) void {
    const dwarf_info = getSelfDebugInfo() catch |err| {
        log.emerg("Unable to dump stack trace: Unable to open debug info: {s}", .{@errorName(err)});
        return;
    };
    writeStackTrace(stack_trace, dwarf_info) catch |err| {
        log.emerg("Unable to dump stack trace: {s}", .{@errorName(err)});
        return;
    };
}

pub fn dumpCurrentStackTrace(start_addr: ?usize) void {
    const dwarf_info = getSelfDebugInfo() catch |err| {
        log.emerg("Unable to dump stack trace: Unable to open debug info: {s}", .{@errorName(err)});
        return;
    };
    writeCurrentStackTrace(dwarf_info, start_addr) catch |err| {
        log.emerg("Unable to dump stack trace: {s}", .{@errorName(err)});
        return;
    };
}

fn printLineFromBuffer(writer: anytype, contents: []const u8, line_info: std.debug.LineInfo) anyerror!void {
    var line: usize = 1;
    var column: usize = 1;
    var abs_index: usize = 0;
    for (contents) |byte| {
        if (line == line_info.line) {
            try writer.writeByte(byte);
            if (byte == '\n') {
                return;
            }
        }
        if (byte == '\n') {
            line += 1;
            column = 1;
        } else {
            column += 1;
        }
    }
    return error.EndOfFile;
}

fn printLineFromFileEmbed(writer: anytype, line_info: std.debug.LineInfo) anyerror!void {
    inline for (.{"main.zig"}) |src_path| {
        if (std.mem.endsWith(u8, line_info.file_name, src_path)) {
            const contents = @embedFile(src_path);
            try printLineFromBuffer(writer, contents[0..], line_info);
            return;
        }
    }
    try writer.print("(source file {s} not added in std/debug.zig)\n", .{line_info.file_name});
}

// sad copy paste from debug.zig
const SymbolInfo = struct {
    symbol_name: []const u8 = "???",
    compile_unit_name: []const u8 = "???",
    line_info: ?std.debug.LineInfo = null,
};

// sad copy paste from debug.zig
pub fn getSymbolAtAddress(
    dwarf: *std.dwarf.DwarfInfo,
    in_address: usize,
) !SymbolInfo {
    // TODO: relocation?!?!?!
    // Translate the VA into an address into this object
    // const relocated_address = in_address - base_address;
    const addr = in_address;

    if (nosuspend dwarf.findCompileUnit(addr)) |compile_unit| {
        return SymbolInfo{
            .symbol_name = nosuspend (dwarf.getSymbolName(addr) orelse "???"),
            .compile_unit_name = compile_unit.die.getAttrString(
                dwarf,
                std.dwarf.AT_name,
            ) catch |err| switch (err) {
                error.MissingDebugInfo, error.InvalidDebugInfo => "???",
                else => return err,
            },
            .line_info = nosuspend dwarf.getLineNumberInfo(
                compile_unit.*,
                addr,
            ) catch |err| switch (err) {
                error.MissingDebugInfo, error.InvalidDebugInfo => null,
                else => return err,
            },
        };
    } else |err| switch (err) {
        error.MissingDebugInfo, error.InvalidDebugInfo => {
            return SymbolInfo{};
        },
        else => return err,
    }

    unreachable;
}

fn printSourceAtAddress(dwarf_info: *std.dwarf.DwarfInfo, address: usize) !void {
    const symbol_info = try getSymbolAtAddress(dwarf_info, address);
    return printLineInfo(
        log_writer,
        symbol_info.line_info,
        address,
        symbol_info.symbol_name,
        symbol_info.compile_unit_name,
        .escape_codes,
        printLineFromFileEmbed,
    );
}
fn printLineInfo(
    out_stream: anytype,
    line_info: ?std.debug.LineInfo,
    address: usize,
    symbol_name: []const u8,
    compile_unit_name: []const u8,
    tty_config: std.debug.TTY.Config,
    comptime printLineFromFile: anytype,
) !void {
    nosuspend {
        tty_config.setColor(out_stream, .White);

        if (line_info) |*li| {
            try out_stream.print("{s}:{d}:{d}", .{ li.file_name, li.line, li.column });
        } else {
            try out_stream.writeAll("???:?:?");
        }

        tty_config.setColor(out_stream, .Reset);
        try out_stream.writeAll(": ");
        tty_config.setColor(out_stream, .Dim);
        try out_stream.print("0x{x} in {s} ({s})", .{ address, symbol_name, compile_unit_name });
        tty_config.setColor(out_stream, .Reset);
        try out_stream.writeAll("\n");

        // Show the matching source code line if possible
        if (line_info) |li| {
            if (printLineFromFile(out_stream, li)) {
                if (li.column > 0) {
                    // The caret already takes one char
                    const space_needed = @intCast(usize, li.column - 1);

                    try out_stream.writeByteNTimes(' ', space_needed);
                    tty_config.setColor(out_stream, .Green);
                    try out_stream.writeAll("^");
                    tty_config.setColor(out_stream, .Reset);
                }
                try out_stream.writeAll("\n");
            } else |err| switch (err) {
                error.EndOfFile, error.FileNotFound => {},
                error.BadPathName => {},
                error.AccessDenied => {},
                else => return err,
            }
        }
    }
}

fn writeCurrentStackTrace(dwarf_info: *std.dwarf.DwarfInfo, start_addr: ?usize) !void {
    var it = std.debug.StackIterator.init(start_addr, null);
    while (it.next()) |return_address| {
        try printSourceAtAddress(dwarf_info, return_address);
    }
}

fn writeStackTrace(stack_trace: *const builtin.StackTrace, dwarf_info: *std.dwarf.DwarfInfo) !void {
    var frame_index: usize = undefined;
    var frames_left: usize = undefined;
    if (stack_trace.index < stack_trace.instruction_addresses.len) {
        frame_index = 0;
        frames_left = stack_trace.index;
    } else {
        frame_index = (stack_trace.index + 1) % stack_trace.instruction_addresses.len;
        frames_left = stack_trace.instruction_addresses.len;
    }

    while (frames_left != 0) : ({
        frames_left -= 1;
        frame_index = (frame_index + 1) % stack_trace.instruction_addresses.len;
    }) {
        const return_address = stack_trace.instruction_addresses[frame_index];
        try printSourceAtAddress(dwarf_info, return_address);
    }
}
