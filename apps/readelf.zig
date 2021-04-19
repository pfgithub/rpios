const std = @import("std");
const elf_file = @embedFile("sample");

pub fn main() !void {
    var fbs = std.io.FixedBufferStream([]const u8){ .buffer = elf_file, .pos = 0 };

    const elf_header = try std.elf.Header.read(&fbs);

    var section_headers = elf_header.section_header_iterator(&fbs);
    while (try section_headers.next()) |section| {
        // name: section.sh_name
        // strtab (.shstrtab | .strtab : does something useful)
        // ah : get the string table section first by looping, then when a string is needed, get it
        std.log.info("section header: {}", .{section});
    }

    var program_headers = elf_header.program_header_iterator(&fbs);
    while (try program_headers.next()) |phdr| {
        std.log.info("program header: {}", .{phdr});
    }

    // oh std.elf doesn't have a symbol table reader thing
}
