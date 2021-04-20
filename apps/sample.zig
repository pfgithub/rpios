//! zig build-exe sample.zig -target aarch64-freestanding -fPIC -fPIE -mcpu cortex_a53 -ofmt=elf

const std = @import("std");

export fn syscall0(number: usize) usize {
    return asm volatile ("svc #0"
        : [ret] "={x0}" (-> usize)
        : [number] "{x8}" (number)
        : "memory", "cc"
    );
}
fn syscall1(number: usize, arg1: usize) usize {
    return asm volatile ("svc #0"
        : [ret] "={x0}" (-> usize)
        : [number] "{x8}" (number),
          [arg1] "{x0}" (arg1)
        : "memory", "cc"
    );
}

export fn _start() void {
    for ("User Program!") |char| _ = syscall1(1, char);
    _ = syscall0(2);
}
