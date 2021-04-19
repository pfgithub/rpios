/*
 * Copyright (C) 2018 bzt (bztsrc@github)
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * https://github.com/bztsrc/raspi3-tutorial/blob/master/05_uart0/start.S
 */

// this assembly file *shouldn't* be necessary, it should be possible to
// export fn _start() linksection(".text.boot) callconv(.Naked) void {}
// have that - check cpuid (inline asm to get mpidr_el1).
//   if ≠ 0, while(true) wfi
// have that set up a stack and call a function to clear bss
// after clearing bss, call main
// this loses the arguments dtb_ptr32: u64, x1: u64, x2: u64, x3: u64
// it is also more annoying to set the stack because zig expects a slice
//   while assembly is able to just set the stack end ptr and extern
//   variables don't have an option to say align(std.Target.stack_align)
// it also is more annoying to clear the bss because ptr math and extern
//   variables don't have an option to say align(16)

.section ".text.boot"

.global _start

_start:
    // read cpu id, stop slave cores
    mrs     x1, mpidr_el1
    and     x1, x1, #3
    cbz     x1, 2f
    // cpu id > 0, stop
1:  wfi // should be wfe to be able to be woken up, but qemu has this max out a thread because it's not being used properly
    b       1b
2:  // cpu id == 0

    // set stack before our code
    ldr     x1, =_start
    mov     sp, x1

    // clear bss
    ldr     x1, =__bss_start
    ldr     w2, =__bss_size
3:  cbz     w2, 4f
    str     xzr, [x1], #8
    sub     w2, w2, #1
    cbnz    w2, 3b
    // nah who needs this.

    // jump to C code, should not return
4:  bl      zigMain
    // for failsafe, halt this core too
    b       1b