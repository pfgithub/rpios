add-symbol-file zig-cache/bin/kernel8.img 0x80000
target remote localhost:1234
set scheduler-locking on
layout split