cp zig-cache/bin/kernel8.img fs/input/boot/
cd fs
    ../src/deps/genimage/genimage
cd ..
qemu-img resize -f raw fs/images/sdcard.img 128M