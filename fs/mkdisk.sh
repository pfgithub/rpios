dd if=/dev/zero bs=2M count=200 > binary.img
sfdisk binary.img < mkdisk.sfdisk
#dd if=/dev/zero bs=512 count=817152 > boot_fs.img
mkfs.fat -F32 -n BOOT --offset=2048 binary.img 408576 # size: (math (817152*512)/1024)
#mkfs.fat -F32 -n BOOT boot_fs.img # fix the offset above ; should be (2048 * 512) / 512 = 2048
#dd if=boot_fs.img of=binary.img bs=512 seek=4 # seek=(math 2048 / 512)

