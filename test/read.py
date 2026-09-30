import struct
from update import BootloaderUpdater

with BootloaderUpdater('COM12', 57600, 1, 20) as u:
    u.chunk_regs = 120
    raw = u.read_flash(0, 16)
    print("前 16 字节 :", raw.hex(' '))

    sp, pc = struct.unpack('<II', raw[0:8])
    print("SP         = 0x%08X" % sp)
    print("PC         = 0x%08X   (bit0 = %d)" % (pc, pc & 1))

    sp_ok = (sp & 0x2FFE0000) == 0x20000000
    pc_ok = (0x08008000 <= pc < 0x08026000) and (pc & 1) == 1
    print("SP 合法    =", sp_ok)
    print("PC 合法    =", pc_ok)
    print("boot_app_is_valid() ->", sp_ok and pc_ok)