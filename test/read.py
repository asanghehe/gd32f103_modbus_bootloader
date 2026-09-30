import argparse
import struct

from update import APP_START_ADDR, BootloaderClient


def main():
    parser = argparse.ArgumentParser(description="读取 APP 向量与 CRC 元数据")
    parser.add_argument("-p", "--port", default="COM12")
    parser.add_argument("-b", "--baud", type=int, default=57600)
    args = parser.parse_args()

    client = BootloaderClient(args.port, args.baud, 5.0)
    try:
        info = client.info()
        raw = client.read_flash(0, 8)
    finally:
        client.close()

    sp, pc = struct.unpack("<II", raw)
    sp_ok = 0x20000000 <= sp <= 0x20010000 and (sp & 3) == 0
    pc_ok = APP_START_ADDR <= (pc & ~1) < APP_START_ADDR + info["size"] and (pc & 1) == 1
    print("APP 地址   = 0x%08X" % info["address"])
    print("APP 长度   = %d" % info["size"])
    print("APP CRC32  = 0x%08X" % info["crc"])
    print("SP         = 0x%08X (合法: %s)" % (sp, sp_ok))
    print("PC         = 0x%08X (合法: %s)" % (pc, pc_ok))
    print("APP_VALID  = %s" % bool(info["status"] & 1))


if __name__ == "__main__":
    main()