#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GD32F103RET6 Modbus-RTU Bootloader 升级工具
协议见 bootloader.h V1.0.0
支持文件: Intel HEX (.hex / .ihx) 与裸二进制 (.bin)

依赖: pip install pymodbus pyserial

示例:
    python fw_update.py -p COM3 -f app.hex                    # Keil 产物直接刷
    python fw_update.py -p COM3 -f app.hex --reset --verify   # 推荐: 自动复位 + 回读校验
    python fw_update.py -p COM3 -f app.bin --base 0x08008000  # bin 也可, 显式给基址
"""

from __future__ import annotations

import argparse
import os
import sys
import time
import zlib

try:
    from pymodbus.client import ModbusSerialClient
except ImportError:
    try:
        from pymodbus.client.sync import ModbusSerialClient
    except ImportError:
        print("请先安装 pymodbus:  pip install pymodbus pyserial")
        sys.exit(2)


# ===========================================================================
#  协议常量 —— 严格对应 bootloader.h
# ===========================================================================
MB_REG_BOOT_VER   = 0
MB_REG_STATUS     = 1
MB_REG_FW_SIZE_H  = 2
MB_REG_FW_SIZE_L  = 3
MB_REG_FW_CRC_H   = 4
MB_REG_FW_CRC_L   = 5
MB_REG_CMD        = 6            # WO
MB_REG_CRC_RESULT = 7
MB_REG_ERRCODE    = 8
MB_REG_APP_ADDR_H = 9
MB_REG_APP_ADDR_L = 10
MB_CTRL_REG_NUM   = 11

MB_DATA_REG_BASE  = 0x1000
MAX_REG_ADDR      = 0xFFFF       # Modbus 地址上限

BOOT_STS_APP_VALID = 0x0001
BOOT_STS_UPDATING  = 0x0002
BOOT_STS_ERASED    = 0x0004
BOOT_STS_WRITABLE  = 0x0008
BOOT_STS_CMD_OK    = 0x0010

BOOT_CMD_ENTER_UPDATE = 1
BOOT_CMD_ERASE        = 2
BOOT_CMD_CHECK_CRC    = 3
BOOT_CMD_JUMP         = 4
BOOT_CMD_RESET        = 5

BOOT_CRC_NOT_RUN = 0
BOOT_CRC_PASS    = 1
BOOT_CRC_FAIL    = 2

BOOT_ERR_NAME = {
    0: "NONE",
    1: "NOT_UPDATE (未进入升级模式)",
    2: "NOT_ERASED (写数据前未擦除)",
    3: "BAD_SIZE   (固件大小非法)",
    4: "BAD_ADDR   (非法寄存器地址)",
    5: "FLASH      (flash 擦写失败)",
    6: "CRC_FAIL   (CRC 校验失败)",
    7: "APP_INVALID(APP 无效, 无法跳转)",
    8: "NO_ERASE   (擦除失败)",
    9: "BUSY       (内部状态冲突)",
}

DEFAULT_SLAVE = 0x01
DEFAULT_BAUD  = 57600
DEFAULT_CHUNK = 120
BOOT_WAIT_MS  = 2000

APP_START_ADDR = 0x08008000
APP_MAX_SIZE   = 0x1E000        # 120 KB
FLASH_PAGE     = 2048


def err_str(code):
    return "0x%04X %s" % (code, BOOT_ERR_NAME.get(code, "UNKNOWN"))


# ===========================================================================
#  Intel HEX 解析
# ===========================================================================
def load_intel_hex(path, base_addr):
    """
    解析 Intel HEX 文件, 返回 (bytes, base_addr)
    数据从 base_addr 开始, 中间空洞填 0xFF, 末尾补 0xFF 至偶数长度。
    """
    data = {}          # addr -> byte
    ext_base = 0
    eof_seen = False

    with open(path, 'r', encoding='ascii', errors='strict') as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            if not line.startswith(':'):
                raise ValueError("第 %d 行不是 Intel HEX 记录: %s" % (line_no, line[:40]))
            try:
                raw = bytes.fromhex(line[1:])
            except ValueError:
                raise ValueError("第 %d 行十六进制解码失败" % line_no)
            if len(raw) < 5:
                raise ValueError("第 %d 行长度不足" % line_no)
            if (sum(raw) & 0xFF) != 0:
                raise ValueError("第 %d 行校验和错误" % line_no)

            count   = raw[0]
            addr    = (raw[1] << 8) | raw[2]
            rtype   = raw[3]
            payload = raw[4:4 + count]
            if len(payload) != count:
                raise ValueError("第 %d 行数据长度不符" % line_no)

            if rtype == 0x00:                       # 数据记录
                full = ext_base + addr
                for i, b in enumerate(payload):
                    data[full + i] = b
            elif rtype == 0x01:                     # 文件结束
                eof_seen = True
                break
            elif rtype == 0x02:                     # 扩展段地址
                ext_base = ((payload[0] << 8) | payload[1]) << 4
            elif rtype == 0x04:                     # 扩展线性地址
                ext_base = ((payload[0] << 8) | payload[1]) << 16
            elif rtype in (0x03, 0x05):             # 起始地址, 忽略
                pass
            else:
                raise ValueError("第 %d 行未知记录类型 0x%02X" % (line_no, rtype))

    if not data:
        raise ValueError("HEX 文件不含任何数据记录")
    if not eof_seen:
        print("  [WARN] HEX 文件缺少 EOF 记录 (:00000001FF)")

    min_addr = min(data)
    max_addr = max(data)

    if min_addr < base_addr:
        raise ValueError(
            "HEX 数据起始 0x%08X 低于 APP 基址 0x%08X。\n"
            "       可能是刷错了 HEX (比如 bootloader 工程), 或 Keil 工程\n"
            "       IROM 起始地址没改成 0x08008000。请检查 APP 工程设置。"
            % (min_addr, base_addr))

    if min_addr > base_addr:
        print("  [WARN] HEX 数据起始 0x%08X 高于 APP 基址 0x%08X,"
              " 前 %d 字节将填充 0xFF"
              % (min_addr, base_addr, min_addr - base_addr))

    total = max_addr + 1 - base_addr
    if total & 1:
        total += 1
    if total > APP_MAX_SIZE:
        raise ValueError("HEX 数据 %d 字节 (0x%08X ~ 0x%08X) 超过协议上限 120KB"
                         % (total, base_addr, max_addr))

    buf = bytearray(b'\xFF' * total)
    for addr, b in data.items():
        if base_addr <= addr <= max_addr:
            buf[addr - base_addr] = b

    return bytes(buf), base_addr


# ===========================================================================
#  统一固件加载
# ===========================================================================
def load_firmware(path, base_addr):
    ext = os.path.splitext(path)[1].lower()
    if ext in ('.hex', '.ihx'):
        print("      格式: Intel HEX")
        data, baddr = load_intel_hex(path, base_addr)
        # 统计实际有效字节数 (非 0xFF 的部分), 仅供参考
        used = sum(1 for b in data if b != 0xFF)
        print("      地址范围: 0x%08X ~ 0x%08X"
              % (baddr, baddr + len(data) - 1))
        print("      有效字节: %d  (含 %d 字节 0xFF 填充)"
              % (used, len(data) - used))
        return data
    elif ext == '.bin':
        print("      格式: 裸二进制 (起始地址视为 0x%08X)" % base_addr)
        with open(path, 'rb') as f:
            data = f.read()
        return data
    else:
        raise ValueError("不支持的文件类型 '%s' (请用 .hex / .ihx / .bin)" % ext)


# ===========================================================================
#  升级客户端
# ===========================================================================
class BootloaderUpdater:

    def __init__(self, port, baudrate=DEFAULT_BAUD, slave=DEFAULT_SLAVE,
                 timeout=20.0):
        self.slave = slave
        self.chunk_regs = DEFAULT_CHUNK
        self.client = ModbusSerialClient(
            port=port, baudrate=baudrate,
            bytesize=8, parity='N', stopbits=1,
            timeout=timeout, retries=2,
        )
        if not self.client.connect():
            raise RuntimeError("串口打开失败: %s" % port)

    def close(self):
        try: self.client.close()
        except Exception: pass

    def __enter__(self):  return self
    def __exit__(self, *exc): self.close()

    # ---- 兼容不同 pymodbus 版本的 slave / device_id / unit ----
    def _call(self, name, *args, **kwargs):
        method = getattr(self.client, name)
        for key in ('slave', 'device_id', 'unit'):
            try:
                return method(*args, **kwargs, **{key: self.slave})
            except TypeError:
                continue
        return method(*args, **kwargs)

    # ------------------------------------------------------------------
    def read_regs(self, addr, count):
        # pymodbus 3.7+ 的 count 是 keyword-only, 必须用关键字传
        rr = self._call('read_holding_registers', addr, count=count)
        if rr.isError():
            raise RuntimeError("读寄存器失败 @0x%04X n=%d: %s" % (addr, count, rr))
        return rr.registers

    def write_regs(self, addr, values):
        # 3.7+ 与 2.x 都是位置参数, 保持不变即可
        rr = self._call('write_registers', addr, values)
        if rr.isError():
            raise RuntimeError("写寄存器失败 @0x%04X n=%d: %s" % (addr, len(values), rr))

    def write_reg(self, addr, value):
        rr = self._call('write_register', addr, value & 0xFFFF)
        if rr.isError():
            raise RuntimeError("写寄存器失败 @0x%04X: %s" % (addr, rr))

    # ---- 协议层 ----
    def read_ctrl_block(self):  return self.read_regs(MB_REG_BOOT_VER, MB_CTRL_REG_NUM)
    def read_status(self):      return self.read_regs(MB_REG_STATUS, 1)[0]
    def read_errcode(self):     return self.read_regs(MB_REG_ERRCODE, 1)[0]
    def read_crc_result(self):  return self.read_regs(MB_REG_CRC_RESULT, 1)[0]

    def set_fw_size(self, size):
        self.write_regs(MB_REG_FW_SIZE_H, [(size >> 16) & 0xFFFF, size & 0xFFFF])

    def set_fw_crc(self, crc):
        self.write_regs(MB_REG_FW_CRC_H, [(crc >> 16) & 0xFFFF, crc & 0xFFFF])

    def send_cmd(self, cmd):
        self.write_reg(MB_REG_CMD, cmd)

    def write_flash(self, offset, data, progress=None):
        if offset & 1:
            raise ValueError("offset 必须 2 字节对齐")
        if len(data) & 1:
            data = data + b'\x00'

        n_regs   = len(data) // 2
        base_reg = MB_DATA_REG_BASE + (offset // 2)
        if base_reg + n_regs - 1 > MAX_REG_ADDR:
            raise ValueError("寄存器地址 0x%04X 超出 Modbus 上限, 固件过大"
                             % (base_reg + n_regs - 1))

        done = 0
        while done < n_regs:
            take = min(self.chunk_regs, n_regs - done)
            chunk = data[done * 2: (done + take) * 2]
            regs = [(chunk[2 * i] << 8) | chunk[2 * i + 1] for i in range(take)]
            self.write_regs(base_reg + done, regs)
            done += take
            if progress:
                progress(done * 2, len(data))

    def read_flash(self, offset, length):
        if offset & 1:
            raise ValueError("offset 必须 2 字节对齐")
        if length & 1:
            length += 1
        n_regs   = length // 2
        base_reg = MB_DATA_REG_BASE + (offset // 2)

        out = bytearray()
        done = 0
        while done < n_regs:
            take = min(self.chunk_regs, n_regs - done)
            regs = self.read_regs(base_reg + done, take)
            for r in regs:
                # 设备 FC03 读回调对小端加载的 halfword 按大端发送,
                # 与 boot_cmd_write() 的字节序相反; 这里换回来才对应 flash 真实字节。
                out.append(r & 0xFF)
                out.append((r >> 8) & 0xFF)
            done += take
        return bytes(out)


# ===========================================================================
#  进入升级模式
# ===========================================================================
def try_enter_update(updater, max_attempts=15, interval=0.12):
    for _ in range(max_attempts):
        try:
            updater.send_cmd(BOOT_CMD_ENTER_UPDATE)
        except Exception:
            time.sleep(interval); continue
        time.sleep(0.05)
        try:
            if updater.read_status() & BOOT_STS_UPDATING:
                return True
        except Exception:
            pass
        time.sleep(interval)
    return False


def ensure_update_mode(updater, allow_reset):
    st = updater.read_status()
    if st & BOOT_STS_UPDATING:
        return True

    if not allow_reset:
        print("      当前不在升级模式 (STATUS=0x%04X)。" % st)
        print("      → 请手动复位设备, 或加 --reset 由脚本自动发复位命令。")
        input("      复位完成后按回车继续 ...")
        return try_enter_update(updater)

    print("      发送 RESET, 抢入 %dms 上电窗口 ..." % BOOT_WAIT_MS)
    try:
        updater.send_cmd(BOOT_CMD_RESET)
    except Exception:
        pass
    if try_enter_update(updater, max_attempts=20, interval=0.08):
        return True
    print("      抢入升级模式失败 (可能 APP 已接管串口)。")
    return False


# ===========================================================================
#  主流程
# ===========================================================================
def do_upgrade(args):
    # ---------- 加载固件 ----------
    if not os.path.isfile(args.file):
        print("错误: 找不到文件 %s" % args.file); return 1
    print("=" * 64)
    print(" 固件文件 : %s" % os.path.abspath(args.file))
    try:
        firmware = load_firmware(args.file, args.base)
    except Exception as e:
        print(" 加载失败: %s" % e); return 2

    if not firmware:
        print(" 错误: 固件为空"); return 2

    fw_size = len(firmware)
    fw_crc  = zlib.crc32(firmware) & 0xFFFFFFFF
    print(" 写入大小 : %d 字节 (%.2f KB)" % (fw_size, fw_size / 1024.0))
    print(" CRC32    : 0x%08X  (zlib / IEEE802.3)" % fw_crc)
    print(" 串口参数 : %s @ %d 8N1, slave=0x%02X"
          % (args.port, args.baud, args.slave))
    print("=" * 64)

    try:
        updater = BootloaderUpdater(args.port, args.baud, args.slave, args.timeout)
    except Exception as e:
        print("错误: %s" % e); return 3

    with updater:
        updater.chunk_regs = args.chunk

        # ---- 1. 读控制区 ----
        print("\n[1/7] 读取 Bootloader 状态 ...")
        try:
            st = updater.read_ctrl_block()
        except Exception as e:
            print("      读控制区失败: %s" % e); return 3

        boot_ver = st[MB_REG_BOOT_VER]
        status   = st[MB_REG_STATUS]
        app_addr = (st[MB_REG_APP_ADDR_H] << 16) | st[MB_REG_APP_ADDR_L]
        print("      BOOT_VER  = 0x%04X  (V%d.%d)"
              % (boot_ver, (boot_ver >> 8) & 0xFF, boot_ver & 0xFF))
        print("      APP_ADDR  = 0x%08X" % app_addr)
        flags = []
        if status & BOOT_STS_APP_VALID: flags.append("APP_VALID")
        if status & BOOT_STS_UPDATING:  flags.append("UPDATING")
        if status & BOOT_STS_ERASED:    flags.append("ERASED")
        if status & BOOT_STS_WRITABLE:  flags.append("WRITABLE")
        if status & BOOT_STS_CMD_OK:    flags.append("CMD_OK")
        print("      STATUS    = 0x%04X  [%s]"
              % (status, ", ".join(flags) if flags else "-"))

        if app_addr != args.base:
            print("  [WARN] 设备 APP_ADDR=0x%08X, 脚本基址=0x%08X, 请确认一致"
                  % (app_addr, args.base))

        # ---- 2. 进入升级模式 ----
        print("\n[2/7] 进入升级模式 ...")
        if not ensure_update_mode(updater, allow_reset=args.reset):
            return 4
        print("      已在升级模式 (STATUS=0x%04X)" % updater.read_status())

        # ---- 3. 写 size/crc ----
        print("\n[3/7] 写入 FW_SIZE / FW_CRC ...")
        updater.set_fw_size(fw_size)
        updater.set_fw_crc(fw_crc)
        print("      FW_SIZE = %d" % fw_size)
        print("      FW_CRC  = 0x%08X" % fw_crc)

        # ---- 4. 擦除 ----
        if args.no_erase:
            print("\n[4/7] 跳过擦除 (--no-erase)")
        else:
            pages = (fw_size + FLASH_PAGE - 1) // FLASH_PAGE
            print("\n[4/7] 擦除 APP 区 (约 %d 页, 每页 ~40ms, 预计 ~%.1fs) ..."
                  % (pages, pages * 0.04))
            t0 = time.time()
            updater.send_cmd(BOOT_CMD_ERASE)
            dt = time.time() - t0
            print("      擦除完成, 耗时 %.2f s" % dt)
            err = updater.read_errcode()
            if err != 0:
                print("      擦除失败: %s" % err_str(err)); return 5
            status = updater.read_status()
            if not (status & BOOT_STS_ERASED):
                print("      WARN: STATUS 未置 ERASED (0x%04X)" % status)
            if not (status & BOOT_STS_WRITABLE):
                print("      WARN: STATUS 未置 WRITABLE, 数据区可能写不进去")

        # ---- 5. 写数据 ----
        print("\n[5/7] 写入固件 (%d 字节) ..." % fw_size)
        t0 = time.time()

        def _progress(done, total):
            pct = done * 100 // max(total, 1)
            filled = 30 * done // max(total, 1)
            bar = "#" * filled + "-" * (30 - filled)
            sys.stdout.write("\r      [%s] %3d%%  %d/%d" % (bar, pct, done, total))
            sys.stdout.flush()

        updater.write_flash(0, firmware, _progress)
        sys.stdout.write("\n")
        dt = time.time() - t0
        print("      写入完成, 耗时 %.2f s  (%.1f KB/s)"
              % (dt, fw_size / max(dt, 1e-6) / 1024.0))
        err = updater.read_errcode()
        if err != 0:
            print("      写入失败: %s" % err_str(err)); return 6

        # ---- 6. 回读校验 ----
        if args.verify:
            print("\n[6/7] 回读 flash 逐字节比对 ...")
            t0 = time.time()
            rb = updater.read_flash(0, fw_size)[:fw_size]
            if rb == firmware:
                print("      回读校验通过 (%.2f s)" % (time.time() - t0))
            else:
                bad = next((i for i in range(fw_size) if rb[i] != firmware[i]), 0)
                print("      回读校验失败! 首个不一致偏移 = 0x%X (写入 0x%02X, 读回 0x%02X)"
                      % (bad, firmware[bad], rb[bad]))
                return 7
        else:
            print("\n[6/7] 跳过回读校验 (--verify 可开启)")

        # ---- 7. CRC + 跳转 ----
        print("\n[7/7] 请求 CRC 校验 ...")
        updater.send_cmd(BOOT_CMD_CHECK_CRC)
        time.sleep(0.2)
        crc_ret = updater.read_crc_result()
        err     = updater.read_errcode()
        print("      CRC_RESULT = %d (%s)"
              % (crc_ret, {0: "NOT_RUN", 1: "PASS", 2: "FAIL"}.get(crc_ret, "?")))
        print("      ERRCODE    = %s" % err_str(err))

        if crc_ret != BOOT_CRC_PASS or err != 0:
            print("\n[失败] CRC 校验未通过, 已终止升级 (未跳转)"); return 8

        if args.no_jump:
            print("\n[完成] 已跳过 JUMP, 设备停留在 Bootloader")
        else:
            print("\n      发送 JUMP, 启动 APP ...")
            try:
                updater.send_cmd(BOOT_CMD_JUMP)
            except Exception:
                pass
            print("\n[完成] 升级成功, APP 已启动")

    return 0


# ===========================================================================
#  命令行
# ===========================================================================
def parse_args():
    p = argparse.ArgumentParser(
        description="GD32F103RET6 Modbus-RTU Bootloader 升级工具 (协议 V1.0.0)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('-p', '--port', required=True, help='串口, 如 COM3 / /dev/ttyUSB0')
    p.add_argument('-f', '--file', required=True,
                   help='待升级固件: .hex / .ihx (Keil 产物) 或 .bin')
    p.add_argument('-b', '--baud', type=int, default=DEFAULT_BAUD,
                   help='波特率')
    p.add_argument('-s', '--slave', type=lambda x: int(x, 0),
                   default=DEFAULT_SLAVE, help='Modbus 从机地址')
    p.add_argument('-t', '--timeout', type=float, default=20.0,
                   help='串口超时(秒), ERASE 阻塞需 >=10s')
    p.add_argument('--chunk', type=int, default=DEFAULT_CHUNK,
                   help='每次 FC16 写入的寄存器数 (<=123)')
    p.add_argument('--base', type=lambda x: int(x, 0), default=APP_START_ADDR,
                   help='APP 基址 (HEX 中数据必须从这里开始)')
    p.add_argument('--reset', action='store_true',
                   help='自动发 RESET 抢占 2s 上电窗口')
    p.add_argument('--no-erase', action='store_true', help='跳过擦除命令')
    p.add_argument('--no-jump',  action='store_true', help='升级完成后不发送 JUMP')
    p.add_argument('--verify',   action='store_true',
                   help='写完 flash 后回读逐字节比对')
    return p.parse_args()


def main():
    args = parse_args()
    if not (1 <= args.chunk <= 123):
        print("错误: --chunk 必须在 1..123 之间"); return 1
    try:
        return do_upgrade(args)
    except KeyboardInterrupt:
        print("\n用户中断"); return 130
    except Exception as e:
        print("\n[异常] %s" % e); return 99


if __name__ == '__main__':
    sys.exit(main())