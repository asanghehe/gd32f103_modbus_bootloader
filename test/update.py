#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Upgrade GD32F103 APP firmware through the simple boot protocol."""

import argparse
import os
import struct
import sys
import time
import zlib

try:
    import serial
except ImportError:
    print("请先安装依赖: pip install pyserial")
    raise SystemExit(2)

SOF = b"\xA5\x5A"
CMD_INFO = 0x01
CMD_BEGIN = 0x10
CMD_DATA = 0x11
CMD_VERIFY = 0x12
CMD_JUMP = 0x13
CMD_RESET = 0x14
CMD_READ = 0x15
RESPONSE = 0x80

APP_START_ADDR = 0x08008000
APP_MAX_SIZE = 0x1E000
MAX_DATA = 260
DEFAULT_BAUD = 57600
BOOT_WAIT_MS = 2000

ERRORS = {
    0: "NONE", 1: "NOT_UPDATE", 2: "NOT_ERASED", 3: "BAD_SIZE",
    4: "BAD_ADDR", 5: "FLASH", 6: "CRC_FAIL", 7: "APP_INVALID",
    8: "NO_ERASE", 9: "BUSY",
}


def crc16(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def make_frame(command, sequence, payload=b""):
    body = struct.pack("<BBH", command, sequence, len(payload)) + payload
    return SOF + body + struct.pack("<H", crc16(body))


def load_intel_hex(path, base_addr):
    data = {}
    upper = 0
    eof_seen = False
    with open(path, "r", encoding="ascii") as source:
        for line_number, line in enumerate(source, 1):
            line = line.strip()
            if not line:
                continue
            if not line.startswith(":"):
                raise ValueError("HEX 第 %d 行格式错误" % line_number)
            record = bytes.fromhex(line[1:])
            if len(record) < 5 or (sum(record) & 0xFF):
                raise ValueError("HEX 第 %d 行校验和或长度错误" % line_number)
            count = record[0]
            address = (record[1] << 8) | record[2]
            kind = record[3]
            payload = record[4:4 + count]
            if len(payload) != count:
                raise ValueError("HEX 第 %d 行数据长度错误" % line_number)
            if kind == 0:
                for index, value in enumerate(payload):
                    data[upper + address + index] = value
            elif kind == 1:
                eof_seen = True
                break
            elif kind == 2:
                upper = int.from_bytes(payload, "big") << 4
            elif kind == 4:
                upper = int.from_bytes(payload, "big") << 16
            elif kind not in (3, 5):
                raise ValueError("HEX 第 %d 行类型不支持: %d" % (line_number, kind))
    if not eof_seen:
        print("警告: HEX 缺少 EOF 记录")
    if not data:
        raise ValueError("HEX 文件没有固件数据")
    first, last = min(data), max(data)
    if first < base_addr:
        raise ValueError("HEX 数据低于 APP 基址 0x%08X" % base_addr)
    size = last + 1 - base_addr
    if size > APP_MAX_SIZE:
        raise ValueError("固件超过 APP 容量 %d 字节" % APP_MAX_SIZE)
    image = bytearray(b"\xFF" * size)
    for address, value in data.items():
        image[address - base_addr] = value
    return bytes(image)


def load_firmware(path, base_addr):
    extension = os.path.splitext(path)[1].lower()
    if extension in (".hex", ".ihx"):
        return load_intel_hex(path, base_addr)
    if extension == ".bin":
        with open(path, "rb") as source:
            return source.read()
    raise ValueError("固件格式只支持 .bin / .hex / .ihx")


class BootloaderClient:
    def __init__(self, port, baudrate, timeout):
        self.timeout = timeout
        self.sequence = 0
        self.serial = serial.Serial(port, baudrate, timeout=0.1, write_timeout=timeout)

    def close(self):
        self.serial.close()

    def _read_exact(self, length, deadline):
        result = bytearray()
        while len(result) < length and time.monotonic() < deadline:
            part = self.serial.read(length - len(result))
            if part:
                result.extend(part)
        if len(result) != length:
            raise TimeoutError("等待 bootloader 响应超时")
        return bytes(result)

    def _read_frame(self):
        deadline = time.monotonic() + self.timeout
        sync = bytearray()
        while time.monotonic() < deadline:
            byte = self.serial.read(1)
            if not byte:
                continue
            sync.append(byte[0])
            if len(sync) > 2:
                del sync[0]
            if bytes(sync) == SOF:
                break
        else:
            raise TimeoutError("没有收到 bootloader 响应")

        header = self._read_exact(4, deadline)
        command, sequence, length = struct.unpack("<BBH", header)
        if length > MAX_DATA:
            raise ValueError("响应帧长度非法: %d" % length)
        payload_crc = self._read_exact(length + 2, deadline)
        body = header + payload_crc[:-2]
        received_crc = struct.unpack("<H", payload_crc[-2:])[0]
        if crc16(body) != received_crc:
            raise ValueError("响应 CRC16 错误")
        return command, sequence, payload_crc[:-2]

    def request(self, command, payload=b""):
        sequence = self.sequence
        self.sequence = (self.sequence + 1) & 0xFF
        self.serial.write(make_frame(command, sequence, payload))
        self.serial.flush()
        reply_command, reply_sequence, response = self._read_frame()
        if reply_command != (command | RESPONSE) or reply_sequence != sequence:
            raise ValueError("响应命令或序号不匹配")
        return response

    def command(self, command, payload=b""):
        response = self.request(command, payload)
        if len(response) != 3:
            raise ValueError("命令响应长度错误")
        result, error = struct.unpack("<BH", response)
        if result:
            raise RuntimeError("设备拒绝命令: %s" % ERRORS.get(error, "ERR_%d" % error))

    def info(self):
        response = self.request(CMD_INFO)
        if len(response) != 22:
            raise ValueError("INFO 响应长度错误")
        version, status, error, size, crc, address, maximum = struct.unpack("<HHHIIII", response)
        return {
            "version": version, "status": status, "error": error, "size": size,
            "crc": crc, "address": address, "maximum": maximum,
        }

    def read_flash(self, offset, length):
        result = bytearray()
        while length:
            count = min(length, 256)
            response = self.request(CMD_READ, struct.pack("<IH", offset, count))
            if len(response) != count + 3 or response[0] != 0:
                raise RuntimeError("Flash 回读失败")
            result.extend(response[3:])
            offset += count
            length -= count
        return bytes(result)


def wait_for_bootloader(client):
    original_timeout = client.timeout
    client.timeout = min(original_timeout, 0.5)
    try:
        info = client.info()
    except Exception:
        input("未检测到 bootloader。请复位设备，并在 %d 秒内按回车继续... " % (BOOT_WAIT_MS // 1000))
        info = client.info()
    finally:
        client.timeout = original_timeout
    return info


def upgrade(args):
    firmware = load_firmware(args.file, args.base)
    if not firmware or len(firmware) > APP_MAX_SIZE:
        raise ValueError("固件大小非法")
    fw_crc = zlib.crc32(firmware) & 0xFFFFFFFF
    print("固件大小: %d 字节, CRC32: 0x%08X" % (len(firmware), fw_crc))

    client = BootloaderClient(args.port, args.baud, args.timeout)
    try:
        info = wait_for_bootloader(client)
        print("Bootloader V%d.%d, APP_VALID=%s" % (
            info["version"] >> 8, info["version"] & 0xFF, bool(info["status"] & 1)))
        if info["address"] != args.base or info["maximum"] != APP_MAX_SIZE:
            raise ValueError("设备 APP 布局与本地设置不匹配")

        print("擦除 APP 并开始升级...")
        client.command(CMD_BEGIN, struct.pack("<II", len(firmware), fw_crc))
        started = time.monotonic()
        for offset in range(0, len(firmware), args.chunk):
            chunk = firmware[offset:offset + args.chunk]
            client.command(CMD_DATA, struct.pack("<I", offset) + chunk)
            done = min(offset + len(chunk), len(firmware))
            print("\r写入: %d%% (%d/%d)" % (done * 100 // len(firmware), done, len(firmware)), end="")
        print("\n写入完成，耗时 %.2f 秒" % (time.monotonic() - started))

        if args.verify:
            readback = client.read_flash(0, len(firmware))
            if readback != firmware:
                mismatch = next(index for index, pair in enumerate(zip(readback, firmware)) if pair[0] != pair[1])
                raise RuntimeError("回读不一致，偏移 0x%X" % mismatch)
            print("Flash 回读比对通过")

        print("执行设备端 CRC32 校验...")
        client.command(CMD_VERIFY)
        if args.no_jump:
            print("升级完成，设备保持在 bootloader")
        else:
            client.command(CMD_JUMP)
            print("CRC32 校验通过，已请求跳转 APP")
    finally:
        client.close()


def main():
    parser = argparse.ArgumentParser(description="GD32F103 simple-frame bootloader 升级工具")
    parser.add_argument("-p", "--port", required=True, help="串口，例如 COM3")
    parser.add_argument("-f", "--file", required=True, help=".bin / .hex / .ihx 固件")
    parser.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("-t", "--timeout", type=float, default=15.0)
    parser.add_argument("--chunk", type=int, default=256, help="数据块字节数，1..256")
    parser.add_argument("--base", type=lambda value: int(value, 0), default=APP_START_ADDR)
    parser.add_argument("--no-jump", action="store_true", help="校验后停留在 bootloader")
    parser.add_argument("--verify", action="store_true", help="逐块回读比对 Flash")
    args = parser.parse_args()
    if not 2 <= args.chunk <= 256 or args.chunk & 1:
        parser.error("--chunk 必须是 2..256 范围内的偶数")
    if not os.path.isfile(args.file):
        parser.error("固件文件不存在: %s" % args.file)
    try:
        upgrade(args)
        return 0
    except KeyboardInterrupt:
        print("\n用户中断")
        return 130
    except Exception as error:
        print("升级失败: %s" % error)
        return 1


if __name__ == "__main__":
    sys.exit(main())