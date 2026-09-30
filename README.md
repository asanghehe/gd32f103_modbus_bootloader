# GD32F103 Simple-Frame Bootloader

GD32F103RET6 RS485 固件升级程序。bootloader 使用简单二进制命令帧，不再使用 Modbus RTU。

## 串口参数

- 57600 baud, 8N1
- RS485 接口及方向控制沿用 `Template/BSP/bsp_usart.h`、PB7
- APP 地址：`0x08008000`，最大镜像长度：`0x1E000` 字节
- 上电后等待 `BOOT_WAIT_MS`（默认 2 秒）接收升级命令；无有效 APP 时留在 bootloader

## 帧格式

所有多字节整数均为小端序。请求帧和响应帧格式相同：

| 字段 | 大小 | 说明 |
| --- | ---: | --- |
| SOF | 2 | 固定 `A5 5A` |
| CMD | 1 | 命令号；响应为请求命令号 OR `0x80` |
| SEQ | 1 | 请求序号；响应回显 |
| LEN | 2 | DATA 长度，最大 260 字节 |
| DATA | LEN | 命令参数或结果 |
| CRC16 | 2 | CRC16-CCITT，初值 `0xFFFF`、多项式 `0x1021`，覆盖 CMD 到 DATA，结果小端序 |

除 INFO 外的命令响应 DATA 为 `RESULT:u8, ERROR:u16`，RESULT 为 0 表示成功。DATA 写入命令最多携带 256 字节固件数据。

| CMD | 名称 | 请求 DATA | 说明 |
| ---: | --- | --- | --- |
| `0x01` | INFO | 空 | 返回 22 字节：版本、状态、错误码、已验证镜像长度/CRC32、APP 地址/最大长度 |
| `0x10` | BEGIN | `SIZE:u32, CRC32:u32` | 失效化旧元数据并擦除镜像区与元数据页 |
| `0x11` | DATA | `OFFSET:u32, BYTES` | 写入镜像偏移；偏移和数据块长度需半字对齐 |
| `0x12` | VERIFY | 空 | 对镜像计算 CRC32；成功后提交持久化元数据 |
| `0x13` | JUMP | 空 | 重新校验镜像 CRC32，成功后跳转 APP |
| `0x14` | RESET | 空 | 发送成功响应后系统复位 |
| `0x15` | READ | `OFFSET:u32, LENGTH:u16` | 读取 APP 内容，单次最多 256 字节 |

镜像 CRC32 使用 IEEE 802.3/zlib 规则。长度、CRC32 和反码保存在 APP 之后的保留页 `0x08026000`；上电自动跳转和显式 JUMP 都会重新校验完整镜像。升级中断时元数据保持无效，设备不会启动未完成镜像。

## 升级

安装依赖 `pip install pyserial`，让设备复位进入 bootloader 后，在 2 秒窗口内运行：

```powershell
python test/update.py -p COM3 -f app.hex --verify
```

脚本支持 `.bin`、`.hex`、`.ihx`。如需手动检查串口数据，可使用 `python test/read.py -p COM3` 查询镜像元数据和向量表。原 FreeModbus 源码保留在仓库中，但不再编入此 bootloader 工程。