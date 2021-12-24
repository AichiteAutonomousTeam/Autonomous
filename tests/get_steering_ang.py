# !/usr/bin/python
# -*- coding: utf-8 -*-
import spidev


# MCP3002からSPI通信で10ビットのデジタル値を取得。0から１の２チャンネル利用可
# https://qiita.com/M_Study/items/1a2779c24de20ee35af8
def readDC(num):
    if (num > 1) or (num < 0):
        return -1

    command1 = 0xd | (num << 1)
    command1 <<= 3
    ret = spi.xfer2([command1, 0, 0])
    return int((ret[0] & 0x3) << 8 | ret[1])

# 右 742 (3.8V)
# ( 中 480 (2.4V) )
# 左 207 (1V)


spi = spidev.SpiDev()
spi.open(0, 0)  # bus0, CE0
spi.max_speed_hz = 1000000  # 1MHz

data = []
try:
    while True:
        data.append(readDC(0))
        print readDC(0)

except KeyboardInterrupt:
    with open("data.txt", "w") as f:
        f.write(str(data))
    spi.close()
