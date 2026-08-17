# K210 前视摄像头固件（MaixPy / MaixPy-v1）——CAR 项目
#
# 功能：QVGA(320x240) 采集 → LCD 本地预览 + JPEG 串口推流给树莓派。
# 树莓派侧由 car_nodes 的 k210_camera_driver 节点接收，发布 /camera/image_raw。
#
# 烧录（替代原 main.py）：
#   1. MaixPy IDE 连接开发板（选对应串口）
#   2. 打开本文件，菜单 工具 → 发送文件到开发板，保存为 main.py
#      （或 文件 → 将当前脚本保存到开发板 的 boot.py/main.py）
#   3. 按复位键，开发板脱离电脑也会自动运行本固件
#
# 接线：USB 线直连树莓派（同时供电）。Pi 上识别为 /dev/ttyUSB*（CH340）。
# 数据协议：连续 JPEG 字节流，帧以 FFD8 开始、FFD9 结束（JPEG 自带边界，
# 接收端按标记切帧，无需额外帧头帧尾）。

import sensor, image, time, lcd
from machine import UART

BAUD = 921600        # 串口波特率，必须与树莓派侧 k210_camera_driver 的 baud 参数一致
JPEG_QUALITY = 60    # JPEG 质量 [1,100]：越大越清晰、帧越大、帧率越低

lcd.init(freq=15000000)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)   # 320x240，LCD 上显示流畅
sensor.skip_frames(time=2000)
# 画面方向不对时按需打开（取决于摄像头/屏幕安装方向）：
# sensor.set_hmirror(True)
# sensor.set_vflip(True)

# Maix 开发板的 USB 转串口芯片（CH340 等）接的就是 UART1（REPL 口）；
# 脚本运行时 REPL 不活动，直接复用该串口经 USB 线向树莓派推流。
# 注意（本部署板实测）：CanMV makerobo 固件的 REPL 不是 machine.UART1，
# UART1 上电未绑定任何引脚，必须显式做 FPIOA 映射——USB 串口 CH9102 的
# K210→PC 方向接 IO5（逐引脚扫描实测）。MaixPy v1 固件的板子可去掉这两行。
from fpioa_manager import fm
fm.register(5, fm.fpioa.UART1_TX, force=True)
uart = UART(UART.UART1, BAUD, 8, 0, 0, timeout=1000, read_buf_len=4096)

while True:
    img = sensor.snapshot()
    lcd.display(img)                    # 本地预览（不需要可注释掉以提帧率）
    jpg = img.compress(quality=JPEG_QUALITY)
    uart.write(jpg)                     # image 对象实现 buffer 协议，直接写 JPEG 字节
                                        # 若个别固件版本报类型错误，改为 uart.write(jpg.to_bytes())
