# usb_comm.py
# --------------------------------------------------
# USB 虚拟串口通信模块
# - 仅负责数据收发与拆包
# - 不解析业务
# - 每条完整命令投递给 StateMachine
# --------------------------------------------------

import pyb
import time
from config import USB_READ_BUF
from log_system import log, INFO


class USBComm:
    def __init__(self, task_queue, handler=None):
        # 保存任务队列
        self.task_queue = task_queue

        # 状态机回调
        self.handler = handler

        # 接收缓冲（支持半包）
        self._rx_buffer = ""

        try:
            # 启用 USB VCP 模式
            pyb.usb_mode('VCP')
            time.sleep(1)

            # 创建 VCP 对象
            self.vcp = pyb.USB_VCP()
            self.vcp.init()

            log(INFO, "USB VCP init OK")

        except Exception as e:
            self.vcp = None
            log(INFO, "USB init failed:", e)

    def set_handler(self, handler):
        self.handler = handler

    def poll(self):
        """
        非阻塞轮询 USB 输入
        """
        try:
            if not self.vcp:
                self.vcp = pyb.USB_VCP()

            if not self.vcp.any():
                return

            raw = self.vcp.read(USB_READ_BUF)
            if not raw:
                return

            try:
                text = raw.decode('utf-8')
            except Exception:
                log(INFO, "USB decode error")
                return

            # 累加到缓冲区
            self._rx_buffer += text

            # 统一分隔符
            self._rx_buffer = (
                self._rx_buffer
                .replace('\r\n', '\n')
                .replace('\r', '\n')
                .replace(';', '\n')
            )

            lines = self._rx_buffer.split('\n')
            self._rx_buffer = lines[-1]

            for line in lines[:-1]:
                cmd = line.strip()
                if cmd and self.handler:
                    self.task_queue.add_task(
                        self.handler.handle_usb_data,
                        cmd.encode('utf-8')
                    )

        except Exception as e:
            log(INFO, "USB poll error:", e)

    def write(self, data: bytes):
        """
        对外接口：写 USB（异步）
        """
        self.task_queue.add_task(self._write_chunks, data)

    def _write_chunks(self, data: bytes):
        """
        分块写 USB，防止阻塞
        """
        try:
            if not self.vcp:
                self.vcp = pyb.USB_VCP()

            for i in range(0, len(data), USB_READ_BUF):
                self.vcp.write(data[i:i + USB_READ_BUF])

        except Exception as e:
            log(INFO, "USB write error:", e)
