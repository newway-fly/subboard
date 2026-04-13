# uart_master.py
# --------------------------------------------------
# UART 主通信模块（SubBoard <-> HostBoard）
# - 非阻塞轮询
# - 接收数据 → TaskQueue → StateMachine
# - 发送数据 → TaskQueue → UART
# --------------------------------------------------

import pyb
from config import UART_PORT, UART_BAUD
from log_system import log, INFO


class UARTMaster:
    def __init__(self, task_queue, handler=None):
        # 保存任务队列引用
        self.task_queue = task_queue

        # 状态机回调
        self.handler = handler

        self._rx_buffer = ""   # ★ 新增：UART 接收缓存

        try:
            # 初始化 UART
            self.uart = pyb.UART(UART_PORT, UART_BAUD)
            log(INFO, "UART init OK:", UART_PORT, UART_BAUD)
        except Exception as e:
            self.uart = None
            log(INFO, "UART init failed:", e)

    def set_handler(self, handler):
        self.handler = handler

    def poll(self):
        """
        非阻塞 UART 轮询（支持多命令）
        """
        try:
            if not self.uart:
                self.uart = pyb.UART(UART_PORT, UART_BAUD)

            if not self.uart.any():
                return

            raw = self.uart.read()
            if not raw:
                return

            text = raw.decode('utf-8', 'ignore')

            # keep only safe ASCII
            clean = ''.join(c for c in text if 32 <= ord(c) <= 126 or c in '\n\r')

            # -------- 累加缓冲 --------
            self._rx_buffer += clean

            # -------- 统一分隔符 --------
            self._rx_buffer = (
                self._rx_buffer
                .replace('\r\n', '\n')
                .replace('\r', '\n')
                .replace(';', '\n')
            )

            lines = self._rx_buffer.split('\n')
            self._rx_buffer = lines[-1]  # 保留半包

            # -------- 每一行单独投递 --------
            for line in lines[:-1]:
                cmd = line.strip()
                if cmd and self.handler:
                    self.task_queue.add_task(
                        self.handler.handle_uart_data,
                        cmd.encode('utf-8')
                    )

        except Exception as e:
            log(INFO, "UART poll error:", e)


    def write(self, data: bytes):
        """
        对外接口：写 UART（异步）
        """
        self.task_queue.add_task(self._write_direct, data)

    def _write_direct(self, data: bytes):
        """
        实际 UART 写入（任务上下文）
        """
        try:
            if not self.uart:
                self.uart = pyb.UART(UART_PORT, UART_BAUD)
            
            self.uart.write(data)

        except Exception as e:
            log(INFO, "UART write failed:", e)

  