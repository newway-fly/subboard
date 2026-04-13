# --------------------------------------------------
# GPIO 控制模块（最终稳定版） gpio_control.py
# --------------------------------------------------

import pyb
from config import GPIO_CONFIG
from log_system import log, INFO


class GPIOControl:
    def __init__(self, task_queue, handler=None):
        # 保存任务队列
        self.task_queue = task_queue
        # 状态机回调
        self.handler = handler

        # GPIO 对象表：NAME -> {pin, mode, default}
        self.pins = {}

        # 初始化所有 GPIO
        self._init_gpio_pins()

    # --------------------------------------------------
    # GPIO 初始化
    # --------------------------------------------------
    def _init_gpio_pins(self):
        for name, config in GPIO_CONFIG.items():
            pin_name, mode_str, default_value = config

            name = name.upper()
            pin_name = pin_name.upper()

            try:
                if mode_str.upper() == "OUT":
                    pin = pyb.Pin(pin_name, pyb.Pin.OUT_PP)
                    if default_value is not None:
                        pin.value(default_value)

                    self.pins[name] = {
                        "pin": pin,
                        "mode": "OUT",
                        "default": default_value,
                    }

                elif mode_str.upper() == "IN":
                    if default_value == 1:
                        pin = pyb.Pin(pin_name, pyb.Pin.IN, pyb.Pin.PULL_UP)
                    elif default_value == 0:
                        pin = pyb.Pin(pin_name, pyb.Pin.IN, pyb.Pin.PULL_DOWN)
                    else:
                        pin = pyb.Pin(pin_name, pyb.Pin.IN)

                    self.pins[name] = {
                        "pin": pin,
                        "mode": "IN",
                        "default": default_value,
                    }

                else:
                    log(INFO, "GPIO invalid mode:", name)
                    self.pins[name] = None

                log(INFO, "GPIO init:", name, pin_name)

            except Exception as e:
                log(INFO, "GPIO init failed:", name, e)
                self.pins[name] = None

    # ===============================
    # Direct 接口（同步、立即执行）
    # ===============================
    def set_pin_direct(self, pin_name, level):
        """
        立即设置 GPIO 引脚电平（同步）
        """
        pin_name = pin_name.upper()
        pin_info = self.pins.get(pin_name)
        if not pin_info or pin_info["mode"] != "OUT":
            return False

        try:
            pin = pin_info["pin"]
            if level in (1, '1', 'HIGH', 'H'):
                pin.high()
            else:
                pin.low()
            return True
        except Exception as e:
            log(INFO, "set_pin_direct error:", e)
            return False

    def read_pin_direct(self, pin_name):
        """
        立即读取 GPIO 引脚电平（同步）
        """
        pin_name = pin_name.upper()
        pin_info = self.pins.get(pin_name)
        if not pin_info:
            return None

        try:
            return pin_info["pin"].value()
        except Exception as e:
            log(INFO, "read_pin_direct error:", e)
            return None                             


    # ==================================================
    # 异步接口（命令系统使用）
    # ==================================================
    def set_pin_task(self, pin_name, level, source="usb"):
        pin_name = pin_name.upper()

        if pin_name not in self.pins:
            return

        pin_info = self.pins[pin_name]
        if pin_info is None or pin_info["mode"] != "OUT":
            return

        pin = pin_info["pin"]

        if level in ("HIGH", "1", 1):
            pin.high()
            value = 1
        elif level in ("LOW", "0", 0):
            pin.low()
            value = 0
        else:
            return

        if self.handler:
            self.task_queue.add_task(
                self.handler.handle_gpio_result,
                (pin_name, value),
                "SET",
                source,
            )

    def read_pin_task(self, pin_name, source="usb"):
        pin_name = pin_name.upper()
        result = []

        if pin_name in self.pins and self.pins[pin_name]:
            try:
                value = self.pins[pin_name]["pin"].value()
                result.append((pin_name, value, self.pins[pin_name]["mode"]))
            except Exception:
                result.append((pin_name, None, "ERROR"))
        else:
            result.append((pin_name, None, "INVALID"))

        if self.handler:
            self.task_queue.add_task(
                self.handler.handle_gpio_result,
                result,
                "READ",
                source,
            )

    def read_all_pins_task(self, source="usb"):
        result = []

        for name, INFO in self.pins.items():
            if INFO is None:
                result.append((name, None, "INVALID"))
                continue

            try:
                value = INFO["pin"].value()
                result.append((name, value, INFO["mode"]))
            except Exception:
                result.append((name, None, "ERROR"))

        if self.handler:
            self.task_queue.add_task(
                self.handler.handle_gpio_result,
                result,
                "READ_ALL",
                source,
            )
