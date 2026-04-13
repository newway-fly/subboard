import pyb
from config import LED_NUM, Delay_MS, Timer_Num

class LedIndicator:
    """
    System‑running LED indicator, using the onboard LED2.
    Uses a hardware timer to ensure a stable blinking period.
    """

    def __init__(self, LED=LED_NUM, period_ms=Delay_MS, Timer_NumBuf = Timer_Num):
        self.led = pyb.LED(LED)
        self.period = period_ms
        self.state = 0
        self.led.off()

        # 使用 Timer 2（任意空闲定时器）
        self.timer = pyb.Timer(Timer_NumBuf)
        self.timer.init(freq=1000//self.period)  # 频率 = 1/半周期
        self.timer.callback(self._toggle)

    def _toggle(self, t):
        """Timer 回调切换 LED 状态"""
        self.state ^= 1
        if self.state:
            self.led.on()
        else:
            self.led.off()

    def on(self):
        """强制亮"""
        self.state = 1
        self.led.on()

    def off(self):
        """强制灭"""
        self.state = 0
        self.led.off()

    def set_period(self, ms):
        """改变闪烁速度"""
        self.period = ms
        self.timer.init(freq=1000//self.period)
