"""
ad_da.py
-------------------------
统一的 AD/DA 模块（任务队列 + 字符串命令支持）
- 完全异步任务化
- 支持索引 / 名称访问 ADC/DAC（1-based）
- 支持单通道 / 多通道
- ADC 支持多次采样取平均，降低抖动
- 所有输出通过 StateMachine 回调

[OPTIMIZED FOR MODE B LOCKING]
- Added Circular DMA DAC output for seamless Dither generation.
- Added in-memory DAC Bias update to prevent hardware glitches (瞬跳).
- Added hardware-timed synchronous ADC multi-read for Coherent Demodulation.
"""

import pyb                                     # MicroPython 硬件库
import math
from array import array
from log_system import log, INFO               # 日志系统
from config import ADC_PINS, DAC_PINS, ADC_SAMPLE_TIMES, ADC_SAMPLE_DELAY_US, \
    ADC_SAMPLE_TIMES_ForSweep, ADC_SAMPLE_DELAY_US_ForSweep, SWeep_ADC_list       # 硬件配置

LOG_DURING_BOOT = False

class AD_DA:
    """
    AD / DA 统一管理模块
    """

    def __init__(self, task_queue, handler=None):
        # 保存任务队列
        self.task_queue = task_queue

        # 保存回调宿主
        self.handler = handler

        # =================================================
        # ADC 平均采样参数（可根据硬件调整）
        # =================================================
        self.ADC_SAMPLE_TIMES     = ADC_SAMPLE_TIMES      # 每次读取采样次数
        self.ADC_SAMPLE_DELAY_US  = ADC_SAMPLE_DELAY_US   # 每次采样之间延时（微秒）

        # ================= ADC 初始化 =================
        self.adcs = {}                          # name -> ADC 对象
        self.adc_list = []                      # [(idx, name, adc)]
        self.SWeep_MonitorADC_list = []
        SWeep_ADC_list_buf = dict(SWeep_ADC_list)

        for idx, (name, pin_name) in enumerate(ADC_PINS, start=1):# 从1开始编号
            log(INFO, f"Init ADC {idx} {name} {pin_name}")   
            try:
                pin = pyb.Pin(pin_name)         
                adc = pyb.ADC(pin)              
            except Exception as e:
                log(INFO, "ADC init failed:", name, e)  
                adc = None

            self.adcs[name] = adc               
            self.adc_list.append((idx, name, adc))  

            # 处理, 获取扫描测试中的需要监控的ADC数组
            try:
                if name in SWeep_ADC_list_buf:
                    self.SWeep_MonitorADC_list.append((idx, name, adc))
            except Exception as e:
                pass   

        # ================= DAC 初始化 =================
        self.dacs = {}                          # name -> DAC 对象
        self.dac_list = []                      # [(idx, name, dac)]
        self._dac_shadow = {}                   # DAC 影子寄存器
        
        # [Added for Mode B] Dictionary to hold circular buffers to prevent GC
        self.dac_circular_buffers = {}

        for idx, (name, pin_name) in enumerate(DAC_PINS, start=1):
            log(INFO, f"Init DAC {idx} {name} {pin_name}")   
            try:
                dac = pyb.DAC(idx, bits=12, buffering=True) 
                self._dac_shadow[name] = 0                  
            except Exception as e:
                log(INFO, "DAC init failed:", name, e)      
                dac = None
                self._dac_shadow[name] = None

            self.dacs[name] = dac               
            self.dac_list.append((idx, name, dac))  


    # =================================================
    # [新增] Mode B 锁相专属接口: 无缝硬件驱动 (Hardware-Level Defense)
    # =================================================

    def init_dither_dac_circular(self, dac_name, timer_id=6, freq=800, fs=19200, amp_code=30, initial_bias=2048):
        """
        [Mode B Exclusive] 
        Initialize a background Circular DMA output for the DAC to generate 
        continuous Dither without CPU intervention.
        """
        dac = self.get_dac_object(dac_name)
        if not dac:
            log(INFO, f"Cannot init circular DMA, DAC {dac_name} not found.")
            return

        # Use 48 points (LCM of 800Hz and 1200Hz at 19.2kHz fs) to allow future dual-tone
        lcm_len = 48 
        buf = array('H', [0] * lcm_len)
        
        # Pre-calculate waveform
        w = 2 * math.pi * freq / fs
        for i in range(lcm_len):
            val = int(initial_bias + amp_code * math.sin(w * i))
            buf[i] = max(min(val, 4095), 0)

        # Store buffer to prevent Garbage Collection and allow in-place modification
        self.dac_circular_buffers[dac_name] = {
            'buf': buf, 'amp': amp_code, 'freq': freq, 'fs': fs
        }
        self._dac_shadow[dac_name] = initial_bias

        try:
            tim_dac = pyb.Timer(timer_id, freq=fs)
            # Start Circular DMA: Hardware runs infinitely in the background
            dac.write_timed(buf, tim_dac, mode=pyb.DAC.CIRCULAR)
            log(INFO, f"Circular DMA started for {dac_name} at {freq}Hz (Timer {timer_id})")
        except Exception as e:
            log(INFO, f"Circular DMA failed to start for {dac_name}:", e)

    def update_dac_bias(self, dac_name, new_bias):
        """
        [Mode B Exclusive] 
        Update the DC bias of the running Dither output seamlessly.
        Modifies memory in-place. Zero hardware glitches (0瞬跳).
        """
        if dac_name not in self.dac_circular_buffers:
            # Fallback to normal sync write if circular buffer is not initialized
            self.set_dac_sync(dac_name, new_bias)
            return

        ctx = self.dac_circular_buffers[dac_name]
        buf = ctx['buf']
        amp = ctx['amp']
        w = 2 * math.pi * ctx['freq'] / ctx['fs']
        
        # In-place mathematical modification of the circular buffer
        for i in range(len(buf)):
            val = int(new_bias + amp * math.sin(w * i))
            buf[i] = max(min(val, 4095), 0)
            
        self._dac_shadow[dac_name] = new_bias


    def update_dither_amplitude(self, dac_name, new_amp_code):
        """
        [Mode B Exclusive] 
        无缝动态修改 Dither 幅度。
        仅在内存中重新计算波形，下一个 DMA 周期自动生效，绝对不引起硬件瞬跳。
        """
        if dac_name not in self.dac_circular_buffers:
            log(INFO, f"Cannot update amplitude, {dac_name} circular buffer not initialized.")
            return

        # 1. 更新内存池中的幅度配置
        ctx = self.dac_circular_buffers[dac_name]
        ctx['amp'] = new_amp_code 
        
        # 2. 获取当前的直流偏置
        current_bias = self._dac_shadow.get(dac_name, 2048)
        
        # 3. 复用 update_dac_bias 重新刷写这一帧的 48 个点
        self.update_dac_bias(dac_name, current_bias)
        log(INFO, f"Dither amplitude for {dac_name} dynamically updated to {new_amp_code} Codes.")


    def read_adc_timed_multi(self, adc_name, buf, timer_id=8, fs=19200):
        """
        [Mode B Exclusive] 
        Hardware-timed synchronous ADC multi-read. 
        Populates the float array directly without TaskQueue overhead.
        """
        adc = self.get_adc_object(adc_name)
        if adc:
            try:
                tim_adc = pyb.Timer(timer_id, freq=fs)
                # Non-blocking trigger. Fills 'buf' in the background via DMA/IRQ.
                pyb.ADC.read_timed_multi((adc,), (buf,), tim_adc)
            except Exception as e:
                log(INFO, f"Hardware timed multi-read failed for {adc_name}:", e)

    def set_dac_sync(self, ch, value):
        """
        Synchronous DAC write without task queue overhead.
        """
        dac = self.get_dac_object(ch)
        if dac:
            try:
                dac.write(value)
                name = ch if isinstance(ch, str) else next((n for idx, n, d in self.dac_list if idx == ch), None)
                if name:
                    self._dac_shadow[name] = value
            except Exception as e:
                log(INFO, f"DAC sync write failed for CH {ch}:", e)


    # =================================================
    # handler 注入
    # =================================================
    def set_handler(self, handler):
        self.handler = handler                  # 更新回调宿主

    # =================================================
    # 私有函数：ADC 多次采样取平均
    # =================================================
    def _read_adc_avg(self, adc):
        """
        对单个 ADC 进行多次采样并取平均
        """
        if adc is None:
            return None

        total = 0
        count = 0

        try:
            for _ in range(self.ADC_SAMPLE_TIMES):
                total += adc.read()
                count += 1
                pyb.udelay(self.ADC_SAMPLE_DELAY_US)
        except Exception as e:
            log(INFO, "ADC avg read failed:", e)
            return None

        return total // count if count else None

    # =================================================
    # ADC：读全部
    # =================================================
    def read_all_adc_task(self, source):
        results = []

        for idx, name, adc in self.adc_list:
            value = self._read_adc_avg(adc)
            results.append((idx, name, value))

        if self.handler:
            self.task_queue.add_task(
                self.handler.handle_adc_data,
                results,
                source
            )

    # =================================================
    # ADC：读单通道
    # =================================================
    def read_adc_task(self, ch, source):
        target = None

        if isinstance(ch, int):
            for idx, name, adc in self.adc_list:
                if idx == ch:
                    target = (idx, name, adc)
                    break
        elif isinstance(ch, str):
            for idx, name, adc in self.adc_list:
                if name == ch:
                    target = (idx, name, adc)
                    break

        if not target:
            log(INFO, "ADC channel invalid:", ch)
            return

        idx, name, adc = target
        value = self._read_adc_avg(adc)

        if self.handler:
            self.task_queue.add_task(
                self.handler.handle_adc_data,
                [(idx, name, value)],
                source
            )
    
    def get_adc_object(self, ch):

        target = None

        if isinstance(ch, int):
            for idx, name, adc in self.adc_list:
                if idx == ch:
                    target = (idx, name, adc)
                    break
        elif isinstance(ch, str):
            for idx, name, adc in self.adc_list:
                if name == ch:
                    target = (idx, name, adc)
                    break

        if not target:
            log(INFO, "ADC channel invalid:", ch)
            return

        idx, name, adc = target

        try:
            return adc
        except Exception as e:
            log(INFO, "get ADC object failed:", name, e)
    
    # =================================================
    # DAC：写
    # =================================================
    def set_dac_task(self, ch, value, source):
        targets = []

        if ch == "ALL":
            targets = self.dac_list[:]
        elif isinstance(ch, int):
            targets = [t for t in self.dac_list if t[0] == ch]
        elif isinstance(ch, str):
            targets = [t for t in self.dac_list if t[1] == ch]

        if not targets:
            log(INFO, "DAC channel invalid:", ch)
            return

        for idx, name, dac in targets:
            if dac is None:
                continue

            try:
                dac.write(value)
                self._dac_shadow[name] = value
                if source != None:
                    if self.handler:
                        self.task_queue.add_task(
                            self.handler.handle_dac_Set_Single,
                            idx,
                            name,
                            value,
                            source
                        )
            except Exception as e:
                log(INFO, "DAC write failed:", name, e)


    def get_dac_object(self, ch):
        targets = []

        if isinstance(ch, int):
            targets = [t for t in self.dac_list if t[0] == ch]
        elif isinstance(ch, str):
            targets = [t for t in self.dac_list if t[1] == ch]

        for idx, name, dac in targets:
 
            try:
                return dac
            except Exception as e:
                log(INFO, "get DAC failed:", name, e)

    # =================================================
    # DAC：读
    # =================================================
    def read_dac_task(self, ch, source):
        targets = []

        if ch == "ALL":
            targets = self.dac_list[:]
        elif isinstance(ch, int):
            targets = [t for t in self.dac_list if t[0] == ch]
        elif isinstance(ch, str):
            targets = [t for t in self.dac_list if t[1] == ch]

        if not targets:
            log(INFO, "DAC channel invalid:", ch)
            return

        results = [(idx, name, self._dac_shadow.get(name)) for idx, name, _ in targets]

        if self.handler:
            if len(results) == 1:
                self.task_queue.add_task(
                    self.handler.handle_dac_Read_Single,
                    results[0][0],
                    results[0][1],
                    results[0][2],
                    source
                )
            else:
                self.task_queue.add_task(
                    self.handler.handle_dac_Read_All,
                    results,
                    source
                )

    # =================================================
    # ADC 快照（供 SWEEP 使用）
    # =================================================
    def read_adc_snapshot(self):
        snapshot = []
        for idx, name, adc in self.SWeep_MonitorADC_list:
            snapshot.append((idx, name, self._read_adc_avg_Sweep(adc, ADC_SAMPLE_TIMES_ForSweep, ADC_SAMPLE_DELAY_US_ForSweep)))
        return snapshot

    # =================================================
    # 私有函数：ADC 多次采样取平均 (Sweep专用)
    # =================================================
    def _read_adc_avg_Sweep(self, adc, ADC_SAMPLE_TIMES_buf, ADC_SAMPLE_DELAY_US_buf):
        """
        对单个 ADC 进行多次采样并取平均
        """
        if adc is None:
            return None

        total = 0
        count = 0

        try:
            for _ in range(ADC_SAMPLE_TIMES_buf):
                total += adc.read()
                count += 1
                pyb.udelay(ADC_SAMPLE_DELAY_US_buf)
        except Exception as e:
            log(INFO, "ADC avg read failed:", e)
            return None

        return total // count if count else None
    
    def read_adc_sync(self, name, for_sweep=False):
        """
        同步读取指定 ADC 通道（不经过 TaskQueue）
        :param name: ADC 名称，例如 "DC_Power"
        :param for_sweep: 是否使用 Sweep 专用采样参数
        :return: ADC 数值 或 None
        """

        adc = self.adcs.get(name)
        if adc is None:
            return None

        if for_sweep:
            return self._read_adc_avg_Sweep(
                adc,
                ADC_SAMPLE_TIMES_ForSweep,
                ADC_SAMPLE_DELAY_US_ForSweep
            )
        else:
            return self._read_adc_avg(adc)