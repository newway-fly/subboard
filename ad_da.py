"""
ad_da.py
-------------------------
Unified AD/DA Module with Circular DMA Dither and Arm Topology Control.
[OPTIMIZED FOR SUBBOARD MODE B]
- Implements a Static Memory Pool (global_adc_pool) to completely eliminate MemoryError.
- [NEW] Uses a Global Shared Timer (sync_timer) for absolute phase alignment.
"""

import pyb
import math
import machine
import time,gc
from array import array
from log_system import log, INFO
from config import ADC_PINS, DAC_PINS, ADC_SAMPLE_TIMES, ADC_SAMPLE_DELAY_US, \
    ADC_SAMPLE_TIMES_ForSweep, ADC_SAMPLE_DELAY_US_ForSweep, SWeep_ADC_list, \
    Bias_Arm_Volt, Bias_Amp_Volt, MCU_ADDA_Vref

class AD_DA:
    def __init__(self, task_queue, handler=None):
        self.task_queue = task_queue
        self.handler = handler
        self.vref = MCU_ADDA_Vref

        # =================================================
        # ADC Average Sampling Parameters
        # =================================================
        self.ADC_SAMPLE_TIMES     = ADC_SAMPLE_TIMES      
        self.ADC_SAMPLE_DELAY_US  = ADC_SAMPLE_DELAY_US   

        # ================= ADC Initialization =================
        self.adcs = {}
        self.adc_list = []
        self.SWeep_MonitorADC_list = []
        SWeep_ADC_list_buf = dict(SWeep_ADC_list)

        for idx, (name, pin_name) in enumerate(ADC_PINS, start=1):
            try:
                pin = pyb.Pin(pin_name)         
                adc = pyb.ADC(pin)              
            except Exception as e:
                adc = None
            self.adcs[name] = adc               
            self.adc_list.append((idx, name, adc))  
            if name in SWeep_ADC_list_buf:
                self.SWeep_MonitorADC_list.append((idx, name, adc))

        # ================= DAC Initialization =================
        self.dacs = {}
        self.dac_list = []
        self._dac_shadow = {}
        for idx, (name, pin_name) in enumerate(DAC_PINS, start=1):
            try:
                dac = pyb.DAC(idx, bits=12, buffering=True) 
                self._dac_shadow[name] = 0                  
            except Exception as e:
                dac = None
                self._dac_shadow[name] = None
            self.dacs[name] = dac               
            self.dac_list.append((idx, name, dac))


        # ================= Mode B Architecture State =================
        self.arm_mode = "P"                 
        self.fixed_unused_dc = Bias_Arm_Volt 
        self.Bias_Amp_Volt = Bias_Amp_Volt       
        self.current_main_code = 2048       
        self.dither_amp_mv = 50             
        
        # 【路线B 核心】：DAC 数组退回单周期 48 点。DMA 会无限循环读取它。
        self.lcm_len = 48                   
        self.sine_buf_p = array('H', [0] * self.lcm_len)
        self.sine_buf_n = array('H', [0] * self.lcm_len)
        
        self.dither_active = False
        self.dma_engine_started = False # 引擎启动标志
        
        self.sync_timer = pyb.Timer(6, freq=19200)
        self.global_adc_pool = array('H', [0] * 4320)   
        self.init_dither(self.dither_amp_mv)

    # =================================================
    # 初始化 bias模式
    # =================================================
    def init_arm_config(self, mode, v1=None, v2=None):
        self.arm_mode = mode.upper()
        if self.arm_mode in ["P", "N"]:
            v_init = v1 if v1 is not None else 1.0
            self.fixed_unused_dc = Bias_Arm_Volt
        elif self.arm_mode == "PN":
            v_init = v1 if v1 is not None else 1.0
            self.Bias_Amp_Volt = v2 if v2 is not None else Bias_Amp_Volt
        else: 
            return
        #初始化完就启动DMA
        self.start_dma_engine_once()
    
        self.update_dac_bias(int(v_init/2.0/self.vref*4096.0))

    def init_dither(self, amp_mv):
        self.dither_amp_mv = amp_mv
        log(INFO, f"Dither pre-calculated with Vpp: {amp_mv} mV")

    def start_dma_engine_once(self):
        """生命周期内唯一一次启动 DMA，开机即运行，永不停机"""
        if self.dma_engine_started: 
            return
        
        dac_p = self.dacs.get("DAC_Parm")
        dac_n = self.dacs.get("DAC_Narm")
        
        # 启动前，先用当前 DC 填平数组
        self.set_lock_dither(False)
        
        # 唯一一次强制配置 DMA
        if dac_p: dac_p.write_timed(self.sine_buf_p, self.sync_timer, mode=pyb.DAC.CIRCULAR)
        if dac_n: dac_n.write_timed(self.sine_buf_n, self.sync_timer, mode=pyb.DAC.CIRCULAR)
        self.dma_engine_started = True

    @micropython.native
    def _recalc_sine_buffer(self):
        import math
        w = 2 * math.pi * 800 / 19200
        # 【精准采纳你的建议】：严格锚定当前 DC 偏压
        p_base = self._dac_shadow.get("DAC_Parm", 2048)
        n_base = self._dac_shadow.get("DAC_Narm", 2048)
        vref_mv = self.vref * 1000.0
        amp_code = (self.dither_amp_mv / vref_mv) * 4095.0 / 2.0/ 2.0 #Vpp/2/(Amp*2)

        for i in range(self.lcm_len):
            wave = amp_code * math.sin(w * i)
            if self.arm_mode == "P":
                val_p, val_n = int(p_base + wave), n_base
            elif self.arm_mode == "N":
                val_p, val_n = p_base, int(n_base + wave)
            elif self.arm_mode == "PN":
                val_p, val_n = int(p_base + wave/2.0), int(n_base - wave/2.0)
                
            self.sine_buf_p[i] = max(min(val_p, 4095), 0)
            self.sine_buf_n[i] = max(min(val_n, 4095), 0)


    def _fire_sync_timer(self, t):
        """发令枪中断回调：瞬间拉起全局心脏 Timer6"""
        machine.mem32[0x40001000] |= 1  # 设置 CEN = 1
        t.deinit()                      # 开完枪即销毁

    def read_adc_timed_multi(self, adc_name, view):

        adc = self.get_adc_object(adc_name)
        if adc:
            # 1. 设定一个 1ms 后的中断，用作延迟发令枪
            trig = pyb.Timer(8, freq=1000)
            trig.callback(self._fire_sync_timer)
            
            # 2. 阻塞调用 ADC 采样
            # 1ms 足够底层配置完所有寄存器并进入待命。
            # 1ms 后，中断触发，Timer6 开始跳动，DAC 和 ADC 绝对同频同相起飞！
            adc.read_timed(view, self.sync_timer)

    @micropython.native
    def set_lock_dither(self, active=True, restart_timer=True):
        """
        :param restart_timer: 布置完数组后，是否立刻启动 Timer。
                              为 False 时用于配合 ADC 延迟发令枪使用。
        """
        mem32 = machine.mem32
        dac_p = self.dacs.get("DAC_Parm")
        dac_n = self.dacs.get("DAC_Narm")

        # 1. 暂停全局心脏 Timer6 (CEN = 0)
        mem32[0x40001000] &= ~1
        # 清零 Timer6 计数器，保证第一枪的绝对准时
        mem32[0x40001024] = 0
        
        # 2. 暂停 DAC DMA (由于 Timer6 已停，此时物理电压死死冻结，绝不跌落)
        if dac_p: 
            mem32[0x40026088] &= ~1
        if dac_n: 
            mem32[0x400260A0] &= ~1
        
        # 死等 DMA 彻底停稳
        if dac_p: 
            while mem32[0x40026088] & 1: 
                pass
        if dac_n:
            while mem32[0x400260A0] & 1: 
                pass
            
        # 3. 在绝对安全区覆写数组
        if active:
            self._recalc_sine_buffer()
        else:
            dc_p = self._dac_shadow.get("DAC_Parm", 2048)
            dc_n = self._dac_shadow.get("DAC_Narm", 2048)
            for i in range(self.lcm_len):
                self.sine_buf_p[i] = dc_p
                self.sine_buf_n[i] = dc_n
                
        # 4. 终极对齐：暴力将 DMA 读指针(NDTR)拨回起跑线
        if dac_p: 
            mem32[0x4002608C] = self.lcm_len
        if dac_n: 
            mem32[0x400260A4] = self.lcm_len
        
        # 清除 DMA HIFCR 标志位，防止死锁
        mem32[0x4002600C] = 0x003F0FC0
        
        # 5. 重新挂载 DMA (此时依然绝对安静，因为 Timer6 还停着)
        if dac_p: 
            mem32[0x40026088] |= 1
        if dac_n:
            mem32[0x400260A0] |= 1
        
        # 6. 根据逻辑决定是否立刻开枪
        if restart_timer:
            mem32[0x40001000] |= 1

    def update_dac_bias(self, main_code):
        """专门给锁定算法使用的码值直写接口，避免电压换算导致精度丢失或越界"""
        self.current_main_code = max(min(main_code, 4095), 0)

        p_code = self._dac_shadow.get("DAC_Parm", 2048)
        n_code = self._dac_shadow.get("DAC_Narm", 2048)
        
        if self.arm_mode == "P":
            p_code = self.current_main_code
        elif self.arm_mode == "N":
            n_code = self.current_main_code
        elif self.arm_mode == "PN":
            p_code = self.current_main_code
            # PN模式下，必须保持双臂电压之和恒定，总码值为：(Bias_Amp_Volt / 2.0 / vref) * 4095
            sum_code = int((self.Bias_Amp_Volt / 2.0 / self.vref) * 4095.0)
            n_code = max(min(sum_code - p_code, 4095), 0)

        self._dac_shadow["DAC_Parm"] = p_code
        self._dac_shadow["DAC_Narm"] = n_code
        
        # 绝不调用单次 dac.write！直接瞬间刷新 DMA 正在读取的内存数组！
        self.set_lock_dither(False)

    def sync_anchor_code(self):
        current_main_code1 = 0
        if self.arm_mode in "P": 
            self.current_main_code = self._dac_shadow.get("DAC_Parm", 2048)
        elif self.arm_mode == "N":       
            self.current_main_code = self._dac_shadow.get("DAC_Narm", 2048)
        elif self.arm_mode == "PN":       
            self.current_main_code = self._dac_shadow.get("DAC_Parm", 2048)
            current_main_code1 = self._dac_shadow.get("DAC_Narm", 2048)
        log(INFO, f"current_main_code updated:{self.current_main_code, current_main_code1}")           

    # =================================================
    # Pool Data Export & Noise Capture
    # =================================================
    def capture_noise_task(self, adc_name, source):
        adc = self.get_adc_object(adc_name)
        if not adc: return
        
        buf = self.global_adc_pool
        total_points = len(buf) 
        
        # 【极致优化】：抛弃所有软件原生循环！
        # 直接让 ADC 挂载到全局的 sync_timer 上，利用底层 DMA 一口气灌满 4320 个点。
        # 耗时精确等于 4320 / 19200 = 225 毫秒！
        adc.read_timed(buf, self.sync_timer)
                
        if self.handler: 
            self.handler._write_response(f"NOISE_DATA_START\r\n", source)
            
        self.task_queue.add_task(self._export_chunk_task, 0, total_points, source)

    def export_pool_data_task(self, points, source):
        max_points = len(self.global_adc_pool)
        points = max(1, min(points, max_points))
        if self.handler:
            self.handler._write_response(f"Export_Sampling_DATA_START\r\n", source)
        log(INFO, f"Starting asynchronous export of {points} points...")
        self.task_queue.add_task(self._export_chunk_task, 0, points, source)
    
    def _export_chunk_task(self, start_idx, total_points, source):
        chunk_size = 20
        end_idx = min(start_idx + chunk_size, total_points)

        for i in range(start_idx, end_idx):
            voltage = (self.global_adc_pool[i] / 4095.0) * self.vref
            if self.handler:
                self.handler._write_response(f"{voltage:.4f}\r\n", source)

        if end_idx < total_points:
            self.task_queue.add_task(self._export_chunk_task, end_idx, total_points, source)
        else:
            gc.collect()
            if self.handler:
                self.handler._write_response(f"Export_DATA_END\r\n", source)
            log(INFO, "Pool data export complete.")

    # =================================================
    # Legacy Wrappers (Unchanged)
    # =================================================
    def set_handler(self, handler): 
        self.handler = handler

    def _read_adc_avg(self, adc):
        if adc is None: return None
        total, count = 0, 0
        try:
            for _ in range(self.ADC_SAMPLE_TIMES):
                total += adc.read()
                count += 1
                pyb.udelay(self.ADC_SAMPLE_DELAY_US)
        except Exception as e: return None
        return total // count if count else None
    
    def read_all_adc_task(self, source):
        results = [(idx, name, self._read_adc_avg(adc)) for idx, name, adc in self.adc_list]
        if self.handler: self.task_queue.add_task(self.handler.handle_adc_data, results, source)

    def read_adc_task(self, ch, source):
        target = next((t for t in self.adc_list if t[0] == ch or t[1] == ch), None)
        if not target: return
        idx, name, adc = target
        val = self._read_adc_avg(adc)
        if self.handler: self.task_queue.add_task(self.handler.handle_adc_data, [(idx, name, val)], source)
    
    def get_adc_object(self, ch):
        target = next((t for t in self.adc_list if t[0] == ch or t[1] == ch), None)
        return target[2] if target else None
    
    def set_dac_task(self, ch, value, source):
        targets = self.dac_list[:] if ch == "ALL" else [t for t in self.dac_list if t[0] == ch or t[1] == ch]
        for idx, name, dac in targets:
            if dac:
                try:
                    # dac.write(value)
                    # self._dac_shadow[name] = value

                    # 1. 永远先更新影子寄存器
                    self._dac_shadow[name] = value
                    # 2. 【核心修复】：判断该通道是否已被无缝 DMA 全局接管？
                    if self.dma_engine_started and name in ["DAC_Parm", "DAC_Narm"]:
                        # DMA存在则直接呼叫内存刷新，让DMA 瞬间读到新 DC 值
                        self.set_lock_dither(False) 
                    else:
                        # 对于板子上未被 DMA 接管的普通低频 DAC 通道，保持正常软件写入
                        dac.write(value)

                    if source and self.handler:
                        self.task_queue.add_task(self.handler.handle_dac_Set_Single, idx, name, value, source)
                except Exception as e: pass

    def get_dac_object(self, ch):
        targets = [t for t in self.dac_list if t[0] == ch or t[1] == ch]
        return targets[0][2] if targets else None

    def read_dac_task(self, ch, source):
        targets = self.dac_list[:] if ch == "ALL" else [t for t in self.dac_list if t[0] == ch or t[1] == ch]
        results = [(idx, name, self._dac_shadow.get(name)) for idx, name, _ in targets]
        if self.handler:
            if len(results) == 1:
                self.task_queue.add_task(self.handler.handle_dac_Read_Single, results[0][0], results[0][1], results[0][2], source)
            else:
                self.task_queue.add_task(self.handler.handle_dac_Read_All, results, source)

    def read_adc_snapshot(self):
        return [(idx, name, self._read_adc_avg_Sweep(adc, ADC_SAMPLE_TIMES_ForSweep, ADC_SAMPLE_DELAY_US_ForSweep)) for idx, name, adc in self.SWeep_MonitorADC_list]

    def _read_adc_avg_Sweep(self, adc, times, delay_us):
        if adc is None: return None
        total, count = 0, 0
        try:
            for _ in range(times):
                total += adc.read()
                count += 1
                pyb.udelay(delay_us)
        except Exception as e: return None
        return total // count if count else None
    
    def read_adc_sync(self, name, for_sweep=False):
        adc = self.adcs.get(name)
        if adc is None: return None
        if for_sweep: return self._read_adc_avg_Sweep(adc, ADC_SAMPLE_TIMES_ForSweep, ADC_SAMPLE_DELAY_US_ForSweep)
        return self._read_adc_avg(adc)