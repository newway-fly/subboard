"""
locking.py
-------------------------
High-Rigidity Locking Engine ported from ADuCM410 Mode B.
[Subboard Version - Single Channel Autonomous FSM]
- Evaluates 800Hz / 1600Hz dot products directly.
- Triggers hardware Dither output strictly on start/stop.
- PI controller relies on in-place DAC bias modifications.
- [NEW] Implements Atomic Execution (IRQ Disable) for absolute synchronization.
- [NEW] Adds non-blocking DAC Settle State (WAIT_DAC).
"""

import pyb
import math
import machine
from array import array
from log_system import log, INFO, WARN, DEBUG, ERROR

# 19.2kHz, 1440 points = 60 cycles @ 800Hz
# ADC_LEN = const(1440)

ADC_LEN = const(1440)     # 75ms，用于纯净算法解调的黄金数据 (60个周期)
WARMUP_LEN = const(72)    # 3.75ms，用于吸收物理电路瞬态的“垃圾桶” (2个周期)
TOTAL_LEN = const(1512)   # 78.75ms，DMA 实际搬运的总长度

class Lock_TimeMUX:
    def __init__(self, ad_da, uart_master, handler):
        self.ad_da = ad_da
        self.uart = uart_master
        self.running = False
        self.handler = handler
        
        self.adc_target_name = "AC_Lock" 
        self.BiasPoint = ['MIN','MAX','QUAD']
  

        # 【总视图】：交给底层的 ad_da.read_adc_timed_multi 使用
        # 让 DMA 一口气搬运 1512 个点
        # self.adc_view_total = memoryview(self.ad_da.global_adc_pool)[0:TOTAL_LEN]
        # 【净视图】：交给 process 和 calibrate_phase 里的算法使用
        # 巧妙地切片：跳过前 72 个带有模拟电路畸变的点，只保留最后绝对稳定的 1440 个点！
        # self.adc_view = memoryview(self.ad_da.global_adc_pool)[WARMUP_LEN:ADC_LEN]

        # 【净视图】：交给 process 和 calibrate_phase 里的算法使用
        self.adc_view = memoryview(self.ad_da.global_adc_pool)[0:ADC_LEN]


        self.ref_1f = array('f', [0.0] * ADC_LEN)
        self.ref_2f = array('f', [0.0] * ADC_LEN)
        
        # Hardware calibration phase 1F=32.74,2F-147.26（已根据 180 度翻转修正 2F）
        self.phase_1f = 124.52  
        self.phase_2f = -141.48

        self.params = {
            'errLpfAlpha': 0.85, 
            'i2Th': 0.001,       
            'jumpErr': 0.07,     
            
            'thFast': 0.07,      
            'thMid': 0.01,       
            
            'stepFast': 15,      
            'kMid': 250.0,       
            'kFine': 150.0,      
            'kInt': 100.0,       # [微调1]：略微增强积分力度
            
            # [核心微调2]：放大积分限幅！
            # 允许积攒最大 0.015 的误差，0.015 * 100 = 1.0。
            # 这保证了积分器在关键时刻，绝对能爆发出 1 个码值的力量，冲破 int() 的 0 截断！
            'intLimit': 0.015,   
            'deadZone': 0.001,  # [微调3]：收紧死区，鼓励系统往极值点钻
            'stepMax': 15        
        }

        self.err_int = 0.0
        self.last_err = 0.0
        self.target = 1        
        self.iters_rem = 0
        self.quad_slope = 1
        
        self.update_dac_en = True
        
        # DAC Settle Time Configuration
        self.dac_settle_ms = 20     # 每次锁定迭代后的硬件物理稳定延时
        self.settle_ticks = 0       
        
        self.stage = "IDLE"
        self._generate_refs()

    def _generate_refs(self):
        w1 = 2 * math.pi * 800 / 19200
        w2 = 2 * math.pi * 1600 / 19200
        ph1 = math.radians(self.phase_1f)
        ph2 = math.radians(self.phase_2f)
        for i in range(ADC_LEN):
            self.ref_1f[i] = math.sin(w1 * i + ph1)
            self.ref_2f[i] = math.cos(w2 * i + ph2)

    def start_lock(self, target, iters=100):
        self.target = target
        self.iters_rem = iters
        self.err_int = 0.0
        self.last_err = 0.0

        self.handler._write_response(f" LOCK_B START TGT={self.BiasPoint[self.target]}, Anchor={self.ad_da.current_main_code}\r\n", 'uart')

        self.ad_da.sync_anchor_code()
        self.running = True
        self.stage = "START_DITHER"

    def stop_lock(self):
        self.running = False
        self.stage = "IDLE"
        self.iters_rem = 0
        self.ad_da.set_lock_dither(False)

    def set_update_dac(self, enable):
        self.update_dac_en = enable
        if not enable:
            self.err_int = 0.0

    def set_settle_time(self, ms):
        self.dac_settle_ms = ms

    def calibrate_phase(self, base_freq=800):
        mean = sum(self.adc_view) / ADC_LEN
        w1 = 2 * math.pi * base_freq / 19200
        w2 = 2 * math.pi * (base_freq * 2) / 19200
        i1f, q1f, i2f, q2f = 0.0, 0.0, 0.0, 0.0
        
        for i in range(ADC_LEN):
            v = self.adc_view[i] - mean
            i1f += v * math.cos(w1 * i)
            q1f += v * -math.sin(w1 * i)
            i2f += v * math.cos(w2 * i)
            q2f += v * -math.sin(w2 * i)
        

        p1 = math.degrees(math.atan2(q1f, i1f)) + 90.0
        self.phase_1f = p1 if p1 <= 180 else p1 - 360
        
        # 计算 2F 相位并强制减去 180 度，维持标准化定义
        p2 = math.degrees(math.atan2(q2f, i2f)) - 180.0
        self.phase_2f = p2 if p2 >= -180 else p2 + 360

        self._generate_refs()
        self.handler._write_response(f"CALIB_DONE:1F={self.phase_1f:.2f},2F={self.phase_2f:.2f}\r\n", 'uart')
        self.handler._write_response(f"CALIB_DONE:1F={self.phase_1f:.2f},2F={self.phase_2f:.2f}\r\n",'usb')

    def process(self):
        if not self.running: return
        
        if self.stage == "START_DITHER":
            if self.iters_rem <= 0:
                self.stop_lock()
                return

            # self.ad_da.start_dma_engine_once()
            # 暂停时间，布置波形，不开枪
            self.ad_da.set_lock_dither(True, restart_timer=False)
            
            # ADC 就位，1ms 延迟中断开枪，绝对同相采样
            # self.ad_da.read_adc_timed_multi(self.adc_target_name, self.adc_view_total)
            self.ad_da.read_adc_timed_multi(self.adc_target_name, self.adc_view)
            pyb.udelay(125)
            # 切回直流，立刻开枪
            self.ad_da.set_lock_dither(False, restart_timer=True)
            
            self.stage = "CALC"

        elif self.stage == "CALC":

            mean = sum(self.adc_view) / ADC_LEN
            
            acc1, acc2, ref_energy = 0.0, 0.0, 0.0
            for i in range(ADC_LEN):
                v = self.adc_view[i] - mean
                r1 = self.ref_1f[i]
                r2 = self.ref_2f[i]
                acc1 += v * r1
                acc2 += v * r2
                ref_energy += r1 * r1
                
            if ref_energy < 1e-6: ref_energy = 1.0
            
            scale_factor = self.ad_da.vref / 4095.0
            i1 = (acc1 / ref_energy) * scale_factor
            i2 = (acc2 / ref_energy) * scale_factor
            
            # C. Jump Escape Defense & Error Mapping (标准化后的 I2 逻辑)
            error_signal = 0.0
            
            if self.target == 1: # MAX 点
                # 正常情况 I2 为负。若 I2 > i2Th，说明误入 MIN 谷底，强制输出负向逃逸信号
                # error_signal = -self.params['jumpErr'] if i2 >= self.params['i2Th'] else i1
                error_signal = i1
                
            elif self.target == 0: # MIN 点
                # 正常情况 I2 为正。若 I2 < -i2Th，说明误入 MAX 山峰，强制输出正极逃逸信号
                # error_signal = self.params['jumpErr'] if i2 <= -self.params['i2Th'] else -i1
                if i2 <= -self.params['i2Th']:
                    error_signal = i1
                else:
                    error_signal = -i1
           
                
            elif self.target == 2: # QUAD 点
                error_signal = i2 if self.quad_slope > 0 else -i2
                error_signal *= 1.5


            alpha = self.params['errLpfAlpha']
            smooth_err = alpha * error_signal + (1 - alpha) * self.last_err
            self.last_err = smooth_err

            abs_err = abs(smooth_err)
            step = 0
            
            current_code = self.ad_da.current_main_code
            new_code = current_code
            
            if self.update_dac_en:
                if abs_err > self.params['thFast']:
                    self.err_int = 0.0
                    step = self.params['stepFast'] if smooth_err > 0 else -self.params['stepFast']
                elif abs_err > self.params['thMid']:
                    self.err_int = 0.0
                    step = int(smooth_err * self.params['kMid'])
                else:
                    self.err_int += smooth_err
                    if self.err_int > self.params['intLimit']: 
                        self.err_int = self.params['intLimit']
                    if self.err_int < -self.params['intLimit']: 
                        self.err_int = -self.params['intLimit']

                    pi_step = int(smooth_err * self.params['kFine'] + self.err_int * self.params['kInt'])
                    step = 0 if abs_err <= self.params['deadZone'] else pi_step
                        
                if step > self.params['stepMax']: 
                    step = self.params['stepMax']
                if step < -self.params['stepMax']: 
                    step = -self.params['stepMax']
                
                new_code = max(min(current_code + step, 4095), 0)
                self.ad_da.update_dac_bias(new_code)
            else:
                new_code = current_code
                self.err_int = 0.0
            
            self.iters_rem -= 1
            log_str = f"LOCK: C={new_code}, I1={i1:.5f}V, I2={i2:.5f}V, Err={smooth_err:.5f}"
            if not self.update_dac_en:
                log_str += ", UPDATE_DAC OFF"

            self.handler._write_response(f" {log_str} \r\n", 'uart')
            log(INFO, f"{log_str}")

            # =================================================
            # 【第三道防线：物理稳定延时】
            # 进入 WAIT_DAC 状态，让光路有充足时间适应新的直流电压
            # =================================================
            self.settle_ticks = pyb.millis()
            self.stage = "WAIT_DAC"

        elif self.stage == "WAIT_DAC":
            if pyb.elapsed_millis(self.settle_ticks) >= self.dac_settle_ms:
                self.stage = "START_DITHER"