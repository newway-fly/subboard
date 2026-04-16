"""
locking.py
-------------------------
High-Rigidity Locking Engine ported from ADuCM410 Mode B.
[Subboard Version - Single Channel Autonomous FSM]
- Evaluates 800Hz / 1600Hz dot products directly.
- Triggers hardware Dither output strictly on start/stop.
- PI controller relies on in-place DAC bias modifications.
- [NEW] Uses MemoryView to share the static global_adc_pool natively.
"""

import pyb
import math
from array import array

# 19.2kHz, 1440 points = 60 cycles @ 800Hz
ADC_LEN = const(1440)

class Lock_TimeMUX:
    def __init__(self, ad_da, uart_master):
        self.ad_da = ad_da
        self.uart = uart_master
        self.running = False
        
        # Internal configuration
        self.adc_target_name = "AC_Lock" 
        
        # [NEW] Zero-Allocation Memory Aliasing
        # Maps a view exclusively to the first 1440 elements of the global memory pool.
        # This allows locking and noise-capture to safely share exactly the same SRAM space.
        self.adc_view = memoryview(self.ad_da.global_adc_pool)[0:ADC_LEN]
        
        self.ref_1f = array('f', [0.0] * ADC_LEN)
        self.ref_2f = array('f', [0.0] * ADC_LEN)
        
        # Hardware calibration phase
        self.phase_1f = 68.52  
        self.phase_2f = -5.2   
        
        # Golden PI Parameters from C code
        self.params = {
            'errLpfAlpha': 0.85, 'i2Th': 0.00008, 'jumpErr': 0.01,
            'thFast': 0.004, 'thMid': 0.001, 'stepFast': 40,
            'kMid': 4000.0, 'kFine': 3000.0, 'kInt': 1000.0,
            'intLimit': 0.001, 'deadZone': 0.0001, 'stepMax': 80
        }

        # Single Engine State
        self.err_int = 0.0
        self.last_err = 0.0
        self.last_i2 = 0.0
        self.target = 1        # 1:MAX, 0:MIN, 2:QUAD
        self.iters_rem = 0
        self.quad_slope = 1
        
        # DAC Update Enable Flag (Closed Loop vs Open Loop)
        self.update_dac_en = True
        
        self.stage = "IDLE"
        self._generate_refs()

    def _generate_refs(self):
        """Pre-compute mathematical NCO references."""
        w1 = 2 * math.pi * 800 / 19200
        w2 = 2 * math.pi * 1600 / 19200
        ph1 = math.radians(self.phase_1f)
        ph2 = math.radians(self.phase_2f)
        for i in range(ADC_LEN):
            self.ref_1f[i] = math.sin(w1 * i + ph1)
            self.ref_2f[i] = math.cos(w2 * i + ph2)

    def start_lock(self, target, iters=100):
        """API to arm and trigger the physical locking engine."""
        self.target = target
        self.iters_rem = iters
        self.err_int = 0.0
        self.last_err = 0.0
        self.last_i2 = 0.0
        
        # Instruct AD/DA module to physically start emitting the Sine Wave
        self.ad_da.set_lock_dither(True)
        
        self.running = True
        self.stage = "START"
        self.uart.write(f"ACK: Lock Started. TGT={target}, Iters={iters}\n".encode('utf-8'))

    def stop_lock(self):
        """Force stop all locking calculations and physical outputs."""
        self.running = False
        self.stage = "IDLE"
        self.iters_rem = 0
        
        # Instruct AD/DA module to halt Dither and restore pure DC bias
        self.ad_da.set_lock_dither(False)
        self.uart.write("ACK: Mode B Lock Stopped\n".encode('utf-8'))

    def set_update_dac(self, enable):
        """
        Dynamically enable or disable DAC feedback (Closed vs Open loop).
        Clears integral windup immediately when disabling feedback.
        """
        self.update_dac_en = enable
        if not enable:
            self.err_int = 0.0

    def calibrate_phase(self, base_freq=800):
        """Standard DFT Phase Extractor based on memoryview data."""
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
        self.phase_2f = math.degrees(math.atan2(q2f, i2f))
        
        self._generate_refs()
        self.uart.write(f"CALIB_DONE:1F={self.phase_1f:.2f},2F={self.phase_2f:.2f}\n".encode('utf-8'))

    def process(self):
        """Non-blocking FSM heartbeat for TaskQueue execution."""
        if not self.running: return

        if self.stage == "START":
            if self.iters_rem <= 0:
                self.stop_lock()
                return
            
            # Non-blocking trigger of the ADC capture targeting the shared memoryview
            self.ad_da.read_adc_timed_multi(self.adc_target_name, self.adc_view, timer_id=7)
            self.start_ticks = pyb.millis()
            self.stage = "WAIT"

        elif self.stage == "WAIT":
            # 1440 points at 19.2kHz = 75ms. Wait 80ms for safety.
            if pyb.elapsed_millis(self.start_ticks) > 80: 
                self.stage = "CALC"

        elif self.stage == "CALC":
            # A. DC Removal (Compatible with unsigned short memoryview)
            mean = sum(self.adc_view) / ADC_LEN
            
            # B. Coherent Demodulation
            acc1, acc2, ref_energy = 0.0, 0.0, 0.0
            for i in range(ADC_LEN):
                # On-the-fly math yields float automatically
                v = self.adc_view[i] - mean
                r1 = self.ref_1f[i]
                r2 = self.ref_2f[i]
                acc1 += v * r1
                acc2 += v * r2
                ref_energy += r1 * r1
                
            if ref_energy < 1e-6: ref_energy = 1.0
            i1 = acc1 / ref_energy
            i2 = acc2 / ref_energy
            
            # C. Jump Escape Defense & Error Mapping
            error_signal = 0.0
            if self.target == 1: # MAX
                error_signal = -self.params['jumpErr'] if i2 > self.params['i2Th'] else i1
            elif self.target == 0: # MIN
                error_signal = self.params['jumpErr'] if i2 < -self.params['i2Th'] else -i1
            elif self.target == 2: # QUAD
                error_signal = i2 if self.quad_slope > 0 else -i2
                error_signal *= 1.5

            # LPF Filter
            alpha = self.params['errLpfAlpha']
            smooth_err = alpha * error_signal + (1 - alpha) * self.last_err
            self.last_err = smooth_err
            self.last_i2 = alpha * i2 + (1 - alpha) * self.last_i2
            
            abs_err = abs(smooth_err)
            step = 0
            
            current_code = self.ad_da.current_main_code
            new_code = current_code
            
            # D. Conditional PI Execution & DAC Update
            if self.update_dac_en:
                if abs_err > self.params['thFast']:
                    self.err_int = 0.0
                    step = self.params['stepFast'] if smooth_err > 0 else -self.params['stepFast']
                elif abs_err > self.params['thMid']:
                    self.err_int = 0.0
                    step = int(smooth_err * self.params['kMid'])
                else:
                    self.err_int += smooth_err
                    if self.err_int > self.params['intLimit']: self.err_int = self.params['intLimit']
                    if self.err_int < -self.params['intLimit']: self.err_int = -self.params['intLimit']
                    pi_step = int(smooth_err * self.params['kFine'] + self.err_int * self.params['kInt'])
                    step = 0 if abs_err <= self.params['deadZone'] else pi_step
                        
                # Hard Clamping
                if step > self.params['stepMax']: step = self.params['stepMax']
                if step < -self.params['stepMax']: step = -self.params['stepMax']
                
                # Write back to hardware AD_DA manager (updates mathematical relationship)
                new_code = max(min(current_code + step, 4095), 0)
                self.ad_da.update_dac_bias(new_code)
            else:
                self.err_int = 0.0
            
            # E. Telemetry Log
            self.iters_rem -= 1
            if self.iters_rem % 5 == 0:
                log_str = f"LOCK: C={new_code}, I1={i1:.5f}, I2={i2:.5f}, Err={smooth_err:.5f}"
                if not self.update_dac_en:
                    log_str += ", UPDATE_DAC OFF"
                self.uart.write((log_str + "\n").encode('utf-8'))
            
            self.stage = "START"