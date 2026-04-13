import pyb
import math
from array import array

# --- Engineering Constants ---
# 19.2kHz, 1440 points = 60 cycles @ 800Hz
ADC_LEN = const(1440)

class Lock_TimeMUX:
    """
    High-Rigidity Locking Engine ported from ADuCM410 Mode B.
    Features: Orthogonal Dot-product extraction, 1f/2f Harmonic Analysis, 
    and 3-stage PI Gain Scheduling.
    """
    def __init__(self, ad_da, uart_master):
        self.ad_da = ad_da
        self.uart = uart_master
        self.running = False
        
        # --- Critical Target Names (Must match config.py) ---
        # The ADC channel name used for reading the Photodiode / Power Meter
        self.adc_target_name = "AC_Lock" 
        
        # --- Pre-allocated Memory Pools (Zero-GC during lock) ---
        self.adc_buf = array('f', [0.0] * ADC_LEN)
        self.ref_1f = array('f', [0.0] * ADC_LEN)
        self.ref_2f = array('f', [0.0] * ADC_LEN)
        
        # --- Calibration Constants (Updated by CALIB command) ---
        self.phase_1f = 68.52  # Default Sine phase offset
        self.phase_2f = -5.2   # Default Cosine 2f phase offset
        
        # --- PI & Defense Parameters (Dict for easy tuning) ---
        self.params = {
            'errLpfAlpha': 0.85, 'i2Th': 0.00008, 'jumpErr': 0.01,
            'thFast': 0.004, 'thMid': 0.001, 'stepFast': 40,
            'kMid': 4000.0, 'kFine': 3000.0, 'kInt': 1000.0,
            'intLimit': 0.001, 'deadZone': 0.0001, 'stepMax': 80
        }

        self.bias_p = [2048] * 6      # Current DAC codes for 6 channels
        self.err_int = [0.0] * 6      # Integral accumulators
        self.last_err = [0.0] * 6     # LPF history
        self.targets = [1] * 6        # 1:MAX, 0:MIN, 2:QUAD
        self.iters_rem = [0] * 6      # Remaining iteration counter
        
        self.active_ch = 0
        self.stage = "IDLE"
        self._generate_refs()

    def _generate_refs(self):
        """Update internal reference tables with phase compensation."""
        w1 = 2 * math.pi * 800 / 19200
        w2 = 2 * math.pi * 1600 / 19200
        ph1 = math.radians(self.phase_1f)
        ph2 = math.radians(self.phase_2f)
        for i in range(ADC_LEN):
            self.ref_1f[i] = math.sin(w1 * i + ph1)
            self.ref_2f[i] = math.cos(w2 * i + ph2)

    def start_lock(self, ch_idx, target, iters=100):
        """API to trigger locking via UART/USB commands."""
        if 0 <= ch_idx < 6:
            self.targets[ch_idx] = target
            self.iters_rem[ch_idx] = iters
            self.running = True
            self.active_ch = ch_idx
            self.stage = "START"
            self.uart.write(f"ACK: Locking CH{ch_idx} to TGT={target} for {iters} iters\n".encode('utf-8'))

    def stop_lock(self):
        """Force stop all locking activities."""
        self.running = False
        self.stage = "IDLE"
        for i in range(6): 
            self.iters_rem[i] = 0
        self.uart.write("ACK: Mode B Lock Stopped\n".encode('utf-8'))

    def calibrate_phase(self, base_freq=800):
        """Single-point DFT to calibrate phase offsets (The 410 logic)."""
        mean = sum(self.adc_buf) / ADC_LEN
        w1 = 2 * math.pi * base_freq / 19200
        w2 = 2 * math.pi * (base_freq * 2) / 19200
        i1f, q1f, i2f, q2f = 0.0, 0.0, 0.0, 0.0
        
        for i in range(ADC_LEN):
            v = self.adc_buf[i] - mean
            i1f += v * math.cos(w1 * i)
            q1f += v * -math.sin(w1 * i)
            i2f += v * math.cos(w2 * i)
            q2f += v * -math.sin(w2 * i)
        
        p1 = math.degrees(math.atan2(q1f, i1f)) + 90.0
        self.phase_1f = p1 if p1 <= 180 else p1 - 360
        self.phase_2f = math.degrees(math.atan2(q2f, i2f))
        
        self._generate_refs() # Refresh mathematical reference arrays
        self.uart.write(f"CALIB_DONE:1F={self.phase_1f:.2f},2F={self.phase_2f:.2f}\n".encode('utf-8'))

    def process(self):
        """Non-blocking FSM - Must be called frequently by TaskQueue/Poll."""
        if not self.running: return

        if self.stage == "START":
            ch = self.active_ch
            if self.iters_rem[ch] <= 0:
                self.active_ch = (self.active_ch + 1) % 6
                if all(it <= 0 for it in self.iters_rem): 
                    self.running = False
                return
            
            # Non-blocking trigger of the ADC capture (Specify the ADC name)
            self.ad_da.read_adc_timed_multi(self.adc_target_name, self.adc_buf)
            self.start_ticks = pyb.millis()
            self.stage = "WAIT"

        elif self.stage == "WAIT":
            # 1440 points at 19.2kHz = 75ms. We wait 80ms to be safe.
            if pyb.elapsed_millis(self.start_ticks) > 80: 
                self.stage = "CALC"

        elif self.stage == "CALC":
            ch = self.active_ch
            # A. DC Removal
            mean = sum(self.adc_buf) / ADC_LEN
            
            # B. Coherent Demodulation
            i1, i2 = 0.0, 0.0
            for i in range(ADC_LEN):
                v = self.adc_buf[i] - mean
                i1 += v * self.ref_1f[i]
                i2 += v * self.ref_2f[i]
            i1 /= (ADC_LEN/2)
            i2 /= (ADC_LEN/2)
            
            # C. Mode B Jump/Escape Logic & PI
            err = 0.0
            if self.targets[ch] == 1: # Target: MAX
                err = i1 if i2 <= self.params['i2Th'] else -self.params['jumpErr']
            elif self.targets[ch] == 0: # Target: MIN
                err = -i1 if i2 >= -self.params['i2Th'] else self.params['jumpErr']
            
            # LPF Filter for Error
            smooth_err = self.params['errLpfAlpha'] * err + (1 - self.params['errLpfAlpha']) * self.last_err[ch]
            self.last_err[ch] = smooth_err
            abs_err = abs(smooth_err)
            
            # Gain Scheduling
            step = 0
            if abs_err > self.params['thFast']:
                self.err_int[ch] = 0.0 # Reset Integral
                step = self.params['stepFast'] if smooth_err > 0 else -self.params['stepFast']
            else:
                self.err_int[ch] = max(min(self.err_int[ch] + smooth_err, self.params['intLimit']), -self.params['intLimit'])
                step = int(smooth_err * self.params['kFine'] + self.err_int[ch] * self.params['kInt'])
            
            # D. Update Hardware (Seamless memory-based update)
            self.bias_p[ch] = max(min(self.bias_p[ch] + step, 4095), 0)
            
            # Look up DAC name from ad_da module safely based on channel index
            dac_name = self.ad_da.dac_list[ch][1] if ch < len(self.ad_da.dac_list) else None
            if dac_name:
                self.ad_da.update_dac_bias(dac_name, self.bias_p[ch])
            
            self.iters_rem[ch] -= 1
            if self.iters_rem[ch] % 5 == 0:
                self.uart.write(f"LOCK:CH{ch},BIAS={self.bias_p[ch]},I1={i1:.5f},I2={i2:.5f}\n".encode('utf-8'))
            
            self.stage = "START"