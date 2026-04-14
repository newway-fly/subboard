"""
ad_da.py
-------------------------
Unified AD/DA Module with Circular DMA Dither and Arm Topology Control.
[OPTIMIZED FOR SUBBOARD MODE B]
- Supports P, N, and PN Arm configurations.
- Calculates 800Hz Sine wave array in advance without outputting.
- Starts/Stops physical Dither only when instructed by the Locking engine.
"""

import pyb
import math
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
        self.arm_mode = "P"                 # "P", "N", or "PN"
        self.fixed_unused_dc = Bias_Arm_Volt # Fixed voltage for idle arm
        self.bias_sum = Bias_Amp_Volt       # Total voltage for PN mode
        self.current_main_code = 2048       # Current active DC bias code
        
        # Dither parameters
        self.dither_amp = 30                # Amplitude in DAC codes
        self.dither_active = False          # Physical output flag
        self.lcm_len = 48                   # 48 points for 800Hz @ 19.2kHz
        
        # Pre-allocate 1D Memory Pools to avoid GC
        self.sine_buf_p = array('H', [0] * self.lcm_len)
        self.sine_buf_n = array('H', [0] * self.lcm_len)
        self.tim_dac = None

    # =================================================
    # [Mode B] Arm Topology & Dither Engine
    # =================================================
    def init_arm_config(self, mode, v1=None, v2=None):
        """
        Configures the P/N arm relationships.
        V1: Initial voltage for active arm. V2: Bias sum for PN mode.
        """
        self.arm_mode = mode.upper()
        
        if self.arm_mode in ["P", "N"]:
            v_init = v1 if v1 is not None else 1.0
            self.fixed_unused_dc = Bias_Arm_Volt
        elif self.arm_mode == "PN":
            v_init = v1 if v1 is not None else 0.5
            self.bias_sum = v2 if v2 is not None else Bias_Amp_Volt
        else:
            return

        # Convert voltage to DAC code and update output immediately
        initial_code = int((v_init / self.vref) * 4095)
        self.update_dac_bias(initial_code)
        log(INFO, f"ARM Configured: {self.arm_mode}, Initial Code: {initial_code}")

    def init_dither(self, amp_code):
        """
        Sets the amplitude and pre-calculates the array. 
        Does NOT start physical output.
        """
        self.dither_amp = amp_code
        self._recalc_sine_buffer()
        log(INFO, f"Dither pre-calculated with Amp: {amp_code} Codes")

    def _recalc_sine_buffer(self):
        """
        Calculates the actual output arrays based on arm mode and DC bias.
        """
        w = 2 * math.pi * 800 / 19200
        p_base = self._dac_shadow.get("DAC_Parm", 2048)
        n_base = self._dac_shadow.get("DAC_Narm", 2048)
        amp = self.dither_amp
        
        for i in range(self.lcm_len):
            wave = amp * math.sin(w * i)
            # Apply dither strictly based on arm topology
            if self.arm_mode == "P":
                val_p = int(p_base + wave)
                val_n = n_base
            elif self.arm_mode == "N":
                val_p = p_base
                val_n = int(n_base + wave)
            elif self.arm_mode == "PN":
                # In Push-Pull, usually Dither is applied anti-phase or to one arm.
                # Here we apply it to Parm, and keep Narm steady, or you can invert it.
                # Defaulting to apply only to P-arm for simplicity in PN mode as requested.
                val_p = int(p_base + wave)
                val_n = n_base
                
            self.sine_buf_p[i] = max(min(val_p, 4095), 0)
            self.sine_buf_n[i] = max(min(val_n, 4095), 0)

    def set_lock_dither(self, active=True):
        """
        Starts or Stops the circular DMA output physically.
        """
        dac_p = self.dacs.get("DAC_Parm")
        dac_n = self.dacs.get("DAC_Narm")
        
        if active:
            # Ensure buffer is up-to-date before launching
            self._recalc_sine_buffer()
            self.dither_active = True
            if self.tim_dac is None:
                self.tim_dac = pyb.Timer(6, freq=19200)
            
            # Start Circular DMA for both arms (even if one is flat DC)
            if dac_p: dac_p.write_timed(self.sine_buf_p, self.tim_dac, mode=pyb.DAC.CIRCULAR)
            if dac_n: dac_n.write_timed(self.sine_buf_n, self.tim_dac, mode=pyb.DAC.CIRCULAR)
            log(INFO, "Hardware Dither DMA Started.")
        else:
            self.dither_active = False
            if self.tim_dac:
                self.tim_dac.deinit()
                self.tim_dac = None
            # Restore pure DC landing pads
            if dac_p: dac_p.write(self._dac_shadow.get("DAC_Parm", 2048))
            if dac_n: dac_n.write(self._dac_shadow.get("DAC_Narm", 2048))
            log(INFO, "Hardware Dither DMA Stopped. Restored pure DC.")

    def update_dac_bias(self, new_main_code):
        """
        Updates the mathematical DC relationship between P and N arms.
        Modifies memory in-place for zero-glitch tuning during locking.
        """
        self.current_main_code = new_main_code
        v_main = (new_main_code / 4095.0) * self.vref
        
        v_p, v_n = 0.0, 0.0
        if self.arm_mode == "P":
            v_p, v_n = v_main, self.fixed_unused_dc
        elif self.arm_mode == "N":
            v_p, v_n = self.fixed_unused_dc, v_main
        elif self.arm_mode == "PN":
            v_p = v_main
            v_n = max(0, self.bias_sum - v_p)
            
        p_code = int((v_p / self.vref) * 4095)
        n_code = int((v_n / self.vref) * 4095)
        
        self._dac_shadow["DAC_Parm"] = max(min(p_code, 4095), 0)
        self._dac_shadow["DAC_Narm"] = max(min(n_code, 4095), 0)
        
        if self.dither_active:
            self._recalc_sine_buffer()
        else:
            # Direct static write if not sweeping
            if self.dacs.get("DAC_Parm"): self.dacs["DAC_Parm"].write(self._dac_shadow["DAC_Parm"])
            if self.dacs.get("DAC_Narm"): self.dacs["DAC_Narm"].write(self._dac_shadow["DAC_Narm"])

    def read_adc_timed_multi(self, adc_name, buf, timer_id=8, fs=19200):
        """Hardware-timed synchronous ADC multi-read."""
        adc = self.get_adc_object(adc_name)
        if adc:
            try:
                tim_adc = pyb.Timer(timer_id, freq=fs)
                pyb.ADC.read_timed_multi((adc,), (buf,), tim_adc)
            except Exception as e:
                log(INFO, f"ADC Multi-read failed: {e}")

    # =================================================
    # Legacy TaskQueue Wrappers (Unchanged)
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
                    dac.write(value)
                    self._dac_shadow[name] = value
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
        if for_sweep:
            return self._read_adc_avg_Sweep(adc, ADC_SAMPLE_TIMES_ForSweep, ADC_SAMPLE_DELAY_US_ForSweep)
        return self._read_adc_avg(adc)