# --------------------------------------------------
# Sweep 扫描模块（Timer 驱动 + 非阻塞状态机）
# --------------------------------------------------

import pyb,gc
from config import MCU_ADDA_Vref, Volt_Bias_Amp

class Sweep:
    """
    Sweep Scanning Module (Slave‑Only)
    Features:

    - Timer‑driven cadence (truly non‑blocking)
    - TaskQueue executes the core logic
    - Supports immediate STOP response
    - Fully implements MODE 1 / 2 / 3
    - After the scan completes, data is forwarded in one batch
    """

    # ===================== Scan Mode =====================
    
    MODE_PARM_SCAN  = 1     # DAC_Parm scanning, DAC_Narm fixed
    MODE_NARM_SCAN  = 2     # DAC_Narm scanning, DAC_Parm fixed
    MODE_FORMULA    = 3     # DAC_Parm scanning, DAC_Narm computed by formula
    MODE_FORMULA_AD = 4     # DAC_Parm scanning, DAC_Narm computed by DA formula


    def __init__(self, task_queue, ad_da=None, 
                 gpio=None, handler=None):
        self.task_queue = task_queue
        self.ad_da = ad_da
        self.gpio = gpio
        self.handler = handler

        # ---------------- Sweep 状态 ----------------
        self.running = False
        self._timer = None
        # Timer → 主循环信号（IRQ-safe）
        self._tick_flag = False 

        # ---------------- 扫描参数 ----------------
        self.start = 0
        self.stop = 0
        self.step = 1
        self.delay = 1    # ms
        self.mode = self.MODE_PARM_SCAN

        self.fixed_parm = 0
        self.fixed_narm = 0

        # 电压参数
        self.vref = MCU_ADDA_Vref
        self.volt_bias_amp = Volt_Bias_Amp

        # ---------------- 游标 ----------------
        self.current = 0
        self.step_idx = 0

        # ---------------- 数据缓存 ----------------
        self.results = []

    # ==================================================
    # Sweep 控制接口
    # ==================================================
    def start_sweep(self, params: dict, source):
        """
        启动 Sweep（Timer 驱动，非阻塞）
        """
        if self.running:
            return

        
        # -------- 参数解析 --------
        self.start = params.get("start", 100)
        self.stop  = params.get("stop", 4095)
        # -------------------- 计算步长 --------------------
        points = params.get("points", 10)
        if points > 1:
            self.step = int( (self.stop - self.start) // (points - 1) )
        else:
            self.step = 1
        
        self.delay = params.get("delay", 1)
        if int(self.delay) <= 35:#ms
            self.delay=35
            self.handler._write_response("[INFO]: DelayTime(min) = 35ms\n", source)


        self.mode  = params.get("mode", self.MODE_PARM_SCAN)

        # self.fixed_parm = params.get("fixed_arm", 620)
        # self.fixed_narm = params.get("fixed_arm", 620)
        # self.vref = params.get("vref", MCU_ADDA_Vref)
        # self.volt_bias_amp = params.get("volt_bias_amp", Volt_Bias_Amp)

        self.fixed_parm = 620 if params.get("fixed_arm") is None else params["fixed_arm"]
        self.fixed_narm = 620 if params.get("fixed_arm") is None else params["fixed_arm"]

        self.vref = MCU_ADDA_Vref if params.get("vref") is None else params["vref"]
        self.volt_bias_amp = Volt_Bias_Amp if params.get("volt_bias_amp") is None else params["volt_bias_amp"]


        # -------- 初始化状态 --------
        self.current = self.start
        self.step_idx = 0
        self.results.clear()
        
        self.running = True
        self.handler._write_response("SWEEP START, For DEBUG\n", source)
        self.handler._write_response("SWEEP DATA:"   \
                    +"  step_idx"\
                    +"  dac_parm"\
                    +"  dac_narm"\
                    +"  ADC_V_Parm"\
                    +"  ADC_I_Parm"\
                    +"  ADC_V_Narm"\
                    +"  ADC_I_Narm"\
                    +"  DC_Power"\
                    +"  Pam4_MPD_Sel_Res"+"\n", 
                    "usb")   

        self._run_step()#先执行一次，配置初始值且做一次延时，确保第一个点也是正常的扫描步骤
      
        # -------- 启动 Timer --------
        self._tick_flag = False
        self._start_timer()

    def stop_sweep(self, source):
        """
        即时停止 Sweep
        """
        self.running = False
        self._tick_flag = False
        self._stop_timer()
        self.handler._write_response("SWEEP STOP\n", source)
    # ==================================================
    # Timer（IRQ-safe）
    # ==================================================
    def _start_timer(self):
        """
        启动 Sweep Timer，仅作为节拍器
        """
        if self._timer:
            self._timer.deinit()

        freq = max(1, int(1000 / self.delay))
        self._timer = pyb.Timer(
            8,
            freq=freq,
            callback=self._timer_cb
        )
        

    def _stop_timer(self):
        """
        停止 Sweep Timer
        """
        if self._timer:
            self._timer.deinit()
            self._timer = None
       
    def _timer_cb(self, t):
        """
        Timer IRQ：
        - 仅作为节拍信号源
        - 严格禁止 tick 堆积
        - 不做任何业务逻辑
        """
        if not self.running:
            return

        # 防止上一个 tick 还未被 poll 消费
        if self._tick_flag:
            return

        self._tick_flag = True
    # ==================================================
    # DC_Power + GPIO 特殊采样
    # ==================================================
    def _read_dc_power_with_gpio(self):
        """
        DC_Power ADC + Pam4_MPD_Sel_Res 联合判断
        """
        gpio_level = self.gpio.read_pin_direct("Pam4_MPD_Sel_Res")
        adc_val = self.ad_da.read_adc_sync("DC_Power", for_sweep=True)

        need_retry = False

        if gpio_level == 1 and adc_val is not None and adc_val > 3900:
            self.gpio.set_pin_direct("Pam4_MPD_Sel_Res", 0)
            need_retry = True

        elif gpio_level == 0 and adc_val is not None and adc_val < 200:
            self.gpio.set_pin_direct("Pam4_MPD_Sel_Res", 1)
            need_retry = True

        if need_retry:
            pyb.delay(1)
            gpio_level = self.gpio.read_pin_direct("Pam4_MPD_Sel_Res")
            pyb.delay(5)
            adc_val = self.ad_da.read_adc_sync("DC_Power", for_sweep=True)

        return adc_val, gpio_level

    # ==================================================
    # 核心：单 Step 执行（TaskQueue）
    # ==================================================
    def _run_step(self):
        """
        Sweep 单步执行（非阻塞）
        """
        if not self.running:
            return

        if self.current > self.stop:
            self._finish()
            return

        # ---------------- DAC 计算 ----------------
        dac_parm = self.fixed_parm
        dac_narm = self.fixed_narm

        if self.mode == self.MODE_PARM_SCAN:
            dac_parm = self.current
            self.ad_da.set_dac_task("DAC_Parm", dac_parm, None)#"uart"
            if self.step_idx == 0:
                self.ad_da.set_dac_task("DAC_Narm", dac_narm, None)
        elif self.mode == self.MODE_NARM_SCAN:
            dac_narm = self.current
            self.ad_da.set_dac_task("DAC_Narm", dac_narm, None)
            if self.step_idx == 0:
                self.ad_da.set_dac_task("DAC_Parm", dac_parm, None)
        elif self.mode == self.MODE_FORMULA:
            dac_parm = self.current
            dac_narm = int( (self.volt_bias_amp - (dac_parm / 4096.0 * self.vref * 2))
                / 2.0 / self.vref * 4096 )
            self.ad_da.set_dac_task("DAC_Parm", dac_parm, None)
            self.ad_da.set_dac_task("DAC_Narm", dac_narm, None)
            if self.step_idx == 0:
                pyb.delay(10)
                       
        elif self.mode == self.MODE_FORMULA_AD:
            dac_parm = self.current
            self.ad_da.set_dac_task("DAC_Parm", dac_parm, None)
            if self.step_idx == 0:
                self.ad_da.set_dac_task("DAC_Narm", dac_narm, None)
                
        # ---------------- 写 DAC ----------------
        # self.ad_da.set_dac_task("DAC_Parm", dac_parm, "uart")
        # self.ad_da.set_dac_task("DAC_Narm", dac_narm, "uart")

        # MODE 4：AD公式计算
        if self.mode == self.MODE_FORMULA_AD:
            if int(self.delay) <= 35:#ms
                pyb.delay(10)
            else:# delay for ADC sampling
                pyb.delay(int(self.delay*0.3))
            adc_v_parm = self.ad_da.read_adc_sync("ADC_V_Parm", True)
            dac_narm = int( (self.volt_bias_amp - (adc_v_parm / 4096.0 * self.vref / 150.0 * 350)) 
                           / 2.0 / self.vref * 4096 )
            self.ad_da.set_dac_task("DAC_Narm", dac_narm, None)#"uart"


        if int(self.delay) <= 35:#ms
            pyb.delay(10)
        else:# delay for ADC sampling
            pyb.delay(int(self.delay*0.3))

        # ---------------- ADC 同步采样 & MPD+IO控制 ----------------
        adc_snapshot = {}
        for _, name, val in self.ad_da.read_adc_snapshot():
            if name != "DC_Power":
                adc_snapshot[name] = val
        dc_val, dc_gpio = self._read_dc_power_with_gpio()
        adc_snapshot["DC_Power"] = dc_val

        
        # ---------------- 缓存数据 ----------------
        # self.results.append({
        #     "step": self.step_idx,
        #     "dac_parm": dac_parm,
        #     "dac_narm": dac_narm,
        #     "adc": adc_snapshot,
        #     "gpio": {"Pam4_MPD_Sel_Res": dc_gpio},#记录IO状态
        # })
        # ---------- Debug 输出,可注释 ----------
        line = (
            "SWEEP DATA %d %d %d %d %d %d %d %d %d\n"
            % (
                self.step_idx,
                dac_parm,
                dac_narm,
                adc_snapshot.get("ADC_V_Parm", -1),
                adc_snapshot.get("ADC_I_Parm", -1),
                adc_snapshot.get("ADC_V_Narm", -1),
                adc_snapshot.get("ADC_I_Narm", -1),
                dc_val if dc_val is not None else -1,
                dc_gpio if dc_gpio is not None else -1
            )
        )

        self.handler._write_response(line, "uart")#用在同步 光功率计等设备的调用
        pyb.delay(1)
        self.handler._write_response(line, "usb")#用在同步 光功率计等设备的调用
        # ---------------- 游标推进 ----------------
        self.step_idx += 1
        self.current += self.step

    # ==================================================
    # 扫描完成,一次性返回所有数据
    # ==================================================
    def _finish(self,):
        """
        Sweep 正常结束
        """
        self.running = False
        self._stop_timer()


        self.handler._write_response(
            "SWEEP END, Times: %d\n" % self.step_idx,
            "uart"
        )
        self.handler._write_response(
            "SWEEP END, Times: %d\n" % self.step_idx,
            "usb"
        )


        # self.handler._write_response(
        #     "SWEEP END, Times: %d\n" % len(self.results),
        #     "uart"
        # )
        # self.handler._write_response("SWEEP"\
        #             +"  step_idx"\
        #             +"  dac_parm"\
        #             +"  dac_narm"\
        #             +"  ADC_V_Parm"\
        #             +"  ADC_I_Parm"\
        #             +"  ADC_V_Narm"\
        #             +"  ADC_I_Narm"\
        #             +"  DC_Power"\
        #             +"  Pam4_MPD_Sel_Res"+"\n", 
        #             "uart")   
        # for item in self.results:
        #     line = (
        #         "SWEEP %d %d %d %d %d %d %d %d %d\n"
        #         % (
        #             item["step"],
        #             item["dac_parm"],
        #             item["dac_narm"],
        #             item["adc"].get("ADC_V_Parm", -1),
        #             item["adc"].get("ADC_I_Parm", -1),
        #             item["adc"].get("ADC_V_Narm", -1),
        #             item["adc"].get("ADC_I_Narm", -1),
        #             item["adc"].get("DC_Power", -1),
        #             item["gpio"].get("Pam4_MPD_Sel_Res", -1),
        #         )
        #     )
        #     self.handler._write_response(line, "uart")


        # self.handler._write_response(
        #     "SWEEP END, Times: %d\n" % len(self.results),
        #     "usb")
        # self.handler._write_response("SWEEP"\
        #             +"  step_idx"\
        #             +"  dac_parm"\
        #             +"  dac_narm"\
        #             +"  ADC_V_Parm"\
        #             +"  ADC_I_Parm"\
        #             +"  ADC_V_Narm"\
        #             +"  ADC_I_Narm"\
        #             +"  DC_Power"\
        #             +"  Pam4_MPD_Sel_Res"+"\n", 
        #             "usb")            
        # for item in self.results:
        #     line = (
        #         "SWEEP %d %d %d %d %d %d %d %d %d\n"
        #         % (
        #             item["step"],
        #             item["dac_parm"],
        #             item["dac_narm"],
        #             item["adc"].get("ADC_V_Parm", -1),
        #             item["adc"].get("ADC_I_Parm", -1),
        #             item["adc"].get("ADC_V_Narm", -1),
        #             item["adc"].get("ADC_I_Narm", -1),
        #             item["adc"].get("DC_Power", -1),
        #             item["gpio"].get("Pam4_MPD_Sel_Res", -1),
        #         )
        #     )
        #     self.handler._write_response(line, "usb")
            
            
        self.results.clear()
        gc.collect()            
# 执行序顺序：

# UART → StateMachine

# start_sweep()
# → running = True
# → Timer 启动
# → 回复 SWEEP START

# 100ms 后 Timer IRQ
# → _tick_flag = True

# main loop
# → sm.poll()
# → enqueue _run_step

# TaskQueue
# → 执行一次 _run_step

# 下一次 Timer IRQ
# → 下一步

# current > stop
# → _finish()
# → Timer deinit
# → 输出 SWEEP END + 数据