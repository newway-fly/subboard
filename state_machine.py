# state_machine.py
# --------------------------------------------------
# SubBoard 从机中央状态机
# - 统一管理模块调用
# - 支持 USB/Host UART 命令解析
# - 支持自身命令执行（ADC/DAC/POWER/GPIO/SWEEP/EXT）
# - 支持 HostBoard 透传命令
# - 执行完成后通过 USB/Host UART 回传
# --------------------------------------------------

import pyb
from log_system import log, INFO, WARN, DEBUG, ERROR
from config import SubBoard_ID, uart_delay_ms  # 从机 ID 配置&简单延时

class StateMachine:
    """
    SubBoard 中央状态机
    """

    def __init__(self, task_queue):
        """
        初始化状态机
        :param task_queue: TaskQueue 实例，用于异步任务调度
        """
        self.head_ID = b''#从机uart传递信息到主机uart,再透传到PC端，可以加上前缀

        self.task_queue = task_queue

        # 模块引用（后续通过 set_modules 注入）
        self.usb = None
        self.uart = None
        self.ad_da = None
        self.power_seq = None
        self.gpio = None
        self.sweep = None
        self.lock = None

        # 扩展函数字典，可动态添加
        self.ext_func_dict = {}

    # --------------------------------------------------
    # 模块回注
    # --------------------------------------------------
    def set_modules(self, usb=None, uart=None, ad_da=None,
                    gpio=None, sweep=None, lock=None):
        self.usb = usb or self.usb
        self.uart = uart or self.uart
        self.ad_da = ad_da or self.ad_da
        self.gpio = gpio or self.gpio
        self.sweep = sweep or getattr(self, 'sweep', None)
        self.lock = lock or getattr(self, 'lock', None)

    # --------------------------------------------------
    # USB / UART 数据入口逻辑
    # --------------------------------------------------
    def handle_usb_data(self, data: bytes):
        try:
            cmd_str = data.decode('utf-8').strip()
            if not cmd_str: return
            self.task_queue.add_task(self._process_str_cmd, cmd_str, "usb")
        except Exception as e:
            log(INFO, "handle_usb_data error:", e)

    def handle_uart_data(self, data: bytes):
        try:
            text = data.decode('utf-8').strip()
            if not text: return
            parts = text.split()
            if not parts: return

            head = parts[0].upper()
            self.head_ID = ("S%d " % SubBoard_ID).encode("utf-8")

            if head.startswith("S") and len(head) == 2 and head[1].isdigit():
                target_id = int(head[1])
                if target_id == SubBoard_ID:
                    cmd_str = " ".join(parts[1:])  
                    self.task_queue.add_task(self._process_str_cmd, cmd_str, "uart")
                    
                    resp_data = "CMD: "+ text+'\n'
                    if self.usb: self.usb.write(resp_data.encode('utf-8'))
                    return
                else:
                    return

            if self.usb:
                self.usb.write(data)

        except Exception as e:
            log(INFO, "handle_uart_data error:", e)

    # --------------------------------------------------
    # 字符串命令解析 (核心业务路由)
    # --------------------------------------------------
    def _process_str_cmd(self, cmd_str: str, source='usb'):
        params = {}
        try:
            parts = cmd_str.split()
            if not parts: return
            head = parts[0].upper()

            # ========== ADC ==========
            if head == "ADC" and self.ad_da:
                if len(parts) >= 2:
                    arg = parts[1].upper()
                    if arg == "ALL":
                        self.task_queue.add_task(self.ad_da.read_all_adc_task, source)
                    else:
                        ch = int(arg) if arg.isdigit() else arg
                        self.task_queue.add_task(self.ad_da.read_adc_task, ch, source)
                return

            # ========== DAC ==========
            elif head == "DAC" and self.ad_da:
                if len(parts) == 2:
                    arg = parts[1]
                    ch = int(arg) if arg.isdigit() else arg
                    self.task_queue.add_task(self.ad_da.read_dac_task, ch, source)
                elif len(parts) >= 3:
                    arg = parts[1]
                    ch = int(arg) if arg.isdigit() else arg
                    val = int(parts[2])
                    self.task_queue.add_task(self.ad_da.set_dac_task, ch, val, source)
                return

            # ========== GPIO ==========
            elif head == "GPIO" and self.gpio:
                if len(parts) >= 3:
                    self.task_queue.add_task(self.gpio.set_pin_task, parts[1], parts[2].upper(), source)
                elif len(parts) == 2:
                    self.task_queue.add_task(self.gpio.read_pin_task, parts[1], source)
                elif len(parts) == 1:
                    self.task_queue.add_task(self.gpio.read_all_pins_task, source)
                return

            # ========== SWEEP ==========
            elif head == "SWEEP" and self.sweep:
                if len(parts) == 2 and parts[1].upper() == "STOP":
                    self.task_queue.add_task(self.sweep.stop_sweep, source)
                    return
                if len(parts) < 5: return
                params = {
                    "start": int(parts[1]), "stop": int(parts[2]),
                    "points": int(parts[3]) if int(parts[3]) <= 800 else 800, 
                    "delay": int(parts[4]),
                    "mode": int(parts[5]) if len(parts) >= 6 else None,
                    "fixed_arm": int(parts[6]) if len(parts) >= 7 else None,
                    "vref": float(parts[7]) if len(parts) >= 8 else None,
                    "volt_bias_amp": float(parts[8]) if len(parts) >= 9 else None,
                }
                self.task_queue.add_task(self.sweep.start_sweep, params, source)
                return

            # ========== MODE B SUBBOARD LOCKING PROTOCOL ==========
            
            elif head == "INIT_ARM":
                """
                Command: INIT_ARM <MODE> [V1] [V2]
                Example: INIT_ARM PN 1.0 5.0
                """
                if len(parts) >= 2 and self.ad_da:
                    mode = parts[1].upper()
                    v1 = float(parts[2]) if len(parts) > 2 else None
                    v2 = float(parts[3]) if len(parts) > 3 else None
                    self.task_queue.add_task(self.ad_da.init_arm_config, mode, v1, v2)
                    self._write_response(f"ACK: INIT_ARM {mode}\r\n", source)
                return

            elif head == "INIT_DITHER":
                """
                Command: INIT_DITHER <AMP>
                Example: INIT_DITHER 30
                """
                if len(parts) >= 2 and self.ad_da:
                    amp = int(parts[1])
                    self.task_queue.add_task(self.ad_da.init_dither, amp)
                    self._write_response(f"ACK: INIT_DITHER Amp={amp}\r\n", source)
                return

            elif head == "LOCK_B":
                """
                Command: LOCK_B START <TGT> <ITERS>  |  LOCK_B STOP  |  LOCK_B AMP <AMP>
                Example: LOCK_B START MAX 200
                """
                if len(parts) >= 2 and self.lock:
                    sub_cmd = parts[1].upper()
                    
                    if sub_cmd == "START" and len(parts) >= 3:
                        tgt_map = {"MIN": 0, "MAX": 1, "QUAD": 2}
                        target = tgt_map.get(parts[2].upper(), 1)
                        iters = int(parts[3]) if len(parts) > 3 else 100
                        self.task_queue.add_task(self.lock.start_lock, target, iters)
                        self._write_response(f"ACK: LOCK_B START TGT={parts[2]}\r\n", source)
                        
                    elif sub_cmd == "STOP":
                        self.task_queue.add_task(self.lock.stop_lock)
                        
                    elif sub_cmd == "AMP" and len(parts) >= 3:
                        new_amp = int(parts[2])
                        # Dynamically update the in-memory sine buffer without stopping hardware
                        self.task_queue.add_task(self.ad_da.init_dither, new_amp)
                        self._write_response(f"ACK: LOCK_B AMP = {new_amp}\r\n", source)
                return

            elif head == "CALIB":
                """
                Command: CALIB <freq>
                [Plan A] Automated Sequence: Dither ON -> Sample -> Dither OFF -> Calculate.
                This ensures calibration is done with active physical perturbation.
                """
                freq = int(parts[1]) if len(parts) >= 2 else 800
                if self.lock and self.ad_da:
                    log(INFO, f"Executing CALIB Plan A at {freq}Hz...")
                    
                    # 1. Start hardware dither output automatically
                    self.ad_da.set_lock_dither(True)
                    pyb.delay(20) # Hardware stabilization delay
                    
                    # 2. Trigger hardware-timed sampling for phase extraction (1440 points)
                    # Note: uses self.lock.adc_buf (size 1440)
                    self.ad_da.read_adc_timed_multi(self.lock.adc_target_name, self.lock.adc_buf, timer_id=7)
                    
                    # 3. Wait for DMA transfer to finish (~75ms + safety)
                    pyb.delay(85)
                    
                    # 4. Stop hardware dither immediately to return to DC state
                    self.ad_da.set_lock_dither(False)
                    
                    # 5. Enqueue the phase calculation logic as a background task
                    self.task_queue.add_task(self.lock.calibrate_phase, freq)
                return

            elif head == "GET_ADC_NOISE":
                """
                Command: GET_ADC_NOISE [CH_NAME]
                Captures 4320 points for raw noise evaluation.
                """
                ch_name = parts[1] if len(parts) >= 2 else "AC_Lock"
                if self.ad_da:
                    # Enqueue as a task to keep the state machine responsive
                    self.task_queue.add_task(self.ad_da.capture_noise_task, ch_name, source)
                return
            




            else:
                log(INFO, "Unknown command:", cmd_str)
            return

        except Exception as e:
            log(INFO, "_process_str_cmd error:", e)

    # ===================================================
    # 回调输出接口
    # ===================================================
    def _write_response(self, msg: str, source: str):
        pyb.delay(uart_delay_ms)
        source = source.lower()
        try:
            data = msg.encode('utf-8')
            if source == 'usb' and self.usb:
                self.usb.write(data)
            elif source == 'uart' and self.uart:
                data = self.head_ID + data
                self.uart.write(data)
        except Exception as e:
            log(INFO, "_write_response error:", e)

    # 各种 handler 维持原状
    def handle_adc_data(self, results, source='uart'):
        try:
            for idx, name, val in results:
                self._write_response(f"RX: ADC {idx} {name} = {val}\r\n", source)
        except Exception as e: pass

    def handle_dac_Set_Single(self, ch, name, value, source='usb'):
        try:
            self._write_response(f"DAC {ch} {name} = {value}\r\n", source)
        except Exception as e: pass

    def handle_dac_Read_Single(self, ch, name, value, source='usb'):
        try:
            self._write_response(f"DAC {ch} {name} = {value}\r\n", source)
        except Exception as e: pass

    def handle_dac_Read_All(self, results, source='usb'):
        try:
            for idx, name, val in results:
                self._write_response(f"DAC {idx} {name} = {val}\r\n", source)
        except Exception as e: pass

    def handle_gpio_result(self, data, operation, source='usb'):
        # 省略具体内容，保持原有实现
        pass

    def poll(self):
        """
        主循环轮询（IRQ → TaskQueue 的安全过渡点，并为 Locking 引擎供电）
        """
        # 1. 扫描轮询 (Sweep)
        if self.sweep and self.sweep.running:
            if self.sweep._tick_flag:
                self.sweep._tick_flag = False
                self.task_queue.add_task(self.sweep._run_step)

        # 2. 锁定引擎轮询 (Locking Engine FSM Heartbeat)
        # 只要跑起来，就疯狂抽样，内部通过 millis 判定是否需要实质运算，纯非阻塞
        if self.lock and getattr(self.lock, 'running', False):
            self.lock.process()