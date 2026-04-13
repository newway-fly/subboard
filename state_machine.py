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

            # ========== LOCKING (Mode B 独家指令) ==========
            elif head == "CALIB":
                """
                Command: CALIB <freq>
                触发物理链路相位标定，解决 TIA 与运放带来的群延迟偏移。
                """
                freq = int(parts[1]) if len(parts) >= 2 else 800
                if self.lock and self.ad_da:
                    log(INFO, f"Starting CALIB Phase Extraction at {freq}Hz")
                    # 1. 强制采集当前光功率 (阻塞模式保证数据实时)
                    self.ad_da.read_adc_timed_multi(self.lock.adc_target_name, self.lock.adc_buf)
                    pyb.delay(80) # 1440点物理需要 75ms 缓冲
                    
                    # 2. 调用算法解算
                    self.task_queue.add_task(self.lock.calibrate_phase, freq)
                return

            elif head == "LOCKMODE_B_CH":
                """
                Command: LOCKMODE_B_CH <CH> <TARGET> <ITERS>
                Example: LOCKMODE_B_CH XI MAX 200
                """
                if len(parts) >= 3 and self.lock:
                    ch_map = {"XI":0, "XQ":1, "XP":2, "YI":3, "YQ":4, "YP":5}
                    tgt_map = {"MIN":0, "MAX":1, "QUAD":2, "OFF":3}
                    
                    ch_idx = ch_map.get(parts[1].upper(), -1)
                    target = tgt_map.get(parts[2].upper(), 1)
                    iters = int(parts[3]) if len(parts) > 3 else 100
                    
                    if ch_idx != -1:
                        if target == 3: # OFF
                            self.task_queue.add_task(self.lock.stop_lock)
                        else:
                            self.task_queue.add_task(self.lock.start_lock, ch_idx, target, iters)
                    else:
                        log(ERROR, f"Invalid Lock Channel: {parts[1]}")
                return
                
            elif head == "LOCKMODE_B_STOP":
                if self.lock:
                    self.task_queue.add_task(self.lock.stop_lock)
                return
            elif head == "LOCKMODE_B_AMP":
                """
                Command: LOCKMODE_B_AMP <CH> <AMP_CODE>
                Example: LOCKMODE_B_AMP XI 50
                """
                if len(parts) >= 3 and self.ad_da:
                    ch_map = {"XI":0, "XQ":1, "XP":2, "YI":3, "YQ":4, "YP":5}
                    ch_idx = ch_map.get(parts[1].upper(), -1)
                    new_amp = int(parts[2])
                    
                    if ch_idx != -1:
                        # 从底层 dac_list 中安全获取真实的 dac_name
                        dac_name = self.ad_da.dac_list[ch_idx][1] if ch_idx < len(self.ad_da.dac_list) else None
                        if dac_name:
                            self.task_queue.add_task(self.ad_da.update_dither_amplitude, dac_name, new_amp)
                    else:
                        log(ERROR, f"Invalid Lock Channel for AMP update: {parts[1]}")
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