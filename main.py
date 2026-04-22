"""
main.py
Main program entry (Architecture B)
Initialization order:
    1) Create TaskQueue
    2) Create StateMachine (inject the TaskQueue into it)
    3) Create and inject modules (USBComm, UARTMaster, AD_DA, PowerSequence, LedIndicator)
    4) Provide module objects back to the StateMachine via set_modules
    5) Main loop:
        - Call each module’s poll() to convert external events into queued tasks (USB, UART)
        - Execute some tasks (task_queue.run_all(limit))
        - Use a short delay to avoid 100% CPU usage (pyb.delay(1))
"""

# main.py
# --------------------------------------------------
# SubBoard 主程序入口（架构 B）
# --------------------------------------------------

import pyb, gc
# import micropython
from locking import Lock_TimeMUX
from sweep import Sweep
from ad_da import AD_DA
from task_queue import TaskQueue
from usb_comm import USBComm
from uart_master import UARTMaster
from state_machine import StateMachine
from led_indicator import LedIndicator
from log_system import log, INFO
from gpio_control import GPIOControl


                         
from config import SubBoard_ID, Verdion

def main():
    """
    SubBoard 主程序入口
    - 初始化任务队列、状态机、模块
    - 将模块注入状态机
    - 主循环 poll 外设 + 执行任务队列
    """

    # 创建任务队列，最大容量 512
    task_q = TaskQueue(max_size=512)

    # 创建中央状态机，传入任务队列
    sm = StateMachine(task_q)

    # 创建各模块
    led = LedIndicator()                             # LED 指示
    usb = USBComm(task_q, handler=sm)                # USB 通信
    uart = UARTMaster(task_q, handler=sm)            # UART 通信
    gpio = GPIOControl(task_q, handler=sm)          # GPIO 控制
    ad_da = AD_DA(task_q, handler=sm)                # ADC/DAC 控制

    # 创建 Sweep 扫描模块，注入任务队列、状态机和 AD_DA 对象
    sweep = Sweep(task_queue=task_q, ad_da=ad_da, gpio = gpio, handler=sm)
    lock = Lock_TimeMUX(ad_da=ad_da, uart_master=uart, handler=sm)

    # 模块回注到状态机，状态机可统一调度
    # This allows the StateMachine to call into modules and coordinate
    # cross-module operations (e.g., starting a sweep from a USB command, using ADC/DAC and GPIO)
    sm.set_modules(
        usb=usb,
        uart=uart,
        ad_da=ad_da,
        # led=led,
        gpio=gpio,
        sweep=sweep,    # 注入 Sweep 模块
        lock=lock
    )

    # 打印系统启动日志
    log(INFO, "==============================")
    log(INFO, "SubBoard Version Update: "+str(Verdion))
    log(INFO, "SubBoard system started for S"+str(SubBoard_ID))
    
    log(INFO, f"MEMORY AFTER BOOTING: {gc.mem_free()}")
    log(INFO, "==============================")

    # 主循环：poll 外设 -> 执行任务队列
    while True:
        try:
            # 1. 外设事件 → 任务（USB / UART）
            usb.poll()
            uart.poll()

            # 2. 状态机轮询（Sweep Timer → TaskQueue）
            sm.poll()

            # 3. 执行任务队列（限制执行数量，避免阻塞）
            if lock.running:
                executed = task_q.run_all(limit=1)
            else:
                executed = task_q.run_all(limit=40)

            # 4. 空闲让出 CPU，如果没有任务执行，延迟 1ms 避免 CPU 占用 100%
            if executed == 0:
                pyb.delay(1)
                # 定期垃圾回收
                gc.collect()

        except Exception as e:
            log(INFO, "Main loop error:", e)


if __name__ == '__main__':
    # 上电延迟 5 秒，确保 USB / 电源稳定
    pyb.delay(5000)
    main()
