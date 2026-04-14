# config.py
# =========================
# pyb11_SubBoard 全局配置文件
# =========================

# USB 配置
USB_READ_BUF = 128  # USB 读缓冲大小

# SubBoard_ID从机ID配置
SubBoard_ID = 1  # ID

# ID_to_loc_map = {
#     '1': {'P_arm': 'XI', 'N_arm': 'XQ'},
#     '2': {'P_arm': 'YI', 'N_arm': 'YQ'},
#     '3': {'P_arm': 'X', 'N_arm': 'Y'},
# }

# P_arm_location = ID_to_loc_map[f"{SubBoard_ID}"]["P_arm"]
# N_arm_location = ID_to_loc_map[f"{SubBoard_ID}"]["N_arm"]

# === Mode B Physics Constants ===
Bias_Arm_Volt = 1.0       # Fixed DC voltage for the unused arm in Single-Arm mode (V)
Bias_Amp_Volt = 5.0       # Default total voltage sum for PN Dual-Arm mode (V)
MCU_ADDA_Vref = 3.382     # MCU Reference Voltage (V)


uart_delay_ms = 0   #(SubBoard_ID-1)*5

# UART 配置（SubBoard ↔ 主机板通信）
UART_PORT = 4
UART_BAUD = 115200

# 状态机状态定义
STATE_IDLE = 0
STATE_PROCESS_USB_CMD = 1
STATE_PROCESS_UART_RESP = 2
STATE_RUN_POWER_SEQ = 3
STATE_UPDATE_AD_DA = 4
STATE_ERROR = 99

# 从机 ID
SLAVE_IDS = [1, 2, 3]

# 动作超时
ACTION_TIMEOUT_MS = 1000

# LED 指示灯配置
LED_NUM = 4        # PC9 已在 mpconfigboard.h 映射为 LED_Blue
Delay_MS = 500
Timer_Num = 2

# 改成列表+元组，保证顺序
# ADC 配置
ADC_PINS = [
    ("DC_Power", "A0"),
    ("AC_Lock", "A1"),
    ("ADC_IN2", "A2"),
    ("ADC_IN3", "A3"),
    ("ADC_IN6", "A6"),
    ("ADC_IN7", "A7"),
    ("ADC_IN8", "B0"),
    ("ADC_IN9", "B1"),
    ("ADC_V_Parm", "C0"),
    ("ADC_I_Parm", "C1"),
    ("ADC_V_Narm", "C2"),
    ("ADC_I_Narm", "C3"),
    ("ADC_IN14", "C4"),
    ("ADC_IN15", "C5")
]
ADC_SAMPLE_TIMES = 5            #重复采样次数   
ADC_SAMPLE_DELAY_US = 2000      #采样间隔us
ADC_SAMPLE_TIMES_ForSweep = 5
ADC_SAMPLE_DELAY_US_ForSweep = 200#1000


# MCU_ADDA_Vref = 3.382   #单位：V
# PSdriver_Amp = 2 #Amplification factor of the PS driver stage
# Volt_Bias_Amp = 5.0       #单位：V


SWeep_ADC_list = [   
    ("DC_Power", "A0"),
    ("ADC_V_Parm", "C0"),
    ("ADC_I_Parm", "C1"),
    ("ADC_V_Narm", "C2"),
    ("ADC_I_Narm", "C3"),
]
# DAC 配置
DAC_PINS = [
    ("DAC_Parm", "A4"),
    ("DAC_Narm", "A5")
]
DAC_BUFFER_MODE = True  # 启用缓冲模式保证输出稳定

# I2C 配置
I2C_PINS = {
    "I2C1_SCL": "B6",
    "I2C1_SDA": "B7",
}

# SPI 配置
SPI_PINS = {
    "SPI2_NSS":  "B12",
    "SPI2_SCK":  "B13",
    "SPI2_MISO": "B14",
    "SPI2_MOSI": "B15",
    "SPI3_NSS":  "A15",
    "SPI3_SCK":  "B3",
    "SPI3_MISO": "B4",
    "SPI3_MOSI": "B5",
}

# GPIO配置字典结构：引脚名 -> [引脚号, 模式, 初始值]
GPIO_CONFIG = {
    "HostMode_EnVBus"   : ["A8",    "OUT",  1   ],  # 输出模式，默认高电平
    "USRSW"             : ["C13",   "IN",   None],  # 输入模式，无初始值
    "Pam4_MPD_Sel_Res"  : ["C5",    "OUT",  0   ],  # 输出模式，初始低电平
    "Seq_In"            : ["B8",    "OUT",  1   ],  # 输出模式，默认高电平
    "Seq_Out"           : ["B9",    "IN",   None],  # 输入模式，无初始值
}
# 从 GPIO_CONFIG 生成 GPIO_MAP 用于向后兼容
GPIO_MAP = {name: config[0] for name, config in GPIO_CONFIG.items()}


# 未使用功能占位
CAN_PINS = {}
I2S_PINS = {}
ETH_PINS = {}

# import pyb
# for name in dir(pyb.Pin.board):
# if not name.startswith('_'):
# pin = getattr(pyb.Pin.board, name)
# print(f"{name}: {pin}")
#加3次回车键

# Pyb所有引脚定义
# AC_Lock: Pin(Pin.cpu.A1, mode=Pin.ANALOG)
# ADC_IN14: Pin(Pin.cpu.C4, mode=Pin.ANALOG)
# ADC_IN2: Pin(Pin.cpu.A2, mode=Pin.ANALOG)
# ADC_IN3: Pin(Pin.cpu.A3, mode=Pin.ANALOG)
# ADC_IN6: Pin(Pin.cpu.A6, mode=Pin.ANALOG)
# ADC_IN7: Pin(Pin.cpu.A7, mode=Pin.ANALOG)
# ADC_IN8: Pin(Pin.cpu.B0, mode=Pin.ANALOG)
# ADC_IN9: Pin(Pin.cpu.B1, mode=Pin.ANALOG)
# ADC_I_Narm: Pin(Pin.cpu.C3, mode=Pin.ANALOG)
# ADC_I_Parm: Pin(Pin.cpu.C1, mode=Pin.ANALOG)
# ADC_V_Narm: Pin(Pin.cpu.C2, mode=Pin.ANALOG)
# ADC_V_Parm: Pin(Pin.cpu.C0, mode=Pin.ANALOG)
# BOOT1: Pin(Pin.cpu.B2, mode=Pin.IN)
# DAC_Narm: Pin(Pin.cpu.A5, mode=Pin.ANALOG)
# DAC_Parm: Pin(Pin.cpu.A4, mode=Pin.ANALOG)
# DC_Power: Pin(Pin.cpu.A0, mode=Pin.ANALOG)
# HostMode_EnVBus: Pin(Pin.cpu.A8, mode=Pin.OUT)
# I2C1_SCL: Pin(Pin.cpu.B6, mode=Pin.IN)
# I2C1_SDA: Pin(Pin.cpu.B7, mode=Pin.IN)
# LED_BLUE: Pin(Pin.cpu.C9, mode=Pin.OUT)
# LED_GREEN: Pin(Pin.cpu.C7, mode=Pin.OUT)
# LED_RED: Pin(Pin.cpu.C6, mode=Pin.OUT)
# LED_YELLOW: Pin(Pin.cpu.C8, mode=Pin.OUT)
# OSC32_IN: Pin(Pin.cpu.C14, mode=Pin.IN)
# OSC32_OUT: Pin(Pin.cpu.C15, mode=Pin.OUT)
# OSC_IN: Pin(Pin.cpu.H0, mode=Pin.IN)
# OSC_OUT: Pin(Pin.cpu.H1, mode=Pin.IN)
# Pam4_MPD_Sel_Res: Pin(Pin.cpu.C5, mode=Pin.IN)
# SPI2_CS: Pin(Pin.cpu.B12, mode=Pin.IN)
# SPI2_MISO: Pin(Pin.cpu.B14, mode=Pin.IN)
# SPI2_MOSI: Pin(Pin.cpu.B15, mode=Pin.IN)
# SPI2_SCK: Pin(Pin.cpu.B13, mode=Pin.IN)
# SPI3_CS: Pin(Pin.cpu.A15, mode=Pin.ALT, pull=Pin.PULL_UP, alt=0)
# SPI3_MISO: Pin(Pin.cpu.B4, mode=Pin.ALT, pull=Pin.PULL_UP, alt=0)
# SPI3_MOSI: Pin(Pin.cpu.B5, mode=Pin.IN)
# SPI3_SCK: Pin(Pin.cpu.B3, mode=Pin.ALT, alt=0)
# SWCLK: Pin(Pin.cpu.A14, mode=Pin.ALT, pull=Pin.PULL_DOWN, alt=0)
# SWDIO: Pin(Pin.cpu.A13, mode=Pin.ALT, pull=Pin.PULL_UP, alt=0)
# Seq_In: Pin(Pin.cpu.B8, mode=Pin.OUT)
# Seq_Out: Pin(Pin.cpu.B9, mode=Pin.IN)
# UART4_RX: Pin(Pin.cpu.C11, mode=Pin.ALT, pull=Pin.PULL_UP, alt=Pin.AF8_UART4)
# UART4_TX: Pin(Pin.cpu.C10, mode=Pin.ALT, alt=Pin.AF8_UART4)
# UART5_RX: Pin(Pin.cpu.D2, mode=Pin.IN)
# UART5_TX: Pin(Pin.cpu.C12, mode=Pin.IN)
# USART3_RX: Pin(Pin.cpu.B11, mode=Pin.IN)
# USART3_TX: Pin(Pin.cpu.B10, mode=Pin.IN)
# USB_DM: Pin(Pin.cpu.A11, mode=Pin.ALT, alt=10)
# USB_DP: Pin(Pin.cpu.A12, mode=Pin.ALT, alt=10)
# USB_ID: Pin(Pin.cpu.A10, mode=Pin.IN)
# USB_VBUS: Pin(Pin.cpu.A9, mode=Pin.IN)
# USRSW: Pin(Pin.cpu.C13, mode=Pin.IN)