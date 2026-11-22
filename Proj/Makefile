##########################################################################################################################
# Makefile for GD32F303CCT6 Pressure Relief Module Project
# Based on arm-none-eabi-gcc toolchain (reference: PlanbPro GD32H75E)
##########################################################################################################################

######################################
# target
######################################
TARGET = planb_discharge

######################################
# building variables
######################################
DEBUG = 1
OPT = -Og

#######################################
# paths
#######################################
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
../Core/User/main.c \
../Core/User/gd32f30x_it.c \
../Core/User/systick.c \
../Core/User/syscalls.c \
../Core/Bsp/bsp.c \
../Core/Bsp/bsp_uart_log.c \
../Core/Bsp/bsp_analog.c \
../Core/Bsp/bsp_discharge.c \
../Core/Bsp/bsp_can.c \
../Core/Bsp/bsp_spi.c \
../Core/Bsp/bsp_fram.c \
../Core/Bsp/bsp_wdg.c \
../Core/Bsp/bsp_timer.c \
../Core/Bsp/bsp_config.c \
../Core/App/uart_log.c \
../Core/App/app.c \
../Core/App/app_monitor.c \
../Core/App/app_protect.c \
../Core/App/app_discharge.c \
../Core/App/app_safety.c \
../Core/App/app_blackbox.c \
../Core/App/app_comm.c \
../Core/App/canx_proto.c \
../Core/App/canx_proto_port.c \
../Core/App/app_can_slave.c \
../Core/Lib/CMSIS/GD/GD32F30x/Source/system_gd32f30x.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_adc.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_bkp.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_can.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_crc.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_ctc.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_dac.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_dbg.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_dma.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_enet.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_exmc.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_exti.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_fmc.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_fwdgt.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_gpio.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_i2c.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_misc.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_pmu.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_rcu.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_rtc.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_sdio.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_spi.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_timer.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_usart.c \
../Core/Lib/GD32F30x_standard_peripheral/Source/gd32f30x_wwdgt.c \

# BSP 单元测试源文件: 仅当 EXTRA_CFLAGS 含 -DBSP_UNIT_TEST 时编入
# 用法: make all EXTRA_CFLAGS="-DBSP_UNIT_TEST -DUART_LOG_ENABLE=1"
ifneq (,$(findstring BSP_UNIT_TEST,$(EXTRA_CFLAGS)))
C_SOURCES += \
../Core/Test/test_runner.c \
../Core/Test/test_bsp_uart_log.c \
../Core/Test/test_bsp_analog.c \
../Core/Test/test_bsp_discharge.c \
../Core/Test/test_bsp_can.c \
../Core/Test/test_bsp_spi.c \
../Core/Test/test_bsp_fram.c \
../Core/Test/test_bsp_wdg.c \
../Core/Test/test_bsp_config.c \
../Core/Test/test_ovp_logic.c \
../Core/Test/test_canx_proto.c \
../Core/Test/test_canx_port.c \
../Core/Test/test_canx_slave.c \
../Core/Test/test_brake_special.c
endif
# 注: test_bsp_flash.c 暂时停用（配置改存 FRAM）；如需切回，恢复上面 bsp_flash.c
#     与此处 test_bsp_flash.c，并将 test_runner 中 TEST_ENABLE_FLASH 设回 1。

# ASM sources
ASM_SOURCES =  \
startup_gd32f30x_hd_gcc.s

# ASMM sources
ASMM_SOURCES =

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

AS_DEFS =

# UART_LOG_ENABLE=0 -> 默认关闭调试打印（生产固件）
# 调试打印时使用: make debug  或  make all EXTRA_CFLAGS="-UUART_LOG_ENABLE -DUART_LOG_ENABLE=1"
#
# 时钟方案切换 (默认 25M晶振→100MHz):
#   换 8M 晶振配 120MHz 时, 需同步两处:
#     1) 把下面的 HXTAL_VALUE 改为 8000000U
#     2) 在 system_gd32f30x.c 顶部把 BSP_CLK_PLAN 改为 BSP_CLK_PLAN_8M_120M
#        (或在此追加 -DBSP_CLK_PLAN=2 覆盖)
C_DEFS =  \
-DUART_LOG_ENABLE=0 \
-DGD32F30X_HD \
-DUSE_STDPERIPH_DRIVER \
-DHXTAL_VALUE=8000000U

AS_INCLUDES =

C_INCLUDES =  \
-I../Core/Lib/CMSIS \
-I../Core/Lib/CMSIS/GD/GD32F30x/Include \
-I../Core/User \
-I../Core/Bsp \
-I../Core/App \
-I../Core/Test \
-I../Core/Lib/GD32F30x_standard_peripheral/Include

# Source / execution character set - force UTF-8 so that GBK-saved
# legacy headers or Chinese comments do not silently corrupt string
# literals on a Windows host with default code page 936.
CHARSET_FLAGS = -finput-charset=UTF-8 -fexec-charset=UTF-8

ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

# Optional extras
EXTRA_CFLAGS ?=
CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) $(CHARSET_FLAGS) $(EXTRA_CFLAGS) -Wall -Wno-misleading-indentation -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
LDSCRIPT = GD32F30x_FLASH.ld

LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# build the application
#######################################
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASMM_SOURCES:.S=.o)))
vpath %.S $(sort $(dir $(ASMM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@
$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	if not exist $@ mkdir $@

#######################################
# clean up
#######################################
clean:
	-@del /f /q $(BUILD_DIR)\*.o  2>nul
	-@del /f /q $(BUILD_DIR)\*.d  2>nul
	-@del /f /q $(BUILD_DIR)\*.lst 2>nul
	-@del /f /q $(BUILD_DIR)\*.elf 2>nul
	-@del /f /q $(BUILD_DIR)\*.hex 2>nul
	-@del /f /q $(BUILD_DIR)\*.bin 2>nul
	-@del /f /q $(BUILD_DIR)\*.map 2>nul
	@echo clean done.

# Flash via J-Link
flash: all
	flash_jlink.bat

# Flash via OpenOCD (if configured)
flash-ocd: all
	openocd -f interface/jlink.cfg -f target/gd32f30x.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

# 调试固件: 开启 UART LOG 打印，方便开发调试
#   make debug
# 等价: make all EXTRA_CFLAGS="-UUART_LOG_ENABLE -DUART_LOG_ENABLE=1"
# 注: 先 clean 再编, 避免改了编译宏但 .o 时间戳未变导致 Make 复用旧目标文件,
#     从而出现"开了打印却没有输出"的假象。
.PHONY: debug
debug: clean
	$(MAKE) all EXTRA_CFLAGS="-UUART_LOG_ENABLE -DUART_LOG_ENABLE=1"

# 生产固件: 关闭所有调试打印 (默认)
#   make release
# 等价: make all (UART_LOG_ENABLE=0)
.PHONY: release
release: clean
	$(MAKE) all

# ================================================================
# 单元测试编译目标 (对标 PlanbPro: make analog-dma-test 等)
# 统一命令: make test
# 单个模块: make test-<模块名>
# ================================================================
.PHONY: test
test: clean
	$(MAKE) all EXTRA_CFLAGS="-DBSP_UNIT_TEST -UUART_LOG_ENABLE -DUART_LOG_ENABLE=1"

.PHONY: test-uart
test-uart: clean
	$(MAKE) all EXTRA_CFLAGS="-DBSP_UNIT_TEST -UUART_LOG_ENABLE -DUART_LOG_ENABLE=1 -DTEST_ENABLE_ANALOG=0 -DTEST_ENABLE_DISCHARGE=0 -DTEST_ENABLE_CAN=0 -DTEST_ENABLE_SPI=0 -DTEST_ENABLE_FRAM=0 -DTEST_ENABLE_WDG=0 -DTEST_ENABLE_OVP=0 -DTEST_ENABLE_FLASH=0 -DTEST_ENABLE_CONFIG=0"

.PHONY: test-analog
test-analog: clean
	$(MAKE) all EXTRA_CFLAGS="-DBSP_UNIT_TEST -UUART_LOG_ENABLE -DUART_LOG_ENABLE=1 -DTEST_ENABLE_UART_LOG=0 -DTEST_ENABLE_DISCHARGE=0 -DTEST_ENABLE_CAN=0 -DTEST_ENABLE_SPI=0 -DTEST_ENABLE_FRAM=0 -DTEST_ENABLE_WDG=0 -DTEST_ENABLE_OVP=0 -DTEST_ENABLE_FLASH=0 -DTEST_ENABLE_CONFIG=0"

.PHONY: test-discharge
test-discharge: clean
	$(MAKE) all EXTRA_CFLAGS="-DBSP_UNIT_TEST -UUART_LOG_ENABLE -DUART_LOG_ENABLE=1 -DTEST_ENABLE_UART_LOG=0 -DTEST_ENABLE_ANALOG=0 -DTEST_ENABLE_CAN=0 -DTEST_ENABLE_SPI=0 -DTEST_ENABLE_FRAM=0 -DTEST_ENABLE_WDG=0 -DTEST_ENABLE_OVP=0 -DTEST_ENABLE_FLASH=0 -DTEST_ENABLE_CONFIG=0"

.PHONY: test-can
test-can: clean
	$(MAKE) all EXTRA_CFLAGS="-DBSP_UNIT_TEST -UUART_LOG_ENABLE -DUART_LOG_ENABLE=1 -DTEST_ENABLE_UART_LOG=0 -DTEST_ENABLE_ANALOG=0 -DTEST_ENABLE_DISCHARGE=0 -DTEST_ENABLE_SPI=0 -DTEST_ENABLE_FRAM=0 -DTEST_ENABLE_WDG=0 -DTEST_ENABLE_OVP=0 -DTEST_ENABLE_FLASH=0 -DTEST_ENABLE_CONFIG=0"

.PHONY: test-spi
test-spi: clean
	$(MAKE) all EXTRA_CFLAGS="-DBSP_UNIT_TEST -UUART_LOG_ENABLE -DUART_LOG_ENABLE=1 -DTEST_ENABLE_UART_LOG=0 -DTEST_ENABLE_ANALOG=0 -DTEST_ENABLE_DISCHARGE=0 -DTEST_ENABLE_CAN=0 -DTEST_ENABLE_FRAM=0 -DTEST_ENABLE_WDG=0 -DTEST_ENABLE_OVP=0 -DTEST_ENABLE_FLASH=0 -DTEST_ENABLE_CONFIG=0"

.PHONY: test-fram
test-fram: clean
	$(MAKE) all EXTRA_CFLAGS="-DBSP_UNIT_TEST -UUART_LOG_ENABLE -DUART_LOG_ENABLE=1 -DTEST_ENABLE_UART_LOG=0 -DTEST_ENABLE_ANALOG=0 -DTEST_ENABLE_DISCHARGE=0 -DTEST_ENABLE_CAN=0 -DTEST_ENABLE_SPI=0 -DTEST_ENABLE_WDG=0 -DTEST_ENABLE_OVP=0 -DTEST_ENABLE_FLASH=0 -DTEST_ENABLE_CONFIG=0"

.PHONY: test-wdg
test-wdg: clean
	$(MAKE) all EXTRA_CFLAGS="-DBSP_UNIT_TEST -UUART_LOG_ENABLE -DUART_LOG_ENABLE=1 -DTEST_ENABLE_UART_LOG=0 -DTEST_ENABLE_ANALOG=0 -DTEST_ENABLE_DISCHARGE=0 -DTEST_ENABLE_CAN=0 -DTEST_ENABLE_SPI=0 -DTEST_ENABLE_FRAM=0 -DTEST_ENABLE_OVP=0 -DTEST_ENABLE_FLASH=0 -DTEST_ENABLE_CONFIG=0"

.PHONY: test-flash
test-flash:
	@echo "FLASH 模块暂时停用：配置已改存 FRAM（见 bsp_config.c / test_bsp_config.c）。"
	@echo "如需启用片上 Flash：在 Makefile 恢复 bsp_flash.c 与 test_bsp_flash.c，并设 -DTEST_ENABLE_FLASH=1。"

.PHONY: test-config
test-config: clean
	$(MAKE) all EXTRA_CFLAGS="-DBSP_UNIT_TEST -UUART_LOG_ENABLE -DUART_LOG_ENABLE=1 -DTEST_ENABLE_UART_LOG=0 -DTEST_ENABLE_ANALOG=0 -DTEST_ENABLE_DISCHARGE=0 -DTEST_ENABLE_CAN=0 -DTEST_ENABLE_SPI=0 -DTEST_ENABLE_FRAM=0 -DTEST_ENABLE_WDG=0 -DTEST_ENABLE_OVP=0 -DTEST_ENABLE_FLASH=0"

.PHONY: test-ovp
test-ovp: clean
	$(MAKE) all EXTRA_CFLAGS="-DBSP_UNIT_TEST -UUART_LOG_ENABLE -DUART_LOG_ENABLE=1 -DTEST_ENABLE_UART_LOG=0 -DTEST_ENABLE_ANALOG=0 -DTEST_ENABLE_DISCHARGE=0 -DTEST_ENABLE_CAN=0 -DTEST_ENABLE_SPI=0 -DTEST_ENABLE_FRAM=0 -DTEST_ENABLE_WDG=0 -DTEST_ENABLE_FLASH=0 -DTEST_ENABLE_CONFIG=0"

# ================================================================
# 实时监视模式 (输入类模块): 循环采集并打印, 供改变物理输入时随动观测/标定。
#   一次性 RunAll 的快照对实时采集无参考价值, 这些目标改跑连续监视循环。
#   烧录后打开串口终端观察, 退出: 发送 q/ESC 或复位 (CAN 发 0xFF)。
#   make analog-live     模拟量(电压/电流/温度/功率/滤波) 随动
#   make discharge-live  泄放 Brake_FB(PA9) 反馈电平 随动 (PWM 全程关断)
#   make uart-live       串口 RX 回显, 验证接收通路
#   make can-loop        CAN 主站命令交互应答, 配合上位机验证收发
# ================================================================
.PHONY: analog-live
analog-live: clean
	$(MAKE) all EXTRA_CFLAGS="-DBSP_UNIT_TEST -DBSP_MONITOR -DMONITOR_ANALOG -UUART_LOG_ENABLE -DUART_LOG_ENABLE=1"

.PHONY: discharge-live
discharge-live: clean
	$(MAKE) all EXTRA_CFLAGS="-DBSP_UNIT_TEST -DBSP_MONITOR -DMONITOR_DISCHARGE -UUART_LOG_ENABLE -DUART_LOG_ENABLE=1"

.PHONY: uart-live
uart-live: clean
	$(MAKE) all EXTRA_CFLAGS="-DBSP_UNIT_TEST -DBSP_MONITOR -DMONITOR_UART -UUART_LOG_ENABLE -DUART_LOG_ENABLE=1"

.PHONY: can-loop
can-loop: clean
	$(MAKE) all EXTRA_CFLAGS="-DBSP_UNIT_TEST -DBSP_MONITOR -DCAN_INTERACTIVE_TEST -UUART_LOG_ENABLE -DUART_LOG_ENABLE=1"

# 专项极限测试: 跑真实 App(App_Init/App_Run 完整保护) + 串口遥测钩子(只读观测)。
#   负载仪器逐保护功能极限测试用; 台架加压/加热/加载时串口看真实固件反应。
#   烧录后打开串口(PB6/115200)看 [TELE] 遥测行与 [ALARM]/[STATE]/[DCH]/[CNT] 事件行。
#   已关 20s 兜底(APP_DCH_MAXON_TIMEOUT_ENABLE=0): 测温升需持续泄放远超20s; 正式固件默认开。
#   详见《泄放从站专项极限测试规程》。可加 -DTELE_PERIOD_MS=100 调遥测周期。
.PHONY: brake-special
brake-special: clean
	$(MAKE) all EXTRA_CFLAGS="-DBSP_UNIT_TEST -DBSP_MONITOR -DMONITOR_BRAKE_SPECIAL -DAPP_DCH_MAXON_TIMEOUT_ENABLE=0 -UUART_LOG_ENABLE -DUART_LOG_ENABLE=1"

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***

#######################################
# 分级泄放功能验证 —— 三独立测试固件（《泄放从站专项极限测试规程》测试需求调整）
#   均基于 brake-special 机制：真实 App(App_Init/App_Run 完整保护) + 串口只读遥测。
#   打开串口(PB6/115200)看 [TELE]（含 A9=Brake_FB 原始电平）与 [ALARM]/[LEVEL]/[DCH]/[CNT] 事件。
#######################################

# 固件A：步骤1-3。看门狗→A9 验证(上电前10s不喂狗观测A9,再自动转正常喂狗) + 一级/二级泄放到55V。
#   BST_WDG_NOFEED_MS=10000：上电前10s不喂外部看门狗，台架观测 A9 电平；到时自动转每圈喂狗。
#   20s 兜底保持开启（步骤3 正常泄放到55V不会触发）。
.PHONY: brake-func
brake-func: clean
	$(MAKE) all EXTRA_CFLAGS="-DBSP_UNIT_TEST -DBSP_MONITOR -DMONITOR_BRAKE_SPECIAL -DBST_WDG_NOFEED_MS=10000 -UUART_LOG_ENABLE -DUART_LOG_ENABLE=1"

# 固件B：步骤4。验证"连续泄放超20s兜底保护"。
#   占空比强制1%(START/MAX/STEP/L2_FULL 全=1)：泄放极弱、永远压不到55V → 触发20s兜底停机。
#   20s 兜底保持开启（本固件正是测它）。
.PHONY: brake-timeout
brake-timeout: clean
	$(MAKE) all EXTRA_CFLAGS="-DBSP_UNIT_TEST -DBSP_MONITOR -DMONITOR_BRAKE_SPECIAL -DBSP_DISCHARGE_DUTY_START=1 -DBSP_DISCHARGE_DUTY_MAX=1 -DBSP_DISCHARGE_DUTY_STEP=1 -DBSP_DISCHARGE_DUTY_L2_FULL=1 -UUART_LOG_ENABLE -DUART_LOG_ENABLE=1"

# 固件C：步骤5。验证过温保护（100℃关断 / 冷却到65℃才重新支持泄放）。
#   过温关断阈值用默认 100℃、恢复阈值 65℃(bsp.h 默认)；100℃置OVER_TEMP禁泄放(60V也不开)，冷却≤65℃才解除。
#   APP_DCH_MAXON_TIMEOUT_ENABLE=0：关20s兜底，避免升温未到100℃就被兜底打断。
.PHONY: brake-otp
brake-otp: clean
	$(MAKE) all EXTRA_CFLAGS="-DBSP_UNIT_TEST -DBSP_MONITOR -DMONITOR_BRAKE_SPECIAL -DAPP_DCH_MAXON_TIMEOUT_ENABLE=0 -UUART_LOG_ENABLE -DUART_LOG_ENABLE=1"
