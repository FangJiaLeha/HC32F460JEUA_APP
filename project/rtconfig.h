#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

/* Automatically generated file; DO NOT EDIT. */
/* RT-Thread Configuration */

/* RT-Thread Kernel */

#define RT_NAME_MAX 12
#define RT_ALIGN_SIZE 4
#define RT_THREAD_PRIORITY_32
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 500
#define RT_USING_OVERFLOW_CHECK
#define RT_USING_HOOK
#define RT_USING_IDLE_HOOK
#define RT_IDLE_HOOK_LIST_SIZE 4
#define IDLE_THREAD_STACK_SIZE 256
#define RT_USING_TIMER_SOFT
#define RT_TIMER_THREAD_PRIO 4
#define RT_TIMER_THREAD_STACK_SIZE 512
#define RT_DEBUG
#define RT_DEBUG_COLOR
#define RT_DEBUG_INIT_CONFIG
#define RT_DEBUG_INIT 1
#define RT_DEBUG_MODULE_CONFIG
#define RT_DEBUG_MODULE 1

/* Inter-Thread communication */

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_EVENT
#define RT_USING_MAILBOX
#define RT_USING_MESSAGEQUEUE
#define RT_USING_SIGNALS

/* Memory Management */

#define RT_USING_MEMPOOL
#define RT_USING_MEMHEAP
#define RT_USING_SMALL_MEM
#define RT_USING_MEMTRACE
#define RT_USING_HEAP

/* Kernel Device Object */

#define RT_USING_DEVICE
#define RT_USING_DEVICE_OPS
#define RT_USING_INTERRUPT_INFO
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 128
#define RT_CONSOLE_DEVICE_NAME "uart1"
#define RT_VER_NUM 0x30104

/* RT-Thread Components */

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE 1024
#define RT_MAIN_THREAD_PRIORITY 6

/* C++ features */


/* Command shell */

#define RT_USING_FINSH
#define FINSH_THREAD_NAME "tshell"
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES 5
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
#define FINSH_THREAD_PRIORITY 20
#define FINSH_THREAD_STACK_SIZE 4096
#define FINSH_CMD_SIZE 80
#define FINSH_USING_MSH
#define FINSH_USING_MSH_DEFAULT
#define FINSH_ARG_MAX 20

/* Device virtual file system */

#define RT_USING_DFS
#define DFS_USING_WORKDIR
#define DFS_FILESYSTEMS_MAX 2
#define DFS_FILESYSTEM_TYPES_MAX 2
#define DFS_FD_MAX 4
#define RT_USING_DFS_ELMFAT

/* elm-chan's FatFs, Generic FAT Filesystem Module */

#define RT_DFS_ELM_CODE_PAGE 437
#define RT_DFS_ELM_WORD_ACCESS
#define RT_DFS_ELM_USE_LFN_0
#define RT_DFS_ELM_USE_LFN 0
#define RT_DFS_ELM_MAX_LFN 255
#define RT_DFS_ELM_DRIVES 2
#define RT_DFS_ELM_MAX_SECTOR_SIZE 4096
#define RT_DFS_ELM_USE_ERASE
#define RT_DFS_ELM_REENTRANT
#define RT_USING_DFS_DEVFS

/* Device Drivers */

#define RT_USING_DEVICE_IPC
#define RT_PIPE_BUFSZ 512
#define RT_USING_SERIAL
#define RT_SERIAL_USING_DMA
#define RT_SERIAL_RB_BUFSZ 64
#define RT_USING_CAN
#define RT_USING_I2C
#define RT_USING_I2C_BITOPS
#define RT_USING_PIN
#define RT_USING_PWM
#define RT_USING_WDT
#define RT_USING_SENSOR
#define RT_USING_SENSOR_CMD

/* Using USB */


/* POSIX layer and C standard library */

#define RT_USING_LIBC

/* Network */

/* Socket abstraction layer */


/* Network interface device */


/* light weight TCP/IP stack */


/* AT commands */


/* VBUS(Virtual Software BUS) */


/* Utilities */

#define RT_USING_ULOG
#define ULOG_OUTPUT_LVL_W
#define ULOG_OUTPUT_LVL 4
#define ULOG_USING_ISR_LOG
#define ULOG_ASSERT_ENABLE
#define ULOG_LINE_BUF_SIZE 128

/* log format */

#define ULOG_OUTPUT_FLOAT
#define ULOG_USING_COLOR
#define ULOG_OUTPUT_TIME
#define ULOG_OUTPUT_LEVEL
#define ULOG_OUTPUT_TAG
#define ULOG_OUTPUT_THREAD_NAME
#define ULOG_BACKEND_USING_CONSOLE

/* RT-Thread online packages */

/* IoT - internet of things */


/* Wi-Fi */

/* Marvell WiFi */


/* Wiced WiFi */


/* IoT Cloud */


/* security packages */


/* language packages */


/* multimedia packages */


/* tools packages */

#define PKG_USING_CMBACKTRACE
#define PKG_CMBACKTRACE_PLATFORM_M4
#define PKG_CMBACKTRACE_DUMP_STACK
#define PKG_CMBACKTRACE_PRINT_ENGLISH
#define PKG_USING_CMBACKTRACE_V10400
#define PKG_CMBACKTRACE_VER_NUM 0x10400

/* system packages */

/* acceleration: Assembly language or algorithmic acceleration packages */


/* Micrium: Micrium software products porting for RT-Thread */


/* peripheral libraries and drivers */


/* AI packages */


/* miscellaneous packages */


/* samples: kernel and components samples */


/* entertainment: terminal games and other interesting software packages */


/* Select Chip Type */

#define CHIP_SEL_HC32F460JEUA

/* Hardware Drivers Config */

#define SOC_GD32303R
#define SOC_USING_USART1
#define SOC_USING_I2C0
#define SOC_USING_CAN0
#define SOC_USING_PWM0
#define SOC_USING_PWM0_CH0
#define SOC_USING_ONEWIRE3
#define SOC_USING_ONEWIRE3_CH2

/* Pudu BSP Config */

/* Device Drivers */

#define RT_USING_ONEWIRE

/* BSP Pallet Check */

#define BSP_USING_PALLET_LED_BAR
#define BSP_PALLET_LED_BAR_PWMX "pwm0"
#define BSP_PALLET_LED_BAR_CHANNEL 1
#define BSP_PALLET_LED_BAR_DEV "PLedBar"
#define BSP_USING_HEARTBEAT_LED
#define BSP_HEARTBEAT_PIN 32

/* PCF8574 */

#define BSP_USING_I2C_PCF8574
#define BSP_I2C_PCF8574_BUS "i2c1"
#define BSP_I2C_PCF8574_DEV "touch_io"
#define BSP_I2C_PCF8574_ADDRESS 0x27

/* TLC59108 */

#define BSP_USING_I2C_TLC59108
#define BSP_I2C_TLC59108_BUS "i2c1"
#define BSP_I2C_TLC59108_DEV "tlc59108"
#define BSP_I2C_TLC59108_ADDRESS 0x4F

/* VL53L0X */

#define BSP_USING_I2C_VL53L0X
#define BSP_I2C_VL53L0X_BUS "i2c0"
#define BSP_I2C_VL53L0X_DEV "vl53l0x"
#define BSP_I2C_VL53L0X_ADDRESS 0x29
#define BSP_I2C_VL53L0X_XSHUT1_PIN 17
#define BSP_I2C_VL53L0X_XSHUT2_PIN 18
#define BSP_I2C_VL53L0X_XSHUT3_PIN 19
#define BSP_I2C_VL53L0X_XSHUT4_PIN 20

/* WS2812 */

#define BSP_USING_PWM_WS2812
#define BSP_PWM_WS2812_PWMX "onewire3"
#define BSP_PWM_WS2812_PWM_CHANNEL 2
#define BSP_PWM_WS2812_DEV "ws2812"

/* Pudu C-Packages Config */

/* led ctrl frame */

#define BSP_USING_LED_FRAME_MOD

/* Applications Config */


#endif
