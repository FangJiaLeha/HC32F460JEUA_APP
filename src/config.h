#ifndef __CONFIG_H__
#define __CONFIG_H__
#undef STM32F10X_LD
#undef STM32F10X_LD_VL
#undef STM32F10X_MD
#undef STM32F10X_MD_VL
#undef STM32F10X_HD
#undef STM32F10X_HD_VL
#undef STM32F10X_XL
#undef STM32F10X_CL

/**
 * @defgroup configs 嵌入式软件库配置
 *
 * @brief 这是嵌入式软件库的配置描述，入口为config.h文件
 *
 * @{
 */



/**
 * @brief      生成程序版本号
 *
 * @param      major   主版本号
 * @param      minor   次版本号
 * @param      suffix  修订版本号
 *
 * @return     返回生成的版本号数据类型为uint32
 */
#define MK_PROGRAM_VERSION( major, minor, suffix )			\
( ( ( (major) & 0xff ) << 16 ) |  ( ( (minor) & 0xff ) << 8 ) | ( (suffix) & 0xff) )

/**
 * @brief      生成硬件版本号
 *
 * @param      major   主版本号
 * @param      minor   次版本号
 * @param      suffix  修订版本号
 *
 * @return     返回生成的版本号数据类型为uint32
 */
#define MK_HARDWARE_VERSION( major, minor, suffix )			\
( ( ( (major) & 0xff ) << 16 ) |  ( ( (minor) & 0xff ) << 8 ) | ( (suffix) & 0xff) )

/**
 * @}
 * <!-- end CONSTANT_DEFS 配置相关常量定义 -->
 */

/**
 * @defgroup soc_base_configs soc 相关配置
 * @{
 */

/**
 * 使能CMSIS模块
 */
#define USING_CMSIS



/**
 * 使能rt thread 实时系统， rt-thread配置参考 @ref rt-thread_configs
 */
#define USE_RT_THREAD_OS


/**
 * 当前程序版本，即如果PROGRAM_ROLE为ROLE_BOOTLOADER则PROGRAM_VERSION为bootloader版本号，
 * 版本编码规则参考@ref
 */
#define PROGRAM_VERSION			MK_PROGRAM_VERSION( 0, 1 , 8 )

/**
 * 配置当前硬件版本，版本编码规则参考 @ref
 */
#define HARDWARE_VERSION		MK_HARDWARE_VERSION( 0, 1, 0 )

//start SOC_SERIAL
#define STM32F10X_HD
//end SOC_SERIAL

#define ROM_START_ADDR 0x8000000

#define ROM_MAXLEN 0x40000

#define SRAM_START_ADDR  0x20000000

#define SRAM_MAXLEN 0xc000


/**
 * @}
 * <!-- end configs 嵌入式软件库配置 -->
 */

#ifdef __CC_ARM
#define inline	__inline
#endif

#endif
