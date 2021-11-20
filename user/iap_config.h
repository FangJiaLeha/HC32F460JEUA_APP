#ifndef _IAP_CONFIG_H
#define _IAP_CONFIG_H

#include "hc32_ddl.h"
#include "user_config.h"

/***************************************************************************************************/
// RAM base address.
#define RESET_SIGNATURE_ADDRESS                   ( 0x1FFF8000 )
// Offset base of @FLASH_APP_ADDR.
#define FIRMWARE_SIGNATURE_OFFSET                 ( 0xF80 )
/***************************************************************************************************
                                        
***************************************************************************************************/
// The two macros are used to app jump to boot for ensuring that MCU work in boot mode.
#define RESET_SIGNATURE0_UPDATE_FIRMWARE          ( 0XFD002244 )
#define RESET_SIGNATURE1_UPDATE_FIRMWARE          ( 0XFD113355 )

// The four macros are used to debug and release.
#define FIRMWARE_SIGNATURE0_DEBUG                 ( 0XFDEB0044 )
#define FIRMWARE_SIGNATURE1_DEBUG                 ( 0XFDEB1155 )
#define FIRMWARE_SIGNATURE0_RELEASE               ( 0XFE00AACC )
#define FIRMWARE_SIGNATURE1_RELEASE               ( 0XFE117799 )

// The macro @FIRMWARE_SIZE_SIGNATURE is used to CRC32.exe for the updated firmware 
#define FIRMWARE_SIZE_SIGNATURE                   ( 0x12345678 )
/**************************************************************************************************/
extern volatile uint32_t ResetSignature[2];
#endif
