#include "iap_config.h"

/*******************************************************************************
The variables and address defined in these segments are not allowed to be modifed    
*******************************************************************************/
volatile uint32_t ResetSignature[2] __attribute__( ( at( RESET_SIGNATURE_ADDRESS + 0x00u) ) );

/*******************************************************************************
                                    Bootloader
*******************************************************************************/
#if ( defined( _USING_DEBUG ) &&  (_USING_DEBUG != 0))  // Set the macro @_USING_DEBUG to 0x01 for debug
const uint32_t FirmwareSignature[2] __attribute__( ( at( FLASH_APP_ADDR + FIRMWARE_SIGNATURE_OFFSET + 0x00u ) ) ) =
{
    FIRMWARE_SIGNATURE0_DEBUG,
    FIRMWARE_SIGNATURE1_DEBUG
};
#else   // Reset the macro @_USING_DEBUG to 0x00 for release
const uint32_t FirmwareSignature[2] __attribute__( ( at( FLASH_APP_ADDR + FIRMWARE_SIGNATURE_OFFSET + 0x00u ) ) ) = 
{
    FIRMWARE_SIGNATURE0_RELEASE,
    FIRMWARE_SIGNATURE1_RELEASE
};
#endif
// The variable @FirmwareSize is used to CRC32.exe for firmware that update. It must be valued @FIRMWARE_SIZE_SIGNATURE(0x12345678).
const uint32_t FirmwareSize __attribute__( ( at( FLASH_APP_ADDR + FIRMWARE_SIGNATURE_OFFSET + 0x08u ) ) ) = FIRMWARE_SIZE_SIGNATURE;

// The variable @mcu_type is used to check firmware mcu whether valid in bootloader.
const uint8_t mcu_type[12]  __attribute__( ( at( FLASH_APP_ADDR + FIRMWARE_SIGNATURE_OFFSET + 0x0Cu ) ) ) = "hc32f460";

