#ifndef _USER_CONFIG_H
#define _USER_CONFIG_H

// Debug macro
#define _USING_DEBUG        (0x00u)

// App addr and boot size set
#define FLASH_BASE_ADDR     (0x00000000ul)
#define FLASH_BOOT_MAXSIZE  (1024 * 64)
#define FLASH_APP_ADDR      (FLASH_BASE_ADDR + FLASH_BOOT_MAXSIZE)

// Iap macro
#define _USING_IAP          (0x01u)

#endif
