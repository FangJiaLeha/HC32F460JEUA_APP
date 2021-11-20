#ifndef _IAP_H
#define _IAP_H

#include "hc32_ddl.h"
#include "user_config.h"

#define COMPARE_VALUE(value1, value2)   ( value1 == value2 ? 1 : 0 )

#if defined(_USING_IAP)
void iap_control(uint16_t id, uint8_t *req, uint8_t req_len, uint8_t *ret, uint32_t *ret_len);
#endif

#endif
