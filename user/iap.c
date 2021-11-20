#include "iap.h"
#include "config.h"

//==============================================================================
typedef enum mcu_work_mode_type
{
    WORK_IN_APP,
    WORK_IN_BOOT
}McuWorkModeType;

typedef enum can_cmd_list_type
{
    CmdSoftwareReset = 0x01u,
    CmdCheckBootOrApp,
    CmdGetSoftwareVersion,
    CmdOpenOrCloseCan
}CanCmdListType;

//==============================================================================
static uint8_t get_check_sum(uint8_t *canBuff, uint8_t canLen)
{
	uint8_t cnt,sum = 0;
	for (cnt = 0; cnt < canLen; cnt++)
	{
		sum += *canBuff;
		canBuff++;
	}
	return sum;
}

//==============================================================================
#if ( defined(_USING_IAP) && (_USING_IAP == 0x01u) )
void iap_control(uint16_t id, uint8_t *req, uint8_t req_len, uint8_t *ret, uint32_t *ret_len)
{
    uint8_t can_cmd_type, check_sum;
    *ret_len = 0;
    extern uint8_t get_pallet_id( void );
    if (req[1] != get_pallet_id() + 0x07u) {
    } else {
        can_cmd_type = req[2];
        switch(can_cmd_type)
        {
            case CmdSoftwareReset:
            case CmdOpenOrCloseCan:
                check_sum = get_check_sum(req, 7);
                if ( COMPARE_VALUE(check_sum, req[7]) ) {
                    *ret_len = req_len;
                    memmove(ret, (const void *)req, req_len);
                }
            break;
            case CmdCheckBootOrApp:
                check_sum = get_check_sum(req, 7);
                if ( COMPARE_VALUE(check_sum, req[7]) ) {
                    *ret_len = req_len;
                    memmove(ret, (const void *)req, req_len);
                    ret[3] = WORK_IN_APP;
                    ret[7] = get_check_sum(ret, 7);
                }
            break;
            case CmdGetSoftwareVersion:
                check_sum = get_check_sum(req, 7);
                if ( COMPARE_VALUE(check_sum, req[7]) ) {
                    *ret_len = req_len;
                    memmove(ret, (const void *)req, req_len);
                    ret[3] = (PROGRAM_VERSION >> 16) & 0xFF;
                    ret[4] = (PROGRAM_VERSION >> 8)  & 0xFF;
                    ret[5] = (PROGRAM_VERSION >> 0)  & 0xFF;
                    ret[7] = get_check_sum(ret, 7);
                }
            break;
            default:
            break;
        }
    }
}
#endif
