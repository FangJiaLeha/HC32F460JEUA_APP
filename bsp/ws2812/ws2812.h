#ifndef __WS2812_H__
#define __WS2812_H__

#include <rtdevice.h>

struct WS2812_DEV {
    struct rt_device parent;
    uint32_t led_num;
    uint16_t *dma_buff;
    uint8_t (*render_buff)[3];
    struct rt_event dma_evt;
    struct rt_mutex lock;
    rt_timer_t update_timer;
    // uint32_t update_pos;
    // uint32_t update_count;
    uint8_t need_update;
    
    struct rt_device_onewire *onewire_dev;
    int onewire_channel;
};

struct WS2812_BAR_CTRLPACK{
    uint8_t color[3];
    uint32_t start;
    uint32_t count;
};

rt_err_t init_ws2812_device( struct WS2812_DEV *dev );

#define WS2812_CTRL_INIT                   0x105
#define WS2812_CTRL_GET_DISBUFF            0x106
#define WS2812_CTRL_UPDATE_DEVDATA         0X107
#define WS2812_CTRL_BAR_COLOR              0x108

// #define DEVICE_CTRL_SET_RGBLED_BAR         0x100
// #define DEVICE_CTRL_ACTIVE_RGBLED_BAR      0x101
// #define DEVICE_CTRL_DISACTIVE_RGBLED_BAR   0x102

struct RGBLED_BAR_CTRL_PACK{
    uint8_t id;
    uint8_t color[3]; //!< 起始颜色值
    uint8_t end_color[3]; //!< 终止颜色值
    uint8_t mode;
    /**
     * @brief 控制参数
     * 如果 mode 为BAR_CTRL_MODE_SECTRL（分段控制模式）则参数值范围为0~15，单位为100ms
     * 
     */
    uint32_t arg;
};

#endif
