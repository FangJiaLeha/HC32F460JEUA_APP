/**
 * @file            bsp.c
 * @author          philZeng@outlook.com
 * @version         V0.0.1
 * @date            2018年8月6号
 * @brief           功能板板级支持
 * @attention       
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <soc.h>
#include "bsp.h"

static void init_bsp_gpio(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE );

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init( GPIOB, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_14 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOC, &GPIO_InitStructure );
}

uint8_t is_pallet_installed(void)
{
    if( GPIO_ReadInputDataBit( GPIOB, GPIO_Pin_2 ) )
    {
        return 0;
    }
    return 1;
}

uint8_t get_pallet_id_low( void )
{
    uint8_t id = 0;
    if ( GPIO_ReadInputDataBit( GPIOC, GPIO_Pin_14 ) )
    {
        id |= 0x01;
    }
    if( GPIO_ReadInputDataBit( GPIOC, GPIO_Pin_13 ) )
    {
        id |= 0x02;
    }
    return id;
}

#ifdef USING_WS2812_LED_DEV
static struct WS2812_DEV ws2812_dev;
#endif

#ifdef USING_ADFLED_MATRIX_DEV
static struct ADFLED_MATRIX_DEV adfled_dev;
#endif

#ifdef RT_USING_SPI_W25QXX
static struct w25qxx_dev w25qxx_device;
#endif

#ifdef USING_EXTEND_IO_DEV
static struct IO_EXTEND_DEV io_extend_dev;
#endif

#ifdef RT_USING_I2C

#ifdef RT_USING_I2C_BITOPS
#define I2C_BUS_NAME        "i2cs1" //"i2cs1"
#elif defined RT_USING_I2C3
#define I2C_BUS_NAME        "i2c3" //"i2cs1"
#elif defined RT_USING_FMPI2C1
#define I2C_BUS_NAME        "fmpi2c1"
#endif

#ifdef RT_USING_I2C_BITOPS
static struct rt_i2c_bus_device soft_i2cbus_dev;
static struct rt_i2c_bus_device touch_soft_i2cbus_dev;
#endif

#ifdef RT_USING_I2C_TLC59108
static struct TLC59108_DEV tlc59108_dev;
#endif

#ifdef RT_USING_I2C_PCF8574
static struct PCF8574_DEV touch_oi;
#endif

#ifdef RT_USING_I2C_VL53L0X
#define VL53L0X_ID_BASE             0x30
static VL53L0X_Dev_t vl53l0x[4];

//static VL53L0X_Dev_t vfoot[2];

static const uint16_t pin_list[4] = { GPIO_Pin_4,  GPIO_Pin_5,  GPIO_Pin_6,  GPIO_Pin_7};

static void init_vl53l0x_xshut_pin(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_Init( GPIOA, &GPIO_InitStructure );
}

static void vl53l0x_enable( uint8_t id, uint8_t enable )
{
    rt_device_t dev;
    int cmd;
    uint8_t chip_num = ( id - VL53L0X_ID_BASE ) % 4;
    uint8_t pin_num;
    
    if( enable )
    {
        //cmd = PCF8574_CTRL_SETBIT;
        GPIO_SetBits( GPIOA, pin_list[chip_num] );
    } else {
        //cmd = PCF8574_CTRL_CLRBIT;
        GPIO_ResetBits( GPIOA, pin_list[chip_num] );
    }
}
#endif // end #ifdef RT_USING_I2C_VL53L0X

#endif // end #ifdef RT_USING_I2C



#ifdef USING_PALLET_LED_BAR_DEV
static struct rt_device pallet_led_bar_dev;
#endif

#ifdef USING_HEARTBEAT_LED_DEV
static struct rt_device heartbeat;
#endif

void rt_hw_board_init(void)
{
    int i;
    char name[RT_NAME_MAX] = { 'v','l','5','3','l','0','x', 0 };
    rt_hw_soc_init();
    init_bsp_gpio();
#ifdef USING_WS2812_LED_DEV
    init_ws2812_device( &ws2812_dev );
#endif


#ifdef RT_USING_I2C

#ifdef RT_USING_I2C_BITOPS
    rt_soft_i2c_bus_init( &soft_i2cbus_dev, I2C_BUS_NAME );
    touch_soft_i2c_bus_init( &touch_soft_i2cbus_dev, "i2cs2" );
#endif

#ifdef RT_USING_I2C_PCF8574
    pcf8574_regist_and_connnect_i2c( &touch_oi, "touch_io", "i2cs2", 0x27);
#endif

#ifdef RT_USING_I2C_TLC59108
    tlc59108_regist_and_connect_i2c( &tlc59108_dev, "tlc59108", "i2cs2", 0x4F );
#endif

#ifdef RT_USING_I2C_VL53L0X
    init_vl53l0x_xshut_pin();
    for( i = 0; i < 4; i++ )
    {
        name[7] = '0' + i;
        vl53l0x_regist_and_connect_i2c( (VL53L0X_DEV)&vl53l0x[i], ( VL53L0X_ID_BASE + i), name, I2C_BUS_NAME, vl53l0x_enable );
    }
    
    //vl53l0x_regist_and_connect_i2c( (VL53L0X_DEV)&vl53l0x[i], ( VL53L0X_ID_BASE + i), name, I2C_BUS_NAME, vl53l0x_enable );
#endif

#endif

#ifdef USING_PWR_DEV
regist_power_device( &pwr_dev, "PWRDEV" );
#endif

#ifdef BSP_USING_TOUCH_DEV
touch_dev_register( &touch_dev, "TOUCHDEV" );
#endif

#ifdef USING_PALLET_LED_BAR_DEV
    init_pallet_led_device( &pallet_led_bar_dev, "PLedBar" );
#endif

#ifdef USING_HEARTBEAT_LED_DEV
    init_heartbeat_led_dev( &heartbeat, "HeartBeat" );
#endif

}
