#include "soft_i2c.h"

#define I2C2_SDA_GPIO       GPIOB
#define I2C2_SDA_GPIO_PIN   GPIO_Pin_7
#define I2C2_SCL_GPIO       GPIOB
#define I2C2_SCL_GPIO_PIN   GPIO_Pin_6

static void stm32_set_sda(void *data, rt_int32_t state);
static void stm32_set_scl(void *data, rt_int32_t state);
static rt_int32_t stm32_get_sda(void *data);
static rt_int32_t stm32_get_scl(void *data);
static void stm32_udelay(rt_uint32_t us);

static const struct  rt_i2c_bit_ops stm32_i2c_bit_ops =
{
    (void*)0xaa,     //no use in set_sda,set_scl,get_sda,get_scl
    stm32_set_sda,
    stm32_set_scl,
    stm32_get_sda,
    stm32_get_scl,
    stm32_udelay,
    20, 
    1
};

rt_err_t rt_soft_i2c_bus_init( struct rt_i2c_bus_device* bus_dev, const char *bus_name )
{
    //static struct rt_i2c_bus_device stm32_i2c[4];
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE );

    GPIO_InitStructure.GPIO_Pin   = I2C2_SDA_GPIO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_Init( I2C2_SDA_GPIO, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin   = I2C2_SCL_GPIO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_Init( I2C2_SCL_GPIO, &GPIO_InitStructure );
    
//    GPIO_InitStructure.GPIO_Pin   = I2C2_SCL_GPIO_PIN;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
//    GPIO_Init(I2C2_SCL_GPIO, &GPIO_InitStructure);
    

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_DeInit( TIM2 );
    TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );
    TIM_TimeBaseStructure.TIM_Prescaler = 71; 
    TIM_TimeBaseStructure.TIM_Period    = 50000;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
    TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure );
    TIM_Cmd( TIM2, ENABLE );

    rt_memset( (void *)bus_dev, 0, sizeof( struct rt_i2c_bus_device ) );
    
    bus_dev->priv = (void *)&stm32_i2c_bit_ops;
    return rt_i2c_bit_add_bus( bus_dev, bus_name );
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
static void stm32_set_sda(void *data, rt_int32_t state)
{
    if(state == 1)
        GPIO_SetBits(I2C2_SDA_GPIO, I2C2_SDA_GPIO_PIN);
    else if(state == 0)
        GPIO_ResetBits(I2C2_SDA_GPIO, I2C2_SDA_GPIO_PIN);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
static void stm32_set_scl(void *data, rt_int32_t state)
{
    if(state == 1)
        GPIO_SetBits(I2C2_SCL_GPIO , I2C2_SCL_GPIO_PIN);
    else if(state == 0)
        GPIO_ResetBits(I2C2_SCL_GPIO , I2C2_SCL_GPIO_PIN);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
static rt_int32_t stm32_get_sda(void *data)
{
    return (rt_int32_t)GPIO_ReadInputDataBit(I2C2_SDA_GPIO , I2C2_SDA_GPIO_PIN);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
static rt_int32_t stm32_get_scl(void *data)
{
    return (rt_int32_t)GPIO_ReadInputDataBit(I2C2_SCL_GPIO , I2C2_SCL_GPIO_PIN);
}

static void stm32_udelay(rt_uint32_t us)
{
    rt_uint32_t delta;
    TIM2->CNT = 0;
    TIM_Cmd( TIM2, ENABLE );
    while (us > TIM2->CNT );
    TIM_Cmd( TIM2, DISABLE );
}
