#include "soft_i2c.h"

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * 168;
    while ((DWT->CYCCNT - start) < ticks);
}

void IIC_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    IIC_SCL(1); IIC_SDA(1);
    delay_us(10);

    /* 发9个时钟脉冲释放可能卡死的从机 */
    for (uint8_t i = 0; i < 9; i++)
    {
        IIC_SCL(0); delay_us(5);
        IIC_SCL(1); delay_us(5);
    }
    /* Stop条件 */
    IIC_SCL(0); delay_us(5);
    IIC_SDA(0); delay_us(5);
    IIC_SCL(1); delay_us(5);
    IIC_SDA(1); delay_us(5);
}

int IIC_Start(void)
{
    SDA_OUT();
    IIC_SDA(1); delay_us(5);
    IIC_SCL(1); delay_us(5);
    IIC_SDA(0); delay_us(5);
    IIC_SCL(0); delay_us(5);
    return 1;
}

void IIC_Stop(void)
{
    SDA_OUT();
    IIC_SCL(0); delay_us(5);
    IIC_SDA(0); delay_us(5);
    IIC_SCL(1); delay_us(5);
    IIC_SDA(1); delay_us(5);
}

int IIC_Wait_Ack(void)
{
    uint8_t timeout = 0;
    SDA_IN();
    delay_us(5);
    IIC_SCL(1);
    delay_us(5);
    while (READ_SDA)
    {
        if (++timeout > 50)
        {
            IIC_Stop();
            return 0;
        }
        delay_us(1);
    }
    IIC_SCL(0);
    delay_us(5);
    return 1;
}

void IIC_Ack(void)
{
    IIC_SCL(0);
    SDA_OUT();
    IIC_SDA(0); delay_us(5);
    IIC_SCL(1); delay_us(5);
    IIC_SCL(0); delay_us(5);
}

void IIC_NAck(void)
{
    IIC_SCL(0);
    SDA_OUT();
    IIC_SDA(1); delay_us(5);
    IIC_SCL(1); delay_us(5);
    IIC_SCL(0); delay_us(5);
}

void IIC_Send_Byte(uint8_t txd)
{
    SDA_OUT();
    IIC_SCL(0);
    for (uint8_t t = 0; t < 8; t++)
    {
        IIC_SDA((txd & 0x80) >> 7);
        txd <<= 1;
        delay_us(5);
        IIC_SCL(1); delay_us(5);
        IIC_SCL(0); delay_us(5);
    }
}

uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t receive = 0;
    SDA_IN();
    for (uint8_t i = 0; i < 8; i++)
    {
        IIC_SCL(0); delay_us(5);
        IIC_SCL(1); delay_us(5);
        receive <<= 1;
        if (READ_SDA) receive++;
    }
    IIC_SCL(0); delay_us(5);
    if (ack) IIC_Ack();
    else     IIC_NAck();
    return receive;
}

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    if (!IIC_Start()) return 1;
    IIC_Send_Byte(addr << 1);
    if (!IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    IIC_Send_Byte(reg);
    if (!IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    for (uint8_t i = 0; i < len; i++)
    {
        IIC_Send_Byte(data[i]);
        if (!IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    }
    IIC_Stop();
    return 0;
}

int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!IIC_Start()) return 1;
    IIC_Send_Byte(addr << 1);
    if (!IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    IIC_Send_Byte(reg);
    if (!IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 0x01);
    if (!IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    while (len)
    {
        *buf = IIC_Read_Byte(len > 1 ? 1 : 0);
        buf++;
        len--;
    }
    IIC_Stop();
    return 0;
}

uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
    uint8_t count = 0;
    IIC_Start();
    IIC_Send_Byte(dev);
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(dev | 0x01);
    IIC_Wait_Ack();
    for (count = 0; count < length; count++)
    {
        if (count != length - 1) data[count] = IIC_Read_Byte(1);
        else                     data[count] = IIC_Read_Byte(0);
    }
    IIC_Stop();
    return count;
}

uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
    IIC_Start();
    IIC_Send_Byte(dev);
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    for (uint8_t count = 0; count < length; count++)
    {
        IIC_Send_Byte(data[count]);
        IIC_Wait_Ack();
    }
    IIC_Stop();
    return 1;
}

uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data)
{
    return (uint8_t)(i2cRead(dev >> 1, reg, 1, data) == 0 ? 1 : 0);
}

uint8_t IICwriteByte(uint8_t dev, uint8_t reg, uint8_t data)
{
    return IICwriteBytes(dev, reg, 1, &data);
}
