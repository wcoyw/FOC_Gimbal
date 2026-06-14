#ifndef __SOFT_I2C_H__
#define __SOFT_I2C_H__

#include "main.h"

/* PB8=SCL, PB9=SDA
 * STM32F4 MODER寄存器：每引脚2位
 *   00=输入  01=输出  10=复用  11=模拟
 * PB8 → MODER[17:16]   PB9 → MODER[19:18]
 */

/* SDA(PB9) 切换为推挽输出 */
#define SDA_OUT() do {                          \
    GPIOB->MODER &= ~(0x3U << 18);             \
    GPIOB->MODER |=  (0x1U << 18);             \
} while(0)

/* SDA(PB9) 切换为输入 */
#define SDA_IN()  do {                          \
    GPIOB->MODER &= ~(0x3U << 18);             \
} while(0)

/* GPIO位操作 */
#define IIC_SCL(x)  do { if(x) GPIOB->BSRR = GPIO_PIN_8; \
                         else   GPIOB->BSRR = (uint32_t)GPIO_PIN_8 << 16; } while(0)
#define IIC_SDA(x)  do { if(x) GPIOB->BSRR = GPIO_PIN_9; \
                         else   GPIOB->BSRR = (uint32_t)GPIO_PIN_9 << 16; } while(0)
#define READ_SDA    ((GPIOB->IDR & GPIO_PIN_9) ? 1 : 0)

void    IIC_Init(void);
int     IIC_Start(void);
void    IIC_Stop(void);
int     IIC_Wait_Ack(void);
void    IIC_Ack(void);
void    IIC_NAck(void);
void    IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(uint8_t ack);

int     i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int     i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data);
uint8_t IICwriteByte(uint8_t dev, uint8_t reg, uint8_t data);

#endif /* __SOFT_I2C_H__ */
