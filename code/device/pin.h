#ifndef _PIN_H_
#define _PIN_H_

// camera
#define CAMERA1_COF_IIC_SCL (P10_5)
#define CAMERA1_COF_IIC_SDA (P10_6)
#define CAMERA1_VSYNC_PIN (ERU_CH5_REQ1_P15_8) // 场中断引脚
#define CAMERA1_PCLK_PIN (ERU_CH0_REQ0_P15_4)  // PCLK 触发信号 TIM_ETR 引脚禁止随意修改
#define CAMERA1_DATA_PIN (P02_0)
#define CAMERA1_DMA_CH (IfxDma_ChannelId_7)

#define CAMERA2_COF_IIC_SCL (P00_8)
#define CAMERA2_COF_IIC_SDA (P10_1)
#define CAMERA2_VSYNC_PIN (ERU_CH2_REQ2_P10_2) // 场中断引脚
#define CAMERA2_PCLK_PIN (ERU_CH3_REQ3_P10_3)  // PCLK 触发信号 TIM_ETR 引脚禁止随意修改
#define CAMERA2_DATA_PIN (P00_0)               // 数据引脚 这里是 只能是 GPIOx0 或者 GPIOx8 开始 连续八个引脚例如 P00_0-P00_7
#define CAMERA2_DMA_CH (IfxDma_ChannelId_6)

// encoder
#define ENCODER_BOTTOM TIM4_ENCODER
#define ENCODER_PIN0_BOTTOM TIM4_ENCODER_CH1_P02_8
#define ENCODER_PIN1_BOTTOM TIM4_ENCODER_CH2_P00_9

// motor
#define MOTOR_BOTTOM_PIN ATOM2_CH1_P33_5
#define MOTOR_BOTTOM_DIR P33_4
#define MOTOR_BOTTOM_ENABLE P33_6

// wireless uart
#define WL_UART_INDEX (UART_2)          // 无线串口对应使用的串口号
#define WL_UART_BUAD_RATE (230400)      // 无线串口对应使用的串口波特率
#define WL_UART_TX_PIN (UART2_TX_P14_2) // 单片机TX引脚
#define WL_UART_RX_PIN (UART2_RX_P14_3) // 单片机RX引脚
#define WL_UART_RTS_PIN (P14_4)         // 无线串口对应模块的 RTS 引脚

// tft180
#define LCD_SPI (SPI_0)                   // TFT180 使用的 SPI 号
#define LCD_SCL_PIN (SPI0_SCLK_P20_11)    // TFT180 硬件 SPI SCK 引脚
#define LCD_SDA_PIN (SPI0_MOSI_P20_14)    // TFT180 硬件 SPI MOSI 引脚
#define LCD_SDA_PIN_IN (SPI0_MISO_P20_12) // TFT180 硬件 SPI MISO 引脚
#define LCD_RES_PIN (P22_1)               // TFT180 液晶复位引脚定义
#define LCD_DC_PIN (P22_2)                // TFT180 液晶命令位引脚定义
#define LCD_CS_PIN (P20_13)               // TFT180 CS 片选引脚
#define LCD_BL_PIN (P22_3)                // TFT180 液晶背光引脚定义

// sd card
#define SD_SPIN (SPI_4)
#define SD_SCL (SPI4_SCLK_P33_11)
#define SD_SDA (SPI4_MOSI_33_12)
#define SD_SDA_IN (SPI4_MISO_P33_13)
#define SD_CS (SPI4_CS0_P33_10)
#define SD_CS_PIN (P33_10)

// imu
#define IMU_SPI (SPI_2)
#define IMU_SPC_PIN (SPI2_SCLK_P15_3) // 硬件 SPI SCK 引脚
#define IMU_SDI_PIN (SPI2_MOSI_P15_5) // 硬件 SPI MOSI 引脚
#define IMU_SDO_PIN (SPI2_MISO_P15_7) // 硬件 SPI MISO 引脚
#define IMU_CS_PIN (P14_6)            // CS 片选引脚

#endif