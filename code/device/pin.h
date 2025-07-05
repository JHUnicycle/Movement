#ifndef _PIN_H_
#define _PIN_H_

#define EMPTY_nSLEEP P33_0

// camera1
#define CAMERA1_COF_UART (UART_1) // 摄像头配置所使用的串口
#define CAMERA1_COF_BAUD (9600) // 摄像头配置串口波特率
#define CAMERA1_COF_UART_TX (UART1_RX_P11_10) // 摄像头配置串口 TX 引脚
#define CAMERA1_COF_UART_RX (UART1_TX_P11_12) // 摄像头配置串口 RX 引脚
// 以上四个暂时没用，但是线画了，先留着
#define CAMERA1_COF_IIC_SCL (P11_10)
#define CAMERA1_COF_IIC_SDA (P11_12)
#define CAMERA1_VSYNC_PIN (ERU_CH5_REQ1_P15_8) // 场中断引脚
#define CAMERA1_PCLK_PIN (ERU_CH0_REQ0_P15_4)  // PCLK 触发信号 TIM_ETR 引脚禁止随意修改
#define CAMERA1_DATA_PIN (P02_0)
#define CAMERA1_DMA_CH (IfxDma_ChannelId_7)
// camera2
#define CAMERA2_COF_UART (UART_10) // 摄像头配置所使用的串口
#define CAMERA2_COF_BAUD (9600) // 摄像头配置串口波特率
#define CAMERA2_COF_UART_TX (UART10_RX_P13_1) // 摄像头配置串口 TX 引脚
#define CAMERA2_COF_UART_RX (UART10_TX_P13_0) // 摄像头配置串口 RX 引脚
// 以上四个暂时没用，但是线画了，先留着
#define CAMERA2_COF_IIC_SCL (P13_1)
#define CAMERA2_COF_IIC_SDA (P13_0)
#define CAMERA2_VSYNC_PIN (ERU_CH2_REQ2_P10_2) // 场中断引脚
#define CAMERA2_PCLK_PIN (ERU_CH3_REQ3_P10_3)  // PCLK 触发信号 TIM_ETR 引脚禁止随意修改
#define CAMERA2_DATA_PIN (P00_0)               // 数据引脚 这里是 只能是 GPIOx0 或者 GPIOx8 开始 连续八个引脚例如 P00_0-P00_7
#define CAMERA2_DMA_CH (IfxDma_ChannelId_6)

// encoder
#define ENCODER_BOTTOM (TIM4_ENCODER)
#define ENCODER_PIN0_BOTTOM (TIM4_ENCODER_CH1_P02_8)
#define ENCODER_PIN1_BOTTOM (TIM4_ENCODER_CH2_P00_9)

// motor
#define MOTOR_BOTTOM_PIN (ATOM2_CH4_P33_8)
#define MOTOR_BOTTOM_DIR (P33_7)
#define MOTOR_BOTTOM_ENABLE (EMPTY_nSLEEP)
#define MOTOR_MOMENTUM_BAUDRATE (230400)
#define MOTOR_LEFT_UART (UART_1)
#define MOTOR_LEFT_RX (UART1_TX_P20_10)
#define MOTOR_LEFT_TX (UART1_RX_P20_9)
#define MOTOR_RIGHT_UART (UART_6)
#define MOTOR_RIGHT_RX (UART6_TX_P22_0)
#define MOTOR_RIGHT_TX (UART6_RX_P23_1)

// wireless uart
#define WL_UART_INDEX (UART_3)          // 无线串口对应使用的串口号
#define WL_UART_BUAD_RATE (230400)      // 无线串口对应使用的串口波特率
#define WL_UART_TX_PIN (UART3_TX_P21_7) // 单片机TX引脚
#define WL_UART_RX_PIN (UART3_RX_P20_3) // 单片机RX引脚
#define WL_UART_RTS_PIN (P20_0)         // 无线串口对应模块的 RTS 引脚
#define WL_UART_RST_PIN (P20_2)         // 无线串口对应模块的 RST 引脚（WiFi SPI用到 RST 引脚， 先留着）

// tft180
#define LCD_SPI (SPI_2)                   // TFT180 使用的 SPI 号
#define LCD_SCL_PIN (SPI2_SCLK_P15_6)    // TFT180 硬件 SPI SCK 引脚
#define LCD_SDA_PIN (SPI2_MOSI_P15_5)    // TFT180 硬件 SPI MOSI 引脚
#define LCD_SDA_PIN_IN (SPI2_MISO_P15_2) // TFT180 硬件 SPI MISO 引脚
#define LCD_RES_PIN (P15_3)               // TFT180 液晶复位引脚定义
#define LCD_DC_PIN (P15_7)                // TFT180 液晶命令位引脚定义
#define LCD_CS_PIN (P14_0)                // TFT180 CS 片选引脚
#define LCD_BL_PIN (P14_1)                // TFT180 液晶背光引脚定义

// sd card
#define SD_SPIN (SPI_1)
#define SD_SCL (SPI1_SCLK_P11_6)
#define SD_SDA (SPI1_MOSI_P11_9)
#define SD_SDA_IN (SPI1_MISO_P11_3)
#define SD_CS (SPI1_CS5_P11_2)
#define SD_CS_PIN (P11_2)

// imu
#define IMU_SPI (SPI_4)
#define IMU_SPC_PIN (SPI4_SCLK_P33_11) // 硬件 SPI SCK 引脚
#define IMU_SDI_PIN (SPI4_MOSI_P33_12) // 硬件 SPI MOSI 引脚
#define IMU_SDO_PIN (SPI4_MISO_P33_13) // 硬件 SPI MISO 引脚
#define IMU_CS_PIN (P33_10)            // CS 片选引脚

// key
#define MKEY_LIST P21_6, P21_4, P21_2, P22_2, P21_5

// switch
#define SWITCH_NAME SWITCH_1, SWITCH_2, SWITCH_3, SWITCH_4
#define SWITCH_LIST P33_4, P20_6, P20_8, P20_7

// LED&BUZZER
#define DIODE_NAME BUZZER
#define DIODE_LIST P33_6

#endif