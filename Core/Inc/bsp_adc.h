#ifndef CPROJECT_BSP_ADC_H
#define CPROJECT_BSP_ADC_H

#include "stm32f1xx.h"

#define VREF                                3.3f     // 参考电压，理论上是3.3，可通过实际测量得3.258
#define GET_ADC_VAL(val)                    ((float)val/(float)4096.0*VREF)          // 得到电压值

#define POSITION_ADC					    ADC1
#define POSITION_ADC_CLK_ENABLE()           __HAL_RCC_ADC1_CLK_ENABLE();
#define ADC_NUM_MAX                         2046       // ADC 转换结果缓冲区最大值

#define ADC_POSITION_IRQ					ADC_IRQn
#define ADC_POSITION_IRQHandler				ADC_IRQHandler

/*********************** 位置采集 ******************/

/************** 上电机位置采集 ******************/
#define POSITONUP_ADC_GPIO_PORT             GPIOB
#define POSITONUP_ADC_GPIO_PIN              GPIO_PIN_0
#define POSITONUP_ADC_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()
#define POSITONUP_ADC_CHANNEL               ADC_CHANNEL_8

/************** 下电机位置采集 ******************/
#define POSITONDOWN_ADC_GPIO_PORT           GPIOB
#define POSITONDOWN_ADC_GPIO_PIN            GPIO_PIN_1
#define POSITONDOWN_ADC_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define POSITONDOWN_ADC_CHANNEL             ADC_CHANNEL_9

#define Dropping_ADC_GPIO_PORT           GPIOA
#define Dropping_ADC_GPIO_PIN            GPIO_PIN_4
#define Dropping_ADC_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define Dropping_ADC_CHANNEL             ADC_CHANNEL_4

#define Rotation1_ADC_GPIO_PORT           GPIOA
#define Rotation1_ADC_GPIO_PIN            GPIO_PIN_5
#define Rotation1_ADC_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define Rotation1_ADC_CHANNEL             ADC_CHANNEL_5

#define Rotation2_ADC_GPIO_PORT           GPIOA
#define Rotation2_ADC_GPIO_PIN            GPIO_PIN_6
#define Rotation2_ADC_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define Rotation2_ADC_CHANNEL             ADC_CHANNEL_6

#define Rotation3_ADC_GPIO_PORT           GPIOA
#define Rotation3_ADC_GPIO_PIN            GPIO_PIN_7
#define Rotation3_ADC_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define Rotation3_ADC_CHANNEL             ADC_CHANNEL_7

/** ADC DR寄存器宏定义，ADC转换后的数字值则存放在这里 */
#define POSITION_ADC_DR_ADDR                ((uint32_t)ADC1+0x4c)

/** ADC DMA 通道宏定义，这里我们使用DMA传输 */
#define POSITION_ADC_DMA_CLK_ENABLE()       __HAL_RCC_DMA1_CLK_ENABLE()
#define POSITION_ADC_DMA_STREAM             DMA1_Channel1

/** ADC DMA 通道宏定义，这里我们使用DMA传输 */
#define ADC_DMA_IRQ                         DMA1_Channel1_IRQn
#define ADC_DMA_IRQ_Handler                 DMA1_Channel1_IRQHandler

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef DMA_Init_Handle;
void ADC_Init(void);
float get_positionup_val(void);
float get_positiondown_val(void);
extern int32_t positionup_adc_mean;   			    // 位置上电压 ACD 采样结果平均值
extern int32_t positiondown_adc_mean; 				// 位置下电压 ACD 采样结果平均值
extern int32_t Rotation1_adc_mean;
extern int32_t Rotation2_adc_mean;
extern int32_t Rotation3_adc_mean;
extern int32_t Dropping_adc_mean;

#endif
