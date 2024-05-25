//
// Created by Giselle on 2024/3/17.
//

#ifndef CPROJECT_BSP_USART_BLT_H
#define CPROJECT_BSP_USART_BLT_H


#include "stm32f1xx.h"
#include <stdio.h>

#define BLE_UART_BAUD_RATE         256000


#define BLE_UARTx 					    UART4
#define BLE_UART_CLK()				    __HAL_RCC_UART4_CLK_ENABLE()

#define BLE_UART_RX_PORT                   GPIOC
#define UART_RX_BLE_CLK_ENABLE()             __HAL_RCC_GPIOC_CLK_ENABLE()
#define BLE_UART_RX_PIN                    GPIO_PIN_11

#define BLE_UART_TX_PORT                    GPIOC
#define UART_TX_BLE_CLK_ENABLE()              __HAL_RCC_GPIOC_CLK_ENABLE()
#define BLE_UART_TX_PIN                     GPIO_PIN_10

#define BLE_UARTx_IRQHandler            UART4_IRQHandler
#define BLE_UARTx_IRQ                   UART4_IRQn


#define UART_BUFF_SIZE      1024


typedef struct
{
    volatile    uint16_t datanum;
    uint8_t     uart_buff[UART_BUFF_SIZE];
    uint8_t     receive_data_flag;
}ReceiveData;


void BLE_USART_Config(void);
void Usart_SendStr_length( USART_TypeDef * pUSARTx, uint8_t *str,uint32_t strlen );
void BLE_Usart_SendString(uint8_t *str);

void bsp_USART_Process(void);
char *get_rebuff(uint16_t *len);
void clean_rebuff(void);



#endif //CPROJECT_BSP_USART_BLT_H