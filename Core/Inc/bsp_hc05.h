//
// Created by Giselle on 2024/3/17.
//

#ifndef __BSP_HC05_H
#define __BSP_HC05_H


#include "stm32f1xx.h"
#include "core_delay.h"

#define hc05_delay_ms   Delay_ms
#define HC05_USART   	BLT_USARTx



/* 定义LED连接的GPIO端口, 用户只需要修改下面的代码即可改变控制的LED引脚 */
#define BLE_INT_GPIO_PORT    	GPIOD			              /* GPIO端口 */
#define BLE_INT_GPIO_CLK 	    __HAL_RCC_GPIOD_CLK_ENABLE()		/* GPIO端口时钟 */
#define BLE_INT_GPIO_PIN		GPIO_PIN_0			          /* 连接到HC05 INT引脚的GPIO */


#define BLE_WAKEUP_GPIO_PORT    	GPIOC			              /* GPIO端口 */
#define BLE_WAKEUP_GPIO_CLK 	    __HAL_RCC_GPIOC_CLK_ENABLE()		/* GPIO端口时钟 */
#define BLE_WAKEUP_GPIO_PIN		GPIO_PIN_12		          /* 连接到HC05 KEY引脚的GPIO */



#define BLE_WAKEUP_HIGHT  		  HAL_GPIO_WritePin(BLE_WAKEUP_GPIO_PORT, BLE_WAKEUP_GPIO_PIN,1);
#define BLE_WAKEUP_LOW  		  HAL_GPIO_WritePin(BLE_WAKEUP_GPIO_PORT,BLE_WAKEUP_GPIO_PIN,0)

//IS_HC05_CONNECTED用于检查模块是否处于配对状态
#define IS_BLE_CONNECTED()          (HAL_GPIO_ReadPin(BLE_INT_GPIO_PORT, BLE_INT_GPIO_PIN) == 0)



/*信息输出*/
#define BLE_DEBUG_ON         0
#define BLE_DEBUG_FUNC_ON    0

#define BLE_INFO(fmt,arg...)           printf("<<-BLE-INFO->> "fmt"\n",##arg)
#define BLE_ERROR(fmt,arg...)          printf("<<-BLE-ERROR->> "fmt"\n",##arg)
#define BLE_DEBUG(fmt,arg...)          do{\
                                          if(BLE_DEBUG_ON)\
                                          printf("<<-BLE-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                          }while(0)

#define BLE_DEBUG_FUNC()               do{\
                                         if(BLE_DEBUG_FUNC_ON)\
                                         printf("<<-BLE-FUNC->> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)




//#define ENABLE_LCD_DISPLAY    //切换液晶显示宏 使用野火【电阻触摸屏ILI9341_XPT2046_3.2_2.8寸】





//最大蓝牙设备数量
#define BLEDEV_MAX_NUM 5



/*蓝牙地址，数字形式，分NAP，UAP，LAP段*/
typedef  struct
{
    uint16_t NAP;
    uint8_t     UAP;
    uint32_t LAP;
}BLEAddr;

typedef  struct
{
    uint8_t num;		//扫描到的蓝牙设备数量

    BLEAddr addr[BLEDEV_MAX_NUM];	//蓝牙设备地址，数字形式

    char unpraseAddr[BLEDEV_MAX_NUM][50];	//蓝牙设备地址，字符串形式，方便扫描时和连接时使用

    char name[BLEDEV_MAX_NUM][50];	//蓝牙设备的名字

}BLEDev;

//蓝牙设备列表，在 bsp_hc05.c 文件中定义
extern  BLEDev bleDevList;


enum
{
    BLE_DEFAULT_TIMEOUT = 200,
    BLE_INQUIRY_DEFAULT_TIMEOUT = 10000,
    BLE_PAIRING_DEFAULT_TIMEOUT = 10000,
    BLE_PASSWORD_MAXLEN = 16,
    HC05_PASSWORD_BUFSIZE = BLE_PASSWORD_MAXLEN + 1,
    BLE_NAME_MAXLEN = 32,
    HC05_NAME_BUFSIZE = BLE_NAME_MAXLEN + 1,
    BLE_ADDRESS_MAXLEN = 14,
    BLE_ADDRESS_BUFSIZE = BLE_ADDRESS_MAXLEN + 1,
};

uint8_t BLE_Init(void);
uint8_t BLE_Send_CMD(char* cmd,uint8_t clean);
void BLE_SendString(char* str);
void strBLEAddr(BLEDev *bltDev,char delimiter);
uint8_t getRemoteDeviceName(BLEDev *bltDev);
void printBLEInfo(BLEDev *bltDev);
uint8_t linkBLE(void);
int get_line(char* line, char* stream ,int max_size);

#endif //CPROJECT_BSP_HC05_H
