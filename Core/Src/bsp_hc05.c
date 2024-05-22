#include "bsp_hc05.h"
#include "bsp_usart_blt.h"
#include <string.h>
#include <stdio.h>

//蓝牙设备列表，在main文件中定义
extern  BLEDev bleDevList;

/**
 * @brief  初始化控制GPIO引脚定义
 * @param  无
 * @retval 无
 */
static void BLE_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /*开启GPIO时钟*/
    BLE_INT_GPIO_CLK;
    BLE_WAKEUP_GPIO_CLK;

    /* 配置INT引脚为复用功能  */
    GPIO_InitStruct.Pin = BLE_INT_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BLE_INT_GPIO_PORT, &GPIO_InitStruct);

    /* 配置WAKEUP引脚为复用功能  */
    GPIO_InitStruct.Pin = BLE_WAKEUP_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL ;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BLE_WAKEUP_GPIO_PORT, &GPIO_InitStruct);

    BLE_WAKEUP_HIGHT
}


/**
 * @brief  向BLE模块发送命令并检查OK。只适用于具有OK应答的命令
   * @param  cmd:命令的完整字符串，需要加\r\n。
   * @param	clean 命令结束后是否清除接收缓冲区，1 清除，0 不清除
   * @template  复位命令：	BLE_Send_CMD("AT+RESET\r\n",1);
 * @retval 0,设置成功;其他,设置失败.
 */
uint8_t BLE_Send_CMD(char* cmd,uint8_t clean)
{
    uint8_t retry=5;
    uint8_t i,result=1;


    while(retry--)
    {
        BLE_WAKEUP_LOW;     //BLE进入准备接收命令的模式
        HAL_Delay(10);
        BLE_Usart_SendString((uint8_t *)cmd);  //USART向BLE发送命令

        for(i=0;i<20;i++)   //循环迭代20次
        {
            uint16_t len;
            char * redata;

            HAL_Delay(100);

            redata = get_rebuff(&len);   //从缓存区获取数据，长度为len
            if(len>0)
            {
                if(redata[0]!=0)   //检查第一个字节是否不等于0，第一个字节为0表示无有效数据
                {
                    BLE_DEBUG("send CMD: %s",cmd);

                    BLE_DEBUG("receive %s",redata);
                }
                if(strstr(redata,"AT+OK\r"))    //搜索接收到的数据是否包含子字符串“OK”，找到OK代表响应
                {
                    if(clean==1)
                        clean_rebuff();      //清除接收缓存区
                    return 0;
                }
                else
                {

                }
            }
            else
            {
                HAL_Delay(100);
            }
        }
        BLE_DEBUG("BLE send CMD fail %d times",retry);
    }

    BLE_DEBUG("BLE send CMD fail ");

    if(clean==1)
        clean_rebuff();

    return result ;

}

/**
 * @brief  使用HC05透传字符串数据
   * @param  str,要传输的字符串
 * @retval 无
 */
void BLE_SendString(char* str)
{
    BLE_WAKEUP_LOW;

    BLE_Usart_SendString((uint8_t *)str);

}



/**
 * @brief  初始化GPIO及检测BLE模块
 * @param  无
 * @retval BLE状态，0 正常，非0异常
 */
uint8_t BLE_Init(void)
{
    uint8_t i;

    BLE_GPIO_Config();

    BLE_USART_Config();

//    for(i=0;i<BLEDEV_MAX_NUM;i++)
//    {
//        sprintf(bleDevList.unpraseAddr[i]," ");
//        sprintf(bleDevList.name[i]," ");
//
//    }
//    bleDevList.num = 0;
    return 0;
//    return BLE_Send_CMD("AT\r\n",1);
}





/**
 * @brief  把接收到的字符串转化成16进制形式的数字变量(主要用于转化蓝牙地址)
   * @param  纯粹的数字字符串
 * @retval 转化后的数字变量
 */
unsigned long htoul(const char *str)
{


    long result = 0;

    if (!str)
        return 0;

    while (*str)
    {
        uint8_t value;

        if (*str >= 'a' && *str <= 'f')
            value = (*str - 'a') + 10;
        else if (*str >= 'A' && *str <= 'F')
            value = (*str - 'A') + 10;
        else if (*str >= '0' && *str <= '9')
            value = *str - '0';
        else
            break;

        result = (result * 16) + value;
        ++str;
    }

    return result;
}


/**
 * @brief  在str中，跳过它前面的prefix字符串,
                       如str为"abcdefg",prefix为"abc"，则调用本函数后返回指向"defg"的指针
   * @param  str,原字符串
   * @param  str_length，字符串长度
   * @param 	prefix，要跳过的字符串
 * @retval 跳过prefix后的字符串指针
 */
char *skipPrefix(char *str, size_t str_length, const char *prefix)
{

    uint16_t prefix_length = strlen(prefix);

    if (!str || str_length == 0 || !prefix)
        return 0;

    if (str_length >= prefix_length && strncmp(str, prefix, prefix_length) == 0)
        return str + prefix_length;

    return 0;
}

/**
 * @brief  从stream中获取一行字符串到line中
   * @param  line,存储获得行的字符串数组
   * @param  stream，原字符串数据流
   * @param 	max_size，stream的大小
 * @retval line的长度，若stream中没有‘\0’，'\r'，'\n'，则返回0
 */
int get_line(char* line, char* stream ,int max_size)
{
    char *p;
    int len = 0;
    p = stream;
    while( *p != '\0' && len < max_size)  //遇到结束符或者最大容量跳出循环
    {
        line[len++] = *p;

        if('\n' == *p || '\r'==*p)
        {
            break;
        }
        p++;
    }
    line[len] = '\0';
    return len;
}

//int get_line(char* line, char* stream, int max_size)
//{
//    char *p = stream;
//    int len = 0;
//
//    // 查找帧头
//    while (*p != '\0' && *p != '<') {
//        p++;
//    }
//
//    if (*p == '<') {
//        p++; // 跳过帧头
//    } else {
//        return 0; // 没有找到帧头，返回 0
//    }
//
//    // 读取数据直到帧尾或最大长度
//    while (*p != '\0' && *p != '>' && len < max_size - 1) {
//        line[len++] = *p++;
//    }
//
//    if (*p == '>') {
//        line[len] = '\0'; // 确保字符串以 '\0' 结尾
//        return len;
//    } else {
//        return 0; // 没有找到帧尾，返回 0
//    }
//}

/**
 * @brief  向BLE写入命令，不检查模块的响应
   * @param  command ，要发送的命令
   * @param  arg，命令参数，为0时不带参数，若command也为0时，发送"AT"命令
 * @retval 无
 */
void writeCommand(const char *command, const char *arg)
{
    char str_buf[50];

    BLE_WAKEUP_HIGHT;
    HAL_Delay(10);

    if (arg && arg[0] != 0)
        sprintf(str_buf,"AT+%s%s\r\n",command,arg);
    else if (command && command[0] != 0)
    {
        sprintf(str_buf,"AT+%s\r\n",command);
    }
    else
        sprintf(str_buf,"AT\r\n");

    BLE_DEBUG("CMD send:%s",str_buf);

    BLE_Usart_SendString((uint8_t *)str_buf);

}



/**
 * @brief  扫描周边的蓝牙设备，并存储到设备列表中。
   * @param  bleDev ，蓝牙设备列表指针
 * @retval 是否扫描到设备，0表示扫描到，非0表示没有扫描到
 */
uint8_t parseBluetoothAddress(BLEDev *bleDev)
{
    /* Address should look like "+ADDR:<NAP>:<UAP>:<LAP>",
     * where actual address will look like "1234:56:abcdef".
     */

    char* redata;
    uint16_t len;

    char linebuff[50];
    uint16_t linelen;

    uint16_t getlen=0;
    uint8_t linenum=0;


    uint8_t i;

    char *p;


    BLE_Send_CMD("AT+INQ\r\n",0);

    redata =get_rebuff(&len);

    if(redata[0] != 0 && strstr(redata, "+INQ:") != 0)
    {
        BLE_DEBUG("rebuf =%s",redata);

        getNewLine:
        while(getlen < len-2*linenum )
        {
            linelen = get_line(linebuff,redata+getlen+2*linenum,len);
            if(linelen>50 && linelen != 0)
            {
                BLE_Send_CMD("AT+INQC\r\n",1);//退出前中断查询
                return 1;
            }

            getlen += linelen;
            linenum++;

            p = skipPrefix(linebuff,linelen,"+INQ:");
            if(p!=0)
            {
                uint8_t num ;
                num = bleDev->num;

                strBLEAddr(bleDev,':');

                for(i=0;i<=num;i++)
                {
                    if(strstr(linebuff,bleDev->unpraseAddr[i]) != NULL)
                    {
                        goto getNewLine;	//!=null时，表示该地址与解码语句的地址相同
                    }
                }

                /*若蓝牙设备不在列表中，对地址进行解码*/
                bleDev->addr[num].NAP = htoul(p);
                p = strchr(p,':');

                if (p == 0)
                {
                    BLE_Send_CMD("AT+INQC\r\n",1);//退出前中断查询
                    return 1;
                }

                bleDev->addr[num].UAP = htoul(++p);
                p = strchr(p,':');

                if (p == 0)
                {
                    BLE_Send_CMD("AT+INQC\r\n",1);//退出前中断查询
                    return 1;
                }

                bleDev->addr[num].LAP = htoul(++p);

                /*存储蓝牙地址(字符串形式)*/
                sprintf(bleDev->unpraseAddr[num],"%X:%X:%X",bleDev->addr[num].NAP,bleDev->addr[num].UAP,bleDev->addr[num].LAP);

                bleDev->num++;

            }


        }

        clean_rebuff();

        BLE_Send_CMD("AT+INQC\r\n",1);//退出前中断查询
        return 0;
    }

    else
    {
        clean_rebuff();

        BLE_Send_CMD("AT+INQC\r\n",1);//退出前中断查询
        return 1;
    }

}

/**
 * @brief  把蓝牙地址转化成字符串形式
   * @param  bleDev ，蓝牙设备列表指针
   *	@param  delimiter, 分隔符。 根据需要使用':'或','。
 * @retval 无
 */
void strBLTAddr(BLEDev *bleDev,char delimiter)
{
    uint8_t i;


    if(bleDev->num==0)
    {
        BLE_DEBUG("/*******No other BLT Device********/");
    }
    else
    {
        for(i=0;i<bleDev->num;i++)
        {
            sprintf(bleDev->unpraseAddr[i],"%X%c%X%c%X",bleDev->addr[i].NAP,delimiter,bleDev->addr[i].UAP,delimiter,bleDev->addr[i].LAP);
        }
    }

}


/**
 * @brief  获取远程蓝牙设备的名称
   * @param  bleDev ，蓝牙设备列表指针
 * @retval 0获取成功，非0不成功
 */
uint8_t getRemoteDeviceName(BLEDev *bleDev)
{
    uint8_t i;
    char *redata;
    uint16_t len;

    char linebuff[50];
    uint16_t linelen;
    char *p;

    char cmdbuff[100];

    strBLTAddr(bleDev,',');

    BLE_DEBUG("device num =%d",bleDev->num);

    for(i=0;i<bleDev->num;i++)
    {
        sprintf(cmdbuff,"AT+RNAME?%s\r\n",bleDev->unpraseAddr[i]);
        BLE_Send_CMD(cmdbuff,0);

        redata =get_rebuff(&len);
        if(redata[0] != 0 && strstr(redata, "OK") != 0)
        {

            linelen = get_line(linebuff,redata,len);
            if(linelen>50 && linelen !=0 ) linebuff[linelen]='\0';	//超长截断

            p = skipPrefix(linebuff,linelen,"+RNAME:");
            if(p!=0)
            {
                strcpy(bleDev->name[i],p);
            }

        }
        else
        {
            clean_rebuff();
            return 1;
        }

        clean_rebuff();

    }

    return 0;

}

/**
 * @brief  输出蓝牙设备列表
   * @param  bltDev ，蓝牙设备列表指针
 * @retval 无
 */
void printBLEInfo(BLEDev *bleDev)
{
    uint8_t i;

    if(bleDev->num==0)
    {
        BLE_DEBUG("/*******No remote BLT Device or in SLAVE mode********/");
    }
    else
    {
        BLE_DEBUG("扫描到 %d 个蓝牙设备",bleDev->num);

        for(i=0;i<bleDev->num;i++)
        {
            BLE_INFO("/*******Device[%d]********/",i);
            BLE_INFO("Device Addr: %s",bleDev->unpraseAddr[i]);
            BLE_INFO("Device name: %s",bleDev->name[i]);
        }
    }

}



/**
 * @brief  扫描蓝牙设备，并连接名称中含有"BLE"的设备
   * @param  无
 * @retval 0获取成功，非0不成功
 */
uint8_t linkBLE(void)
{
    uint8_t i=0;
    char cmdbuff[100];

    parseBluetoothAddress(&bleDevList);
    getRemoteDeviceName(&bleDevList);
    printBLEInfo(&bleDevList);
    BLE_INFO("bleDevList.num = %d", bleDevList.num);


    for(i=0;i<=bleDevList.num;i++)
    {
        if(strstr(bleDevList.name[i],"One") != NULL) //非NULL表示找到有名称部分为BLE的设备
        {
            BLE_INFO("搜索到远程BLE模块，即将进行配对连接...");
            BLE_INFO("%s", bleDevList.unpraseAddr[i]);

            strBLEAddr(&bleDevList,',');

            //配对
            sprintf(cmdbuff,"AT+PAIR=%s,20\r\n",bleDevList.unpraseAddr[i]);
            BLE_Send_CMD(cmdbuff,0);

            //连接
            sprintf(cmdbuff,"AT+LINK=%s\r\n",bleDevList.unpraseAddr[i]);

            return BLE_Send_CMD(cmdbuff,0);
        }

    }

    return 1;

}








