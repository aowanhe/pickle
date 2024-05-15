#include "bsp_adc.h"
#include "bsp_debug_usart.h"
#include "bsp_led.h"

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef DMA_Init_Handle;
static uint16_t adc_buff[ADC_NUM_MAX];     			    /** 电压采集缓冲区 */
int32_t positionup_adc_mean = 0;   					    /** 位置上电压 ACD 采样结果平均值 */
int32_t positiondown_adc_mean = 0; 					    /** 位置下电压 ACD 采样结果平均值 */
int32_t Rotation1_adc_mean = 0;
int32_t Rotation2_adc_mean = 0;
int32_t Rotation3_adc_mean = 0;
int32_t Dropping_adc_mean = 0;

/**
  * @brief  ADC 通道引脚初始化
  * @param  无
  * @retval 无
  */
static void ADC_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    POSITONUP_ADC_GPIO_CLK_ENABLE();
    POSITONDOWN_ADC_GPIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStructure.Pin = POSITONUP_ADC_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL ;             /** 不上拉不下拉 */
    HAL_GPIO_Init(POSITONUP_ADC_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = POSITONDOWN_ADC_GPIO_PIN;
    HAL_GPIO_Init(POSITONDOWN_ADC_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = Dropping_ADC_GPIO_PIN;
    HAL_GPIO_Init(Dropping_ADC_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = Rotation1_ADC_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL ;             /** 不上拉不下拉 */
    HAL_GPIO_Init(Rotation1_ADC_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = Rotation2_ADC_GPIO_PIN;
    HAL_GPIO_Init(Rotation2_ADC_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = Rotation3_ADC_GPIO_PIN;
    HAL_GPIO_Init(Rotation3_ADC_GPIO_PORT, &GPIO_InitStructure);
}

void ADC_DNA_INIT(void)
{
    /** ------------------DMA Init 结构体参数 初始化-------------------------- */
    // 开启DMA时钟
    POSITION_ADC_DMA_CLK_ENABLE();
    // 数据传输通道
    DMA_Init_Handle.Instance = POSITION_ADC_DMA_STREAM;
    // 数据传输方向为外设到存储器
    DMA_Init_Handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    // 外设寄存器只有一个，地址不用递增
    DMA_Init_Handle.Init.PeriphInc = DMA_PINC_DISABLE;
    // 存储器地址固定
    DMA_Init_Handle.Init.MemInc = DMA_MINC_ENABLE;
    // 外设数据大小为半字，即两个字节
    DMA_Init_Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    //	存储器数据大小也为半字，跟外设数据大小相同
    DMA_Init_Handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    // 循环传输模式
    DMA_Init_Handle.Init.Mode = DMA_CIRCULAR;
    // DMA 传输通道优先级为高，当使用一个DMA通道时，优先级设置不影响
    DMA_Init_Handle.Init.Priority = DMA_PRIORITY_HIGH;

    HAL_DMA_Init(&DMA_Init_Handle);

    __HAL_LINKDMA(&hadc1,DMA_Handle,DMA_Init_Handle);
}

/**
  * @brief  ADC 和 DMA 初始化
  * @param  无
  * @retval 无
  */
static void ADC_Mode_Config(void)
{
    // 开启ADC时钟
    POSITION_ADC_CLK_ENABLE();
    // ADC1
    hadc1.Instance = POSITION_ADC;
    // 使能扫描模式，多通道采集才需要
    hadc1.Init.ScanConvMode = ENABLE;
    // 连续转换
    hadc1.Init.ContinuousConvMode = ENABLE;
    // 非连续转换
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    // 非连续转换个数
    hadc1.Init.NbrOfDiscConversion   = 0;
    //使用软件触发
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    //数据右对齐
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    //转换通道 6个
    hadc1.Init.NbrOfConversion = 6;
    // 初始化ADC
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef ADC_Config;

    ADC_Config.Channel      = POSITONUP_ADC_CHANNEL;
    ADC_Config.Rank         = 1;
    // 采样时间间隔
    ADC_Config.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(&hadc1, &ADC_Config);

    ADC_Config.Channel 			= POSITONDOWN_ADC_CHANNEL;
    ADC_Config.Rank 			= 2;
    // 采样时间间隔
    ADC_Config.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(&hadc1, &ADC_Config);

    ADC_Config.Channel 			= Rotation1_ADC_CHANNEL;
    ADC_Config.Rank 			= 3;
    // 采样时间间隔
    ADC_Config.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &ADC_Config);

    ADC_Config.Channel 			= Rotation2_ADC_CHANNEL;
    ADC_Config.Rank 			= 4;
    // 采样时间间隔
    ADC_Config.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &ADC_Config);

    ADC_Config.Channel 			= Rotation3_ADC_CHANNEL;
    ADC_Config.Rank 			= 5;
    // 采样时间间隔
    ADC_Config.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &ADC_Config);

    ADC_Config.Channel 			= Dropping_ADC_CHANNEL;
    ADC_Config.Rank 			= 6;
    // 采样时间间隔
    ADC_Config.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &ADC_Config);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_buff, ADC_NUM_MAX);
}


void ADC_Init(void)
{
    ADC_GPIO_Config();
    ADC_DNA_INIT();
    ADC_Mode_Config();
}

/**
  * @brief  常规转换在非阻塞模式下完成回调
  * @param  hadc: ADC  句柄.
  * @retval 无
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    uint32_t adc_sum = 0;        //用于储存ADC的平均值
    HAL_ADC_Stop_DMA(hadc);       // 停止 ADC 采样，处理完一次数据在继续采样
    for(uint32_t count = 0; count < ADC_NUM_MAX; count += 6)            //开始一个循环，从0开始，每次增加2，直到 ADC_NUM_MAX。这个循环用于计算ADC转换结果的平均值。
    {
        adc_sum += (uint32_t)adc_buff[count];                          //在每次迭代中，将 adc_buff 数组中的元素累加到 adc_mean 变量中。
    }
    positionup_adc_mean = adc_sum / (ADC_NUM_MAX / 6);                 //计算ADC转换结果的平均值。这个平均值被存储到 positionup_adc_mean 变量中。

    adc_sum = 0;
    for(uint32_t count = 1; count < ADC_NUM_MAX; count += 6)
    {
        adc_sum += (uint32_t)adc_buff[count];
    }
    positiondown_adc_mean = adc_sum / (ADC_NUM_MAX / 6);    // 计算ADC转换结果的平均值。这个平均值被存储到 positiondown_adc_mean 变量中。

    adc_sum = 0;
    for(uint32_t count = 2; count < ADC_NUM_MAX; count += 6)
    {
        adc_sum += (uint32_t)adc_buff[count];
    }
    Rotation1_adc_mean = adc_sum / (ADC_NUM_MAX / 6);    // 计算ADC转换结果的平均值。这个平均值被存储到 Rotation1_adc_mean 变量中。

    adc_sum = 0;
    for(uint32_t count = 3; count < ADC_NUM_MAX; count += 6)
    {
        adc_sum += (uint32_t)adc_buff[count];
    }
    Rotation2_adc_mean = adc_sum / (ADC_NUM_MAX / 6);    // 计算ADC转换结果的平均值。这个平均值被存储到 Rotation2_adc_mean 变量中。

    adc_sum = 0;
    for(uint32_t count = 4; count < ADC_NUM_MAX; count += 6)
    {
        adc_sum += (uint32_t)adc_buff[count];
    }
    Rotation3_adc_mean = adc_sum / (ADC_NUM_MAX / 6);    // 计算ADC转换结果的平均值。这个平均值被存储到 Rotation3_adc_mean 变量中。

    adc_sum = 0;
    for(uint32_t count = 5; count < ADC_NUM_MAX; count += 6)
    {
        adc_sum += (uint32_t)adc_buff[count];
    }
    Dropping_adc_mean = adc_sum / (ADC_NUM_MAX / 6);    // 计算ADC转换结果的平均值。这个平均值被存储到 Dropping_adc_mean 变量中。

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_buff, ADC_NUM_MAX);    // 开始 ADC 采样
}
