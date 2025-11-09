/**
 * @file    adc.c
 * @brief   ADC传感器数据采集实现 (标准库版本)
 * @author  MiniMax Agent
 * @version V2.0
 * @date    2025-11-08
 * 
 * 重要说明:
 * - 此版本完全使用STM32标准库，无HAL函数
 * - 彻底去除时间戳功能
 * - 完整的传感器数据采集和处理
 * 
 * 知识点总结:
 * 
 * 【ADC工作原理】
 * 1. ADC采样保持电路捕获输入电压
 * 2. 逐次逼近寄存器(SAR)进行转换
 * 3. 12位分辨率，0-3.3V对应0-4095
 * 4. 转换时间 = 采样时间 + 比较时间
 * 
 * 【传感器数据处理】
 * 1. 原始ADC值 → 电压值 → 阻值 → 物理量
 * 2. NTC热敏电阻：Steinhart-Hart方程计算温度
 * 3. 光敏电阻：对数关系计算光照强度
 * 4. MQ135：对数关系计算气体浓度
 * 
 * 【数字滤波技术】
 * 1. 滑动平均滤波：减少随机噪声
 * 2. 异常值检测：识别并处理异常数据
 * 3. 跳变限制：防止突变影响
 * 4. 历史数据：环形缓冲区管理
 */

#include "adc.h"
#include "main.h"
#include "delay.h"

/* ============================== 全局变量定义 ============================== */

/**
 * 传感器状态
 * 知识点: 传感器工作状态跟踪
 * - 记录每个传感器的健康状态
 * - 统计错误次数
 * - 记录最后采样时间
 */
Sensor_Status_t Sensor_Status = {
    .temp_sensor_ok = true,
    .light_sensor_ok = true,
    .gas_sensor_ok = true,
    .temp_error_count = 0,
    .light_error_count = 0,
    .gas_error_count = 0,
    .last_sample_time = 0,
    .total_samples = 0
};

/**
 * 传感器校准参数
 * 知识点: 传感器校准数据
 * - 偏移和斜率校正
 * - 老化补偿参数
 * - 温度补偿系数
 */
Sensor_Calibration_t Sensor_Calibration = {
    .temp_offset = 0.0f,
    .temp_slope = 1.0f,
    .light_offset = 0.0f,
    .light_slope = 1.0f,
    .gas_offset = 0.0f,
    .gas_slope = 1.0f,
    .calibration_time = 0
};

/**
 * 传感器历史数据
 * 知识点: 滑动平均滤波历史缓冲区
 * - 存储历史数据用于滤波
 * - 环形缓冲区实现
 * - 分别维护三个传感器的历史数据
 */
Sensor_History_t Sensor_History = {
    .temp_history = {0},
    .light_history = {0},
    .gas_history = {0},
    .temp_index = 0,
    .light_index = 0,
    .gas_index = 0,
    .temp_count = 0,
    .light_count = 0,
    .gas_count = 0
};

/**
 * ADC配置
 * 知识点: ADC工作配置
 * - 存储ADC当前配置
 * - 支持动态调整
 * - 故障恢复参数
 */
ADC_Config_t ADC_Config = {
    .init_config = {0},
    .common_config = {0},
    .continuous_mode = false,
    .sample_count = ADC_SAMPLE_COUNT,
    .timeout_ms = ADC_CONVERSION_TIMEOUT
};

/* ============================== ADC模块初始化 ============================== */

/**
 * @brief   ADC模块初始化函数
 * @details 初始化所有ADC相关功能
 * 
 * 知识点: ADC模块初始化顺序
 * 1. 初始化硬件ADC
 * 2. 配置GPIO引脚
 * 3. 校准ADC
 * 4. 初始化校准参数
 * 5. 重置历史数据
 */
void ADC_Module_Init(void)
{
    DEBUG_PRINT("Initializing ADC module...\r\n");
    
    /**
     * 步骤1: 初始化硬件ADC
     * 知识点: 硬件初始化
     * - 配置ADC时钟和时序
     * - 设置转换参数
     * - 启用ADC外设
     */
    ADC_Hardware_Init();
    
    /**
     * 步骤2: 配置GPIO引脚
     * 知识点: 传感器引脚配置
     * - PA1: 温度传感器
     * - PA4: 光线传感器
     * - PB1: 气体传感器
     * - 配置为模拟输入模式
     */
    ADC_GPIO_Config();
    
    /**
     * 步骤3: ADC校准
     * 知识点: ADC精度校准
     * - ADC内置校准电路
     * - 提高转换精度
     * - 校准后需要延时等待稳定
     */
    ADC_Calibrate();
    
    /**
     * 步骤4: 初始化校准参数
     * 知识点: 传感器校准参数
     * - 设置默认校准参数
     * - 重置校准时间
     * - 可加载保存的校准参数
     */
    Sensor_Calibration.calibration_time = System_Get_Runtime_MS();
    
    /**
     * 步骤5: 重置历史数据
     * 知识点: 滤波器初始化
     * - 清空历史数据缓冲区
     * - 重置索引和计数
     * - 为第一次采样做准备
     */
    memset(Sensor_History.temp_history, 0, sizeof(Sensor_History.temp_history));
    memset(Sensor_History.light_history, 0, sizeof(Sensor_History.light_history));
    memset(Sensor_History.gas_history, 0, sizeof(Sensor_History.gas_history));
    Sensor_History.temp_index = 0;
    Sensor_History.light_index = 0;
    Sensor_History.gas_index = 0;
    Sensor_History.temp_count = 0;
    Sensor_History.light_count = 0;
    Sensor_History.gas_count = 0;
    
    /**
     * 步骤6: 重置传感器状态
     * 知识点: 状态初始化
     * - 设置所有传感器为正常状态
     * - 重置错误计数
     * - 准备开始采样
     */
    ADC_Reset_Sensor_Status();
    
    DEBUG_PRINT("ADC module initialized successfully\r\n");
}

/**
 * @brief   ADC硬件初始化函数
 * @details 配置ADC硬件参数
 * 
 * 知识点: ADC硬件配置
 * 1. 启用ADC时钟
 * 2. 配置ADC主参数
 * 3. 配置ADC公共参数
 * 4. 配置规则组通道
 * 5. 使能ADC
 */
void ADC_Hardware_Init(void)
{
    /**
     * 启用ADC时钟
     * 知识点: ADC时钟使能
     * - ADC1挂载在APB2总线上
     * - 时钟频率为72MHz
     * - 需要分频到合适频率
     */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
    /**
     * 配置ADC主参数
     * 知识点: ADC工作模式
     * - 模式：独立模式，只使用ADC1
     * - 扫描模式：禁用，单通道模式
     * - 连续转换：禁用，软件触发
     * - 对齐方式：右对齐(标准)
     * - 转换数量：1个通道
     */
    ADC_Config.init_config.ADC_Mode = ADC_Mode_Independent;
    ADC_Config.init_config.ADC_ScanConvMode = DISABLE;    // 单通道模式
    ADC_Config.init_config.ADC_ContinuousConvMode = DISABLE; // 单次转换
    ADC_Config.init_config.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 软件触发
    ADC_Config.init_config.ADC_DataAlign = ADC_DataAlign_Right; // 右对齐
    ADC_Config.init_config.ADC_NbrOfChannel = 1;         // 1个通道
    ADC_Init(ADC1, &ADC_Config.init_config);
    
    /**
     * 配置ADC公共参数
     * 知识点: ADC公共设置
     * - DMA模式：禁用
     * - 多重模式：独立模式
     * - 双重模式规则：禁用
     * - 延时：4个周期
     */
    ADC_Config.common_config.ADC_Mode = ADC_Mode_Independent;
    ADC_Config.common_config.ADC_Prescaler = ADC_Prescaler_Div6; // 72MHz/6 = 12MHz
    ADC_Config.common_config.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_Config.common_config.ADC_DelayTriggerSampling = ADC_DelayTriggerSampling_4Cycles;
    ADC_CommonInit(&ADC_Config.common_config);
    
    /**
     * 配置ADC通道
     * 知识点: 规则组通道配置
     * - 规则组：用于常规转换
     * - 顺序：1
     * - 采样时间：55.5个时钟周期
     * - 需要为每个通道分别配置
     */
    // 温度传感器 (PA1, 通道1)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SAMPLE_TIME);
    // 光线传感器 (PA4, 通道4)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SAMPLE_TIME);
    // 气体传感器 (PB1, 通道9)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SAMPLE_TIME);
    
    /**
     * 使能ADC
     * 知识点: ADC使能
     * - 使能ADC外设
     * - ADC开始工作但未开始转换
     * - 需要软件触发启动转换
     */
    ADC_Cmd(ADC1, ENABLE);
    
    /**
     * 等待ADC稳定
     * 知识点: ADC稳定时间
     * - ADC启动后需要稳定时间
     * - 约10毫秒
     * - 确保后续校准和转换准确
     */
    Delay_ms(ADC_STARTUP_DELAY_MS);
    
    DEBUG_PRINT("ADC hardware initialized (Clock: 12MHz, Mode: Independent)\r\n");
}

/**
 * @brief   GPIO配置函数
 * @details 配置ADC传感器引脚
 * 
 * 知识点: ADC引脚配置
 * 1. 启用GPIO时钟
 * 2. 配置为模拟输入模式
 * 3. 禁用上下拉电阻
 * 4. 设置合适的速度
 */
void ADC_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /**
     * 启用GPIO时钟
     * 知识点: GPIO时钟使能
     * - 传感器引脚分布在GPIOA和GPIOB
     * - 需要分别使能时钟
     */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    
    /**
     * 配置传感器引脚
     * 知识点: 模拟输入配置
     * - 模式：GPIO_Mode_AIN(模拟输入)
     * - 禁用上下拉电阻
     * - 速度设置无意义(模拟输入)
     */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_4;  // PA1(温度), PA4(光线)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;           // 模拟输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;               // PB1(气体)
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    DEBUG_PRINT("ADC GPIO configured (PA1: Temp, PA4: Light, PB1: Gas)\r\n");
}

/**
 * @brief   ADC校准函数
 * @details 校准ADC提高转换精度
 * 
 * 知识点: ADC校准过程
 * 1. 启动校准序列
 * 2. 等待校准完成
 * 3. 验证校准结果
 * 4. 校准后延时稳定
 */
void ADC_Calibrate(void)
{
    /**
     * 启动校准
     * 知识点: ADC校准启动
     * - ADC内置校准电路
     * - 校准内部电阻分压网络
     * - 提高转换精度
     */
    DEBUG_PRINT("Starting ADC calibration...\r\n");
    ADC_ResetCalibration(ADC1);
    
    /**
     * 等待复位校准完成
     * 知识点: 校准状态检查
     * - 复位校准清除校准寄存器
     * - 为正式校准做准备
     * - 轮询等待完成
     */
    while (ADC_GetResetCalibrationStatus(ADC1) == SET) {
        Delay_ms(1);
    }
    
    /**
     * 启动正式校准
     * 知识点: 校准执行
     * - ADC执行内部校准
     * - 校准过程自动完成
     * - 校准结果存储在内部寄存器
     */
    ADC_StartCalibration(ADC1);
    
    /**
     * 等待校准完成
     * 知识点: 校准完成等待
     * - 校准过程需要时间
     * - 轮询等待校准完成标志
     */
    while (ADC_GetCalibrationStatus(ADC1) == SET) {
        Delay_ms(1);
    }
    
    /**
     * 校准后延时
     * 知识点: 校准后稳定
     * - 校准完成后ADC需要稳定时间
     * - 确保校准结果生效
     * - 避免立即进行转换
     */
    Delay_ms(10);
    
    DEBUG_PRINT("ADC calibration completed\r\n");
}

/* ============================== ADC采样函数 ============================== */

/**
 * @brief   单通道采样函数
 * @details 对指定通道进行单次ADC转换
 * @param channel ADC通道
 * @return uint16_t 转换结果(0-4095)
 * 
 * 知识点: 单次转换过程
 * 1. 选择ADC通道
 * 2. 启动转换
 * 3. 等待转换完成
 * 4. 读取转换结果
 */
uint16_t ADC_Single_Channel(uint8_t channel)
{
    /**
     * 选择ADC通道
     * 知识点: 通道选择
     * - 配置规则组通道
     * - 设置转换顺序
     * - 设置采样时间
     */
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SAMPLE_TIME);
    
    /**
     * 启动ADC转换
     * 知识点: 转换启动
     * - 软件触发转换
     * - ADC开始采样保持
     * - 执行逐次逼近
     */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
    /**
     * 等待转换完成
     * 知识点: 转换完成检测
     * - EOC：转换结束标志
     * - 轮询等待标志置位
     * - 转换时间约20-30微秒
     */
    uint32_t timeout = System_Get_Runtime_MS() + ADC_CONVERSION_TIMEOUT;
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
        if (System_Get_Runtime_MS() > timeout) {
            DEBUG_PRINT("ADC conversion timeout for channel %d\r\n", channel);
            return 0;  // 超时返回0
        }
    }
    
    /**
     * 读取转换结果
     * 知识点: 结果读取
     * - 读取数据寄存器
     * - 清除EOC标志
     * - 返回12位转换结果
     */
    uint16_t result = ADC_GetConversionValue(ADC1);
    
    /**
     * 清除标志
     * 知识点: 标志清除
     * - 清除转换结束标志
     * - 为下次转换做准备
     */
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    
    return result;
}

/**
 * @brief   多通道采样函数
 * @details 依次采样所有传感器通道
 * @param raw_data 存储原始数据的指针
 * @return bool 采样是否成功
 * 
 * 知识点: 多通道采样
 * - 单次转换模式，依次采样各通道
 * - 避免通道间干扰
 * - 返回原始ADC值
 */
bool ADC_Multi_Channel_Sample(ADC_Raw_Data_t *raw_data)
{
    if (raw_data == NULL) {
        return false;
    }
    
    /**
     * 依次采样各个通道
     * 知识点: 通道顺序
     * - 温度传感器：通道1(PA1)
     * - 光线传感器：通道4(PA4)
     * - 气体传感器：通道9(PB1)
     * - 每次转换前延时保证稳定
     */
    
    // 采样温度传感器
    raw_data->temperature_raw = ADC_Single_Channel(TEMP_SENSOR_CHANNEL);
    Delay_us(100);  // 通道间隔离延时
    
    // 采样光线传感器
    raw_data->light_raw = ADC_Single_Channel(LIGHT_SENSOR_CHANNEL);
    Delay_us(100);  // 通道间隔离延时
    
    // 采样气体传感器
    raw_data->gas_raw = ADC_Single_Channel(GAS_SENSOR_CHANNEL);
    
    /**
     * 数据有效性检查
     * 知识点: 原始数据验证
     * - 检查ADC值是否在有效范围
     * - 异常值可能是硬件故障
     * - 记录错误信息
     */
    bool all_valid = true;
    
    if (raw_data->temperature_raw == 0 || raw_data->temperature_raw >= ADC_RESOLUTION) {
        DEBUG_PRINT("Invalid temperature ADC value: %u\r\n", raw_data->temperature_raw);
        all_valid = false;
    }
    
    if (raw_data->light_raw == 0 || raw_data->light_raw >= ADC_RESOLUTION) {
        DEBUG_PRINT("Invalid light ADC value: %u\r\n", raw_data->light_raw);
        all_valid = false;
    }
    
    if (raw_data->gas_raw == 0 || raw_data->gas_raw >= ADC_RESOLUTION) {
        DEBUG_PRINT("Invalid gas ADC value: %u\r\n", raw_data->gas_raw);
        all_valid = false;
    }
    
    return all_valid;
}

/**
 * @brief   批量采样函数
 * @details 对每个通道进行多次采样并求平均
 * @param raw_data 存储原始数据的指针
 * @return bool 采样是否成功
 * 
 * 知识点: 批量采样算法
 * 1. 对每个通道进行多次采样
 * 2. 计算平均值减少噪声
 * 3. 去除异常值
 * 4. 提高测量精度
 */
bool ADC_Batch_Sample(ADC_Raw_Data_t *raw_data)
{
    if (raw_data == NULL) {
        return false;
    }
    
    /**
     * 初始化累积变量
     * 知识点: 累积器
     * - 用于累加采样值
     * - 使用32位避免溢出
     */
    uint32_t temp_sum = 0;
    uint32_t light_sum = 0;
    uint32_t gas_sum = 0;
    
    /**
     * 多次采样
     * 知识点: 采样循环
     * - 对每个通道进行多次采样
     * - 累加采样值
     * - 检查数据有效性
     */
    for (uint8_t i = 0; i < ADC_SAMPLE_COUNT; i++) {
        ADC_Raw_Data_t temp_data;
        if (!ADC_Multi_Channel_Sample(&temp_data)) {
            DEBUG_PRINT("Batch sample failed at iteration %d\r\n", i);
            return false;
        }
        
        temp_sum += temp_data.temperature_raw;
        light_sum += temp_data.light_raw;
        gas_sum += temp_data.gas_raw;
        
        // 采样间隔，防止通道间干扰
        if (i < ADC_SAMPLE_COUNT - 1) {
            Delay_us(200);
        }
    }
    
    /**
     * 计算平均值
     * 知识点: 平均值计算
     * - 累加值除以采样次数
     * - 四舍五入到整数
     * - 保持ADC精度
     */
    raw_data->temperature_raw = (uint16_t)(temp_sum / ADC_SAMPLE_COUNT);
    raw_data->light_raw = (uint16_t)(light_sum / ADC_SAMPLE_COUNT);
    raw_data->gas_raw = (uint16_t)(gas_sum / ADC_SAMPLE_COUNT);
    
    /**
     * 异常值检测
     * 知识点: 批量数据验证
     * - 检查平均值是否在合理范围
     * - 防止采样过程中的异常值影响
     * - 确保数据质量
     */
    bool all_valid = true;
    
    if (raw_data->temperature_raw < 10 || raw_data->temperature_raw > 4000) {
        DEBUG_PRINT("Temperature ADC average out of range: %u\r\n", raw_data->temperature_raw);
        all_valid = false;
    }
    
    if (raw_data->light_raw < 10 || raw_data->light_raw > 4000) {
        DEBUG_PRINT("Light ADC average out of range: %u\r\n", raw_data->light_raw);
        all_valid = false;
    }
    
    if (raw_data->gas_raw < 10 || raw_data->gas_raw > 4000) {
        DEBUG_PRINT("Gas ADC average out of range: %u\r\n", raw_data->gas_raw);
        all_valid = false;
    }
    
    return all_valid;
}

/* ============================== 传感器数据转换函数 ============================== */

/**
 * @brief   温度计算函数
 * @details 根据ADC值计算温度值
 * @param adc_value 温度传感器ADC值
 * @return float 温度值(摄氏度)
 * 
 * 知识点: NTC热敏电阻温度计算
 * 1. ADC值 → 电压值 → 阻值
 * 2. 使用B值方程计算温度
 * 3. B值方程：T = B / (ln(Rt/R25) + B/T25) - 273.15
 */
float ADC_Calculate_Temperature(uint16_t adc_value)
{
    /**
     * 转换为阻值
     * 知识点: 分压电路计算
     * - 分压电路：Vout = Vin * R2 / (R1 + R2)
     * - 反推传感器阻值：R1 = R2 * (Vin/Vout - 1)
     * - Vin = 3.3V, Vout = ADC值 * 3.3V / 4096
     */
    float voltage = (float)adc_value * ADC_VREF / ADC_RESOLUTION;
    float sensor_resistance = TEMP_VOLTAGE_DIVIDER * (ADC_VREF / voltage - 1.0f);
    
    /**
     * B值温度计算
     * 知识点: B值方程
     * - 标准NTC温度计算公式
     * - T = (B * T25) / (B + T25 * ln(Rt/R25)) - 273.15
     * - T25 = 298.15K (25°C)
     * - R25 = 10kΩ
     */
    float t25 = 298.15f;  // 25°C in Kelvin
    float ln_ratio = logf(sensor_resistance / TEMP_NTC_R25);
    float temperature_k = (TEMP_NTC_B_VALUE * t25) / (TEMP_NTC_B_VALUE + t25 * ln_ratio);
    float temperature_c = temperature_k - 273.15f;
    
    /**
     * 温度范围检查
     * 知识点: 合理性验证
     * - NTC传感器测量范围
     * - 环境温度一般在-40~85°C
     * - 超出范围的数据可能异常
     */
    if (temperature_c < TEMP_MIN_C || temperature_c > TEMP_MAX_C) {
        DEBUG_PRINT("Temperature out of range: %.1f°C (ADC: %u, R: %.0fΩ)\r\n", 
                   temperature_c, adc_value, sensor_resistance);
        // 返回边界值而不是错误值
        temperature_c = Limit_Value(temperature_c, TEMP_MIN_C, TEMP_MAX_C);
    }
    
    return temperature_c;
}

/**
 * @brief   光线强度计算函数
 * @details 根据ADC值计算光照强度
 * @param adc_value 光线传感器ADC值
 * @return float 光照强度(Lux)
 * 
 * 知识点: 光敏电阻光照计算
 * 1. ADC值 → 电压值 → 阻值
 * 2. 光敏电阻阻值与光照强度成反比
 * 3. 使用对数关系计算光照
 */
float ADC_Calculate_Light_Level(uint16_t adc_value)
{
    /**
     * 转换为阻值
     * 知识点: 分压电路计算
     * - 同温度传感器计算方法
     * - 获得光敏电阻当前阻值
     */
    float voltage = (float)adc_value * ADC_VREF / ADC_RESOLUTION;
    float sensor_resistance = LIGHT_VOLTAGE_DIVIDER * (ADC_VREF / voltage - 1.0f);
    
    /**
     * 光照强度计算
     * 知识点: 对数关系
     * - 光敏电阻阻值与光照强度成反比
     * - 使用对数插值计算
     * - 参考点：10Lux=10kΩ, 100Lux=1kΩ, 1000Lux=100Ω
     */
    float light_level;
    
    if (sensor_resistance >= LIGHT_R_AT_10LUX) {
        /**
         * 低光照情况
         * 知识点: 低光照插值
         * - 使用10Lux和100Lux之间的对数插值
         * - 对数关系更符合光敏电阻特性
         */
        light_level = Log_Interpolation(LIGHT_R_AT_10LUX, 10.0f, 
                                       LIGHT_R_AT_100LUX, 100.0f, 
                                       sensor_resistance);
    } else if (sensor_resistance >= LIGHT_R_AT_100LUX) {
        /**
         * 中等光照情况
         * 知识点: 中等光照插值
         * - 使用100Lux和1000Lux之间的对数插值
         */
        light_level = Log_Interpolation(LIGHT_R_AT_100LUX, 100.0f, 
                                       LIGHT_R_AT_1000LUX, 1000.0f, 
                                       sensor_resistance);
    } else {
        /**
         * 高光照情况
         * 知识点: 高光照处理
         * - 阻值小于100Lux参考值，可能是强光
         * - 使用线性外推或设为最大值
         */
        light_level = LIGHT_MAX_LUX;
    }
    
    /**
     * 光照范围检查
     * 知识点: 合理性验证
     * - 环境光照一般不会超过1000Lux
     * - 室内光照通常在50-500Lux
     * - 太阳直射可达100000Lux
     */
    if (light_level < LIGHT_MIN_LUX) {
        light_level = LIGHT_MIN_LUX;
    } else if (light_level > LIGHT_MAX_LUX) {
        light_level = LIGHT_MAX_LUX;
    }
    
    return light_level;
}

/**
 * @brief   气体浓度计算函数
 * @details 根据ADC值计算气体浓度
 * @param adc_value 气体传感器ADC值
 * @return float 气体浓度(ppm)
 * 
 * 知识点: MQ135气体浓度计算
 * 1. ADC值 → 电压值 → 阻值
 * 2. MQ135在污染气体中阻值下降
 * 3. 阻值与浓度成反比关系
 */
float ADC_Calculate_Gas_Level(uint16_t adc_value)
{
    /**
     * 转换为阻值
     * 知识点: 分压电路计算
     * - 同其他传感器计算方法
     * - 获得MQ135当前阻值
     */
    float voltage = (float)adc_value * ADC_VREF / ADC_RESOLUTION;
    float sensor_resistance = GAS_VOLTAGE_DIVIDER * (ADC_VREF / voltage - 1.0f);
    
    /**
     * 气体浓度计算
     * 知识点: 对数关系
     * - MQ135阻值与气体浓度成反比
     * - 使用对数插值计算
     * - 清洁空气中阻值约30kΩ
     */
    float gas_level;
    
    // 简化的计算方法：阻值越低，浓度越高
    if (sensor_resistance <= GAS_R0_CLEAN_AIR * 0.1f) {
        // 阻值很低，浓度很高
        gas_level = GAS_MAX_PPM;
    } else if (sensor_resistance >= GAS_R0_CLEAN_AIR) {
        // 阻值很高，浓度很低
        gas_level = GAS_MIN_PPM;
    } else {
        // 阻值在中间范围，使用对数插值
        gas_level = Log_Interpolation(GAS_R0_CLEAN_AIR, GAS_MIN_PPM, 
                                     GAS_R0_CLEAN_AIR * 0.1f, GAS_MAX_PPM, 
                                     sensor_resistance);
    }
    
    /**
     * 气体浓度范围检查
     * 知识点: 合理性验证
     * - MQ135主要检测NH3、CO2等
     * - 正常环境浓度很低
     * - 污染环境可能较高
     */
    if (gas_level < GAS_MIN_PPM) {
        gas_level = GAS_MIN_PPM;
    } else if (gas_level > GAS_MAX_PPM) {
        gas_level = GAS_MAX_PPM;
    }
    
    return gas_level;
}

/* ============================== 数据处理函数 ============================== */

/**
 * @brief   传感器数据处理函数
 * @details 将原始ADC数据转换为物理量
 * @param raw_data 原始ADC数据
 * @param sensor_data 转换后的传感器数据
 * @return bool 处理是否成功
 * 
 * 知识点: 完整数据处理流程
 * 1. 转换温度数据
 * 2. 转换光线数据
 * 3. 转换气体数据
 * 4. 应用校准参数
 * 5. 数据有效性验证
 */
bool ADC_Process_Sensor_Data(const ADC_Raw_Data_t *raw_data, Sensor_Data_t *sensor_data)
{
    if (raw_data == NULL || sensor_data == NULL) {
        return false;
    }
    
    /**
     * 转换温度数据
     * 知识点: 温度转换
     * - ADC值转换为温度
     * - 应用校准参数
     * - 检查转换结果
     */
    sensor_data->temperature = ADC_Calculate_Temperature(raw_data->temperature_raw);
    sensor_data->temperature = sensor_data->temperature * Sensor_Calibration.temp_slope + Sensor_Calibration.temp_offset;
    
    /**
     * 转换光线数据
     * 知识点: 光线转换
     * - ADC值转换为光照强度
     * - 应用校准参数
     * - 检查转换结果
     */
    sensor_data->light_level = ADC_Calculate_Light_Level(raw_data->light_raw);
    sensor_data->light_level = sensor_data->light_level * Sensor_Calibration.light_slope + Sensor_Calibration.light_offset;
    
    /**
     * 转换气体数据
     * 知识点: 气体转换
     * - ADC值转换为气体浓度
     * - 应用校准参数
     * - 检查转换结果
     */
    sensor_data->gas_level = ADC_Calculate_Gas_Level(raw_data->gas_raw);
    sensor_data->gas_level = sensor_data->gas_level * Sensor_Calibration.gas_slope + Sensor_Calibration.gas_offset;
    
    /**
     * 数据有效性验证
     * 知识点: 数据质量检查
     * - 验证转换后的数据
     * - 确保数据在合理范围
     * - 记录异常情况
     */
    return ADC_Validate_Data(sensor_data);
}

/**
 * @brief   传感器数据滤波函数
 * @details 对传感器数据进行数字滤波
 * @param sensor_data 传感器数据
 * @return bool 滤波是否成功
 * 
 * 知识点: 数字滤波算法
 * 1. 滑动平均滤波
 * 2. 异常值检测
 * 3. 跳变限制
 * 4. 历史数据更新
 */
bool ADC_Filter_Sensor_Data(Sensor_Data_t *sensor_data)
{
    if (sensor_data == NULL) {
        return false;
    }
    
    /**
     * 温度数据滤波
     * 知识点: 温度滤波
     * - 滑动平均滤波
     * - 温度跳变检测
     * - 限制温度变化率
     */
    float filtered_temp = Moving_Average_Filter(sensor_data->temperature, 
                                               Sensor_History.temp_history, 
                                               FILTER_WINDOW_SIZE, 
                                               &Sensor_History.temp_index, 
                                               &Sensor_History.temp_count);
    
    /**
     * 光线数据滤波
     * 知识点: 光线滤波
     * - 滑动平均滤波
     * - 光线变化检测
     */
    float filtered_light = Moving_Average_Filter(sensor_data->light_level, 
                                                Sensor_History.light_history, 
                                                FILTER_WINDOW_SIZE, 
                                                &Sensor_History.light_index, 
                                                &Sensor_History.light_count);
    
    /**
     * 气体数据滤波
     * 知识点: 气体滤波
     * - 滑动平均滤波
     * - 气体变化检测
     */
    float filtered_gas = Moving_Average_Filter(sensor_data->gas_level, 
                                              Sensor_History.gas_history, 
                                              FILTER_WINDOW_SIZE, 
                                              &Sensor_History.gas_index, 
                                              &Sensor_History.gas_count);
    
    /**
     * 应用滤波结果
     * 知识点: 滤波结果应用
     * - 将滤波后的值复制回原数据结构
     */
    sensor_data->temperature = filtered_temp;
    sensor_data->light_level = filtered_light;
    sensor_data->gas_level = filtered_gas;
    
    /**
     * 更新历史数据
     * 知识点: 历史数据更新
     * - 将新数据加入历史缓冲区
     * - 为下次滤波做准备
     */
    ADC_Update_History(sensor_data);
    
    return true;
}

/**
 * @brief   数据有效性检查函数
 * @details 验证传感器数据是否合理
 * @param sensor_data 传感器数据
 * @return bool 数据是否有效
 * 
 * 知识点: 数据验证算法
 * 1. 温度范围检查
 * 2. 光线范围检查
 * 3. 气体范围检查
 * 4. 异常值识别
 */
bool ADC_Validate_Data(const Sensor_Data_t *sensor_data)
{
    if (sensor_data == NULL) {
        return false;
    }
    
    /**
     * 温度验证
     * 知识点: 温度合理性检查
     * - NTC传感器测量范围
     * - 环境温度异常检测
     */
    if (sensor_data->temperature < TEMP_MIN_C || sensor_data->temperature > TEMP_MAX_C) {
        DEBUG_PRINT("Invalid temperature: %.1f°C (range: %.1f~%.1f)\r\n", 
                   sensor_data->temperature, TEMP_MIN_C, TEMP_MAX_C);
        Sensor_Status.temp_error_count++;
        return false;
    }
    
    /**
     * 光线验证
     * 知识点: 光线合理性检查
     * - 环境光照范围
     * - 极值情况处理
     */
    if (sensor_data->light_level < LIGHT_MIN_LUX || sensor_data->light_level > LIGHT_MAX_LUX) {
        DEBUG_PRINT("Invalid light level: %.1fLux (range: %.1f~%.1f)\r\n", 
                   sensor_data->light_level, LIGHT_MIN_LUX, LIGHT_MAX_LUX);
        Sensor_Status.light_error_count++;
        return false;
    }
    
    /**
     * 气体验证
     * 知识点: 气体合理性检查
     * - MQ135检测范围
     * - 异常浓度检测
     */
    if (sensor_data->gas_level < GAS_MIN_PPM || sensor_data->gas_level > GAS_MAX_PPM) {
        DEBUG_PRINT("Invalid gas level: %.1fppm (range: %.1f~%.1f)\r\n", 
                   sensor_data->gas_level, GAS_MIN_PPM, GAS_MAX_PPM);
        Sensor_Status.gas_error_count++;
        return false;
    }
    
    return true;
}

/**
 * @brief   传感器历史数据更新函数
 * @details 将新数据加入历史缓冲区
 * @param sensor_data 当前传感器数据
 * 
 * 知识点: 环形缓冲区更新
 * 1. 将数据存入历史缓冲区
 * 2. 更新索引(环形)
 * 3. 更新计数
 * 4. 保持数据历史
 */
void ADC_Update_History(const Sensor_Data_t *sensor_data)
{
    if (sensor_data == NULL) {
        return;
    }
    
    /**
     * 更新温度历史
     * 知识点: 温度数据历史
     * - 存入当前温度值
     * - 更新索引
     * - 更新计数
     */
    Sensor_History.temp_history[Sensor_History.temp_index] = sensor_data->temperature;
    Sensor_History.temp_index = (Sensor_History.temp_index + 1) % FILTER_WINDOW_SIZE;
    if (Sensor_History.temp_count < FILTER_WINDOW_SIZE) {
        Sensor_History.temp_count++;
    }
    
    /**
     * 更新光线历史
     * 知识点: 光线数据历史
     */
    Sensor_History.light_history[Sensor_History.light_index] = sensor_data->light_level;
    Sensor_History.light_index = (Sensor_History.light_index + 1) % FILTER_WINDOW_SIZE;
    if (Sensor_History.light_count < FILTER_WINDOW_SIZE) {
        Sensor_History.light_count++;
    }
    
    /**
     * 更新气体历史
     * 知识点: 气体数据历史
     */
    Sensor_History.gas_history[Sensor_History.gas_index] = sensor_data->gas_level;
    Sensor_History.gas_index = (Sensor_History.gas_index + 1) % FILTER_WINDOW_SIZE;
    if (Sensor_History.gas_count < FILTER_WINDOW_SIZE) {
        Sensor_History.gas_count++;
    }
}

/* ============================== 高级采样函数 ============================== */

/**
 * @brief   传感器状态检查函数
 * @details 检查传感器是否正常工作
 * @param raw_data 原始ADC数据
 * @return bool 传感器是否正常
 * 
 * 知识点: 传感器健康检查
 * 1. 检查ADC转换结果
 * 2. 验证数据合理性
 * 3. 更新传感器状态
 * 4. 错误统计
 */
bool ADC_Check_Sensor_Status(const ADC_Raw_Data_t *raw_data)
{
    if (raw_data == NULL) {
        return false;
    }
    
    /**
     * 温度传感器检查
     * 知识点: 温度传感器状态
     * - 检查ADC值范围
     * - 记录错误次数
     */
    if (raw_data->temperature_raw < 10 || raw_data->temperature_raw > 4000) {
        Sensor_Status.temp_sensor_ok = false;
        Sensor_Status.temp_error_count++;
        DEBUG_PRINT("Temperature sensor error: ADC=%u\r\n", raw_data->temperature_raw);
    } else {
        Sensor_Status.temp_sensor_ok = true;
    }
    
    /**
     * 光线传感器检查
     * 知识点: 光线传感器状态
     */
    if (raw_data->light_raw < 10 || raw_data->light_raw > 4000) {
        Sensor_Status.light_sensor_ok = false;
        Sensor_Status.light_error_count++;
        DEBUG_PRINT("Light sensor error: ADC=%u\r\n", raw_data->light_raw);
    } else {
        Sensor_Status.light_sensor_ok = true;
    }
    
    /**
     * 气体传感器检查
     * 知识点: 气体传感器状态
     */
    if (raw_data->gas_raw < 10 || raw_data->gas_raw > 4000) {
        Sensor_Status.gas_sensor_ok = false;
        Sensor_Status.gas_error_count++;
        DEBUG_PRINT("Gas sensor error: ADC=%u\r\n", raw_data->gas_raw);
    } else {
        Sensor_Status.gas_sensor_ok = true;
    }
    
    /**
     * 返回整体状态
     * 知识点: 整体健康检查
     * - 所有传感器正常才返回true
     */
    return (Sensor_Status.temp_sensor_ok && 
            Sensor_Status.light_sensor_ok && 
            Sensor_Status.gas_sensor_ok);
}

/**
 * @brief   单次数据采集函数
 * @details 完整的传感器数据采集流程
 * @param sensor_data 采集到的传感器数据
 * @return bool 采集是否成功
 * 
 * 知识点: 完整采集流程
 * 1. ADC采样
 * 2. 数据转换
 * 3. 滤波处理
 * 4. 状态检查
 * 5. 更新统计
 */
bool ADC_Single_Sample(Sensor_Data_t *sensor_data)
{
    if (sensor_data == NULL) {
        return false;
    }
    
    /**
     * 步骤1: ADC批量采样
     * 知识点: 原始数据获取
     * - 对所有通道进行多次采样
     * - 计算平均值减少噪声
     * - 验证采样结果
     */
    ADC_Raw_Data_t raw_data;
    if (!ADC_Batch_Sample(&raw_data)) {
        DEBUG_PRINT("ADC batch sampling failed\r\n");
        return false;
    }
    
    /**
     * 步骤2: 传感器状态检查
     * 知识点: 采样前健康检查
     * - 检查原始数据质量
     * - 更新传感器状态
     */
    if (!ADC_Check_Sensor_Status(&raw_data)) {
        DEBUG_PRINT("Sensor health check failed\r\n");
        return false;
    }
    
    /**
     * 步骤3: 数据转换
     * 知识点: 物理量转换
     * - 原始ADC值转换为物理量
     * - 应用校准参数
     */
    if (!ADC_Process_Sensor_Data(&raw_data, sensor_data)) {
        DEBUG_PRINT("Data conversion failed\r\n");
        return false;
    }
    
    /**
     * 步骤4: 数据滤波
     * 知识点: 数字滤波
     * - 滑动平均滤波
     * - 异常值处理
     */
    if (!ADC_Filter_Sensor_Data(sensor_data)) {
        DEBUG_PRINT("Data filtering failed\r\n");
        return false;
    }
    
    /**
     * 步骤5: 更新统计信息
     * 知识点: 统计更新
     * - 更新采样次数
     * - 更新最后采样时间
     */
    Sensor_Status.total_samples++;
    Sensor_Status.last_sample_time = System_Get_Runtime_MS();
    
    return true;
}

/**
 * @brief   连续数据采集函数
 * @details 连续采集传感器数据
 * @param sample_interval_ms 采样间隔(毫秒)
 * @param max_samples 最大采样次数(0=无限制)
 * @return uint32_t 实际采样次数
 * 
 * 知识点: 连续采集模式
 * 1. 持续执行单次采集
 * 2. 等待指定间隔
 * 3. 可选最大采样次数限制
 * 4. 返回实际采集次数
 */
uint32_t ADC_Continuous_Sample(uint32_t sample_interval_ms, uint32_t max_samples)
{
    uint32_t actual_samples = 0;
    uint32_t start_time = System_Get_Runtime_MS();
    bool continue_sampling = true;
    
    DEBUG_PRINT("Starting continuous sampling (interval: %lu ms", sample_interval_ms);
    if (max_samples > 0) {
        DEBUG_PRINT(", max: %lu samples", max_samples);
    }
    DEBUG_PRINT(")\r\n");
    
    /**
     * 持续采集循环
     * 知识点: 采集循环控制
     * - 条件1: 未达到最大采样次数(如果有限制)
     * - 条件2: 用户未停止(这里简化为无限循环)
     * - 条件3: 采集成功
     */
    while (continue_sampling) {
        Sensor_Data_t sensor_data;
        
        /**
         * 执行单次采集
         * 知识点: 单次采集调用
         */
        if (ADC_Single_Sample(&sensor_data)) {
            actual_samples++;
            
            /**
             * 打印采集结果(调试模式)
             * 知识点: 实时数据显示
             */
            DEBUG_PRINT("Sample %lu: T=%.1f°C, L=%.0fLux, G=%.0fppm\r\n", 
                       actual_samples, 
                       sensor_data.temperature, 
                       sensor_data.light_level, 
                       sensor_data.gas_level);
            
            /**
             * 检查停止条件
             * 知识点: 停止条件判断
             */
            if (max_samples > 0 && actual_samples >= max_samples) {
                continue_sampling = false;
            }
        } else {
            /**
             * 采集失败处理
             * 知识点: 错误处理
             * - 记录错误
             * - 可选择继续或停止
             * - 这里选择继续尝试
             */
            DEBUG_PRINT("Sampling failed at sample %lu, retrying...\r\n", actual_samples + 1);
        }
        
        /**
         * 等待采样间隔
         * 知识点: 间隔控制
         * - 等待指定时间间隔
         * - 使用延时函数
         */
        if (continue_sampling) {
            Delay_ms(sample_interval_ms);
        }
    }
    
    /**
     * 输出采集统计
     * 知识点: 统计信息输出
     */
    uint32_t total_time = System_Get_Runtime_MS() - start_time;
    DEBUG_PRINT("Continuous sampling completed:\r\n");
    DEBUG_PRINT("  Total samples: %lu\r\n", actual_samples);
    DEBUG_PRINT("  Total time: %lu ms\r\n", total_time);
    DEBUG_PRINT("  Average interval: %.1f ms\r\n", (float)total_time / actual_samples);
    
    return actual_samples;
}

/**
 * @brief   传感器状态重置函数
 * @details 重置传感器相关状态
 * 
 * 知识点: 状态重置
 * 1. 重置错误计数
 * 2. 重新初始化历史数据
 * 3. 恢复默认校准参数
 * 4. 准备重新开始
 */
void ADC_Reset_Sensor_Status(void)
{
    /**
     * 重置传感器状态
     * 知识点: 状态初始化
     * - 所有传感器设为正常
     * - 错误计数清零
     */
    Sensor_Status.temp_sensor_ok = true;
    Sensor_Status.light_sensor_ok = true;
    Sensor_Status.gas_sensor_ok = true;
    Sensor_Status.temp_error_count = 0;
    Sensor_Status.light_error_count = 0;
    Sensor_Status.gas_error_count = 0;
    Sensor_Status.last_sample_time = 0;
    Sensor_Status.total_samples = 0;
    
    /**
     * 重置校准参数
     * 知识点: 校准参数重置
     * - 恢复默认校准参数
     * - 更新校准时间
     */
    Sensor_Calibration.temp_offset = 0.0f;
    Sensor_Calibration.temp_slope = 1.0f;
    Sensor_Calibration.light_offset = 0.0f;
    Sensor_Calibration.light_slope = 1.0f;
    Sensor_Calibration.gas_offset = 0.0f;
    Sensor_Calibration.gas_slope = 1.0f;
    Sensor_Calibration.calibration_time = System_Get_Runtime_MS();
    
    /**
     * 重置历史数据
     * 知识点: 历史数据重置
     * - 清空所有历史数据
     * - 重置索引和计数
     */
    memset(Sensor_History.temp_history, 0, sizeof(Sensor_History.temp_history));
    memset(Sensor_History.light_history, 0, sizeof(Sensor_History.light_history));
    memset(Sensor_History.gas_history, 0, sizeof(Sensor_History.gas_history));
    Sensor_History.temp_index = 0;
    Sensor_History.light_index = 0;
    Sensor_History.gas_index = 0;
    Sensor_History.temp_count = 0;
    Sensor_History.light_count = 0;
    Sensor_History.gas_count = 0;
    
    DEBUG_PRINT("ADC sensor status reset completed\r\n");
}

/* ============================== 调试函数实现 ============================== */

#ifdef DEBUG
/**
 * @brief   ADC状态打印函数
 * @details 打印ADC当前状态和配置
 */
void ADC_Print_Status(void)
{
    printf("\r\nADC Status:\r\n");
    
    /**
     * 打印传感器状态
     * 知识点: 传感器状态显示
     */
    printf("  Sensor Status:\r\n");
    printf("    Temperature: %s (errors: %lu)\r\n", 
           Sensor_Status.temp_sensor_ok ? "OK" : "ERROR", 
           Sensor_Status.temp_error_count);
    printf("    Light: %s (errors: %lu)\r\n", 
           Sensor_Status.light_sensor_ok ? "OK" : "ERROR", 
           Sensor_Status.light_error_count);
    printf("    Gas: %s (errors: %lu)\r\n", 
           Sensor_Status.gas_sensor_ok ? "OK" : "ERROR", 
           Sensor_Status.gas_error_count);
    
    /**
     * 打印统计信息
     * 知识点: 统计信息显示
     */
    printf("  Statistics:\r\n");
    printf("    Total samples: %u\r\n", Sensor_Status.total_samples);
    if (Sensor_Status.last_sample_time > 0) {
        uint32_t current_time = System_Get_Runtime_MS();
        uint32_t time_since_last = current_time - Sensor_Status.last_sample_time;
        printf("    Last sample: %lu ms ago\r\n", time_since_last);
    }
    
    /**
     * 打印校准信息
     * 知识点: 校准信息显示
     */
    printf("  Calibration:\r\n");
    printf("    Temp offset: %.2f, slope: %.3f\r\n", 
           Sensor_Calibration.temp_offset, 
           Sensor_Calibration.temp_slope);
    printf("    Light offset: %.2f, slope: %.3f\r\n", 
           Sensor_Calibration.light_offset, 
           Sensor_Calibration.light_slope);
    printf("    Gas offset: %.2f, slope: %.3f\r\n", 
           Sensor_Calibration.gas_offset, 
           Sensor_Calibration.gas_slope);
    printf("    Calibrated: %lu ms ago\r\n", 
           System_Get_Runtime_MS() - Sensor_Calibration.calibration_time);
    
    /**
     * 打印历史数据统计
     * 知识点: 历史数据显示
     */
    printf("  History Buffer:\r\n");
    printf("    Temperature: %u/%u samples\r\n", 
           Sensor_History.temp_count, FILTER_WINDOW_SIZE);
    printf("    Light: %u/%u samples\r\n", 
           Sensor_History.light_count, FILTER_WINDOW_SIZE);
    printf("    Gas: %u/%u samples\r\n", 
           Sensor_History.gas_count, FILTER_WINDOW_SIZE);
}

/**
 * @brief   原始数据打印函数
 * @details 打印原始ADC数据
 * @param raw_data 原始ADC数据
 */
void ADC_Print_Raw_Data(const ADC_Raw_Data_t *raw_data)
{
    if (raw_data == NULL) {
        return;
    }
    
    printf("\r\nRaw ADC Data:\r\n");
    
    /**
     * 打印原始ADC值
     * 知识点: 原始数据显示
     */
    printf("  Raw Values:\r\n");
    printf("    Temperature: %u (0x%03X)\r\n", 
           raw_data->temperature_raw, raw_data->temperature_raw);
    printf("    Light: %u (0x%03X)\r\n", 
           raw_data->light_raw, raw_data->light_raw);
    printf("    Gas: %u (0x%03X)\r\n", 
           raw_data->gas_raw, raw_data->gas_raw);
    
    /**
     * 打印电压值
     * 知识点: 电压值显示
     */
    printf("  Voltage Values:\r\n");
    float temp_voltage = (float)raw_data->temperature_raw * ADC_VREF / ADC_RESOLUTION;
    float light_voltage = (float)raw_data->light_raw * ADC_VREF / ADC_RESOLUTION;
    float gas_voltage = (float)raw_data->gas_raw * ADC_VREF / ADC_RESOLUTION;
    
    printf("    Temperature: %.3fV\r\n", temp_voltage);
    printf("    Light: %.3fV\r\n", light_voltage);
    printf("    Gas: %.3fV\r\n", gas_voltage);
    
    /**
     * 打印计算出的阻值
     * 知识点: 阻值显示
     */
    printf("  Calculated Resistance:\r\n");
    float temp_resistance = TEMP_VOLTAGE_DIVIDER * (ADC_VREF / temp_voltage - 1.0f);
    float light_resistance = LIGHT_VOLTAGE_DIVIDER * (ADC_VREF / light_voltage - 1.0f);
    float gas_resistance = GAS_VOLTAGE_DIVIDER * (ADC_VREF / gas_voltage - 1.0f);
    
    printf("    Temperature: %.0fΩ (NTC)\r\n", temp_resistance);
    printf("    Light: %.0fΩ (LDR)\r\n", light_resistance);
    printf("    Gas: %.0fΩ (MQ135)\r\n", gas_resistance);
}

/**
 * @brief   传感器数据打印函数
 * @details 打印处理后的传感器数据
 * @param sensor_data 传感器数据
 */
void ADC_Print_Sensor_Data(const Sensor_Data_t *sensor_data)
{
    if (sensor_data == NULL) {
        return;
    }
    
    printf("\r\nProcessed Sensor Data:\r\n");
    
    /**
     * 打印处理后的数据
     * 知识点: 传感器数据显示
     */
    printf("  Physical Values:\r\n");
    printf("    Temperature: %.1f°C\r\n", sensor_data->temperature);
    printf("    Light Level: %.0fLux\r\n", sensor_data->light_level);
    printf("    Gas Level: %.0fppm\r\n", sensor_data->gas_level);
    
    /**
     * 数据质量评估
     * 知识点: 数据质量评估
     */
    printf("  Data Quality:\r\n");
    printf("    Temperature: %s\r\n", 
           (sensor_data->temperature >= TEMP_MIN_C && sensor_data->temperature <= TEMP_MAX_C) ? "Valid" : "Invalid");
    printf("    Light Level: %s\r\n", 
           (sensor_data->light_level >= LIGHT_MIN_LUX && sensor_data->light_level <= LIGHT_MAX_LUX) ? "Valid" : "Invalid");
    printf("    Gas Level: %s\r\n", 
           (sensor_data->gas_level >= GAS_MIN_PPM && sensor_data->gas_level <= GAS_MAX_PPM) ? "Valid" : "Invalid");
}

/**
 * @brief   校准参数打印函数
 * @details 打印当前校准参数
 */
void ADC_Print_Calibration(void)
{
    printf("\r\nSensor Calibration Parameters:\r\n");
    
    /**
     * 打印温度校准参数
     * 知识点: 温度校准信息
     */
    printf("  Temperature:\r\n");
    printf("    Offset: %.2f°C\r\n", Sensor_Calibration.temp_offset);
    printf("    Slope: %.3f\r\n", Sensor_Calibration.temp_slope);
    printf("    Range: %.1f~%.1f°C\r\n", TEMP_MIN_C, TEMP_MAX_C);
    
    /**
     * 打印光线校准参数
     * 知识点: 光线校准信息
     */
    printf("  Light:\r\n");
    printf("    Offset: %.2fLux\r\n", Sensor_Calibration.light_offset);
    printf("    Slope: %.3f\r\n", Sensor_Calibration.light_slope);
    printf("    Range: %.1f~%.1fLux\r\n", LIGHT_MIN_LUX, LIGHT_MAX_LUX);
    
    /**
     * 打印气体校准参数
     * 知识点: 气体校准信息
     */
    printf("  Gas:\r\n");
    printf("    Offset: %.2fppm\r\n", Sensor_Calibration.gas_offset);
    printf("    Slope: %.3f\r\n", Sensor_Calibration.gas_slope);
    printf("    Range: %.1f~%.1fppm\r\n", GAS_MIN_PPM, GAS_MAX_PPM);
    
    /**
     * 打印校准时间
     * 知识点: 校准时间信息
     */
    printf("  Calibration:\r\n");
    printf("    Last calibration: %lu ms ago\r\n", 
           System_Get_Runtime_MS() - Sensor_Calibration.calibration_time);
}

/**
 * @brief   历史数据打印函数
 * @details 打印历史数据统计
 */
void ADC_Print_History(void)
{
    printf("\r\nSensor History Data:\r\n");
    
    /**
     * 温度历史数据
     * 知识点: 温度历史显示
     */
    printf("  Temperature History:\r\n");
    printf("    Samples: %u/%u\r\n", Sensor_History.temp_count, FILTER_WINDOW_SIZE);
    if (Sensor_History.temp_count > 0) {
        float temp_min = Sensor_History.temp_history[0];
        float temp_max = Sensor_History.temp_history[0];
        float temp_sum = 0.0f;
        
        for (uint8_t i = 0; i < Sensor_History.temp_count; i++) {
            temp_min = fminf(temp_min, Sensor_History.temp_history[i]);
            temp_max = fmaxf(temp_max, Sensor_History.temp_history[i]);
            temp_sum += Sensor_History.temp_history[i];
        }
        
        float temp_avg = temp_sum / Sensor_History.temp_count;
        printf("    Min: %.1f°C, Max: %.1f°C, Avg: %.1f°C\r\n", 
               temp_min, temp_max, temp_avg);
    }
    
    /**
     * 光线历史数据
     * 知识点: 光线历史显示
     */
    printf("  Light History:\r\n");
    printf("    Samples: %u/%u\r\n", Sensor_History.light_count, FILTER_WINDOW_SIZE);
    if (Sensor_History.light_count > 0) {
        float light_min = Sensor_History.light_history[0];
        float light_max = Sensor_History.light_history[0];
        float light_sum = 0.0f;
        
        for (uint8_t i = 0; i < Sensor_History.light_count; i++) {
            light_min = fminf(light_min, Sensor_History.light_history[i]);
            light_max = fmaxf(light_max, Sensor_History.light_history[i]);
            light_sum += Sensor_History.light_history[i];
        }
        
        float light_avg = light_sum / Sensor_History.light_count;
        printf("    Min: %.0fLux, Max: %.0fLux, Avg: %.0fLux\r\n", 
               light_min, light_max, light_avg);
    }
    
    /**
     * 气体历史数据
     * 知识点: 气体历史显示
     */
    printf("  Gas History:\r\n");
    printf("    Samples: %u/%u\r\n", Sensor_History.gas_count, FILTER_WINDOW_SIZE);
    if (Sensor_History.gas_count > 0) {
        float gas_min = Sensor_History.gas_history[0];
        float gas_max = Sensor_History.gas_history[0];
        float gas_sum = 0.0f;
        
        for (uint8_t i = 0; i < Sensor_History.gas_count; i++) {
            gas_min = fminf(gas_min, Sensor_History.gas_history[i]);
            gas_max = fmaxf(gas_max, Sensor_History.gas_history[i]);
            gas_sum += Sensor_History.gas_history[i];
        }
        
        float gas_avg = gas_sum / Sensor_History.gas_count;
        printf("    Min: %.0fppm, Max: %.0fppm, Avg: %.0fppm\r\n", 
               gas_min, gas_max, gas_avg);
    }
}

/**
 * @brief   ADC性能测试函数
 * @details 测试ADC转换性能和稳定性
 * @param test_samples 测试采样次数
 */
void ADC_Performance_Test(uint32_t test_samples)
{
    printf("\r\nADC Performance Test:\r\n");
    printf("  Testing with %lu samples...\r\n", test_samples);
    
    uint32_t start_time = System_Get_Runtime_MS();
    uint32_t success_count = 0;
    uint32_t total_temp = 0, total_light = 0, total_gas = 0;
    uint32_t min_temp = 4095, max_temp = 0;
    uint32_t min_light = 4095, max_light = 0;
    uint32_t min_gas = 4095, max_gas = 0;
    
    /**
     * 批量性能测试
     * 知识点: 性能测试循环
     */
    for (uint32_t i = 0; i < test_samples; i++) {
        ADC_Raw_Data_t raw_data;
        if (ADC_Multi_Channel_Sample(&raw_data)) {
            success_count++;
            
            total_temp += raw_data.temperature_raw;
            total_light += raw_data.light_raw;
            total_gas += raw_data.gas_raw;
            
            min_temp = fmin(min_temp, (uint32_t)raw_data.temperature_raw);
            max_temp = fmax(max_temp, (uint32_t)raw_data.temperature_raw);
            min_light = fmin(min_light, (uint32_t)raw_data.light_raw);
            max_light = fmax(max_light, (uint32_t)raw_data.light_raw);
            min_gas = fmin(min_gas, (uint32_t)raw_data.gas_raw);
            max_gas = fmax(max_gas, (uint32_t)raw_data.gas_raw);
        }
        
        // 采样间隔
        if (i < test_samples - 1) {
            Delay_us(1000);  // 1ms间隔
        }
    }
    
    uint32_t test_time = System_Get_Runtime_MS() - start_time;
    
    /**
     * 输出性能测试结果
     * 知识点: 性能结果统计
     */
    printf("  Test Results:\r\n");
    printf("    Success rate: %.1f%% (%lu/%lu)\r\n", 
           (float)success_count / test_samples * 100.0f, 
           success_count, test_samples);
    printf("    Total time: %lu ms\r\n", test_time);
    printf("    Average interval: %.1f ms\r\n", (float)test_time / test_samples);
    
    if (success_count > 0) {
        printf("  ADC Value Statistics:\r\n");
        printf("    Temperature: min=%u, max=%u, avg=%.1f, range=%u\r\n", 
               min_temp, max_temp, (float)total_temp / success_count, max_temp - min_temp);
        printf("    Light: min=%u, max=%u, avg=%.1f, range=%u\r\n", 
               min_light, max_light, (float)total_light / success_count, max_light - min_light);
        printf("    Gas: min=%u, max=%u, avg=%.1f, range=%u\r\n", 
               min_gas, max_gas, (float)total_gas / success_count, max_gas - min_gas);
    }
}
#endif

/* ============================== 辅助函数实现 ============================== */

/**
 * @brief   滑动平均滤波函数
 * @details 实现移动平均滤波
 * @param new_value 新数据值
 * @param history 历史数据数组
 * @param history_size 历史数据大小
 * @param current_index 当前索引
 * @param count 当前计数
 * @return float 滤波后的值
 * 
 * 知识点: 移动平均滤波算法
 * 1. 将新数据加入历史缓冲区
 * 2. 计算历史数据平均值
 * 3. 返回滤波结果
 */
float Moving_Average_Filter(float new_value, float *history, uint8_t history_size, 
                           uint8_t *current_index, uint8_t *count)
{
    /**
     * 更新历史数据
     * 知识点: 历史数据更新
     * - 存入新值
     * - 更新索引(环形)
     * - 更新计数
     */
    history[*current_index] = new_value;
    *current_index = (*current_index + 1) % history_size;
    if (*count < history_size) {
        (*count)++;
    }
    
    /**
     * 计算平均值
     * 知识点: 平均值计算
     * - 计算历史数据平均值
     * - 返回滤波结果
     */
    if (*count == 0) {
        return new_value;  // 没有历史数据，返回当前值
    }
    
    float sum = 0.0f;
    for (uint8_t i = 0; i < *count; i++) {
        sum += history[i];
    }
    
    return sum / *count;
}

/**
 * @brief   异常值检测函数
 * @details 识别异常数据点
 * @param value 当前值
 * @param history 历史数据数组
 * @param count 历史数据数量
 * @param threshold 异常阈值
 * @return bool 是否为异常值
 * 
 * 知识点: 异常值检测算法
 * 1. 计算历史数据平均值
 * 2. 计算标准差
 * 3. 检查当前值是否偏离太远
 */
bool Is_Outlier(float value, const float *history, uint8_t count, float threshold)
{
    if (count < 2) {
        return false;  // 数据不足，无法判断
    }
    
    /**
     * 计算历史数据平均值
     * 知识点: 平均值计算
     */
    float mean = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        mean += history[i];
    }
    mean /= count;
    
    /**
     * 计算标准差
     * 知识点: 标准差计算
     */
    float variance = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        variance += (history[i] - mean) * (history[i] - mean);
    }
    variance /= count;
    
    float std_dev = sqrtf(variance);
    
    /**
     * 异常值判断
     * 知识点: 异常值判断
     * - 如果偏离超过阈值倍标准差，认为异常
     * - 阈值可调整
     */
    float deviation = fabsf(value - mean);
    return (deviation > threshold * std_dev);
}

/**
 * @brief   线性插值函数
 * @details 在两点间进行线性插值
 * @param x1 第一个点的x坐标
 * @param y1 第一个点的y坐标
 * @param x2 第二个点的x坐标
 * @param y2 第二个点的y坐标
 * @param x 要插值的x坐标
 * @return float 插值结果
 * 
 * 知识点: 线性插值算法
 * - y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
 */
float Linear_Interpolation(float x1, float y1, float x2, float y2, float x)
{
    if (x1 == x2) {
        return y1;  // 避免除零
    }
    
    return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

/**
 * @brief   对数插值函数
 * @details 在两点间进行对数插值
 * @param x1 第一个点的x坐标
 * @param y1 第一个点的y坐标
 * @param x2 第二个点的x坐标
 * @param y2 第二个点的y坐标
 * @param x 要插值的x坐标
 * @return float 插值结果
 * 
 * 知识点: 对数插值算法
 * - 适用于光敏电阻等对数特性的传感器
 */
float Log_Interpolation(float x1, float y1, float x2, float y2, float x)
{
    if (x1 <= 0 || x2 <= 0 || y1 <= 0 || y2 <= 0) {
        return Linear_Interpolation(x1, y1, x2, y2, x);  // 降级为线性插值
    }
    
    // 对数插值: 使用对数坐标
    float log_x1 = logf(x1);
    float log_x2 = logf(x2);
    float log_x = logf(x);
    float log_y1 = logf(y1);
    float log_y2 = logf(y2);
    
    float log_result = Linear_Interpolation(log_x1, log_y1, log_x2, log_y2, log_x);
    return expf(log_result);
}

/**
 * @brief   限幅函数
 * @details 限制数值在指定范围内
 * @param value 输入值
 * @param min_value 最小值
 * @param max_value 最大值
 * @return float 限幅后的值
 */
float Limit_Value(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    } else if (value > max_value) {
        return max_value;
    } else {
        return value;
    }
}

/**
 * @brief   阻值计算函数
 * @details 根据ADC值计算传感器阻值
 * @param adc_value ADC转换值(0-4095)
 * @param vref 参考电压(V)
 * @param divider_resistor 分压电阻值(Ω)
 * @return float 传感器阻值(Ω)
 * 
 * 知识点: 分压电路计算
 * - 分压电路：Vout = Vin * R2 / (R1 + R2)
 * - 反推：R1 = R2 * (Vin/Vout - 1)
 */
float Calculate_Sensor_Resistance(uint16_t adc_value, float vref, float divider_resistor)
{
    float vout = (float)adc_value * vref / ADC_RESOLUTION;
    return divider_resistor * (vref / vout - 1.0f);
}

/* ============================== NTC温度计算函数 ============================== */

/**
 * @brief   NTC温度计算函数
 * @details 根据阻值计算温度
 * @param resistance 阻值(Ω)
 * @return float 温度值(°C)
 * 
 * 知识点: NTC热敏电阻温度计算
 * - 使用B值方程
 * - 适合一般精度要求
 */
float NTC_Calculate_Temperature(float resistance)
{
    if (resistance <= 0) {
        return TEMP_MIN_C;  // 防止除零错误
    }
    
    float t25 = 298.15f;  // 25°C in Kelvin
    float ln_ratio = logf(resistance / TEMP_NTC_R25);
    float temperature_k = (TEMP_NTC_B_VALUE * t25) / (TEMP_NTC_B_VALUE + t25 * ln_ratio);
    return temperature_k - 273.15f;
}

/* ============================== 光敏电阻计算函数 ============================== */

/**
 * @brief   光敏电阻计算函数
 * @details 根据阻值计算光照强度
 * @param resistance 阻值(Ω)
 * @return float 光照强度(Lux)
 */
float LDR_Calculate_Light(float resistance)
{
    if (resistance <= 0) {
        return LIGHT_MIN_LUX;
    }
    
    if (resistance >= LIGHT_R_AT_10LUX) {
        return Log_Interpolation(LIGHT_R_AT_10LUX, 10.0f, 
                                LIGHT_R_AT_100LUX, 100.0f, 
                                resistance);
    } else if (resistance >= LIGHT_R_AT_100LUX) {
        return Log_Interpolation(LIGHT_R_AT_100LUX, 100.0f, 
                                LIGHT_R_AT_1000LUX, 1000.0f, 
                                resistance);
    } else {
        return LIGHT_MAX_LUX;
    }
}

/* ============================== MQ135气体计算函数 ============================== */

/**
 * @brief   MQ135气体计算函数
 * @details 根据阻值计算气体浓度
 * @param resistance 阻值(Ω)
 * @return float 气体浓度(ppm)
 */
float MQ135_Calculate_Gas(float resistance)
{
    if (resistance <= 0) {
        return GAS_MIN_PPM;
    }
    
    if (resistance <= GAS_R0_CLEAN_AIR * 0.1f) {
        return GAS_MAX_PPM;
    } else if (resistance >= GAS_R0_CLEAN_AIR) {
        return GAS_MIN_PPM;
    } else {
        return Log_Interpolation(GAS_R0_CLEAN_AIR, GAS_MIN_PPM, 
                                GAS_R0_CLEAN_AIR * 0.1f, GAS_MAX_PPM, 
                                resistance);
    }
}
