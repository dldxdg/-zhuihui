/**
 * @file    adc.h
 * @brief   ADC传感器数据采集头文件 (标准库版本)
 * @author  MiniMax Agent
 * @version V2.0
 * @date    2025-11-08
 * 
 * 知识点总结:
 * 
 * 【ADC模数转换器基础】
 * 1. ADC将模拟信号转换为数字信号
 * 2. STM32F103内置12位ADC，精度较高
 * 3. 转换范围：0-3.3V对应0-4095
 * 4. 支持多通道转换，可同时采样
 * 
 * 【传感器接口设计】
 * 1. 热敏电阻：温度检测，阻值随温度变化
 * 2. 光敏电阻：光线检测，阻值随光照变化
 * 3. MQ135：气体检测，阻值随气体浓度变化
 * 4. 都需要分压电路将电阻变化转换为电压变化
 * 
 * 【数据处理流程】
 * 1. ADC采样原始数据(0-4095)
 * 2. 转换为电压值(0-3.3V)
 * 3. 根据传感器特性转换为实际物理量
 * 4. 数据滤波和平滑处理
 * 5. 异常值检测和处理
 */

#ifndef __ADC_H
#define __ADC_H

/* ============================== 包含头文件 ============================== */

#include "stm32f1xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ============================== 宏定义 ============================== */

/**
 * ADC配置参数
 * 知识点: ADC时钟和时序配置
 * - ADC时钟不能超过14MHz
 * - 采样时间影响转换精度
 * - 连续转换提高采样速度
 */
#define ADC_CLOCK_PRESCALER     6          // ADC时钟预分频：72MHz/6 = 12MHz
#define ADC_SAMPLE_TIME         ADC_SampleTime_55Cycles5  // 采样时间：55.5周期
#define ADC_RESOLUTION          4096       // ADC分辨率：12位
#define ADC_VREF                3.3f       // 参考电压：3.3V

/**
 * 传感器通道定义
 * 知识点: ADC通道分配
 * - 通道1(PA1): 温度传感器
 * - 通道4(PA4): 光线传感器
 * - 通道9(PB1): 气体传感器
 */
#define TEMP_SENSOR_CHANNEL     ADC_Channel_1   // 温度传感器通道
#define LIGHT_SENSOR_CHANNEL    ADC_Channel_4   // 光线传感器通道
#define GAS_SENSOR_CHANNEL      ADC_Channel_9   // 气体传感器通道

/**
 * 采样参数
 * 知识点: 采样配置
 * - 采样次数用于平均滤波
 * - 转换时间计算
 * - 超时时间设置
 */
#define ADC_SAMPLE_COUNT        5           // 每个传感器采样次数
#define ADC_CONVERSION_TIMEOUT  100         // 转换超时时间(毫秒)
#define ADC_STARTUP_DELAY_MS    10          // ADC启动延时

/**
 * 温度传感器参数
 * 知识点: 热敏电阻特性
 * - 25度时的阻值：10kΩ
 * - B值(热敏常数)：3950
 * - 温度范围：-40°C到125°C
 * - 分压电阻：10kΩ
 */
#define TEMP_NTC_R25            10000.0f    // 25°C时NTC阻值(Ω)
#define TEMP_NTC_B_VALUE        3950        // NTC B值
#define TEMP_VOLTAGE_DIVIDER    10000.0f    // 分压电阻值(Ω)
#define TEMP_MIN_C              -40.0f      // 温度最小值(°C)
#define TEMP_MAX_C              125.0f      // 温度最大值(°C)

/**
 * 光线传感器参数
 * 知识点: 光敏电阻特性
 * - 不同光照下的阻值变化
 * - 10Lux：10kΩ
 * - 100Lux：1kΩ
 * - 1000Lux：100Ω
 * - 分压电阻：10kΩ
 */
#define LIGHT_R_AT_10LUX        10000.0f    // 10Lux时阻值(Ω)
#define LIGHT_R_AT_100LUX       1000.0f     // 100Lux时阻值(Ω)
#define LIGHT_R_AT_1000LUX      100.0f      // 1000Lux时阻值(Ω)
#define LIGHT_VOLTAGE_DIVIDER   10000.0f    // 分压电阻值(Ω)
#define LIGHT_MIN_LUX           0.0f        // 最小光照(Lux)
#define LIGHT_MAX_LUX           1000.0f     // 最大光照(Lux)

/**
 * 气体传感器参数
 * 知识点: MQ135气体传感器
 * - 主要检测NH3、CO2、酒精等气体
 * - 清洁空气中阻值约30kΩ
 * - 污染气体中阻值下降到几kΩ
 * - 分压电阻：10kΩ
 */
#define GAS_R0_CLEAN_AIR        30000.0f    // 清洁空气中阻值(Ω)
#define GAS_VOLTAGE_DIVIDER     10000.0f    // 分压电阻值(Ω)
#define GAS_MIN_PPM             10.0f       // 最小检测浓度(ppm)
#define GAS_MAX_PPM             1000.0f     // 最大检测浓度(ppm)

/**
 * 数据滤波参数
 * 知识点: 数字滤波
 * - 滑动平均滤波
 * - 异常值检测
 * - 变化率限制
 */
#define FILTER_WINDOW_SIZE      10          // 滑动平均窗口大小
#define MAX_VALUE_JUMP          100.0f      // 最大允许值跳变
#define TEMP_JUMP_THRESHOLD     5.0f        // 温度跳变阈值(°C)
#define LIGHT_JUMP_THRESHOLD    200.0f      // 光线跳变阈值(Lux)
#define GAS_JUMP_THRESHOLD      100.0f      // 气体跳变阈值(ppm)

/* ============================== 数据结构定义 ============================== */

/**
 * 原始ADC数据结构
 * 知识点: 原始采样数据
 * - 存储未处理的ADC转换结果
 * - 用于故障诊断和调试
 * - 可以回放历史数据
 */
typedef struct {
    uint16_t temperature_raw;      // 温度传感器原始ADC值
    uint16_t light_raw;            // 光线传感器原始ADC值
    uint16_t gas_raw;              // 气体传感器原始ADC值
} ADC_Raw_Data_t;

/**
 * 传感器数据结构 (无时间戳)
 * 知识点: 处理后的传感器数据
 * - 温度：浮点数，摄氏度
 * - 光线：浮点数，Lux
 * - 气体：浮点数，ppm
 * - 无时间戳(按用户要求)
 */
typedef struct {
    float temperature;             // 温度值 (摄氏度, -40~125°C)
    float light_level;             // 光线强度 (Lux, 0~1000Lux)
    float gas_level;               // 气体浓度 (ppm, 10~1000ppm)
} Sensor_Data_t;

/**
 * 传感器校准参数结构
 * 知识点: 传感器校准数据
 * - 存储校准系数和偏移
 * - 支持温度补偿
 * - 老化补偿参数
 */
typedef struct {
    float temp_offset;             // 温度偏移(°C)
    float temp_slope;              // 温度斜率校正
    float light_offset;            // 光线偏移(Lux)
    float light_slope;             // 光线斜率校正
    float gas_offset;              // 气体偏移(ppm)
    float gas_slope;               // 气体斜率校正
    uint32_t calibration_time;     // 校准时间戳(系统运行时间)
} Sensor_Calibration_t;

/**
 * 传感器历史数据结构
 * 知识点: 滑动平均滤波
 * - 存储历史数据用于滤波
 * - 环形缓冲区实现
 * - 异常值检测
 */
typedef struct {
    float temp_history[FILTER_WINDOW_SIZE];    // 温度历史数据
    float light_history[FILTER_WINDOW_SIZE];   // 光线历史数据
    float gas_history[FILTER_WINDOW_SIZE];     // 气体历史数据
    uint8_t temp_index;                        // 温度数据索引
    uint8_t light_index;                       // 光线数据索引
    uint8_t gas_index;                         // 气体数据索引
    uint8_t temp_count;                        // 温度数据计数
    uint8_t light_count;                       // 光线数据计数
    uint8_t gas_count;                         // 气体数据计数
} Sensor_History_t;

/**
 * 传感器状态结构
 * 知识点: 传感器运行状态
 * - 传感器工作状态
 * - 错误计数
 * - 最后采样时间
 */
typedef struct {
    bool temp_sensor_ok;        // 温度传感器状态
    bool light_sensor_ok;       // 光线传感器状态
    bool gas_sensor_ok;         // 气体传感器状态
    uint32_t temp_error_count;  // 温度传感器错误计数
    uint32_t light_error_count; // 光线传感器错误计数
    uint32_t gas_error_count;   // 气体传感器错误计数
    uint32_t last_sample_time;  // 最后采样时间(系统运行时间)
    uint16_t total_samples;     // 总采样次数
} Sensor_Status_t;

/**
 * ADC配置结构
 * 知识点: ADC工作配置
 * - 存储ADC当前配置参数
 * - 用于动态调整
 * - 故障恢复参数
 */
typedef struct {
    ADC_InitTypeDef init_config;     // ADC初始化配置
    ADC_CommonInitTypeDef common_config;  // ADC公共配置
    bool continuous_mode;            // 连续转换模式
    uint16_t sample_count;           // 采样次数
    uint32_t timeout_ms;             // 超时时间
} ADC_Config_t;

/* ============================== 全局变量声明 ============================== */

// 传感器状态
extern Sensor_Status_t Sensor_Status;

// 传感器校准参数
extern Sensor_Calibration_t Sensor_Calibration;

// 传感器历史数据
extern Sensor_History_t Sensor_History;

// ADC配置
extern ADC_Config_t ADC_Config;

/* ============================== 函数声明 ============================== */

/**
 * ADC模块初始化函数
 * 知识点: ADC初始化流程
 * 1. 初始化ADC硬件
 * 2. 配置GPIO引脚
 * 3. 设置校准参数
 * 4. 初始化历史数据缓冲区
 */
void ADC_Module_Init(void);

/**
 * ADC硬件初始化函数
 * 知识点: ADC硬件配置
 * - 配置ADC时钟和时序
 * - 设置转换参数
 * - 启用ADC和校准
 */
void ADC_Hardware_Init(void);

/**
 * GPIO配置函数
 * 知识点: ADC引脚配置
 * - 配置传感器引脚为模拟输入
 * - 设置引脚速度和模式
 * - 禁用上拉下拉(模拟输入)
 */
void ADC_GPIO_Config(void);

/**
 * ADC校准函数
 * 知识点: ADC校准过程
 * - 启动校准序列
 * - 等待校准完成
 * - 验证校准结果
 */
void ADC_Calibrate(void);

/**
 * 单通道采样函数
 * 知识点: 单次ADC转换
 * - 选择ADC通道
 * - 启动转换
 * - 等待转换完成
 * - 返回转换结果
 * @param channel ADC通道
 * @return uint16_t 转换结果(0-4095)
 */
uint16_t ADC_Single_Channel(uint8_t channel);

/**
 * 多通道采样函数
 * 知识点: 多通道ADC转换
 * - 依次采样多个通道
 * - 返回原始ADC数据
 * @param raw_data 存储原始数据的指针
 * @return bool 采样是否成功
 */
bool ADC_Multi_Channel_Sample(ADC_Raw_Data_t *raw_data);

/**
 * 批量采样函数
 * 知识点: 多次采样取平均
 * - 对每个通道进行多次采样
 * - 计算平均值减少噪声
 * - 提高测量精度
 * @param raw_data 存储原始数据的指针
 * @return bool 采样是否成功
 */
bool ADC_Batch_Sample(ADC_Raw_Data_t *raw_data);

/**
 * 温度计算函数
 * 知识点: NTC热敏电阻温度计算
 * - 根据ADC值计算电阻
 * - 使用Steinhart-Hart方程计算温度
 * - B值简化计算方法
 * @param adc_value 温度传感器ADC值
 * @return float 温度值(摄氏度)
 */
float ADC_Calculate_Temperature(uint16_t adc_value);

/**
 * 光线强度计算函数
 * 知识点: 光敏电阻光强计算
 * - 根据ADC值计算电阻
 * - 使用对数关系计算光照强度
 * - 校准曲线拟合
 * @param adc_value 光线传感器ADC值
 * @return float 光照强度(Lux)
 */
float ADC_Calculate_Light_Level(uint16_t adc_value);

/**
 * 气体浓度计算函数
 * 知识点: MQ135气体浓度计算
 * - 根据ADC值计算电阻
 * - 使用对数关系计算气体浓度
 * - 校准基准点：清洁空气
 * @param adc_value 气体传感器ADC值
 * @return float 气体浓度(ppm)
 */
float ADC_Calculate_Gas_Level(uint16_t adc_value);

/**
 * 传感器数据处理函数
 * 知识点: 原始数据转换为物理量
 * - 转换ADC值为物理量
 * - 应用校准参数
 * - 数据有效性检查
 * @param raw_data 原始ADC数据
 * @param sensor_data 转换后的传感器数据
 * @return bool 处理是否成功
 */
bool ADC_Process_Sensor_Data(const ADC_Raw_Data_t *raw_data, Sensor_Data_t *sensor_data);

/**
 * 传感器数据滤波函数
 * 知识点: 数字滤波处理
 * - 滑动平均滤波
 * - 异常值检测
 * - 跳变限制
 * @param sensor_data 传感器数据
 * @return bool 滤波是否成功
 */
bool ADC_Filter_Sensor_Data(Sensor_Data_t *sensor_data);

/**
 * 数据有效性检查函数
 * 知识点: 数据合理性验证
 * - 检查温度范围
 * - 检查光线范围
 * - 检查气体范围
 * @param sensor_data 传感器数据
 * @return bool 数据是否有效
 */
bool ADC_Validate_Data(const Sensor_Data_t *sensor_data);

/**
 * 传感器历史数据更新函数
 * 知识点: 历史数据维护
 * - 将新数据加入历史缓冲区
 * - 更新索引和计数
 * - 保持环形缓冲特性
 * @param sensor_data 当前传感器数据
 */
void ADC_Update_History(const Sensor_Data_t *sensor_data);

/**
 * 传感器状态检查函数
 * 知识点: 传感器健康检查
 * - 检查ADC转换结果
 * - 验证数据合理性
 * - 更新传感器状态
 * @param raw_data 原始ADC数据
 * @return bool 传感器是否正常
 */
bool ADC_Check_Sensor_Status(const ADC_Raw_Data_t *raw_data);

/**
 * 传感器校准函数
 * 知识点: 传感器校准流程
 * - 校准温度传感器
 * - 校准光线传感器
 * - 校准气体传感器
 * - 存储校准参数
 * @return bool 校准是否成功
 */
bool ADC_Calibrate_Sensors(void);

/**
 * 单次数据采集函数
 * 知识点: 完整的数据采集流程
 * 1. 执行ADC采样
 * 2. 转换原始数据
 * 3. 滤波处理
 * 4. 状态检查
 * 5. 更新历史数据
 * @param sensor_data 采集到的传感器数据
 * @return bool 采集是否成功
 */
bool ADC_Single_Sample(Sensor_Data_t *sensor_data);

/**
 * 连续数据采集函数
 * 知识点: 连续采样模式
 * - 持续采集传感器数据
 * - 适用于实时监控
 * - 有停止条件
 * @param sample_interval_ms 采样间隔(毫秒)
 * @param max_samples 最大采样次数(0=无限制)
 * @return uint32_t 实际采样次数
 */
uint32_t ADC_Continuous_Sample(uint32_t sample_interval_ms, uint32_t max_samples);

/**
 * 传感器状态重置函数
 * 知识点: 传感器状态重置
 * - 重置错误计数
 * - 重新初始化历史数据
 * - 恢复默认校准参数
 */
void ADC_Reset_Sensor_Status(void);

/* ============================== 调试函数声明 ============================== */

#ifdef DEBUG
/**
 * ADC状态打印函数
 * 知识点: ADC状态监控
 * - 打印ADC配置信息
 * - 打印传感器状态
 * - 打印校准参数
 */
void ADC_Print_Status(void);

/**
 * 原始数据打印函数
 * 知识点: 原始数据诊断
 * - 打印原始ADC值
 * - 打印转换后的电压值
 * - 便于故障诊断
 * @param raw_data 原始ADC数据
 */
void ADC_Print_Raw_Data(const ADC_Raw_Data_t *raw_data);

/**
 * 传感器数据打印函数
 * 知识点: 传感器数据显示
 * - 打印处理后的传感器数据
 * - 打印数据有效性
 * - 便于监控和调试
 * @param sensor_data 传感器数据
 */
void ADC_Print_Sensor_Data(const Sensor_Data_t *sensor_data);

/**
 * 校准参数打印函数
 * 知识点: 校准信息显示
 * - 打印校准系数
 * - 打印校准时间
 * - 便于验证校准结果
 */
void ADC_Print_Calibration(void);

/**
 * 历史数据打印函数
 * 知识点: 历史数据显示
 * - 打印历史数据统计
 * - 打印滤波效果
 * - 便于分析数据质量
 */
void ADC_Print_History(void);

/**
 * ADC性能测试函数
 * 知识点: ADC性能评估
 * - 测试转换时间
 * - 测试转换精度
 * - 测试稳定性
 * @param test_samples 测试采样次数
 */
void ADC_Performance_Test(uint32_t test_samples);
#endif

/* ============================== 辅助函数声明 ============================== */

/**
 * NTC温度计算函数
 * 知识点: NTC热敏电阻计算
 * @param resistance 阻值(Ω)
 * @return float 温度值(°C)
 */
float NTC_Calculate_Temperature(float resistance);

/**
 * 光敏电阻计算函数
 * 知识点: 光敏电阻计算
 * @param resistance 阻值(Ω)
 * @return float 光照强度(Lux)
 */
float LDR_Calculate_Light(float resistance);

/**
 * MQ135气体计算函数
 * 知识点: MQ135传感器计算
 * @param resistance 阻值(Ω)
 * @return float 气体浓度(ppm)
 */
float MQ135_Calculate_Gas(float resistance);

/**
 * 滑动平均滤波函数
 * 知识点: 移动平均滤波
 * @param new_value 新数据值
 * @param history 历史数据数组
 * @param history_size 历史数据大小
 * @param current_index 当前索引
 * @param count 当前计数
 * @return float 滤波后的值
 */
float Moving_Average_Filter(float new_value, float *history, uint8_t history_size, 
                           uint8_t *current_index, uint8_t *count);

/**
 * 异常值检测函数
 * 知识点: 异常值识别
 * @param value 当前值
 * @param history 历史数据数组
 * @param count 历史数据数量
 * @param threshold 异常阈值
 * @return bool 是否为异常值
 */
bool Is_Outlier(float value, const float *history, uint8_t count, float threshold);

/**
 * 线性插值函数
 * 知识点: 线性插值计算
 * @param x1 第一个点的x坐标
 * @param y1 第一个点的y坐标
 * @param x2 第二个点的x坐标
 * @param y2 第二个点的y坐标
 * @param x  要插值的x坐标
 * @return float 插值结果
 */
float Linear_Interpolation(float x1, float y1, float x2, float y2, float x);

/**
 * 对数插值函数
 * 知识点: 对数插值计算
 * @param x1 第一个点的x坐标
 * @param y1 第一个点的y坐标
 * @param x2 第二个点的x坐标
 * @param y2 第二个点的y坐标
 * @param x  要插值的x坐标
 * @return float 插值结果
 */
float Log_Interpolation(float x1, float y1, float x2, float y2, float x);

/**
 * 限幅函数
 * 知识点: 数值限幅
 * @param value 输入值
 * @param min_value 最小值
 * @param max_value 最大值
 * @return float 限幅后的值
 */
float Limit_Value(float value, float min_value, float max_value);

/**
 * 阻值计算函数
 * 知识点: 分压电路阻值计算
 * @param adc_value ADC转换值(0-4095)
 * @param vref 参考电压(V)
 * @param divider_resistor 分压电阻值(Ω)
 * @return float 传感器阻值(Ω)
 */
float Calculate_Sensor_Resistance(uint16_t adc_value, float vref, float divider_resistor);

#endif /* __ADC_H */
