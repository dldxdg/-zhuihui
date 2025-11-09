/**
 * @file    main.h
 * @brief   STM32F103环境监测系统 - 主程序头文件 (标准库版本)
 * @author  MiniMax Agent
 * @version V2.0
 * @date    2025-11-08
 * 
 * 知识点总结:
 * 
 * 【STM32标准库特点】
 * 1. 直接操作寄存器，性能更高
 * 2. 代码更接近硬件，调试方便
 * 3. 与HAL库功能相同但接口不同
 * 4. 需要手动配置时钟和使能外设
 * 
 * 【头文件作用】
 * 1. 声明全局变量和函数
 * 2. 定义宏和常量
 * 3. 包含其他头文件
 * 4. 防止头文件重复包含
 * 
 * 【数据类型定义】
 * uint8_t/uint16_t/uint32_t: 无符号整数
 * int8_t/int16_t/int32_t: 有符号整数
 * float: 单精度浮点数
 * bool: 布尔值
 */

#ifndef __MAIN_H
#define __MAIN_H

/* ============================== 包含头文件 ============================== */

/* STM32标准库头文件 */
#include "stm32f1xx.h"          // STM32F1xx系列主头文件
#include "system_stm32f1xx.h"   // 系统时钟配置
#include "FreeRTOS.h"           // FreeRTOS实时操作系统
#include "task.h"               // 任务管理
#include "queue.h"              // 队列管理
#include "semphr.h"             // 信号量和互斥锁

/* 项目头文件 */
#include "usart.h"              // 串口通信
#include "adc.h"                // ADC传感器采集
#include "flash.h"              // 内部Flash管理
#include "spi_flash.h"          // SPI Flash管理
#include "gpio.h"               // GPIO控制
#include "delay.h"              // 延时函数
#include "tasks.h"              // FreeRTOS任务

/* 标准C库 */
#include <stdint.h>             // 标准整数类型
#include <stdbool.h>            // 标准布尔类型
#include <stdio.h>              // 标准输入输出
#include <string.h>             // 字符串处理

/* ============================== 宏定义 ============================== */

/**
 * 任务优先级定义
 * 知识点: FreeRTOS任务优先级，数值越大优先级越高
 * - 0: 空闲任务(系统保留)
 * - 1-3: 用户任务(数值越大优先级越高)
 * - 4+: 高优先级任务保留
 */
#define NETWORK_TASK_PRIORITY      3   // 网络管理任务 - 最高优先级
#define DATA_SEND_TASK_PRIORITY    2   // 数据发送任务
#define DATA_COLLECT_TASK_PRIORITY 2   // 数据采集任务
#define SYSTEM_MONITOR_PRIORITY    1   // 系统监控任务 - 最低优先级

/**
 * 任务堆栈大小定义
 * 知识点: 堆栈大小以字(4字节)为单位
 * - 通常每个任务需要128-512字堆栈
 * - 复杂的任务需要更大堆栈
 */
#define NETWORK_TASK_STACK_SIZE    256 // 网络任务堆栈(1024字节)
#define DATA_SEND_TASK_STACK_SIZE  256 // 发送任务堆栈
#define DATA_COLLECT_TASK_STACK_SIZE 256 // 采集任务堆栈
#define SYSTEM_MONITOR_STACK_SIZE  128 // 监控任务堆栈(最小)

/**
 * 队列长度定义
 * 知识点: 队列用于任务间数据传递
 * - 长度太大会浪费内存
 * - 长度太小会导致数据丢失
 */
#define NETWORK_QUEUE_LENGTH       10  // 网络消息队列长度
#define DATA_QUEUE_LENGTH          50  // 数据队列长度

/**
 * 串口缓冲区大小
 * 知识点: 串口接收缓冲区大小
 * - 足够大以避免数据丢失
 * - 考虑最大AT指令长度
 */
#define USART1_RX_BUFFER_SIZE      256 // USART1接收缓冲区
#define USART2_RX_BUFFER_SIZE      256 // USART2接收缓冲区  
#define USART3_RX_BUFFER_SIZE      256 // USART3接收缓冲区

/**
 * 延时时间定义
 * 知识点: 常用延时时间宏定义
 * - 便于统一修改时间参数
 * - 提高代码可读性
 */
#define SYSTEM_START_DELAY_MS      1000    // 系统启动延时1秒
#define WIFI_INIT_DELAY_MS         3000    // WiFi模块初始化延时3秒
#define DATA_SAMPLE_INTERVAL_MS    5000    // 数据采集间隔5秒
#define WIFI_HEARTBEAT_INTERVAL_MS 5000    // WiFi心跳检测间隔5秒
#define DATA_SEND_INTERVAL_MS      10000   // 数据发送间隔10秒
#define SYSTEM_MONITOR_INTERVAL_MS 1000    // 系统监控间隔1秒

/* ============================== 数据结构定义 ============================== */

/**
 * 系统运行状态枚举
 * 知识点: 枚举类型用于定义状态变量
 * - 提高代码可读性
 * - 编译器进行类型检查
 */
typedef enum {
    SYSTEM_STATE_INIT = 0,        // 系统初始化状态
    SYSTEM_STATE_RUNNING,         // 系统运行状态
    SYSTEM_STATE_ERROR,           // 系统错误状态
    SYSTEM_STATE_STOPPED          // 系统停止状态
} System_State_t;

/**
 * 传感器数据结构 (无时间戳)
 * 知识点: 结构体用于组织相关数据
 * - 温度: 浮点数，精确到0.1度
 * - 光线: 16位无符号整数，0-4095范围
 * - 气体: 16位无符号整数，ppm单位
 */
typedef struct {
    float temperature;            // 温度值 (摄氏度, 范围: -40~85°C)
    uint16_t light_level;         // 光线强度 (0-4095, ADC原始值)
    uint16_t gas_level;           // 气体浓度 (ppm, MQ135传感器)
} Sensor_Data_t;

/**
 * 系统配置结构体
 * 知识点: 系统配置参数存储
 * - WiFi网络配置
 * - 服务器连接参数
 * - 系统运行参数
 */
typedef struct {
    char wifi_ssid[32];           // WiFi网络名称
    char wifi_password[32];       // WiFi密码
    char server_ip[16];           // 服务器IP地址
    uint16_t server_port;         // 服务器端口号
    uint8_t sample_interval;      // 采集间隔(秒)
    uint8_t send_interval;        // 发送间隔(秒)
} System_Config_t;

/**
 * 系统状态结构体
 * 知识点: 系统当前状态信息
 * - 实时运行状态
 * - 各种计数器
 * - 错误标志
 */
typedef struct {
    System_State_t state;         // 系统当前状态
    uint32_t start_time;          // 系统启动时间(使用TIM2计数)
    uint32_t error_count;         // 错误计数器
    uint32_t data_collected;      // 已采集数据条数
    uint32_t data_sent;           // 已发送数据条数
    bool wifi_connected;          // WiFi连接状态
    bool bluetooth_connected;     // 蓝牙连接状态
} System_Status_t;

/* ============================== 全局变量声明 ============================== */

/**
 * 全局变量声明 (在main.c中定义)
 * 知识点: extern关键字声明外部变量
 * - 在其他文件中可以使用这些变量
 * - 需要在对应的源文件中定义
 */

// 系统配置和状态
extern System_Config_t System_Config;     // 系统配置
extern System_Status_t System_Status;     // 系统状态

// FreeRTOS对象
extern xQueueHandle NetworkQueue;         // 网络消息队列
extern xQueueHandle DataQueue;            // 数据队列
extern xSemaphoreHandle USART2_Mutex;     // USART2互斥锁
extern xSemaphoreHandle USART3_Mutex;     // USART3互斥锁

// 任务句柄
extern xTaskHandle NetworkTaskHandle;     // 网络管理任务句柄
extern xTaskHandle DataCollectTaskHandle; // 数据采集任务句柄
extern xTaskHandle DataSendTaskHandle;    // 数据发送任务句柄
extern xTaskHandle SystemMonitorTaskHandle; // 系统监控任务句柄

/* ============================== 函数声明 ============================== */

/**
 * 系统初始化函数
 * 知识点: 初始化函数组织
 * - 硬件初始化
 * - 软件初始化
 * - 系统配置加载
 */
void System_Init(void);

/**
 * 硬件初始化函数
 * 知识点: 硬件外设初始化顺序
 * 1. 时钟配置
 * 2. GPIO配置
 * 3. 外设初始化
 * 4. 中断配置
 */
void Hardware_Init(void);

/**
 * FreeRTOS对象创建函数
 * 知识点: FreeRTOS对象创建
 * - 队列创建
 * - 互斥锁创建
 * - 信号量创建
 */
void FreeRTOS_Objects_Create(void);

/**
 * FreeRTOS任务创建函数
 * 知识点: 任务创建
 * - 任务函数
 * - 任务堆栈
 * - 任务优先级
 * - 任务句柄
 */
void FreeRTOS_Tasks_Create(void);

/**
 * 系统配置加载函数
 * 知识点: 配置参数加载
 * - 从内部Flash读取
 * - 默认值设置
 * - 参数验证
 */
void System_Config_Load(void);

/**
 * 系统配置保存函数
 * 知识点: 配置参数保存
 * - 写入内部Flash
 * - 擦除前检查
 * - 写入后验证
 */
void System_Config_Save(void);

/**
 * LED状态控制函数
 * 知识点: LED控制
 * - 系统状态指示
 * - 网络状态指示
 * - 错误状态指示
 */
void System_LED_Control(void);

/**
 * 系统重启函数
 * 知识点: 软件重启
 * - 关闭所有外设
 * - 重置系统状态
 * - 重新初始化
 */
void System_Reboot(void);

/**
 * 系统错误处理函数
 * 知识点: 错误处理机制
 * - 错误日志记录
 * - 错误恢复尝试
 * - 系统状态更新
 */
void System_Error_Handler(void);

/**
 * 系统时间获取函数
 * 知识点: 系统运行时间
 * - 使用TIM2计数器
 * - 返回毫秒数
 * - 32位计数器，溢出处理
 */
uint32_t System_Get_Runtime_MS(void);

/* ============================== 调试和测试函数 ============================== */

#ifdef DEBUG
/**
 * 系统信息打印函数 (调试模式)
 * 知识点: 调试信息输出
 * - 系统状态
 * - 配置信息
 * - 统计信息
 */
void System_Print_Info(void);

/**
 * 内存使用情况打印函数 (调试模式)
 * 知识点: 内存监控
 * - 堆使用情况
 * - 堆栈使用情况
 * - 任务堆栈使用
 */
void System_Print_Memory_Usage(void);

/**
 * 任务状态打印函数 (调试模式)
 * 知识点: 任务监控
 * - 任务状态
 - 任务优先级
 * - 任务运行时间
 */
void System_Print_Task_Status(void);
#endif

/* ============================== 硬件相关函数声明 ============================== */

/**
 * 系统时钟配置函数
 * 知识点: STM32时钟系统
 * - HSE外部高速时钟: 8MHz
 * - PLL倍频: 9倍 = 72MHz
 * - AHB总线时钟: 72MHz
 * - APB1低速总线: 36MHz
 * - APB2高速总线: 72MHz
 */
void SystemClock_Config(void);

/**
 * NVIC中断优先级配置函数
 * 知识点: 中断优先级管理
 * - 优先级分组: 2位抢占优先级, 2位子优先级
 * - 数值越小优先级越高
 * - 重要的中断需要更高优先级
 */
void NVIC_Config(void);

/**
 * GPIO初始化函数
 * 知识点: GPIO配置
 * - LED引脚: PA0, PB8
 * - 蜂鸣器引脚: PB7
 * - 按键引脚: PA11(上拉输入)
 * - 传感器引脚: PA1, PA4, PB1(模拟输入)
 */
void GPIO_Init(void);

/**
 * USART初始化函数
 * 知识点: 串口配置
 * - USART1: PA9(TX), PA10(RX), 9600bps, PC调试
 * - USART2: PA2(TX), PA3(RX), 9600bps, 蓝牙通信
 * - USART3: PB10(TX), PB11(RX), 115200bps, WiFi通信
 */
void USART_Init(void);

/**
 * ADC初始化函数
 * 知识点: ADC配置
 * - ADC1: 12位分辨率
 * - 时钟频率: 12MHz (72MHz/6)
 * - 采样时间: 239.5个时钟周期
 * - 转换时间: 约21微秒
 * - 通道1: PA1(温度传感器)
 * - 通道4: PA4(光线传感器) 
 * - 通道9: PB1(气体传感器)
 */
void ADC_Init(void);

/**
 * TIM2初始化函数
 * 知识点: 定时器配置
 * - TIM2: 通用定时器
 * - 时钟源: APB1(36MHz)
 * - 预分频: 36-1 = 35 (1MHz计数频率)
 * - 自动重载: 0xFFFFFFFF (最大计数)
 * - 用途: 微秒/毫秒级精确延时
 */
void TIM2_Init(void);

/**
 * TIM4初始化函数
 * 知识点: 定时器配置
 * - TIM4: 通用定时器
 * - 时钟源: APB1(36MHz)
 * - 预分频: 36000-1 = 35999 (1kHz计数频率)
 * - 自动重载: 1000-1 = 999 (1秒中断周期)
 * - 用途: 系统心跳，时基统计
 */
void TIM4_Init(void);

/**
 * IWDG看门狗初始化函数
 * 知识点: 看门狗配置
 * - IWDG: 独立看门狗
 * - 时钟源: LSI内部低速RC(40kHz)
 * - 预分频: 64
 * - 重装载值: 1000
 * - 超时时间: (64*1000)/40kHz = 1.6秒
 */
void IWDG_Init(void);

/* ============================== 条件编译 ============================== */

/**
 * 调试模式开关
 * 知识点: 条件编译
 * - DEBUG宏定义时启用调试功能
 * - 发布版本时取消DEBUG定义
 * - 减少发布版本代码量
 */
#ifdef DEBUG
    #define DEBUG_PRINT(fmt, ...) printf("[DEBUG] " fmt, ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(fmt, ...) ((void)0)
#endif

/**
 * 错误处理宏
 * 知识点: 错误处理机制
 * - 发生错误时停止程序运行
 * - 打印错误信息
 * - 进入错误处理函数
 */
#define ERROR_HANDLER() do { \
    printf("ERROR: %s:%d - System error occurred!\r\n", __FILE__, __LINE__); \
    System_Error_Handler(); \
} while(0)

/**
 * 断言宏
 * 知识点: 运行时检查
 * - 检查条件是否成立
 * - 失败时进入错误处理
 * - 只在调试模式生效
 */
#ifdef DEBUG
    #define ASSERT(cond) do { \
        if (!(cond)) { \
            printf("ASSERT FAILED: %s at %s:%d\r\n", #cond, __FILE__, __LINE__); \
            while(1); \
        } \
    } while(0)
#else
    #define ASSERT(cond) ((void)0)
#endif

#endif /* __MAIN_H */
