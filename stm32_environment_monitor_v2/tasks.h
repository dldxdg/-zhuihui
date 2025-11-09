/**
 * @file    tasks.h
 * @brief   FreeRTOS任务系统头文件 (标准库版本)
 * @author  MiniMax Agent
 * @version V2.0
 * @date    2025-11-08
 * 
 * 重要说明:
 * - 此版本完全使用STM32标准库和FreeRTOS，无HAL函数
 * - 实现4个核心任务的完整架构
 * - 无时间戳依赖，所有时序控制基于任务调度
 * - 任务间通信使用FreeRTOS标准机制
 * 
 * 任务架构说明:
 * 
 * 【任务1: 网络管理任务 (NetworkManageTask)】
 * 负责ESP-01s WiFi模块的网络连接管理
 * - WiFi连接建立和维护
 * - AT指令状态机管理
 * - 网络连接状态监控
 * - 断线重连机制
 * 
 * 【任务2: 数据采集任务 (DataCollectTask)】
 * 负责环境传感器数据的周期性采集
 * - ADC多通道数据读取
 * - 数据校准和滤波
 * - 数据格式转换
 * - 采集频率控制
 * 
 * 【任务3: 数据发送任务 (DataSendTask)】
 * 负责数据的上传和存储管理
 * - 数据发送队列处理
 * - Flash存储管理
 * - 发送状态跟踪
 * - 错误重试机制
 * 
 * 【任务4: 系统监控任务 (SystemMonitorTask)】
 * 负责系统整体状态监控和异常处理
 * - 系统资源监控
 * - 错误日志记录
 * - LED状态指示
 * - 任务健康检查
 * 
 * 任务间通信机制:
 * 
 * 【队列 (Queue)】
 * - Data_Queue: 数据采集到发送的数据传输
 *   知识点: 队列是FreeRTOS的核心IPC机制
 *   - 生产者-消费者模式
 *   - 阻塞读/写机制
 *   - 队列长度配置
 *   - 数据拷贝管理
 * 
 * 【互斥锁 (Mutex)】
 * - Flash_Mutex: Flash访问的互斥保护
 *   知识点: 互斥锁解决资源竞争问题
 *   - 防止同时写入Flash
 *   - 优先级继承机制
 *   - 死锁避免策略
 * 
 * 【信号量 (Semaphore)】
 * - Network_Semaphore: 网络状态信号
 *   知识点: 信号量用于任务同步
 *   - 二值信号量管理
 *   - 释放和获取机制
 *   - 超时处理
 * 
 * 【事件组 (Event Group)】
 * - System_Events: 系统事件管理
 *   知识点: 事件组支持多条件同步
 *   - 事件标志位管理
 *   - 等待多个事件
 *   - 自动清除机制
 * 
 * 内存管理:
 * 
 * 【动态内存分配】
 * 任务堆栈分配策略
 * 知识点: FreeRTOS内存管理
 * - 任务堆栈大小计算
 * - 内存碎片化避免
 * - heap_4内存管理算法
 * - 内存泄漏检测
 * 
 * 调度策略:
 * 
 * 【优先级配置】
 * 知识点: 实时任务调度
 * - 优先级范围: 0(低) 到 configMAX_PRIORITIES-1(高)
 * - 优先级继承
 * - 抢占式调度
 * - 时间片轮转
 * 
 * 【任务状态】
 * 知识点: 任务生命周期
 * - Running: 正在运行
 * - Ready: 就绪状态
 * - Blocked: 阻塞状态  
 * - Suspended: 挂起状态
 * - Deleted: 已删除
 */

#ifndef __TASKS_H
#define __TASKS_H

/* ============================== 头文件包含 ============================== */

#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "main.h"
#include "usart.h"
#include "adc.h"
#include "flash.h"
#include "gpio.h"

/* ============================== 宏定义 ============================== */

/**
 * 任务优先级定义
 * 知识点: 优先级数值越大，优先级越高
 * 建议优先级分配:
 * - 高优先级: 网络管理、错误处理
 * - 中优先级: 数据采集、发送
 * - 低优先级: 系统监控
 */
#define NETWORK_MANAGE_TASK_PRIORITY    (tskIDLE_PRIORITY + 3)   /**< 网络管理任务: 高优先级 */
#define DATA_COLLECT_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)   /**< 数据采集任务: 中高优先级 */
#define DATA_SEND_TASK_PRIORITY         (tskIDLE_PRIORITY + 2)   /**< 数据发送任务: 中高优先级 */
#define SYSTEM_MONITOR_TASK_PRIORITY    (tskIDLE_PRIORITY + 1)   /**< 系统监控任务: 中优先级 */

/**
 * 任务堆栈大小定义
 * 知识点: 堆栈大小以字(4字节)为单位
 * 估算方法: 本地变量 + 函数调用深度 + 任务开销
 */
#define NETWORK_MANAGE_TASK_STACK_SIZE  (256)    /**< 网络管理任务堆栈: 256字(1KB) */
#define DATA_COLLECT_TASK_STACK_SIZE    (128)    /**< 数据采集任务堆栈: 128字(512B) */
#define DATA_SEND_TASK_STACK_SIZE       (128)    /**< 数据发送任务堆栈: 128字(512B) */
#define SYSTEM_MONITOR_TASK_STACK_SIZE  (128)    /**< 系统监控任务堆栈: 128字(512B) */

/**
 * 任务运行周期定义
 * 知识点: 任务周期影响系统响应性和资源消耗
 * - 采样频率: 影响数据精度
 * - 发送频率: 影响网络负载
 * - 监控频率: 影响系统开销
 */
#define DATA_COLLECT_PERIOD_MS          (2000)   /**< 数据采集周期: 2秒 */
#define DATA_SEND_PERIOD_MS             (5000)   /**< 数据发送周期: 5秒 */
#define SYSTEM_MONITOR_PERIOD_MS        (3000)   /**< 系统监控周期: 3秒 */
#define NETWORK_CHECK_PERIOD_MS         (1000)   /**< 网络检查周期: 1秒 */

/**
 * 队列配置
 * 知识点: 队列长度影响数据缓冲能力
 * - 数据队列: 缓冲多组传感器数据
 * - 错误队列: 缓冲错误信息
 */
#define DATA_QUEUE_LENGTH               (10)     /**< 数据队列长度: 10条消息 */
#define ERROR_QUEUE_LENGTH              (5)      /**< 错误队列长度: 5条消息 */
#define DATA_QUEUE_ITEM_SIZE            sizeof(Sensor_Data_t)  /**< 数据项大小 */

/**
 * 事件位定义
 * 知识点: 事件组支持多条件同步
 * 每个位代表一个特定事件的发生
 */
#define NETWORK_CONNECTED_EVENT         (1 << 0) /**< 网络连接成功事件 */
#define NETWORK_DISCONNECTED_EVENT      (1 << 1) /**< 网络断开事件 */
#define DATA_READY_EVENT                (1 << 2) /**< 数据就绪事件 */
#define FLASH_FULL_EVENT                (1 << 3) /**< Flash满事件 */
#define SYSTEM_ERROR_EVENT              (1 << 4) /**< 系统错误事件 */
#define KEY_PRESSED_EVENT               (1 << 5) /**< 按键按下事件 */
#define SYSTEM_RESET_EVENT              (1 << 6) /**< 系统重启事件 */

/* ============================== 类型定义 ============================== */

/**
 * 任务状态枚举
 * 知识点: 任务运行状态跟踪
 * 用于监控系统任务运行情况
 */
typedef enum {
    TASK_STATUS_STOPPED = 0,    /**< 任务停止 */
    TASK_STATUS_RUNNING,        /**< 任务运行 */
    TASK_STATUS_SUSPENDED,      /**< 任务挂起 */
    TASK_STATUS_ERROR           /**< 任务错误 */
} Task_Status_t;

/**
 * 任务统计信息结构
 * 知识点: 任务性能监控
 * 记录任务运行时间、循环次数等统计信息
 */
typedef struct {
    Task_Handle_t task_handle;           /**< 任务句柄 */
    Task_Status_t status;                /**< 任务状态 */
    uint32_t cycle_count;                /**< 循环计数 */
    uint32_t error_count;                /**< 错误计数 */
    uint32_t last_execution_time;        /**< 上次执行时间 */
    uint32_t total_execution_time;       /**< 总执行时间 */
    uint32_t max_execution_time;         /**< 最大执行时间 */
    uint32_t min_execution_time;         /**< 最小执行时间 */
    uint32_t stack_high_water_mark;      /**< 堆栈高水位标记 */
} Task_Statistics_t;

/**
 * 错误信息结构
 * 知识点: 错误信息传递
 * 用于任务间错误信息传递和处理
 */
typedef struct {
    uint8_t error_code;                  /**< 错误代码 */
    uint8_t task_id;                     /**< 发生错误的任务ID */
    uint32_t error_count;                /**< 错误累计计数 */
    char error_message[64];              /**< 错误描述信息 */
} Error_Info_t;

/**
 * 任务管理结构
 * 知识点: 任务统一管理
 * 包含所有任务的相关信息
 */
typedef struct {
    Task_Statistics_t network_manage;    /**< 网络管理任务统计 */
    Task_Statistics_t data_collect;      /**< 数据采集任务统计 */
    Task_Statistics_t data_send;         /**< 数据发送任务统计 */
    Task_Statistics_t system_monitor;    /**< 系统监控任务统计 */
    uint32_t system_start_time;          /**< 系统启动时间 */
    uint32_t total_task_errors;          /**< 总任务错误数 */
} Task_Management_t;

/* ============================== 全局变量声明 ============================== */

/**
 * 任务句柄声明
 * 知识点: 任务句柄用于任务控制
 * 每个任务都有对应的句柄用于管理
 */
extern Task_Handle_t NetworkManageTask_Handle;  /**< 网络管理任务句柄 */
extern Task_Handle_t DataCollectTask_Handle;    /**< 数据采集任务句柄 */
extern Task_Handle_t DataSendTask_Handle;       /**< 数据发送任务句柄 */
extern Task_Handle_t SystemMonitorTask_Handle;  /**< 系统监控任务句柄 */

/**
 * 通信对象声明
 * 知识点: 任务间通信机制
 * 队列、信号量、事件组用于任务协调
 */
extern QueueHandle_t Data_Queue;                /**< 数据队列句柄 */
extern QueueHandle_t Error_Queue;               /**< 错误队列句柄 */
extern SemaphoreHandle_t Flash_Mutex;           /**< Flash互斥锁 */
extern SemaphoreHandle_t Network_Semaphore;     /**< 网络状态信号量 */
extern EventGroupHandle_t System_Events;        /**< 系统事件组句柄 */

/**
 * 任务管理信息
 * 知识点: 全局任务管理
 * 包含所有任务的统计信息
 */
extern Task_Management_t Task_Management;

/* ============================== 函数声明 ============================== */

/**
 * @brief   任务系统初始化
 * @details 此函数初始化所有任务相关的资源
 *          包括创建任务、初始化通信对象等
 * @param   无
 * @return  bool: 初始化成功返回true，失败返回false
 * 
 * 知识点: 任务系统初始化流程
 * 1. 创建通信对象(队列、信号量、事件组)
 * 2. 创建任务
 * 3. 初始化统计信息
 * 4. 验证创建结果
 * 
 * 错误处理:
 * - 内存不足: 返回false
 * - 任务创建失败: 清理已创建资源
 * - 通信对象创建失败: 记录错误
 */
bool Tasks_Init(void);

/**
 * @brief   任务系统去初始化
 * @details 此函数清理任务系统资源
 *          包括删除任务、清理通信对象等
 * @param   无
 * @return  bool: 去初始化成功返回true，失败返回false
 * 
 * 知识点: 资源清理
 * 1. 停止所有任务
 * 2. 删除任务句柄
 * 3. 删除通信对象
 * 4. 释放内存
 * 
 * 注意事项:
 * - 只能在系统关闭时调用
 * - 确保没有任务在运行
 */
bool Tasks_DeInit(void);

/**
 * @brief   获取任务统计信息
 * @details 此函数获取指定任务的详细统计信息
 * @param   task_name: 任务名称字符串
 * @return  Task_Statistics_t*: 任务统计信息指针，失败返回NULL
 * 
 * 知识点: 任务监控
 * - 循环次数: 反映任务活跃度
 * - 执行时间: 反映任务性能
 * - 堆栈使用: 反映内存消耗
 * - 错误计数: 反映任务稳定性
 */
Task_Statistics_t* Tasks_GetTaskStatistics(const char* task_name);

/**
 * @brief   挂起指定任务
 * @details 此函数挂起指定任务，暂时停止其执行
 * @param   task_name: 任务名称字符串
 * @return  bool: 挂起成功返回true，失败返回false
 * 
 * 知识点: 任务挂起
 * - 挂起后任务不参与调度
 * - 可以通过ResumeTask恢复
 * - 不影响任务内部状态
 */
bool Tasks_SuspendTask(const char* task_name);

/**
 * @brief   恢复指定任务
 * @details 此函数恢复之前挂起的任务
 * @param   task_name: 任务名称字符串
 * @return  bool: 恢复成功返回true，失败返回false
 * 
 * 知识点: 任务恢复
 * - 恢复后任务重新参与调度
 * - 恢复时立即开始执行
 * - 保持挂起前的所有状态
 */
bool Tasks_ResumeTask(const char* task_name);

/**
 * @brief   设置任务事件
 * @details 此函数设置系统事件组中的事件位
 * @param   event_bits: 要设置的事件位
 * @param   clear_previous: 是否清除之前的事件位
 * @return  bool: 设置成功返回true，失败返回false
 * 
 * 知识点: 事件设置
 * - 可以设置单个或多个事件位
 * - 支持清除之前事件
 * - 事件设置会唤醒等待该事件的任务
 */
bool Tasks_SetEvent(uint32_t event_bits, bool clear_previous);

/**
 * @brief   清除任务事件
 * @details 此函数清除系统事件组中的事件位
 * @param   event_bits: 要清除的事件位
 * @return  bool: 清除成功返回true，失败返回false
 * 
 * 知识点: 事件清除
 * - 可以清除单个或多个事件位
 * - 不影响其他事件位
 * - 用于手动清理事件状态
 */
bool Tasks_ClearEvent(uint32_t event_bits);

/**
 * @brief   等待任务事件
 * @details 此函数等待指定的事件位设置
 * @param   event_bits: 要等待的事件位
 * @param   wait_all: 是否等待所有事件位
 * @param   timeout_ms: 超时时间(毫秒)
 * @return  uint32_t: 返回触发的事件位，超时返回0
 * 
 * 知识点: 事件等待
 * - 支持等待单个或多个事件
 * - 超时机制避免无限等待
 * - 自动清除已等待的事件位
 */
uint32_t Tasks_WaitEvent(uint32_t event_bits, bool wait_all, uint32_t timeout_ms);

/**
 * @brief   网络管理任务
 * @details 负责ESP-01s WiFi模块的网络连接管理
 *          包含连接建立、状态监控、断线重连等功能
 * @param   pvParameters: 任务参数
 * @return  无
 * 
 * 任务流程:
 * 1. 等待网络连接信号量
 * 2. 初始化ESP-01s模块
 * 3. 尝试建立WiFi连接
 * 4. 监控网络状态
 * 5. 处理网络异常和重连
 * 
 * 网络管理策略:
 * - 优先级: 高优先级确保网络及时响应
 * - 周期: 每秒检查网络状态
 * - 错误处理: 自动重连和故障恢复
 * - 状态指示: LED状态反映网络状态
 */
void NetworkManageTask(void *pvParameters);

/**
 * @brief   数据采集任务
 * @details 负责环境传感器数据的周期性采集
 *          包含ADC读取、数据处理、格式转换等功能
 * @param   pvParameters: 任务参数
 * @return  无
 * 
 * 任务流程:
 * 1. 定期触发ADC数据采集
 * 2. 读取温度、光线、气体传感器数据
 * 3. 数据校准和滤波处理
 * 4. 格式转换和验证
 * 5. 发送到数据队列
 * 
 * 采集策略:
 * - 优先级: 中高优先级保证数据实时性
 * - 周期: 2秒采集一次数据
 * - 数据处理: 简单滤波和校准
 * - 队列发送: 异步数据传输
 */
void DataCollectTask(void *pvParameters);

/**
 * @brief   数据发送任务
 * @details 负责数据的网络传输和Flash存储管理
 *          包含发送队列处理、重试机制、存储管理等功能
 * @param   pvParameters: 任务参数
 * @return  无
 * 
 * 任务流程:
 * 1. 等待数据就绪事件
 * 2. 从数据队列获取数据
 * 3. 尝试网络发送数据
 * 4. 发送失败时存储到Flash
 * 5. 管理Flash存储空间
 * 
 * 发送策略:
 * - 优先级: 中高优先级保证数据及时发送
 * - 周期: 5秒检查一次发送队列
 * - 重试机制: 发送失败自动重试
 * - 存储管理: Flash环形缓冲区管理
 */
void DataSendTask(void *pvParameters);

/**
 * @brief   系统监控任务
 * @details 负责系统整体状态监控和异常处理
 *          包含资源监控、错误处理、状态指示等功能
 * @param   pvParameters: 任务参数
 * @return  无
 * 
 * 任务流程:
 * 1. 定期检查系统资源状态
 * 2. 处理错误队列中的错误信息
 * 3. 更新LED状态指示
 * 4. 监控各任务运行状态
 * 5. 执行系统健康管理
 * 
 * 监控策略:
 * - 优先级: 中优先级避免过度占用系统
 * - 周期: 3秒执行一次全面检查
 * - 错误处理: 分类处理不同类型错误
 * - 状态指示: LED和蜂鸣器状态反馈
 */
void SystemMonitorTask(void *pvParameters);

/* ============================== 调试接口 ============================== */

/**
 * @brief   打印任务状态信息
 * @details 此函数打印所有任务的详细状态信息
 *          用于调试和系统监控
 * @param   无
 * @return  无
 * 
 * 知识点: 任务状态调试
 * - 任务句柄: 唯一标识任务
 * - 状态信息: 反映任务当前状态
 * - 性能统计: 反映任务执行效率
 * - 堆栈使用: 反映内存消耗
 */
void Tasks_PrintStatus(void);

/**
 * @brief   打印任务堆栈使用情况
 * @details 此函数打印所有任务的堆栈使用情况
 *          用于内存优化和调试
 * @param   无
 * @return  无
 * 
 * 知识点: 堆栈监控
 * - 高水位标记: 显示最大堆栈使用量
 * - 剩余空间: 显示当前堆栈剩余
 * - 内存泄漏: 通过定期监控发现
 * - 优化建议: 根据使用情况调整堆栈大小
 */
void Tasks_PrintStackUsage(void);

/**
 * @brief   打印通信对象状态
 * @details 此函数打印所有通信对象的状态信息
 *          包括队列长度、信号量数量等
 * @param   无
 * @return  无
 * 
 * 知识点: 通信对象监控
 * - 队列状态: 消息数量、可用空间
 * - 信号量状态: 当前计数
 * - 事件组状态: 各事件位状态
 * - 阻塞任务: 等待资源的任务列表
 */
void Tasks_PrintCommunicationStatus(void);

#endif /* __TASKS_H */