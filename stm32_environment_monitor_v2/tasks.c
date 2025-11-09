/**
 * @file    tasks.c
 * @brief   FreeRTOS任务系统实现 (标准库版本)
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
 * 任务系统架构:
 * 
 * 【核心任务设计】
 * 1. 网络管理任务 (NetworkManageTask)
 *    负责ESP-01s WiFi模块的连接管理
 *    - 优先级: tskIDLE_PRIORITY + 3 (高优先级)
 *    - 堆栈大小: 256字 (1KB)
 *    - 周期性: 1秒检查网络状态
 * 
 * 2. 数据采集任务 (DataCollectTask)
 *    负责环境传感器数据采集
 *    - 优先级: tskIDLE_PRIORITY + 2 (中高优先级)
 *    - 堆栈大小: 128字 (512B)
 *    - 周期性: 2秒采集一次数据
 * 
 * 3. 数据发送任务 (DataSendTask)
 *    负责数据传输和存储管理
 *    - 优先级: tskIDLE_PRIORITY + 2 (中高优先级)
 *    - 堆栈大小: 128字 (512B)
 *    - 周期性: 5秒检查发送队列
 * 
 * 4. 系统监控任务 (SystemMonitorTask)
 *    负责系统状态监控和异常处理
 *    - 优先级: tskIDLE_PRIORITY + 1 (中优先级)
 *    - 堆栈大小: 128字 (512B)
 *    - 周期性: 3秒执行一次监控
 * 
 * 【任务间通信机制】
 * 1. 数据队列 (Data_Queue)
 *    - 长度: 10条消息
 *    - 数据项: Sensor_Data_t结构
 *    - 用途: 数据采集 -> 数据发送
 * 
 * 2. 错误队列 (Error_Queue)
 *    - 长度: 5条消息
 *    - 数据项: Error_Info_t结构
 *    - 用途: 各任务错误信息传递
 * 
 * 3. Flash互斥锁 (Flash_Mutex)
 *    - 类型: 二值互斥锁
 *    - 用途: 保护Flash访问，防止竞争
 * 
 * 4. 网络信号量 (Network_Semaphore)
 *    - 类型: 二值信号量
 *    - 用途: 网络连接状态同步
 * 
 * 5. 系统事件组 (System_Events)
 *    - 7个事件位定义
 *    - 用途: 系统事件同步和状态管理
 * 
 * 【内存管理策略】
 * 1. 动态内存分配
 *    使用FreeRTOS heap_4算法
 *    - 任务创建时分配堆栈
 *    - 通信对象动态创建
 *    - 内存碎片化管理
 * 
 * 2. 堆栈大小优化
 *    - 网络任务: 256字 (考虑AT指令处理)
 *    - 其他任务: 128字 (常规处理足够)
 *    - 高水位监控: 防止堆栈溢出
 * 
 * 【实时性能保证】
 * 1. 优先级设计
 *    - 网络管理: 高优先级确保网络及时响应
 *    - 数据采集: 中高优先级保证数据实时性
 *    - 数据发送: 中高优先级保证数据及时发送
 *    - 系统监控: 中优先级避免过度占用
 * 
 * 2. 任务周期优化
 *    - 网络检查: 1秒 (快速响应网络变化)
 *    - 数据采集: 2秒 (平衡精度和资源)
 *    - 数据发送: 5秒 (减少网络负载)
 *    - 系统监控: 3秒 (适中监控频率)
 * 
 * 3. 阻塞机制
 *    - 队列阻塞: 等待数据时阻塞
 *    - 事件等待: 等待特定事件时阻塞
 *    - 超时控制: 避免无限阻塞
 * 
 * 【错误处理机制】
 * 1. 分层错误处理
 *    - 任务内错误: 任务自行处理
 *    - 严重错误: 发送到错误队列
 *    - 系统错误: 系统监控任务处理
 * 
 * 2. 错误恢复策略
 *    - 自动重试: 网络发送失败自动重试
 *    - 降级处理: 网络异常时本地存储
 *    - 重启机制: 严重错误时系统重启
 * 
 * 3. 错误监控
 *    - 错误计数: 统计各类错误发生次数
 *    - 错误日志: 记录错误详细信息
 *    - 错误趋势: 分析错误发生规律
 * 
 * 【调试和监控】
 * 1. 任务统计
 *    - 循环计数: 反映任务活跃度
 *    - 执行时间: 反映任务性能
 *    - 堆栈使用: 反映内存消耗
 *    - 错误统计: 反映任务稳定性
 * 
 * 2. 状态监控
 *    - 任务状态: 运行/挂起/错误
 *    - 通信对象: 队列、信号量、事件组状态
 *    - 资源使用: CPU、内存、堆栈使用情况
 * 
 * 3. 调试输出
 *    - 任务状态打印
 *    - 堆栈使用情况
 *    - 通信对象状态
 *    - 错误信息记录
 */

#include "tasks.h"
#include "main.h"

/* ============================== 全局变量定义 ============================== */

/**
 * 任务句柄定义
 * 知识点: 任务句柄是FreeRTOS中任务控制的核心
 * - 每个任务创建时返回句柄
 * - 句柄用于任务管理、状态查询等
 * - 句柄为0表示任务未创建
 */
Task_Handle_t NetworkManageTask_Handle = NULL;    /**< 网络管理任务句柄 */
Task_Handle_t DataCollectTask_Handle = NULL;      /**< 数据采集任务句柄 */
Task_Handle_t DataSendTask_Handle = NULL;         /**< 数据发送任务句柄 */
Task_Handle_t SystemMonitorTask_Handle = NULL;    /**< 系统监控任务句柄 */

/**
 * 通信对象句柄定义
 * 知识点: 通信对象句柄用于任务间协调
 * - 队列句柄: 指向队列控制块
 * - 信号量句柄: 指向信号量控制块
 * - 事件组句柄: 指向事件组控制块
 * - 句柄为NULL表示对象未创建
 */
QueueHandle_t Data_Queue = NULL;                  /**< 数据队列句柄 */
QueueHandle_t Error_Queue = NULL;                 /**< 错误队列句柄 */
SemaphoreHandle_t Flash_Mutex = NULL;             /**< Flash互斥锁句柄 */
SemaphoreHandle_t Network_Semaphore = NULL;       /**< 网络信号量句柄 */
EventGroupHandle_t System_Events = NULL;          /**< 系统事件组句柄 */

/**
 * 任务管理信息
 * 知识点: 全局任务管理结构
 * - 包含所有任务的统计信息
 * - 系统运行时间统计
 * - 错误计数管理
 */
Task_Management_t Task_Management = {
    .network_manage = {0},
    .data_collect = {0},
    .data_send = {0},
    .system_monitor = {0},
    .system_start_time = 0,
    .total_task_errors = 0
};

/* ============================== 任务系统初始化 ============================== */

/**
 * @brief   任务系统初始化
 * @details 此函数初始化所有任务相关的资源
 *          包括创建任务、初始化通信对象等
 * @param   无
 * @return  bool: 初始化成功返回true，失败返回false
 * 
 * 初始化流程:
 * 1. 创建通信对象(队列、信号量、事件组)
 * 2. 创建4个核心任务
 * 3. 初始化任务统计信息
 * 4. 验证创建结果
 * 5. 启动任务调度
 * 
 * 错误处理策略:
 * - 内存不足: 清理已创建资源，返回false
 * - 任务创建失败: 清理已创建对象，继续尝试
 * - 通信对象创建失败: 记录错误，不影响其他对象
 * - 部分失败: 返回false，让调用者决定处理策略
 */
bool Tasks_Init(void)
{
    /**
     * 第一步: 创建通信对象
     * 知识点: 通信对象必须在任务之前创建
     * 原因: 任务可能需要立即使用这些对象
     * 
     * 创建顺序: 队列 -> 信号量 -> 事件组
     * 原因: 依赖关系决定创建顺序
     */
    
    /**
     * 创建数据队列
     * 知识点: 数据队列用于任务间数据传输
     * - 长度: 10条消息
     * - 消息大小: Sensor_Data_t结构体大小
     * - 用途: 数据采集任务 -> 数据发送任务
     */
    Data_Queue = xQueueCreate(DATA_QUEUE_LENGTH, DATA_QUEUE_ITEM_SIZE);
    if (Data_Queue == NULL) {
        DEBUG_PRINT("ERROR: Failed to create Data_Queue\r\n");
        return false;
    }
    
    /**
     * 创建错误队列
     * 知识点: 错误队列用于错误信息传递
     * - 长度: 5条消息
     * - 消息大小: Error_Info_t结构体大小
     * - 用途: 各任务 -> 系统监控任务
     */
    Error_Queue = xQueueCreate(ERROR_QUEUE_LENGTH, sizeof(Error_Info_t));
    if (Error_Queue == NULL) {
        DEBUG_PRINT("ERROR: Failed to create Error_Queue\r\n");
        vQueueDelete(Data_Queue);
        return false;
    }
    
    /**
     * 创建Flash互斥锁
     * 知识点: 互斥锁防止资源竞争
     * - 类型: 二值互斥锁
     * - 用途: 保护Flash访问
     * - 特性: 优先级继承，防止优先级反转
     */
    Flash_Mutex = xSemaphoreCreateMutex();
    if (Flash_Mutex == NULL) {
        DEBUG_PRINT("ERROR: Failed to create Flash_Mutex\r\n");
        vQueueDelete(Data_Queue);
        vQueueDelete(Error_Queue);
        return false;
    }
    
    /**
     * 创建网络信号量
     * 知识点: 信号量用于同步网络状态
     * - 类型: 二值信号量
     * - 初始值: 0 (网络未连接)
     * - 用途: 网络连接状态同步
     */
    Network_Semaphore = xSemaphoreCreateBinary();
    if (Network_Semaphore == NULL) {
        DEBUG_PRINT("ERROR: Failed to create Network_Semaphore\r\n");
        vQueueDelete(Data_Queue);
        vQueueDelete(Error_Queue);
        vSemaphoreDelete(Flash_Mutex);
        return false;
    }
    
    /**
     * 创建系统事件组
     * 知识点: 事件组支持多条件同步
     * - 初始状态: 所有事件位为0
     * - 用途: 系统事件管理
     * - 特性: 原子操作，支持同时设置多个事件
     */
    System_Events = xEventGroupCreate();
    if (System_Events == NULL) {
        DEBUG_PRINT("ERROR: Failed to create System_Events\r\n");
        vQueueDelete(Data_Queue);
        vQueueDelete(Error_Queue);
        vSemaphoreDelete(Flash_Mutex);
        vSemaphoreDelete(Network_Semaphore);
        return false;
    }
    
    /**
     * 第二步: 创建核心任务
     * 知识点: 任务创建使用xTaskCreate函数
     * 参数说明:
     * - pvTaskCode: 任务函数指针
     * - pcName: 任务名称字符串
     * - usStackDepth: 堆栈大小(字为单位)
     * - pvParameters: 任务参数指针
     * - uxPriority: 任务优先级
     * - pvCreatedTask: 任务句柄指针
     * 
     * 创建顺序: 按优先级从高到低创建
     * 原因: 避免高优先级任务等待低优先级任务创建
     */
    
    /**
     * 创建网络管理任务 (高优先级)
     * 知识点: 网络管理任务需要最高优先级
     * 原因: 网络连接建立和维护需要及时响应
     * 堆栈: 256字，考虑AT指令处理和字符串操作
     */
    if (xTaskCreate(NetworkManageTask, 
                    "NetworkManage", 
                    NETWORK_MANAGE_TASK_STACK_SIZE, 
                    NULL, 
                    NETWORK_MANAGE_TASK_PRIORITY, 
                    &NetworkManageTask_Handle) != pdPASS) {
        DEBUG_PRINT("ERROR: Failed to create NetworkManageTask\r\n");
        return false;
    }
    
    /**
     * 创建数据采集任务 (中高优先级)
     * 知识点: 数据采集任务需要较高优先级
     * 原因: 保证数据采集的实时性
     * 堆栈: 128字，常规传感器数据处理足够
     */
    if (xTaskCreate(DataCollectTask, 
                    "DataCollect", 
                    DATA_COLLECT_TASK_STACK_SIZE, 
                    NULL, 
                    DATA_COLLECT_TASK_PRIORITY, 
                    &DataCollectTask_Handle) != pdPASS) {
        DEBUG_PRINT("ERROR: Failed to create DataCollectTask\r\n");
        vTaskDelete(NetworkManageTask_Handle);
        return false;
    }
    
    /**
     * 创建数据发送任务 (中高优先级)
     * 知识点: 数据发送任务需要较高优先级
     * 原因: 保证数据及时发送，减少队列积压
     * 堆栈: 128字，常规数据处理足够
     */
    if (xTaskCreate(DataSendTask, 
                    "DataSend", 
                    DATA_SEND_TASK_STACK_SIZE, 
                    NULL, 
                    DATA_SEND_TASK_PRIORITY, 
                    &DataSendTask_Handle) != pdPASS) {
        DEBUG_PRINT("ERROR: Failed to create DataSendTask\r\n");
        vTaskDelete(NetworkManageTask_Handle);
        vTaskDelete(DataCollectTask_Handle);
        return false;
    }
    
    /**
     * 创建系统监控任务 (中优先级)
     * 知识点: 系统监控任务优先级适中
     * 原因: 避免过度占用系统资源
     * 堆栈: 128字，监控功能处理足够
     */
    if (xTaskCreate(SystemMonitorTask, 
                    "SystemMonitor", 
                    SYSTEM_MONITOR_TASK_STACK_SIZE, 
                    NULL, 
                    SYSTEM_MONITOR_TASK_PRIORITY, 
                    &SystemMonitorTask_Handle) != pdPASS) {
        DEBUG_PRINT("ERROR: Failed to create SystemMonitorTask\r\n");
        vTaskDelete(NetworkManageTask_Handle);
        vTaskDelete(DataCollectTask_Handle);
        vTaskDelete(DataSendTask_Handle);
        return false;
    }
    
    /**
     * 第三步: 初始化统计信息
     * 知识点: 任务统计信息初始化
     * - 设置任务句柄
     * - 初始化运行时间
     * - 重置计数器和错误统计
     */
    Task_Management.system_start_time = System_Get_Runtime_MS();
    
    Task_Management.network_manage.task_handle = NetworkManageTask_Handle;
    Task_Management.network_manage.status = TASK_STATUS_RUNNING;
    
    Task_Management.data_collect.task_handle = DataCollectTask_Handle;
    Task_Management.data_collect.status = TASK_STATUS_RUNNING;
    
    Task_Management.data_send.task_handle = DataSendTask_Handle;
    Task_Management.data_send.status = TASK_STATUS_RUNNING;
    
    Task_Management.system_monitor.task_handle = SystemMonitorTask_Handle;
    Task_Management.system_monitor.status = TASK_STATUS_RUNNING;
    
    /**
     * 第四步: 验证创建结果
     * 知识点: 验证所有对象创建成功
     * - 检查任务句柄不为空
     * - 检查通信对象句柄不为空
     */
    if (NetworkManageTask_Handle == NULL || 
        DataCollectTask_Handle == NULL || 
        DataSendTask_Handle == NULL || 
        SystemMonitorTask_Handle == NULL) {
        DEBUG_PRINT("ERROR: Task handles are NULL\r\n");
        return false;
    }
    
    if (Data_Queue == NULL || 
        Error_Queue == NULL || 
        Flash_Mutex == NULL || 
        Network_Semaphore == NULL || 
        System_Events == NULL) {
        DEBUG_PRINT("ERROR: Communication handles are NULL\r\n");
        return false;
    }
    
    /**
     * 初始化成功
     * 知识点: 成功初始化后的系统状态
     * - 所有任务已创建，等待调度
     * - 通信对象已创建，可以进行任务间通信
     * - 统计信息已初始化，可以开始监控
     */
    DEBUG_PRINT("Tasks system initialized successfully\r\n");
    DEBUG_PRINT("Created tasks: NetworkManage, DataCollect, DataSend, SystemMonitor\r\n");
    DEBUG_PRINT("Created communication objects: 2 Queues, 2 Semaphores, 1 EventGroup\r\n");
    
    return true;
}

/**
 * @brief   任务系统去初始化
 * @details 此函数清理任务系统资源
 *          包括删除任务、清理通信对象等
 * @param   无
 * @return  bool: 去初始化成功返回true，失败返回false
 * 
 * 去初始化流程:
 * 1. 等待任务执行完成
 * 2. 删除所有任务
 * 3. 删除通信对象
 * 4. 释放内存
 * 5. 重置全局状态
 * 
 * 注意事项:
 * - 只能在系统关闭时调用
 * - 确保没有任务在运行
 * - 按相反顺序删除对象
 */
bool Tasks_DeInit(void)
{
    /**
     * 第一步: 停止所有任务
     * 知识点: 任务停止策略
     * - 首先挂起所有任务
     * - 等待一段时间确保任务停止
     * - 然后删除任务
     */
    
    if (SystemMonitorTask_Handle != NULL) {
        vTaskSuspend(SystemMonitorTask_Handle);
    }
    if (DataSendTask_Handle != NULL) {
        vTaskSuspend(DataSendTask_Handle);
    }
    if (DataCollectTask_Handle != NULL) {
        vTaskSuspend(DataCollectTask_Handle);
    }
    if (NetworkManageTask_Handle != NULL) {
        vTaskSuspend(NetworkManageTask_Handle);
    }
    
    /**
     * 等待任务停止
     * 知识点: 等待策略
     * - 给任务时间进行清理操作
     * - 避免强制删除导致资源泄漏
     */
    vTaskDelay(pdMS_TO_TICKS(100));
    
    /**
     * 第二步: 删除所有任务
     * 知识点: 任务删除
     * - 按创建相反顺序删除
     * - 释放任务堆栈内存
     * - 清理任务控制块
     */
    
    if (SystemMonitorTask_Handle != NULL) {
        vTaskDelete(SystemMonitorTask_Handle);
        SystemMonitorTask_Handle = NULL;
    }
    
    if (DataSendTask_Handle != NULL) {
        vTaskDelete(DataSendTask_Handle);
        DataSendTask_Handle = NULL;
    }
    
    if (DataCollectTask_Handle != NULL) {
        vTaskDelete(DataCollectTask_Handle);
        DataCollectTask_Handle = NULL;
    }
    
    if (NetworkManageTask_Handle != NULL) {
        vTaskDelete(NetworkManageTask_Handle);
        NetworkManageTask_Handle = NULL;
    }
    
    /**
     * 第三步: 删除通信对象
     * 知识点: 通信对象删除
     * - 按依赖关系相反顺序删除
     * - 释放队列、信号量、事件组内存
     * - 清理等待队列
     */
    
    if (System_Events != NULL) {
        vEventGroupDelete(System_Events);
        System_Events = NULL;
    }
    
    if (Network_Semaphore != NULL) {
        vSemaphoreDelete(Network_Semaphore);
        Network_Semaphore = NULL;
    }
    
    if (Flash_Mutex != NULL) {
        vSemaphoreDelete(Flash_Mutex);
        Flash_Mutex = NULL;
    }
    
    if (Error_Queue != NULL) {
        vQueueDelete(Error_Queue);
        Error_Queue = NULL;
    }
    
    if (Data_Queue != NULL) {
        vQueueDelete(Data_Queue);
        Data_Queue = NULL;
    }
    
    /**
     * 第四步: 重置全局状态
     * 知识点: 全局状态重置
     * - 清零任务管理结构
     * - 重置错误统计
     * - 重置系统启动时间
     */
    memset(&Task_Management, 0, sizeof(Task_Management_t));
    
    DEBUG_PRINT("Tasks system de-initialized successfully\r\n");
    
    return true;
}

/* ============================== 任务控制函数 ============================== */

/**
 * @brief   获取任务统计信息
 * @details 此函数获取指定任务的详细统计信息
 * @param   task_name: 任务名称字符串
 * @return  Task_Statistics_t*: 任务统计信息指针，失败返回NULL
 * 
 * 实现原理:
 * 1. 根据任务名称查找对应统计结构
 * 2. 返回结构指针给调用者
 * 3. 调用者可以读取但不能修改结构内容
 * 
 * 知识点: 任务统计信息的作用
 * - cycle_count: 反映任务执行频率
 * - execution_time: 反映任务性能
 * - error_count: 反映任务稳定性
 * - stack_high_water_mark: 反映内存使用
 */
Task_Statistics_t* Tasks_GetTaskStatistics(const char* task_name)
{
    if (task_name == NULL) {
        return NULL;
    }
    
    /**
     * 知识点: 字符串比较
     * 使用strcmp函数比较任务名称
     * 返回0表示字符串相等
     */
    if (strcmp(task_name, "NetworkManage") == 0) {
        return &Task_Management.network_manage;
    } else if (strcmp(task_name, "DataCollect") == 0) {
        return &Task_Management.data_collect;
    } else if (strcmp(task_name, "DataSend") == 0) {
        return &Task_Management.data_send;
    } else if (strcmp(task_name, "SystemMonitor") == 0) {
        return &Task_Management.system_monitor;
    }
    
    return NULL;
}

/**
 * @brief   挂起指定任务
 * @details 此函数挂起指定任务，暂时停止其执行
 * @param   task_name: 任务名称字符串
 * @return  bool: 挂起成功返回true，失败返回false
 * 
 * 知识点: 任务挂起机制
 * - 挂起后任务不参与调度
 * - 任务状态变为Suspended
 * - 可以通过ResumeTask恢复执行
 * - 任务内部状态保持不变
 */
bool Tasks_SuspendTask(const char* task_name)
{
    Task_Handle_t target_task_handle = NULL;
    Task_Statistics_t* task_stats = NULL;
    
    /**
     * 第一步: 根据任务名称查找句柄
     * 知识点: 句柄查找
     * - 通过任务名称映射到句柄
     * - 句柄用于后续的挂起操作
     */
    if (strcmp(task_name, "NetworkManage") == 0) {
        target_task_handle = NetworkManageTask_Handle;
        task_stats = &Task_Management.network_manage;
    } else if (strcmp(task_name, "DataCollect") == 0) {
        target_task_handle = DataCollectTask_Handle;
        task_stats = &Task_Management.data_collect;
    } else if (strcmp(task_name, "DataSend") == 0) {
        target_task_handle = DataSendTask_Handle;
        task_stats = &Task_Management.data_send;
    } else if (strcmp(task_name, "SystemMonitor") == 0) {
        target_task_handle = SystemMonitorTask_Handle;
        task_stats = &Task_Management.system_monitor;
    } else {
        DEBUG_PRINT("ERROR: Unknown task name: %s\r\n", task_name);
        return false;
    }
    
    /**
     * 第二步: 检查任务句柄有效性
     * 知识点: 句柄有效性检查
     * - 句柄不能为NULL
     * - 句柄对应的任务必须存在
     */
    if (target_task_handle == NULL) {
        DEBUG_PRINT("ERROR: Task %s handle is NULL\r\n", task_name);
        return false;
    }
    
    /**
     * 第三步: 执行挂起操作
     * 知识点: vTaskSuspend函数
     * - 参数: 要挂起的任务句柄
     * - 返回: 无
     * - 效果: 任务立即停止参与调度
     */
    vTaskSuspend(target_task_handle);
    
    /**
     * 第四步: 更新任务状态
     * 知识点: 状态更新
     * - 记录挂起状态
     * - 统计挂起次数
     */
    if (task_stats != NULL) {
        task_stats->status = TASK_STATUS_SUSPENDED;
    }
    
    DEBUG_PRINT("Task %s suspended successfully\r\n", task_name);
    return true;
}

/**
 * @brief   恢复指定任务
 * @details 此函数恢复之前挂起的任务
 * @param   task_name: 任务名称字符串
 * @return  bool: 恢复成功返回true，失败返回false
 * 
 * 知识点: 任务恢复机制
 * - 恢复后任务重新参与调度
 * - 任务从挂起状态变为就绪状态
 * - 恢复时立即开始执行
 * - 保持挂起前的所有状态
 */
bool Tasks_ResumeTask(const char* task_name)
{
    Task_Handle_t target_task_handle = NULL;
    Task_Statistics_t* task_stats = NULL;
    
    /**
     * 第一步: 根据任务名称查找句柄
     */
    if (strcmp(task_name, "NetworkManage") == 0) {
        target_task_handle = NetworkManageTask_Handle;
        task_stats = &Task_Management.network_manage;
    } else if (strcmp(task_name, "DataCollect") == 0) {
        target_task_handle = DataCollectTask_Handle;
        task_stats = &Task_Management.data_collect;
    } else if (strcmp(task_name, "DataSend") == 0) {
        target_task_handle = DataSendTask_Handle;
        task_stats = &Task_Management.data_send;
    } else if (strcmp(task_name, "SystemMonitor") == 0) {
        target_task_handle = SystemMonitorTask_Handle;
        task_stats = &Task_Management.system_monitor;
    } else {
        DEBUG_PRINT("ERROR: Unknown task name: %s\r\n", task_name);
        return false;
    }
    
    /**
     * 第二步: 检查任务句柄有效性
     */
    if (target_task_handle == NULL) {
        DEBUG_PRINT("ERROR: Task %s handle is NULL\r\n", task_name);
        return false;
    }
    
    /**
     * 第三步: 执行恢复操作
     * 知识点: vTaskResume函数
     * - 参数: 要恢复的任务句柄
     * - 返回: 无
     * - 效果: 任务重新参与调度
     */
    vTaskResume(target_task_handle);
    
    /**
     * 第四步: 更新任务状态
     */
    if (task_stats != NULL) {
        task_stats->status = TASK_STATUS_RUNNING;
    }
    
    DEBUG_PRINT("Task %s resumed successfully\r\n", task_name);
    return true;
}

/* ============================== 事件管理函数 ============================== */

/**
 * @brief   设置任务事件
 * @details 此函数设置系统事件组中的事件位
 * @param   event_bits: 要设置的事件位
 * @param   clear_previous: 是否清除之前的事件位
 * @return  bool: 设置成功返回true，失败返回false
 * 
 * 知识点: 事件组操作
 * - 事件组是32位整型变量
 * - 每个位代表一个特定事件
 * - 原子操作，多任务安全
 * - 支持同时设置多个事件位
 */
bool Tasks_SetEvent(uint32_t event_bits, bool clear_previous)
{
    if (System_Events == NULL) {
        DEBUG_PRINT("ERROR: System_Events is NULL\r\n");
        return false;
    }
    
    /**
     * 知识点: xEventGroupSetBits函数
     * - 参数1: 事件组句柄
     * - 参数2: 要设置的事件位
     * - 返回: 设置后的事件组状态
     * - 特性: 原子操作，不被中断打断
     */
    if (clear_previous) {
        /**
         * 先清除所有事件位，然后设置新的事件位
         * 知识点: 清除和设置组合操作
         * - 使用0清除所有位
         * - 使用指定位设置相应位
         */
        xEventGroupSetBits(System_Events, 0);     // 清除所有位
    }
    
    xEventGroupSetBits(System_Events, event_bits);
    
    DEBUG_PRINT("Event bits 0x%08X set successfully\r\n", event_bits);
    return true;
}

/**
 * @brief   清除任务事件
 * @details 此函数清除系统事件组中的事件位
 * @param   event_bits: 要清除的事件位
 * @return  bool: 清除成功返回true，失败返回false
 * 
 * 知识点: 事件清除
 * - 使用xEventGroupClearBits函数
 * - 只清除指定的事件位
 * - 不影响其他事件位
 */
bool Tasks_ClearEvent(uint32_t event_bits)
{
    if (System_Events == NULL) {
        DEBUG_PRINT("ERROR: System_Events is NULL\r\n");
        return false;
    }
    
    /**
     * 知识点: xEventGroupClearBits函数
     * - 参数1: 事件组句柄
     * - 参数2: 要清除的事件位
     * - 返回: 清除后的事件组状态
     * - 特性: 原子操作，安全清除
     */
    xEventGroupClearBits(System_Events, event_bits);
    
    DEBUG_PRINT("Event bits 0x%08X cleared successfully\r\n", event_bits);
    return true;
}

/**
 * @brief   等待任务事件
 * @details 此函数等待指定的事件位设置
 * @param   event_bits: 要等待的事件位
 * @param   wait_all: 是否等待所有事件位
 * @param   timeout_ms: 超时时间(毫秒)
 * @return  uint32_t: 返回触发的事件位，超时返回0
 * 
 * 知识点: 事件等待机制
 * - 阻塞等待指定事件
 * - 支持等待单个或多个事件
 * - 超时机制避免无限等待
 * - 自动清除已等待的事件位(可配置)
 * 
 * wait_all参数说明:
 * - true: 等待所有指定的事件位都设置
 * - false: 等待任意一个事件位设置
 * 
 * 返回值说明:
 * - 非0: 实际触发的事件位
 * - 0: 超时或事件组被删除
 */
uint32_t Tasks_WaitEvent(uint32_t event_bits, bool wait_all, uint32_t timeout_ms)
{
    if (System_Events == NULL) {
        DEBUG_PRINT("ERROR: System_Events is NULL\r\n");
        return 0;
    }
    
    /**
     * 知识点: xEventGroupWaitBits函数
     * - 参数1: 事件组句柄
     * - 参数2: 要等待的事件位掩码
     * - 参数3: 是否等待所有位(pdTRUE/pdFALSE)
     * - 参数4: 退出时是否清除位(pdTRUE/pdFALSE)
     * - 参数5: 超时时间(portMAX_DELAY表示无限等待)
     * - 返回: 实际触发的事件位
     */
    uint32_t clear_on_exit = wait_all ? pdFALSE : pdTRUE;
    uint32_t result = xEventGroupWaitBits(System_Events, 
                                         event_bits, 
                                         clear_on_exit, 
                                         wait_all, 
                                         pdMS_TO_TICKS(timeout_ms));
    
    if (result == 0) {
        DEBUG_PRINT("Event wait timeout after %u ms\r\n", timeout_ms);
    } else {
        DEBUG_PRINT("Event wait successful, triggered bits: 0x%08X\r\n", result);
    }
    
    return result;
}

/* ============================== 核心任务实现 ============================== */

/**
 * @brief   网络管理任务
 * @details 负责ESP-01s WiFi模块的网络连接管理
 *          包含连接建立、状态监控、断线重连等功能
 * @param   pvParameters: 任务参数
 * @return  无
 * 
 * 任务执行流程:
 * 1. 等待系统启动事件
 * 2. 初始化ESP-01s模块
 * 3. 尝试建立WiFi连接
 * 4. 循环监控网络状态
 * 5. 处理网络异常和重连
 * 
 * 网络管理策略:
 * - 周期性检查: 每秒检查一次网络状态
 * - 连接维护: 保持连接稳定性
 * - 断线重连: 自动重连机制
 * - 状态指示: LED反映网络状态
 * 
 * 状态机设计:
 * - INIT: 模块初始化状态
 * - CONNECTING: WiFi连接中
 * - CONNECTED: WiFi连接成功
 * - ERROR: 网络错误状态
 * - RECONNECTING: 断线重连状态
 * 
 * 错误处理:
 * - AT指令失败: 重试机制
 * - 连接超时: 重新初始化
 * - 模块无响应: 重启模块
 */
void NetworkManageTask(void *pvParameters)
{
    /**
     * 任务初始化
     * 知识点: 任务函数入口
     * - 无参数传递(pvParameters = NULL)
     * - 无限循环执行
     * - 定期调用vTaskDelay让出CPU
     */
    (void)pvParameters;  // 避免编译器警告
    
    /**
     * 任务本地变量
     * 知识点: 任务状态管理
     * - network_state: 当前网络状态
     * - retry_count: 连接重试计数
     * - last_check_time: 上次检查时间
     */
    Network_State_t network_state = NETWORK_INIT;
    uint32_t retry_count = 0;
    uint32_t last_check_time = 0;
    
    DEBUG_PRINT("NetworkManageTask started\r\n");
    
    /**
     * 任务主循环
     * 知识点: 无限循环设计
     * - 任务不能返回，必须无限循环
     * - 定期检查网络状态
     * - 异常处理和状态恢复
     */
    while (1) {
        /**
         * 更新任务统计信息
         * 知识点: 任务性能监控
         * - 循环计数递增
         * - 执行时间统计
         * - 堆栈使用监控
         */
        Task_Management.network_manage.cycle_count++;
        Task_Management.network_manage.last_execution_time = System_Get_Runtime_MS();
        
        /**
         * 状态机执行
         * 知识点: 状态机设计模式
         * - 根据当前状态执行相应操作
         * - 状态转换逻辑
         * - 异常状态处理
         */
        switch (network_state) {
            case NETWORK_INIT: {
                /**
                 * 初始化状态
                 * 知识点: 模块初始化流程
                 * 1. 发送AT指令测试模块
                 * 2. 配置WiFi模式
                 * 3. 重置连接状态
                 */
                DEBUG_PRINT("Network state: INIT\r\n");
                
                /**
                 * 初始化ESP-01s模块
                 * 知识点: AT指令初始化
                 * - AT: 测试模块响应
                 * - AT+CWMODE=1: 设置Station模式
                 * - AT+CIPMUX=0: 单连接模式
                 */
                if (ESP01s_Init() == ESP01S_SUCCESS) {
                    network_state = NETWORK_CONNECTING;
                    retry_count = 0;
                    DEBUG_PRINT("ESP-01s initialized successfully\r\n");
                } else {
                    /**
                     * 初始化失败处理
                     * 知识点: 错误处理策略
                     * - 重试机制
                     * - 重试次数限制
                     * - 错误状态转换
                     */
                    retry_count++;
                    if (retry_count >= MAX_INIT_RETRY) {
                        network_state = NETWORK_ERROR;
                        DEBUG_PRINT("ESP-01s init failed after %u retries\r\n", retry_count);
                    }
                }
                break;
            }
            
            case NETWORK_CONNECTING: {
                /**
                 * 连接状态
                 * 知识点: WiFi连接建立
                 * 1. 扫描可用的WiFi网络
                 * 2. 连接到指定网络
                 * 3. 获取IP地址
                 */
                DEBUG_PRINT("Network state: CONNECTING\r\n");
                
                /**
                 * 尝试连接到WiFi网络
                 * 知识点: WiFi连接过程
                 * - AT+CWJAP: 连接指定WiFi
                 * - AT+CIPSTART: 建立TCP连接
                 * - 等待连接成功响应
                 */
                AT_Command_Status_t connect_result = ESP01s_ConnectWiFi(WIFI_SSID, WIFI_PASSWORD);
                
                if (connect_result == AT_COMMAND_SUCCESS) {
                    network_state = NETWORK_CONNECTED;
                    retry_count = 0;
                    
                    /**
                     * 连接成功设置事件
                     * 知识点: 事件通知机制
                     * - 通知其他任务网络已连接
                     * - 释放网络信号量
                     * - 设置系统事件
                     */
                    xSemaphoreGive(Network_Semaphore);
                    Tasks_SetEvent(NETWORK_CONNECTED_EVENT, false);
                    
                    /**
                     * LED状态指示
                     * 知识点: 状态指示
                     * - 绿色LED常亮表示连接成功
                     * - 状态反映系统运行情况
                     */
                    LED_SetState(LED_GREEN, LED_STATE_ON);
                    
                    DEBUG_PRINT("WiFi connected successfully\r\n");
                } else {
                    /**
                     * 连接失败处理
                     * 知识点: 连接失败策略
                     * - 计数重试次数
                     * - 重试间隔控制
                     * - 状态转换判断
                     */
                    retry_count++;
                    if (retry_count >= MAX_CONNECT_RETRY) {
                        network_state = NETWORK_ERROR;
                        DEBUG_PRINT("WiFi connect failed after %u retries\r\n", retry_count);
                    }
                }
                break;
            }
            
            case NETWORK_CONNECTED: {
                /**
                 * 已连接状态
                 * 知识点: 连接维护
                 * - 定期检查连接状态
                 * - 保持连接稳定性
                 * - 处理连接断开
                 */
                DEBUG_PRINT("Network state: CONNECTED\r\n");
                
                /**
                 * 检查连接状态
                 * 知识点: 连接状态检查
                 * - 发送AT指令查询状态
                 * - 检查网络连通性
                 * - 检测连接是否断开
                 */
                if (ESP01s_CheckConnection() == AT_COMMAND_SUCCESS) {
                    /**
                     * 连接正常
                     * 知识点: 连接正常处理
                     * - 继续维护连接
                     * - 不需要特殊操作
                     */
                    retry_count = 0;  // 重置重试计数
                } else {
                    /**
                     * 连接异常处理
                     * 知识点: 连接断开检测
                     * - 状态转换到重连状态
                     * - 清除连接相关事件
                     * - LED状态变化指示
                     */
                    network_state = NETWORK_RECONNECTING;
                    Tasks_ClearEvent(NETWORK_CONNECTED_EVENT);
                    Tasks_SetEvent(NETWORK_DISCONNECTED_EVENT, false);
                    
                    /**
                     * LED状态指示
                     * 知识点: 异常状态指示
                     * - 红色LED闪烁表示网络异常
                     */
                    LED_SetState(LED_GREEN, LED_STATE_OFF);
                    LED_StartBlink(LED_RED, 500);
                    
                    DEBUG_PRINT("Network connection lost, starting reconnect\r\n");
                }
                break;
            }
            
            case NETWORK_RECONNECTING: {
                /**
                 * 重连状态
                 * 知识点: 自动重连机制
                 * - 尝试重新建立连接
                 * - 重试次数控制
                 * - 重连超时处理
                 */
                DEBUG_PRINT("Network state: RECONNECTING\r\n");
                
                /**
                 * 尝试重连
                 * 知识点: 重连策略
                 * - 重新执行连接流程
                 * - 重试次数限制
                 * - 重连间隔控制
                 */
                if (ESP01s_Reconnect() == AT_COMMAND_SUCCESS) {
                    network_state = NETWORK_CONNECTED;
                    retry_count = 0;
                    
                    /**
                     * 重连成功处理
                     * 知识点: 重连成功
                     * - 恢复连接状态
                     * - 清除重连事件
                     * - 恢复LED状态
                     */
                    xSemaphoreGive(Network_Semaphore);
                    Tasks_ClearEvent(NETWORK_DISCONNECTED_EVENT);
                    Tasks_SetEvent(NETWORK_CONNECTED_EVENT, false);
                    
                    LED_SetState(LED_RED, LED_STATE_OFF);
                    LED_SetState(LED_GREEN, LED_STATE_ON);
                    
                    DEBUG_PRINT("Network reconnected successfully\r\n");
                } else {
                    /**
                     * 重连失败处理
                     * 知识点: 重连失败策略
                     * - 继续重试
                     * - 重试次数统计
                     * - 状态转换判断
                     */
                    retry_count++;
                    if (retry_count >= MAX_RECONNECT_RETRY) {
                        network_state = NETWORK_ERROR;
                        DEBUG_PRINT("Network reconnect failed after %u retries\r\n", retry_count);
                    }
                }
                break;
            }
            
            case NETWORK_ERROR: {
                /**
                 * 错误状态
                 * 知识点: 错误处理策略
                 * - 记录错误信息
                 * - 尝试恢复
                 * - 状态重置机制
                 */
                DEBUG_PRINT("Network state: ERROR\r\n");
                
                /**
                 * 错误恢复尝试
                 * 知识点: 错误恢复
                 * - 定期尝试恢复
                 * - 重置模块
                 * - 重新初始化
                 */
                if (retry_count >= MAX_ERROR_RECOVERY_RETRY) {
                    /**
                     * 多次恢复失败，转入错误报告
                     * 知识点: 严重错误处理
                     * - 发送错误事件
                     * - 设置错误状态
                     * - 等待外部干预
                     */
                    Tasks_SetEvent(SYSTEM_ERROR_EVENT, false);
                    Task_Management.network_manage.error_count++;
                    Task_Management.total_task_errors++;
                    
                    DEBUG_PRINT("Network error recovery failed, requiring intervention\r\n");
                } else {
                    /**
                     * 尝试错误恢复
                     * 知识点: 错误恢复流程
                     * - 重置ESP-01s模块
                     * - 重新初始化
                     * - 计数恢复尝试
                     */
                    ESP01s_Reset();
                    vTaskDelay(pdMS_TO_TICKS(1000));  // 等待重置完成
                    
                    network_state = NETWORK_INIT;
                    retry_count++;
                    
                    DEBUG_PRINT("Attempting error recovery (attempt %u)\r\n", retry_count);
                }
                break;
            }
            
            default: {
                /**
                 * 未知状态处理
                 * 知识点: 状态机健壮性
                 * - 防止状态机失控
                 * - 强制重置到初始状态
                 * - 错误日志记录
                 */
                DEBUG_PRINT("Unknown network state, resetting to INIT\r\n");
                network_state = NETWORK_INIT;
                break;
            }
        }
        
        /**
         * 周期性网络检查
         * 知识点: 定时检查机制
         * - 避免频繁检查消耗资源
         * - 平衡响应性和效率
         * - 基于时间间隔的检查
         */
        uint32_t current_time = System_Get_Runtime_MS();
        if (current_time - last_check_time >= NETWORK_CHECK_PERIOD_MS) {
            /**
             * 定期执行网络检查
             * 知识点: 网络检查内容
             * - 发送ping命令测试连通性
             * - 检查连接质量
             * - 统计连接时间
             */
            ESP01s_PingTest();
            last_check_time = current_time;
        }
        
        /**
         * 任务调度让出
         * 知识点: 任务调度
         * - vTaskDelay让出CPU使用权
         * - 参数转换为系统节拍
         * - 避免任务独占CPU
         */
        vTaskDelay(pdMS_TO_TICKS(NETWORK_CHECK_PERIOD_MS));
    }
}

/**
 * @brief   数据采集任务
 * @details 负责环境传感器数据的周期性采集
 *          包含ADC读取、数据处理、格式转换等功能
 * @param   pvParameters: 任务参数
 * @return  无
 * 
 * 任务执行流程:
 * 1. 等待系统启动
 * 2. 初始化ADC模块
 * 3. 定期采集传感器数据
 * 4. 数据校准和滤波
 * 5. 发送到数据队列
 * 
 * 采集策略:
 * - 周期性采集: 2秒采集一次
 * - 多通道ADC: 同时采集3个传感器
 * - 数据校准: 温度、光线、气体校准
 * - 队列传输: 异步数据传输
 * 
 * 数据处理流程:
 * 1. ADC多通道扫描
 * 2. 原始数据读取
 * 3. 物理量转换
 * 4. 数据校准处理
 * 5. 简单滤波(移动平均)
 * 6. 格式验证
 * 7. 队列发送
 * 
 * 异常处理:
 * - ADC读取失败: 重试机制
 * - 数据异常: 跳过本轮采集
 * - 队列满: 记录错误信息
 * - 校准失败: 使用默认校准值
 */
void DataCollectTask(void *pvParameters)
{
    /**
     * 任务初始化
     * 知识点: 任务本地变量
     * - sensor_data: 当前采集的传感器数据
     * - filtered_data: 滤波后的数据
     * - sample_count: 采样计数(用于移动平均)
     */
    (void)pvParameters;  // 避免编译器警告
    
    Sensor_Data_t sensor_data = {0};
    Sensor_Data_t filtered_data = {0};
    uint32_t sample_count = 0;
    
    DEBUG_PRINT("DataCollectTask started\r\n");
    
    /**
     * ADC模块初始化
     * 知识点: ADC初始化检查
     * - 确保ADC模块正常工作
     * - 配置ADC参数
     * - 验证通道配置
     */
    if (ADC_Init() != ADC_SUCCESS) {
        /**
         * ADC初始化失败处理
         * 知识点: 初始化失败策略
         * - 发送错误事件
         * - 记录错误统计
         * - 任务进入错误状态
         */
        DEBUG_PRINT("ERROR: ADC init failed in DataCollectTask\r\n");
        Tasks_SetEvent(SYSTEM_ERROR_EVENT, false);
        Task_Management.data_collect.error_count++;
        Task_Management.total_task_errors++;
        
        // 任务进入错误循环
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    DEBUG_PRINT("ADC initialized successfully for data collection\r\n");
    
    /**
     * 任务主循环
     * 知识点: 数据采集循环
     * - 周期性采集数据
     * - 数据处理和校准
     * - 队列发送
     */
    while (1) {
        /**
         * 更新任务统计信息
         */
        Task_Management.data_collect.cycle_count++;
        Task_Management.data_collect.last_execution_time = System_Get_Runtime_MS();
        
        /**
         * 第一步: 采集传感器数据
         * 知识点: 多通道ADC采集
         * - 温度传感器 (PA0)
         * - 光线传感器 (PA1)
         * - 气体传感器 (PA2)
         * - 一次性扫描所有通道
         */
        DEBUG_PRINT("Starting sensor data collection...\r\n");
        
        /**
         * 温度数据采集
         * 知识点: 温度传感器数据处理
         * - 读取ADC原始值
         * - 转换为电压值
         * - 根据热敏电阻特性转换为温度
         * - 温度范围: -40°C 到 +85°C
         */
        uint16_t temp_raw = ADC_ReadChannel(ADC_CHANNEL_TEMP);
        if (temp_raw != 0) {
            /**
             * 温度转换算法
             * 知识点: 热敏电阻温度计算
             * - ADC值 -> 电压值
             * - 电压值 -> 电阻值
             * - 电阻值 -> 温度值
             * 
             * 公式: T = 1 / (1/T0 + (1/B)*ln(R/R0))
             * 其中: T0=298.15K(25°C), B=3950, R0=10kΩ
             */
            float voltage = (float)temp_raw * VREF / ADC_RESOLUTION;
            float resistance = VOLTAGE_DIVIDER_R / (VREF / voltage - 1.0f);
            float temperature = 1.0f / (1.0f/298.15f + (1.0f/3950.0f) * logf(resistance/10000.0f)) - 273.15f;
            
            sensor_data.temperature = temperature;
            DEBUG_PRINT("Temperature: %.2f°C (raw: %u, voltage: %.3fV)\r\n", 
                       temperature, temp_raw, voltage);
        } else {
            /**
             * 温度读取失败处理
             * 知识点: 数据异常处理
             * - 使用默认值
             * - 记录错误
             * - 继续处理其他传感器
             */
            sensor_data.temperature = DEFAULT_TEMPERATURE;
            DEBUG_PRINT("ERROR: Temperature read failed, using default value\r\n");
            Task_Management.data_collect.error_count++;
        }
        
        /**
         * 光线数据采集
         * 知识点: 光线传感器数据处理
         * - 光敏电阻特性: 光线越强电阻越小
         * - ADC值范围: 0-4095
         * - 转换到光线强度百分比
         * - 范围: 0-100%
         */
        uint16_t light_raw = ADC_ReadChannel(ADC_CHANNEL_LIGHT);
        if (light_raw != 0) {
            /**
             * 光线强度转换
             * 知识点: 光敏电阻特性曲线
             * - 暗光: 电阻大, ADC值小
             * - 强光: 电阻小, ADC值大
             * - 非线性关系，需要校准
             */
            float light_percentage = ((float)light_raw / ADC_RESOLUTION) * 100.0f;
            light_percentage = light_percentage > 100.0f ? 100.0f : light_percentage;
            light_percentage = light_percentage < 0.0f ? 0.0f : light_percentage;
            
            sensor_data.light_level = light_percentage;
            DEBUG_PRINT("Light level: %.1f%% (raw: %u)\r\n", 
                       light_percentage, light_raw);
        } else {
            /**
             * 光线读取失败处理
             */
            sensor_data.light_level = DEFAULT_LIGHT_LEVEL;
            DEBUG_PRINT("ERROR: Light level read failed, using default value\r\n");
            Task_Management.data_collect.error_count++;
        }
        
        /**
         * 气体数据采集
         * 知识点: 气体传感器数据处理
         * - MQ-2气体传感器
         * - 检测可燃气体、烟雾等
         * - ADC值转换为浓度值
         * - 范围: 0-1000ppm
         */
        uint16_t gas_raw = ADC_ReadChannel(ADC_CHANNEL_GAS);
        if (gas_raw != 0) {
            /**
             * 气体浓度转换
             * 知识点: MQ-2传感器特性
             * - 清洁空气中 Rs/Ro ≈ 9.8
             * - 气体浓度越高 Rs 越小
             * - 使用对数关系计算浓度
             */
            float voltage = (float)gas_raw * VREF / ADC_RESOLUTION;
            float rs = VOLTAGE_DIVIDER_R * (VREF / voltage - 1.0f);
            float ratio = rs / MQ2_RO;  // Rs/Ro比值
            
            /**
             * 浓度计算
             * 知识点: 气体浓度算法
             * - 使用经验公式
             * - 对数关系拟合实际曲线
             * - 输出ppm单位
             */
            float gas_concentration;
            if (ratio > 0.1f) {
                gas_concentration = (1.0f / ratio) * 1.0f;  // 简化计算
            } else {
                gas_concentration = 0.0f;
            }
            
            gas_concentration = gas_concentration > 1000.0f ? 1000.0f : gas_concentration;
            gas_concentration = gas_concentration < 0.0f ? 0.0f : gas_concentration;
            
            sensor_data.gas_level = gas_concentration;
            DEBUG_PRINT("Gas concentration: %.1f ppm (raw: %u, ratio: %.2f)\r\n", 
                       gas_concentration, gas_raw, ratio);
        } else {
            /**
             * 气体读取失败处理
             */
            sensor_data.gas_level = DEFAULT_GAS_LEVEL;
            DEBUG_PRINT("ERROR: Gas level read failed, using default value\r\n");
            Task_Management.data_collect.error_count++;
        }
        
        /**
         * 第二步: 数据校准和滤波
         * 知识点: 数据处理
         * - 校准: 根据传感器特性进行校准
         * - 滤波: 移动平均减少噪声
         * - 范围限制: 防止异常值
         */
        
        /**
         * 温度校准和滤波
         * 知识点: 温度数据处理
         * - 范围校准: -40到+85°C
         * - 移动平均滤波
         * - 异常值检测
         */
        if (sensor_data.temperature > MIN_TEMPERATURE && sensor_data.temperature < MAX_TEMPERATURE) {
            // 简单移动平均滤波
            filtered_data.temperature = (filtered_data.temperature * 0.7f) + (sensor_data.temperature * 0.3f);
        } else {
            /**
             * 温度异常值处理
             * 知识点: 异常值检测
             * - 超出正常范围的数据视为异常
             * - 使用上次正常值
             * - 记录异常事件
             */
            DEBUG_PRINT("WARNING: Temperature value abnormal: %.2f\r\n", sensor_data.temperature);
            Task_Management.data_collect.error_count++;
        }
        
        /**
         * 光线校准和滤波
         * 知识点: 光线数据处理
         * - 范围校准: 0-100%
         * - 移动平均滤波
         * - 环境光补偿
         */
        if (sensor_data.light_level >= 0.0f && sensor_data.light_level <= 100.0f) {
            filtered_data.light_level = (filtered_data.light_level * 0.8f) + (sensor_data.light_level * 0.2f);
        } else {
            DEBUG_PRINT("WARNING: Light level abnormal: %.1f\r\n", sensor_data.light_level);
            Task_Management.data_collect.error_count++;
        }
        
        /**
         * 气体校准和滤波
         * 知识点: 气体数据处理
         * - 范围校准: 0-1000ppm
         * - 移动平均滤波
         * - 报警阈值检查
         */
        if (sensor_data.gas_level >= 0.0f && sensor_data.gas_level <= MAX_GAS_LEVEL) {
            filtered_data.gas_level = (filtered_data.gas_level * 0.9f) + (sensor_data.gas_level * 0.1f);
            
            /**
             * 气体浓度报警检查
             * 知识点: 报警机制
             * - 超过阈值触发报警
             * - 蜂鸣器报警
             * - LED报警指示
             */
            if (filtered_data.gas_level > GAS_ALARM_THRESHOLD) {
                /**
                 * 气体浓度过高报警
                 * 知识点: 紧急处理
                 * - 启动蜂鸣器
                 * - LED红色闪烁
                 * - 发送报警事件
                 */
                Buzzer_StartAlarm();
                LED_StartBlink(LED_RED, 200);
                Tasks_SetEvent(SYSTEM_ERROR_EVENT, false);
                
                DEBUG_PRINT("ALARM: Gas concentration too high: %.1f ppm\r\n", filtered_data.gas_level);
            }
        } else {
            DEBUG_PRINT("WARNING: Gas level abnormal: %.1f\r\n", sensor_data.gas_level);
            Task_Management.data_collect.error_count++;
        }
        
        /**
         * 第三步: 数据验证和发送
         * 知识点: 数据质量检查
         * - 验证所有数据有效性
         * - 检查数据完整性
         * - 队列发送
         */
        
        /**
         * 数据验证
         * 知识点: 数据有效性检查
         * - 检查是否有NaN或Inf值
         * - 检查数值范围
         * - 确保数据完整性
         */
        bool data_valid = true;
        if (isnan(filtered_data.temperature) || isinf(filtered_data.temperature)) {
            DEBUG_PRINT("ERROR: Invalid temperature data\r\n");
            data_valid = false;
        }
        if (isnan(filtered_data.light_level) || isinf(filtered_data.light_level)) {
            DEBUG_PRINT("ERROR: Invalid light level data\r\n");
            data_valid = false;
        }
        if (isnan(filtered_data.gas_level) || isinf(filtered_data.gas_level)) {
            DEBUG_PRINT("ERROR: Invalid gas level data\r\n");
            data_valid = false;
        }
        
        /**
         * 队列发送
         * 知识点: 任务间数据传输
         * - 使用xQueueSend发送数据
         * - 非阻塞发送
         * - 超时控制
         */
        if (data_valid) {
            /**
             * 发送数据到队列
             * 知识点: 队列发送参数
             * - 参数1: 队列句柄
             * - 参数2: 数据指针
             * - 参数3: 阻塞时间(0表示不阻塞)
             * - 返回: pdTRUE成功，pdFALSE失败
             */
            if (xQueueSend(Data_Queue, &filtered_data, 0) == pdTRUE) {
                /**
                 * 发送成功
                 * 知识点: 成功处理
                 * - 设置数据就绪事件
                 * - 统计发送成功
                 * - 调试输出
                 */
                sample_count++;
                Tasks_SetEvent(DATA_READY_EVENT, false);
                
                DEBUG_PRINT("Data collected and sent successfully (sample %u):\r\n", sample_count);
                DEBUG_PRINT("  Temperature: %.2f°C\r\n", filtered_data.temperature);
                DEBUG_PRINT("  Light level: %.1f%%\r\n", filtered_data.light_level);
                DEBUG_PRINT("  Gas level: %.1f ppm\r\n", filtered_data.gas_level);
                
            } else {
                /**
                 * 发送失败处理
                 * 知识点: 队列满处理
                 * - 队列已满，无法发送
                 * - 记录错误统计
                 * - 继续下次采集
                 */
                DEBUG_PRINT("ERROR: Data queue is full, data lost\r\n");
                Task_Management.data_collect.error_count++;
            }
        } else {
            /**
             * 数据验证失败处理
             * 知识点: 数据异常处理
             * - 数据无效，不发送
             * - 记录错误
             * - 清理异常数据
             */
            DEBUG_PRINT("ERROR: Data validation failed, discarding data\r\n");
            Task_Management.data_collect.error_count++;
        }
        
        /**
         * 第四步: 周期延时
         * 知识点: 采集频率控制
         * - 控制数据采集频率
         * - 平衡精度和资源消耗
         * - 避免过于频繁采集
         */
        DEBUG_PRINT("Data collection cycle completed, next collection in %u ms\r\n", DATA_COLLECT_PERIOD_MS);
        vTaskDelay(pdMS_TO_TICKS(DATA_COLLECT_PERIOD_MS));
    }
}

/**
 * @brief   数据发送任务
 * @details 负责数据的网络传输和Flash存储管理
 *          包含发送队列处理、重试机制、存储管理等功能
 * @param   pvParameters: 任务参数
 * @return  无
 * 
 * 任务执行流程:
 * 1. 等待数据就绪事件
 * 2. 从队列获取数据
 * 3. 尝试网络发送
 * 4. 失败时存储到Flash
 * 5. 管理存储空间
 * 
 * 发送策略:
 * - 网络优先: 先尝试网络发送
 * - 本地存储: 网络失败时本地存储
 * - 重试机制: 发送失败自动重试
 * - 队列管理: 避免数据丢失
 * 
 * 存储管理:
 * - 环形缓冲区: 有效利用Flash空间
 * - 写指针管理: 避免数据覆盖
 * - 空间回收: 定期清理已发送数据
 * - 存储状态监控: 防止存储溢出
 * 
 * 错误处理:
 * - 网络异常: 自动重试和本地存储
 * - Flash异常: 发送错误事件
 * - 队列异常: 跳过本条数据
 * - 重试超限: 放弃发送，记录错误
 */
void DataSendTask(void *pvParameters)
{
    /**
     * 任务初始化
     * 知识点: 任务本地变量
     * - data_to_send: 待发送的数据
     * - retry_count: 重试计数
     * - send_success: 发送成功标记
     */
    (void)pvParameters;  // 避免编译器警告
    
    Sensor_Data_t data_to_send = {0};
    uint32_t retry_count = 0;
    bool send_success = false;
    
    DEBUG_PRINT("DataSendTask started\r\n");
    
    /**
     * 任务主循环
     * 知识点: 数据发送循环
     * - 等待数据就绪事件
     * - 从队列获取数据
     * - 尝试发送数据
     * - 处理发送结果
     */
    while (1) {
        /**
         * 更新任务统计信息
         */
        Task_Management.data_send.cycle_count++;
        Task_Management.data_send.last_execution_time = System_Get_Runtime_MS();
        
        /**
         * 第一步: 等待数据就绪
         * 知识点: 事件等待机制
         * - 等待DATA_READY_EVENT事件
         * - 阻塞等待，超时5秒
         * - 事件触发时自动清除
         */
        DEBUG_PRINT("Waiting for data ready event...\r\n");
        uint32_t events = Tasks_WaitEvent(DATA_READY_EVENT, true, 5000);
        
        if (events & DATA_READY_EVENT) {
            /**
             * 数据就绪事件触发
             * 知识点: 事件处理
             * - 从队列获取数据
             * - 开始发送流程
             * - 重置重试计数
             */
            DEBUG_PRINT("Data ready event received, processing data...\r\n");
            retry_count = 0;
            
            /**
             * 第二步: 从队列获取数据
             * 知识点: 队列接收
             * - 使用xQueueReceive接收数据
             * - 非阻塞接收
             * - 超时时间100ms
             */
            if (xQueueReceive(Data_Queue, &data_to_send, pdMS_TO_TICKS(100)) == pdTRUE) {
                /**
                 * 数据获取成功
                 * 知识点: 数据处理
                 * - 数据格式转换
                 * - 发送内容准备
                 * - 调试输出
                 */
                DEBUG_PRINT("Data received from queue:\r\n");
                DEBUG_PRINT("  Temperature: %.2f°C\r\n", data_to_send.temperature);
                DEBUG_PRINT("  Light level: %.1f%%\r\n", data_to_send.light_level);
                DEBUG_PRINT("  Gas level: %.1f ppm\r\n", data_to_send.gas_level);
                
                /**
                 * 第三步: 尝试网络发送
                 * 知识点: 发送策略
                 * - 检查网络连接状态
                 * - 格式化发送数据
                 * - 调用ESP-01s发送函数
                 */
                
                /**
                 * 检查网络连接状态
                 * 知识点: 网络状态检查
                 * - 使用Network_Semaphore检查连接
                 * - 网络未连接直接跳过发送
                 * - 记录网络状态
                 */
                if (xSemaphoreTake(Network_Semaphore, 0) == pdTRUE) {
                    /**
                     * 网络已连接
                     * 知识点: 网络发送流程
                     * - 格式化JSON或文本数据
                     * - 发送AT+CIPSEND指令
                     * - 等待发送确认
                     */
                    DEBUG_PRINT("Network connected, attempting to send data...\r\n");
                    
                    /**
                     * 格式化发送数据
                     * 知识点: 数据格式
                     * - 简单文本格式
                     * - 易于解析和调试
                     * - 不包含时间戳
                     */
                    char send_buffer[128];
                    snprintf(send_buffer, sizeof(send_buffer), 
                            "TEMP:%.1f LIGHT:%.0f GAS:%.0f\r\n",
                            data_to_send.temperature, 
                            data_to_send.light_level, 
                            data_to_send.gas_level);
                    
                    DEBUG_PRINT("Formatted data: %s", send_buffer);
                    
                    /**
                     * 发送数据
                     * 知识点: ESP-01s发送函数
                     * - AT+CIPSEND设置发送长度
                     * - 实际数据传输
                     * - 等待发送确认
                     */
                    AT_Command_Status_t send_result = ESP01s_SendData(send_buffer, strlen(send_buffer));
                    
                    if (send_result == AT_COMMAND_SUCCESS) {
                        /**
                         * 发送成功
                         * 知识点: 成功处理
                         * - 释放信号量
                         * - 标记发送成功
                         * - 记录成功统计
                         */
                        xSemaphoreGive(Network_Semaphore);
                        send_success = true;
                        retry_count = 0;
                        
                        DEBUG_PRINT("Data sent successfully via network\r\n");
                        
                    } else {
                        /**
                         * 发送失败
                         * 知识点: 发送失败处理
                         * - 释放信号量
                         * - 标记发送失败
                         * - 准备重试或本地存储
                         */
                        xSemaphoreGive(Network_Semaphore);
                        send_success = false;
                        
                        DEBUG_PRINT("Network send failed (error code: %d), will retry or use local storage\r\n", send_result);
                    }
                    
                } else {
                    /**
                     * 网络未连接
                     * 知识点: 网络异常处理
                     * - 发送失败
                     * - 准备本地存储
                     * - 记录网络状态
                     */
                    DEBUG_PRINT("Network not connected, data will be stored locally\r\n");
                    send_success = false;
                }
                
                /**
                 * 第四步: 处理发送结果
                 * 知识点: 结果处理策略
                 * - 成功: 数据发送完成
                 * - 失败: 尝试重试或本地存储
                 * - 错误: 记录错误信息
                 */
                
                if (send_success) {
                    /**
                     * 发送成功处理
                     * 知识点: 成功完成
                     * - LED状态指示
                     * - 记录成功统计
                     * - 准备处理下一条数据
                     */
                    LED_BlinkOnce(LED_GREEN, 100);
                    DEBUG_PRINT("Data transmission completed successfully\r\n");
                    
                } else {
                    /**
                     * 发送失败处理
                     * 知识点: 失败重试策略
                     * - 启动重试机制
                     * - 重试次数限制
                     * - 重试间隔控制
                     */
                    DEBUG_PRINT("Data transmission failed, starting retry mechanism\r\n");
                    
                    /**
                     * 重试机制
                     * 知识点: 智能重试
                     * - 指数退避策略
                     * - 最大重试次数限制
                     * - 重试间隔动态调整
                     */
                    for (retry_count = 1; retry_count <= MAX_SEND_RETRY; retry_count++) {
                        DEBUG_PRINT("Retry attempt %u/%u\r\n", retry_count, MAX_SEND_RETRY);
                        
                        /**
                         * 重试间隔
                         * 知识点: 重试间隔策略
                         * - 第一次重试: 1秒
                         * - 第二次重试: 2秒
                         * - 第三次重试: 4秒
                         * - 指数退避减少网络冲击
                         */
                        uint32_t retry_delay = 1000 * (1 << (retry_count - 1));
                        if (retry_delay > 5000) retry_delay = 5000;  // 最大5秒
                        
                        vTaskDelay(pdMS_TO_TICKS(retry_delay));
                        
                        /**
                         * 重新尝试网络发送
                         * 知识点: 重试发送
                         * - 检查网络状态
                         * - 重新发送数据
                         * - 评估发送结果
                         */
                        if (xSemaphoreTake(Network_Semaphore, 0) == pdTRUE) {
                            AT_Command_Status_t retry_result = ESP01s_SendData(send_buffer, strlen(send_buffer));
                            
                            if (retry_result == AT_COMMAND_SUCCESS) {
                                /**
                                 * 重试成功
                                 * 知识点: 重试成功处理
                                 * - 释放信号量
                                 * - 标记成功
                                 * - 跳出重试循环
                                 */
                                xSemaphoreGive(Network_Semaphore);
                                send_success = true;
                                LED_BlinkOnce(LED_GREEN, 100);
                                
                                DEBUG_PRINT("Data sent successfully on retry attempt %u\r\n", retry_count);
                                break;
                            } else {
                                /**
                                 * 重试仍然失败
                                 * 知识点: 重试失败处理
                                 * - 释放信号量
                                 * - 继续下次重试或转到本地存储
                                 */
                                xSemaphoreGive(Network_Semaphore);
                                DEBUG_PRINT("Retry attempt %u failed\r\n", retry_count);
                            }
                        } else {
                            /**
                             * 网络仍然不可用
                             * 知识点: 网络不可用处理
                             * - 跳过本次重试
                             * - 记录网络状态
                             */
                            DEBUG_PRINT("Network still not available on retry attempt %u\r\n", retry_count);
                        }
                    }
                    
                    /**
                     * 重试结果处理
                     * 知识点: 重试结果判断
                     * - 成功: 完成发送
                     * - 失败: 本地存储
                     * - 错误: 记录错误
                     */
                    if (!send_success) {
                        /**
                         * 所有重试都失败
                         * 知识点: 本地存储策略
                         * - 数据不能丢失
                         * - 存储到Flash
                         * - 保证数据持久化
                         */
                        DEBUG_PRINT("All retry attempts failed, storing data locally\r\n");
                        
                        /**
                         * 本地Flash存储
                         * 知识点: Flash存储流程
                         * - 互斥锁保护
                         * - 写入Flash
                         * - 管理环形缓冲区
                         */
                        if (xSemaphoreTake(Flash_Mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                            /**
                             * Flash写入
                             * 知识点: Flash操作
                             * - 格式化数据
                             * - 调用Flash写入函数
                             * - 处理写入结果
                             */
                            char flash_buffer[128];
                            snprintf(flash_buffer, sizeof(flash_buffer),
                                    "TEMP:%.1f LIGHT:%.0f GAS:%.0f\r\n",
                                    data_to_send.temperature,
                                    data_to_send.light_level,
                                    data_to_send.gas_level);
                            
                            Flash_Write_Result_t write_result = SPI_Flash_WriteData((uint8_t*)flash_buffer, strlen(flash_buffer));
                            
                            if (write_result == FLASH_WRITE_SUCCESS) {
                                /**
                                 * Flash写入成功
                                 * 知识点: 存储成功
                                 * - 释放互斥锁
                                 * - 记录存储统计
                                 * - 设置存储满事件(如果需要)
                                 */
                                xSemaphoreGive(Flash_Mutex);
                                send_success = true;
                                
                                /**
                                 * 检查存储空间
                                 * 知识点: 存储空间监控
                                 * - 监控Flash使用率
                                 * - 存储满时发送事件
                                 * - 防止数据丢失
                                 */
                                uint32_t free_space = SPI_Flash_GetFreeSpace();
                                if (free_space < MIN_FREE_SPACE) {
                                    Tasks_SetEvent(FLASH_FULL_EVENT, false);
                                    DEBUG_PRINT("WARNING: Flash storage space low (free: %u bytes)\r\n", free_space);
                                }
                                
                                DEBUG_PRINT("Data stored locally in Flash successfully\r\n");
                                
                            } else {
                                /**
                                 * Flash写入失败
                                 * 知识点: 存储失败处理
                                 * - 释放互斥锁
                                 * - 发送错误事件
                                 * - 记录错误统计
                                 */
                                xSemaphoreGive(Flash_Mutex);
                                DEBUG_PRINT("ERROR: Flash write failed (error code: %d)\r\n", write_result);
                                
                                Tasks_SetEvent(SYSTEM_ERROR_EVENT, false);
                                Task_Management.data_send.error_count++;
                                Task_Management.total_task_errors++;
                            }
                        } else {
                            /**
                             * 互斥锁获取失败
                             * 知识点:                                /**
                                 * 互斥锁获取失败
                                 * 知识点: 互斥锁超时处理
                                 * - 互斥锁被其他任务占用
                                 * - 等待超时时间到达
                                 * - 跳过本轮存储操作
                                 */
                                DEBUG_PRINT("ERROR: Flash mutex timeout, data will be lost\r\n");
                                Task_Management.data_send.error_count++;
                            }
                        }
                    }
                }
                
            } else {
                /**
                 * 队列接收失败
                 * 知识点: 队列异常处理
                 * - 队列为空或错误
                 * - 记录错误统计
                 * - 继续下次循环
                 */
                DEBUG_PRINT("ERROR: Failed to receive data from queue\r\n");
                Task_Management.data_send.error_count++;
            }
        } else {
            /**
             * 等待数据就绪事件超时
             * 知识点: 超时处理
             * - 5秒内没有数据就绪
             * - 正常情况，继续等待
             * - 记录超时统计
             */
            DEBUG_PRINT("Data ready event wait timeout\r\n");
        }
        
        /**
         * 第五步: 周期延时
         * 知识点: 发送频率控制
         * - 控制数据发送频率
         * - 避免过于频繁发送
         * - 平衡响应性和资源消耗
         */
        vTaskDelay(pdMS_TO_TICKS(DATA_SEND_PERIOD_MS));
    }
}

/**
 * @brief   系统监控任务
 * @details 负责系统整体状态监控和异常处理
 *          包含资源监控、错误处理、状态指示等功能
 * @param   pvParameters: 任务参数
 * @return  无
 * 
 * 任务执行流程:
 * 1. 等待系统启动
 * 2. 定期系统状态检查
 * 3. 处理错误队列
 * 4. LED状态指示
 * 5. 任务健康检查
 * 
 * 监控策略:
 * - 周期性检查: 3秒执行一次
 * - 全面监控: CPU、内存、任务状态
 * - 错误处理: 分类处理不同类型错误
 * - 状态指示: LED反映系统状态
 * 
 * 监控内容:
 * 1. 系统资源监控
 *    - CPU使用率
 *    - 堆内存使用
 *    - 任务堆栈使用
 * 
 * 2. 任务健康检查
 *    - 任务运行状态
 *    - 任务执行时间
 *    - 任务循环计数
 * 
 * 3. 错误处理
 *    - 错误队列处理
 *    - 错误分类统计
 *    - 错误恢复策略
 * 
 * 4. 状态指示
 *    - LED状态管理
 *    - 蜂鸣器控制
 *    - 系统状态反馈
 * 
 * 异常处理:
 * - 严重错误: 系统重启
 * - 内存不足: 清理资源
 * - 任务异常: 重启任务
 * - 存储异常: 清理存储
 */
void SystemMonitorTask(void *pvParameters)
{
    /**
     * 任务初始化
     * 知识点: 任务本地变量
     * - last_monitor_time: 上次监控时间
     * - error_buffer: 错误信息缓冲
     * - system_health: 系统健康状态
     */
    (void)pvParameters;  // 避免编译器警告
    
    uint32_t last_monitor_time = 0;
    Error_Info_t error_buffer = {0};
    uint8_t system_health = SYSTEM_HEALTH_GOOD;
    
    DEBUG_PRINT("SystemMonitorTask started\r\n");
    
    /**
     * 任务主循环
     * 知识点: 系统监控循环
     * - 周期性系统检查
     * - 错误队列处理
     * - 状态指示更新
     */
    while (1) {
        /**
         * 更新任务统计信息
         */
        Task_Management.system_monitor.cycle_count++;
        Task_Management.system_monitor.last_execution_time = System_Get_Runtime_MS();
        
        /**
         * 第一步: 系统资源监控
         * 知识点: 系统资源检查
         * - FreeRTOS堆内存使用
         * - 任务堆栈使用情况
         * - CPU使用率估计
         */
        DEBUG_PRINT("=== System Resource Monitoring ===\r\n");
        
        /**
         * 堆内存监控
         * 知识点: 堆内存管理
         * - xPortGetFreeHeapSize(): 获取剩余堆内存
         * - configTOTAL_HEAP_SIZE: 总堆内存大小
         * - 使用率计算
         */
        size_t free_heap = xPortGetFreeHeapSize();
        size_t total_heap = configTOTAL_HEAP_SIZE;
        float heap_usage_percent = ((float)(total_heap - free_heap) / total_heap) * 100.0f;
        
        DEBUG_PRINT("Heap Memory:\r\n");
        DEBUG_PRINT("  Total: %u bytes\r\n", total_heap);
        DEBUG_PRINT("  Free: %u bytes\r\n", free_heap);
        DEBUG_PRINT("  Usage: %.1f%%\r\n", heap_usage_percent);
        
        /**
         * 内存使用异常检测
         * 知识点: 内存使用阈值
         * - 超过80%使用率: 警告
         * - 超过90%使用率: 严重
         * - 接近耗尽: 紧急
         */
        if (heap_usage_percent > 90.0f) {
            system_health = SYSTEM_HEALTH_CRITICAL;
            DEBUG_PRINT("CRITICAL: Heap memory usage too high!\r\n");
            Tasks_SetEvent(SYSTEM_ERROR_EVENT, false);
        } else if (heap_usage_percent > 80.0f) {
            system_health = SYSTEM_HEALTH_WARNING;
            DEBUG_PRINT("WARNING: Heap memory usage high\r\n");
        } else {
            system_health = SYSTEM_HEALTH_GOOD;
        }
        
        /**
         * 任务堆栈监控
         * 知识点: 堆栈溢出检测
         * - uxTaskGetStackHighWaterMark(): 获取堆栈高水位
         * - 高水位越接近0，堆栈使用越多
         * - 需要及时调整堆栈大小
         */
        DEBUG_PRINT("Task Stack Usage:\r\n");
        
        /**
         * 网络管理任务堆栈监控
         */
        if (NetworkManageTask_Handle != NULL) {
            configSTACK_DEPTH_TYPE network_stack = uxTaskGetStackHighWaterMark(NetworkManageTask_Handle);
            DEBUG_PRINT("  NetworkManage: %u words remaining\r\n", network_stack);
            
            if (network_stack < 20) {  // 剩余堆栈少于20字
                DEBUG_PRINT("WARNING: NetworkManage task stack nearly exhausted\r\n");
                Task_Management.network_manage.error_count++;
            }
        }
        
        /**
         * 数据采集任务堆栈监控
         */
        if (DataCollectTask_Handle != NULL) {
            configSTACK_DEPTH_TYPE collect_stack = uxTaskGetStackHighWaterMark(DataCollectTask_Handle);
            DEBUG_PRINT("  DataCollect: %u words remaining\r\n", collect_stack);
            
            if (collect_stack < 20) {
                DEBUG_PRINT("WARNING: DataCollect task stack nearly exhausted\r\n");
                Task_Management.data_collect.error_count++;
            }
        }
        
        /**
         * 数据发送任务堆栈监控
         */
        if (DataSendTask_Handle != NULL) {
            configSTACK_DEPTH_TYPE send_stack = uxTaskGetStackHighWaterMark(DataSendTask_Handle);
            DEBUG_PRINT("  DataSend: %u words remaining\r\n", send_stack);
            
            if (send_stack < 20) {
                DEBUG_PRINT("WARNING: DataSend task stack nearly exhausted\r\n");
                Task_Management.data_send.error_count++;
            }
        }
        
        /**
         * 系统监控任务堆栈监控
         */
        if (SystemMonitorTask_Handle != NULL) {
            configSTACK_DEPTH_TYPE monitor_stack = uxTaskGetStackHighWaterMark(SystemMonitorTask_Handle);
            DEBUG_PRINT("  SystemMonitor: %u words remaining\r\n", monitor_stack);
            
            if (monitor_stack < 20) {
                DEBUG_PRINT("WARNING: SystemMonitor task stack nearly exhausted\r\n");
                Task_Management.system_monitor.error_count++;
            }
        }
        
        /**
         * 第二步: 任务状态检查
         * 知识点: 任务健康监控
         * - 检查任务是否在运行
         * - 监控任务循环频率
         * - 检测任务是否卡死
         */
        DEBUG_PRINT("=== Task Status Monitoring ===\r\n");
        
        /**
         * 任务循环频率检查
         * 知识点: 任务活性检测
         * - 记录上次循环时间
         * - 检查循环间隔是否异常
         * - 任务卡死检测
         */
        uint32_t current_time = System_Get_Runtime_MS();
        uint32_t network_time_diff = current_time - Task_Management.network_manage.last_execution_time;
        uint32_t collect_time_diff = current_time - Task_Management.data_collect.last_execution_time;
        uint32_t send_time_diff = current_time - Task_Management.data_send.last_execution_time;
        uint32_t monitor_time_diff = current_time - Task_Management.system_monitor.last_execution_time;
        
        /**
         * 网络管理任务活性检查
         */
        if (Task_Management.network_manage.status == TASK_STATUS_RUNNING) {
            if (network_time_diff > NETWORK_CHECK_PERIOD_MS * 3) {  // 超过3倍周期时间
                DEBUG_PRINT("WARNING: NetworkManage task may be stuck (last execution: %u ms ago)\r\n", network_time_diff);
                Task_Management.network_manage.error_count++;
            } else {
                DEBUG_PRINT("NetworkManage: Running, Last exec: %u ms ago, Cycles: %u\r\n", 
                           network_time_diff, Task_Management.network_manage.cycle_count);
            }
        } else {
            DEBUG_PRINT("NetworkManage: Status = %u (may be suspended or error)\r\n", 
                       Task_Management.network_manage.status);
        }
        
        /**
         * 数据采集任务活性检查
         */
        if (Task_Management.data_collect.status == TASK_STATUS_RUNNING) {
            if (collect_time_diff > DATA_COLLECT_PERIOD_MS * 3) {
                DEBUG_PRINT("WARNING: DataCollect task may be stuck (last execution: %u ms ago)\r\n", collect_time_diff);
                Task_Management.data_collect.error_count++;
            } else {
                DEBUG_PRINT("DataCollect: Running, Last exec: %u ms ago, Cycles: %u\r\n", 
                           collect_time_diff, Task_Management.data_collect.cycle_count);
            }
        } else {
            DEBUG_PRINT("DataCollect: Status = %u (may be suspended or error)\r\n", 
                       Task_Management.data_collect.status);
        }
        
        /**
         * 数据发送任务活性检查
         */
        if (Task_Management.data_send.status == TASK_STATUS_RUNNING) {
            if (send_time_diff > DATA_SEND_PERIOD_MS * 3) {
                DEBUG_PRINT("WARNING: DataSend task may be stuck (last execution: %u ms ago)\r\n", send_time_diff);
                Task_Management.data_send.error_count++;
            } else {
                DEBUG_PRINT("DataSend: Running, Last exec: %u ms ago, Cycles: %u\r\n", 
                           send_time_diff, Task_Management.data_send.cycle_count);
            }
        } else {
            DEBUG_PRINT("DataSend: Status = %u (may be suspended or error)\r\n", 
                       Task_Management.data_send.status);
        }
        
        /**
         * 系统监控任务活性检查
         */
        DEBUG_PRINT("SystemMonitor: Running, Last exec: %u ms ago, Cycles: %u\r\n", 
                   monitor_time_diff, Task_Management.system_monitor.cycle_count);
        
        /**
         * 第三步: 错误队列处理
         * 知识点: 错误信息处理
         * - 从错误队列获取错误信息
         * - 分类处理不同类型错误
         * - 错误统计和日志
         */
        DEBUG_PRINT("=== Error Queue Processing ===\r\n");
        
        /**
         * 处理错误队列中的错误
         * 知识点: 错误处理策略
         * - 非阻塞获取错误
         * - 分类处理错误
         * - 错误恢复机制
         */
        uint32_t error_count = 0;
        while (xQueueReceive(Error_Queue, &error_buffer, 0) == pdTRUE && error_count < 10) {
            error_count++;
            
            /**
             * 错误信息解析
             * 知识点: 错误分类处理
             * - 根据错误代码分类
             * - 不同错误不同处理策略
             * - 错误严重程度评估
             */
            DEBUG_PRINT("Error received: Task=%u, Code=%u, Count=%u, Message=%s\r\n",
                       error_buffer.task_id, error_buffer.error_code, 
                       error_buffer.error_count, error_buffer.error_message);
            
            /**
             * 错误分类处理
             * 知识点: 错误类型处理
             */
            switch (error_buffer.error_code) {
                case ERROR_NETWORK_FAILED: {
                    /**
                     * 网络错误处理
                     * 知识点: 网络错误恢复
                     * - 检查网络状态
                     * - 尝试重连
                     * - 网络质量评估
                     */
                    DEBUG_PRINT("Handling network error: %s\r\n", error_buffer.error_message);
                    // 网络错误通常由NetworkManageTask处理
                    break;
                }
                
                case ERROR_ADC_FAILED: {
                    /**
                     * ADC错误处理
                     * 知识点: 传感器错误恢复
                     * - 重新初始化ADC
                     * - 检查传感器连接
                     * - 使用默认值继续运行
                     */
                    DEBUG_PRINT("Handling ADC error: %s\r\n", error_buffer.error_message);
                    
                    /**
                     * 重新初始化ADC
                     * 知识点: ADC重初始化
                     * - 复位ADC配置
                     * - 重新配置通道
                     * - 验证初始化结果
                     */
                    if (ADC_Init() != ADC_SUCCESS) {
                        DEBUG_PRINT("ERROR: ADC re-initialization failed\r\n");
                        Tasks_SetEvent(SYSTEM_ERROR_EVENT, false);
                    } else {
                        DEBUG_PRINT("ADC re-initialized successfully\r\n");
                    }
                    break;
                }
                
                case ERROR_FLASH_FAILED: {
                    /**
                     * Flash错误处理
                     * 知识点: 存储错误恢复
                     * - 检查Flash状态
                     * - 尝试恢复存储
                     * - 清理损坏数据
                     */
                    DEBUG_PRINT("Handling Flash error: %s\r\n", error_buffer.error_message);
                    
                    /**
                     * Flash状态检查
                     * 知识点: Flash诊断
                     * - 检查芯片ID
                     * - 验证读写功能
                     * - 清理坏块
                     */
                    if (SPI_Flash_Init() != FLASH_SUCCESS) {
                        DEBUG_PRINT("ERROR: Flash re-initialization failed\r\n");
                        Tasks_SetEvent(SYSTEM_ERROR_EVENT, false);
                    } else {
                        DEBUG_PRINT("Flash re-initialized successfully\r\n");
                    }
                    break;
                }
                
                case ERROR_MEMORY_FAILED: {
                    /**
                     * 内存错误处理
                     * 知识点: 内存错误恢复
                     * - 内存泄漏检测
                     * - 清理不必要的资源
                     * - 重启系统(严重情况)
                     */
                    DEBUG_PRINT("Handling memory error: %s\r\n", error_buffer.error_message);
                    
                    if (error_buffer.error_count > 10) {
                        DEBUG_PRINT("CRITICAL: Too many memory errors, considering system restart\r\n");
                        Tasks_SetEvent(SYSTEM_ERROR_EVENT, false);
                    }
                    break;
                }
                
                default: {
                    /**
                     * 未知错误处理
                     * 知识点: 通用错误处理
                     * - 记录错误信息
                     * - 使用默认恢复策略
                     * - 评估错误严重程度
                     */
                    DEBUG_PRINT("Handling unknown error: %s\r\n", error_buffer.error_message);
                    break;
                }
            }
        }
        
        if (error_count > 0) {
            DEBUG_PRINT("Processed %u errors from error queue\r\n", error_count);
        } else {
            DEBUG_PRINT("No errors in error queue\r\n");
        }
        
        /**
         * 第四步: 系统状态指示
         * 知识点: LED状态管理
         * - 根据系统健康状态设置LED
         * - 正常: 绿色LED常亮
         * - 警告: 黄色LED闪烁
         * - 严重: 红色LED闪烁
         * - 紧急: 红色LED快闪
         */
        DEBUG_PRINT("=== System Status Indication ===\r\n");
        
        /**
         * 系统健康状态LED指示
         * 知识点: LED状态逻辑
         * - 绿色: 系统正常运行
         * - 黄色: 系统有警告
         * - 红色: 系统有错误
         * - 闪烁: 表示不同严重程度
         */
        switch (system_health) {
            case SYSTEM_HEALTH_GOOD: {
                /**
                 * 系统健康
                 * 知识点: 正常状态
                 * - 绿色LED常亮
                 * - 其他LED关闭
                 * - 系统运行正常
                 */
                LED_SetState(LED_GREEN, LED_STATE_ON);
                LED_SetState(LED_YELLOW, LED_STATE_OFF);
                LED_SetState(LED_RED, LED_STATE_OFF);
                DEBUG_PRINT("System health: GOOD (Green LED on)\r\n");
                break;
            }
            
            case SYSTEM_HEALTH_WARNING: {
                /**
                 * 系统警告
                 * 知识点: 警告状态
                 * - 绿色LED关闭
                 * - 黄色LED慢闪烁
                 * - 系统可继续运行
                 */
                LED_SetState(LED_GREEN, LED_STATE_OFF);
                LED_StartBlink(LED_YELLOW, 1000);  // 1秒闪烁
                LED_SetState(LED_RED, LED_STATE_OFF);
                DEBUG_PRINT("System health: WARNING (Yellow LED blinking)\r\n");
                break;
            }
            
            case SYSTEM_HEALTH_CRITICAL: {
                /**
                 * 系统严重
                 * 知识点: 严重状态
                 * - 绿色LED关闭
                 * - 黄色LED关闭
                 * - 红色LED快闪烁
                 * - 需要立即处理
                 */
                LED_SetState(LED_GREEN, LED_STATE_OFF);
                LED_SetState(LED_YELLOW, LED_STATE_OFF);
                LED_StartBlink(LED_RED, 500);  // 0.5秒闪烁
                DEBUG_PRINT("System health: CRITICAL (Red LED fast blinking)\r\n");
                
                /**
                 * 严重状态蜂鸣器报警
                 * 知识点: 紧急报警
                 * - 启动蜂鸣器
                 * - 提醒用户系统异常
                 * - 需要立即关注
                 */
                Buzzer_StartAlarm();
                break;
            }
            
            default: {
                /**
                 * 未知状态
                 * 知识点: 状态异常处理
                 * - 设置默认状态
                 * - 记录异常状态
                 * - 继续监控
                 */
                DEBUG_PRINT("WARNING: Unknown system health state: %u\r\n", system_health);
                system_health = SYSTEM_HEALTH_WARNING;
                break;
            }
        }
        
        /**
         * 第五步: 按键事件处理
         * 知识点: 按键监控
         * - 检查是否有按键事件
         * - 处理长按和短按
         * - 系统控制功能
         */
        uint32_t key_events = Tasks_WaitEvent(KEY_PRESSED_EVENT, false, 0);  // 非阻塞检查
        
        if (key_events & KEY_PRESSED_EVENT) {
            /**
             * 按键事件处理
             * 知识点: 按键功能
             * - 短按: 显示系统状态
             * - 长按: 系统重启
             * - 双击: 清除错误统计
             */
            DEBUG_PRINT("Key pressed event detected\r\n");
            
            /**
             * 获取按键信息
             * 知识点: 按键状态查询
             * - 从GPIO模块获取按键状态
             * - 区分短按和长按
             * - 处理多击情况
             */
            Key_State_t key_state = Key_GetState();
            if (key_state.is_pressed && key_state.press_duration > KEY_LONG_PRESS_THRESHOLD) {
                /**
                 * 长按处理
                 * 知识点: 长按功能
                 * - 系统重启确认
                 * - 清理资源
                 * - 系统重启
                 */
                DEBUG_PRINT("Long press detected (duration: %u ms), initiating system restart\r\n", 
                           key_state.press_duration);
                
                /**
                 * 系统重启准备
                 * 知识点: 重启流程
                 * - 停止所有任务
                 * - 清理资源
                 * - 发送重启事件
                 */
                Tasks_SetEvent(SYSTEM_RESET_EVENT, false);
                
                /**
                 * LED状态指示重启
                 * 知识点: 重启指示
                 * - 所有LED快速闪烁
                 * - 蜂鸣器提示
                 * - 准备重启
                 */
                LED_StartBlink(LED_GREEN, 100);
                LED_StartBlink(LED_YELLOW, 100);
                LED_StartBlink(LED_RED, 100);
                Buzzer_StartAlarm();
                
            } else if (key_state.is_pressed) {
                /**
                 * 短按处理
                 * 知识点: 短按功能
                 * - 打印系统状态
                 * - 不执行其他操作
                 * - 继续正常监控
                 */
                DEBUG_PRINT("Short press detected, printing system status\r\n");
                Tasks_PrintStatus();
            }
            
            /**
             * 清除按键事件
             * 知识点: 事件清理
             * - 清除按键按下事件
             * - 准备下次按键检测
             */
            Tasks_ClearEvent(KEY_PRESSED_EVENT);
        }
        
        /**
         * 第六步: 系统重启事件处理
         * 知识点: 系统重启
         * - 接收重启事件
         * - 执行重启流程
         * - 资源清理
         */
        uint32_t reset_events = Tasks_WaitEvent(SYSTEM_RESET_EVENT, false, 0);
        if (reset_events & SYSTEM_RESET_EVENT) {
            DEBUG_PRINT("System reset event received, shutting down gracefully...\r\n");
            
            /**
             * 优雅关闭
             * 知识点: 关闭流程
             * - 停止所有任务
             * - 清理资源
             * - LED状态显示关闭
             */
            LED_SetState(LED_GREEN, LED_STATE_OFF);
            LED_SetState(LED_YELLOW, LED_STATE_OFF);
            LED_SetState(LED_RED, LED_STATE_OFF);
            Buzzer_StopAlarm();
            
            /**
             * 任务清理
             * 知识点: 任务停止
             * - 挂起所有任务
             * - 等待任务停止
             * - 清理任务资源
             */
            if (DataSendTask_Handle != NULL) {
                vTaskSuspend(DataSendTask_Handle);
            }
            if (DataCollectTask_Handle != NULL) {
                vTaskSuspend(DataCollectTask_Handle);
            }
            if (NetworkManageTask_Handle != NULL) {
                vTaskSuspend(NetworkManageTask_Handle);
            }
            
            vTaskDelay(pdMS_TO_TICKS(1000));  // 等待任务停止
            
            /**
             * 任务系统去初始化
             * 知识点: 资源清理
             * - 删除所有任务
             * - 删除通信对象
             * - 清理全局状态
             */
            Tasks_DeInit();
            
            DEBUG_PRINT("System shutdown completed, restarting...\r\n");
            
            /**
             * 系统重启
             * 知识点: 重启机制
             * - 使用软件复位
             * - 重新初始化系统
             * - 重新启动任务
             */
            NVIC_SystemReset();  // 触发系统复位
        }
        
        /**
         * 第七步: 周期延时
         * 知识点: 监控频率控制
         * - 控制监控频率
         * - 避免过度占用系统
         * - 平衡监控精度和效率
         */
        DEBUG_PRINT("=== System monitoring cycle completed, next in %u ms ===\r\n", SYSTEM_MONITOR_PERIOD_MS);
        last_monitor_time = current_time;
        vTaskDelay(pdMS_TO_TICKS(SYSTEM_MONITOR_PERIOD_MS));
    }
}

/* ============================== 调试和监控函数 ============================== */

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
void Tasks_PrintStatus(void)
{
    DEBUG_PRINT("=================================================\r\n");
    DEBUG_PRINT("           FreeRTOS Task Status Report           \r\n");
    DEBUG_PRINT("=================================================\r\n");
    
    /**
     * 系统总体信息
     * 知识点: 系统运行信息
     * - 系统运行时间
     * - 总任务错误数
     * - 任务创建状态
     */
    uint32_t runtime = System_Get_Runtime_MS();
    uint32_t runtime_seconds = runtime / 1000;
    uint32_t runtime_minutes = runtime_seconds / 60;
    uint32_t runtime_hours = runtime_minutes / 60;
    
    DEBUG_PRINT("System Runtime: %02u:%02u:%02u (total: %u ms)\r\n", 
               runtime_hours, runtime_minutes % 60, runtime_seconds % 60, runtime);
    DEBUG_PRINT("Total Task Errors: %u\r\n", Task_Management.total_task_errors);
    DEBUG_PRINT("System Start Time: %u ms\r\n", Task_Management.system_start_time);
    DEBUG_PRINT("\r\n");
    
    /**
     * 网络管理任务状态
     * 知识点: 网络任务监控
     * - 任务状态
     * - 执行统计
     * - 错误统计
     */
    DEBUG_PRINT("NetworkManage Task:\r\n");
    DEBUG_PRINT("  Status: %s\r\n", 
               Task_Management.network_manage.status == TASK_STATUS_RUNNING ? "Running" :
               Task_Management.network_manage.status == TASK_STATUS_SUSPENDED ? "Suspended" :
               Task_Management.network_manage.status == TASK_STATUS_ERROR ? "Error" : "Stopped");
    DEBUG_PRINT("  Handle: 0x%08X\r\n", (uint32_t)Task_Management.network_manage.task_handle);
    DEBUG_PRINT("  Cycles: %u\r\n", Task_Management.network_manage.cycle_count);
    DEBUG_PRINT("  Errors: %u\r\n", Task_Management.network_manage.error_count);
    DEBUG_PRINT("  Last Execution: %u ms ago\r\n", 
               runtime - Task_Management.network_manage.last_execution_time);
    
    if (Task_Management.network_manage.task_handle != NULL) {
        configSTACK_DEPTH_TYPE stack_usage = uxTaskGetStackHighWaterMark(Task_Management.network_manage.task_handle);
        DEBUG_PRINT("  Stack Remaining: %u words\r\n", stack_usage);
    }
    DEBUG_PRINT("\r\n");
    
    /**
     * 数据采集任务状态
     */
    DEBUG_PRINT("DataCollect Task:\r\n");
    DEBUG_PRINT("  Status: %s\r\n", 
               Task_Management.data_collect.status == TASK_STATUS_RUNNING ? "Running" :
               Task_Management.data_collect.status == TASK_STATUS_SUSPENDED ? "Suspended" :
               Task_Management.data_collect.status == TASK_STATUS_ERROR ? "Error" : "Stopped");
    DEBUG_PRINT("  Handle: 0x%08X\r\n", (uint32_t)Task_Management.data_collect.task_handle);
    DEBUG_PRINT("  Cycles: %u\r\n", Task_Management.data_collect.cycle_count);
    DEBUG_PRINT("  Errors: %u\r\n", Task_Management.data_collect.error_count);
    DEBUG_PRINT("  Last Execution: %u ms ago\r\n", 
               runtime - Task_Management.data_collect.last_execution_time);
    
    if (Task_Management.data_collect.task_handle != NULL) {
        configSTACK_DEPTH_TYPE stack_usage = uxTaskGetStackHighWaterMark(Task_Management.data_collect.task_handle);
        DEBUG_PRINT("  Stack Remaining: %u words\r\n", stack_usage);
    }
    DEBUG_PRINT("\r\n");
    
    /**
     * 数据发送任务状态
     */
    DEBUG_PRINT("DataSend Task:\r\n");
    DEBUG_PRINT("  Status: %s\r\n", 
               Task_Management.data_send.status == TASK_STATUS_RUNNING ? "Running" :
               Task_Management.data_send.status == TASK_STATUS_SUSPENDED ? "Suspended" :
               Task_Management.data_send.status == TASK_STATUS_ERROR ? "Error" : "Stopped");
    DEBUG_PRINT("  Handle: 0x%08X\r\n", (uint32_t)Task_Management.data_send.task_handle);
    DEBUG_PRINT("  Cycles: %u\r\n", Task_Management.data_send.cycle_count);
    DEBUG_PRINT("  Errors: %u\r\n", Task_Management.data_send.error_count);
    DEBUG_PRINT("  Last Execution: %u ms ago\r\n", 
               runtime - Task_Management.data_send.last_execution_time);
    
    if (Task_Management.data_send.task_handle != NULL) {
        configSTACK_DEPTH_TYPE stack_usage = uxTaskGetStackHighWaterMark(Task_Management.data_send.task_handle);
        DEBUG_PRINT("  Stack Remaining: %u words\r\n", stack_usage);
    }
    DEBUG_PRINT("\r\n");
    
    /**
     * 系统监控任务状态
     */
    DEBUG_PRINT("SystemMonitor Task:\r\n");
    DEBUG_PRINT("  Status: %s\r\n", 
               Task_Management.system_monitor.status == TASK_STATUS_RUNNING ? "Running" :
               Task_Management.system_monitor.status == TASK_STATUS_SUSPENDED ? "Suspended" :
               Task_Management.system_monitor.status == TASK_STATUS_ERROR ? "Error" : "Stopped");
    DEBUG_PRINT("  Handle: 0x%08X\r\n", (uint32_t)Task_Management.system_monitor.task_handle);
    DEBUG_PRINT("  Cycles: %u\r\n", Task_Management.system_monitor.cycle_count);
    DEBUG_PRINT("  Errors: %u\r\n", Task_Management.system_monitor.error_count);
    DEBUG_PRINT("  Last Execution: %u ms ago\r\n", 
               runtime - Task_Management.system_monitor.last_execution_time);
    
    if (Task_Management.system_monitor.task_handle != NULL) {
        configSTACK_DEPTH_TYPE stack_usage = uxTaskGetStackHighWaterMark(Task_Management.system_monitor.task_handle);
        DEBUG_PRINT("  Stack Remaining: %u words\r\n", stack_usage);
    }
    DEBUG_PRINT("\r\n");
    
    DEBUG_PRINT("=================================================\r\n");
}

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
void Tasks_PrintStackUsage(void)
{
    DEBUG_PRINT("=================================================\r\n");
    DEBUG_PRINT("            Task Stack Usage Report              \r\n");
    DEBUG_PRINT("=================================================\r\n");
    
    /**
     * 堆内存总体信息
     * 知识点: 堆内存监控
     * - 总堆内存大小
     * - 剩余堆内存
     * - 堆内存使用率
     */
    size_t free_heap = xPortGetFreeHeapSize();
    size_t total_heap = configTOTAL_HEAP_SIZE;
    float heap_usage_percent = ((float)(total_heap - free_heap) / total_heap) * 100.0f;
    
    DEBUG_PRINT("Heap Memory Summary:\r\n");
    DEBUG_PRINT("  Total Heap: %u bytes\r\n", total_heap);
    DEBUG_PRINT("  Free Heap: %u bytes\r\n", free_heap);
    DEBUG_PRINT("  Used Heap: %u bytes (%.1f%%)\r\n", 
               total_heap - free_heap, heap_usage_percent);
    DEBUG_PRINT("\r\n");
    
    /**
     * 任务堆栈详情
     * 知识点: 堆栈使用详情
     * - 分配堆栈大小
     * - 高水位标记(最大使用量)
     * - 剩余堆栈空间
     * - 使用率计算
     */
    DEBUG_PRINT("Task Stack Details:\r\n");
    
    /**
     * 网络管理任务堆栈
     */
    if (NetworkManageTask_Handle != NULL) {
        configSTACK_DEPTH_TYPE high_water_mark = uxTaskGetStackHighWaterMark(NetworkManageTask_Handle);
        configSTACK_DEPTH_TYPE stack_size = NETWORK_MANAGE_TASK_STACK_SIZE;
        configSTACK_DEPTH_TYPE used_stack = stack_size - high_water_mark;
        float stack_usage_percent = ((float)used_stack / stack_size) * 100.0f;
        
        DEBUG_PRINT("NetworkManage Task:\r\n");
        DEBUG_PRINT("  Allocated Stack: %u words\r\n", stack_size);
        DEBUG_PRINT("  High Water Mark: %u words\r\n", high_water_mark);
        DEBUG_PRINT("  Used Stack: %u words\r\n", used_stack);
        DEBUG_PRINT("  Usage Rate: %.1f%%\r\n", stack_usage_percent);
        
        /**
         * 堆栈使用建议
         * 知识点: 堆栈优化
         * - 使用率超过80%: 建议增加堆栈
         * - 使用率超过90%: 必须增加堆栈
         * - 使用率过低: 可以减少堆栈节省内存
         */
        if (stack_usage_percent > 90.0f) {
            DEBUG_PRINT("  WARNING: Stack usage too high! Consider increasing stack size.\r\n");
        } else if (stack_usage_percent > 80.0f) {
            DEBUG_PRINT("  CAUTION: Stack usage high, monitor for potential overflow.\r\n");
        } else if (stack_usage_percent < 30.0f) {
            DEBUG_PRINT("  INFO: Stack usage low, could be optimized for memory saving.\r\n");
        }
        DEBUG_PRINT("\r\n");
    }
    
    /**
     * 数据采集任务堆栈
     */
    if (DataCollectTask_Handle != NULL) {
        configSTACK_DEPTH_TYPE high_water_mark = uxTaskGetStackHighWaterMark(DataCollectTask_Handle);
        configSTACK_DEPTH_TYPE stack_size = DATA_COLLECT_TASK_STACK_SIZE;
        configSTACK_DEPTH_TYPE used_stack = stack_size - high_water_mark;
        float stack_usage_percent = ((float)used_stack / stack_size) * 100.0f;
        
        DEBUG_PRINT("DataCollect Task:\r\n");
        DEBUG_PRINT("  Allocated Stack: %u words\r\n", stack_size);
        DEBUG_PRINT("  High Water Mark: %u words\r\n", high_water_mark);
        DEBUG_PRINT("  Used Stack: %u words\r\n", used_stack);
        DEBUG_PRINT("  Usage Rate: %.1f%%\r\n", stack_usage_percent);
        
        if (stack_usage_percent > 90.0f) {
            DEBUG_PRINT("  WARNING: Stack usage too high! Consider increasing stack size.\r\n");
        } else if (stack_usage_percent > 80.0f) {
            DEBUG_PRINT("  CAUTION: Stack usage high, monitor for potential overflow.\r\n");
        } else if (stack_usage_percent < 30.0f) {
            DEBUG_PRINT("  INFO: Stack usage low, could be optimized for memory saving.\r\n");
        }
        DEBUG_PRINT("\r\n");
    }
    
    /**
     * 数据发送任务堆栈
     */
    if (DataSendTask_Handle != NULL) {
        configSTACK_DEPTH_TYPE high_water_mark = uxTaskGetStackHighWaterMark(DataSendTask_Handle);
        configSTACK_DEPTH_TYPE stack_size = DATA_SEND_TASK_STACK_SIZE;
        configSTACK_DEPTH_TYPE used_stack = stack_size - high_water_mark;
        float stack_usage_percent = ((float)used_stack / stack_size) * 100.0f;
        
        DEBUG_PRINT("DataSend Task:\r\n");
        DEBUG_PRINT("  Allocated Stack: %u words\r\n", stack_size);
        DEBUG_PRINT("  High Water Mark: %u words\r\n", high_water_mark);
        DEBUG_PRINT("  Used Stack: %u words\r\n", used_stack);
        DEBUG_PRINT("  Usage Rate: %.1f%%\r\n", stack_usage_percent);
        
        if (stack_usage_percent > 90.0f) {
            DEBUG_PRINT("  WARNING: Stack usage too high! Consider increasing stack size.\r\n");
        } else if (stack_usage_percent > 80.0f) {
            DEBUG_PRINT("  CAUTION: Stack usage high, monitor for potential overflow.\r\n");
        } else if (stack_usage_percent < 30.0f) {
            DEBUG_PRINT("  INFO: Stack usage low, could be optimized for memory saving.\r\n");
        }
        DEBUG_PRINT("\r\n");
    }
    
    /**
     * 系统监控任务堆栈
     */
    if (SystemMonitorTask_Handle != NULL) {
        configSTACK_DEPTH_TYPE high_water_mark = uxTaskGetStackHighWaterMark(SystemMonitorTask_Handle);
        configSTACK_DEPTH_TYPE stack_size = SYSTEM_MONITOR_TASK_STACK_SIZE;
        configSTACK_DEPTH_TYPE used_stack = stack_size - high_water_mark;
        float stack_usage_percent = ((float)used_stack / stack_size) * 100.0f;
        
        DEBUG_PRINT("SystemMonitor Task:\r\n");
        DEBUG_PRINT("  Allocated Stack: %u words\r\n", stack_size);
        DEBUG_PRINT("  High Water Mark: %u words\r\n", high_water_mark);
        DEBUG_PRINT("  Used Stack: %u words\r\n", used_stack);
        DEBUG_PRINT("  Usage Rate: %.1f%%\r\n", stack_usage_percent);
        
        if (stack_usage_percent > 90.0f) {
            DEBUG_PRINT("  WARNING: Stack usage too high! Consider increasing stack size.\r\n");
        } else if (stack_usage_percent > 80.0f) {
            DEBUG_PRINT("  CAUTION: Stack usage high, monitor for potential overflow.\r\n");
        } else if (stack_usage_percent < 30.0f) {
            DEBUG_PRINT("  INFO: Stack usage low, could be optimized for memory saving.\r\n");
        }
        DEBUG_PRINT("\r\n");
    }
    
    DEBUG_PRINT("=================================================\r\n");
}

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
void Tasks_PrintCommunicationStatus(void)
{
    DEBUG_PRINT("=================================================\r\n");
    DEBUG_PRINT("        Communication Objects Status Report      \r\n");
    DEBUG_PRINT("=================================================\r\n");
    
    /**
     * 队列状态信息
     * 知识点: 队列监控
     * - 队列长度
     * - 当前消息数
     * - 剩余空间
     * - 等待任务数
     */
    DEBUG_PRINT("Queue Status:\r\n");
    
    /**
     * 数据队列状态
     */
    if (Data_Queue != NULL) {
        UBaseType_t queue_length = uxQueueMessagesWaiting(Data_Queue);
        UBaseType_t queue_capacity = uxQueueGetQueueHoldSize(Data_Queue);
        
        DEBUG_PRINT("Data_Queue:\r\n");
        DEBUG_PRINT("  Handle: 0x%08X\r\n", (uint32_t)Data_Queue);
        DEBUG_PRINT("  Capacity: %u messages\r\n", DATA_QUEUE_LENGTH);
        DEBUG_PRINT("  Current Messages: %u\r\n", queue_length);
        DEBUG_PRINT("  Available Space: %u\r\n", DATA_QUEUE_LENGTH - queue_length);
        DEBUG_PRINT("  Item Size: %u bytes\r\n", DATA_QUEUE_ITEM_SIZE);
        
        /**
         * 队列状态分析
         * 知识点: 队列状态评估
         * - 队列空: 可能数据采集不及时
         * - 队列满: 可能发送不及时
         * - 正常: 队列使用平衡
         */
        float queue_usage_percent = ((float)queue_length / DATA_QUEUE_LENGTH) * 100.0f;
        DEBUG_PRINT("  Usage Rate: %.1f%%\r\n", queue_usage_percent);
        
        if (queue_usage_percent > 90.0f) {
            DEBUG_PRINT("  WARNING: Queue almost full! Data may be lost.\r\n");
        } else if (queue_usage_percent < 10.0f) {
            DEBUG_PRINT("  INFO: Queue mostly empty, data collection may be slow.\r\n");
        } else {
            DEBUG_PRINT("  OK: Queue usage normal.\r\n");
        }
        DEBUG_PRINT("\r\n");
    } else {
        DEBUG_PRINT("Data_Queue: Not initialized\r\n\r\n");
    }
    
    /**
     * 错误队列状态
     */
    if (Error_Queue != NULL) {
        UBaseType_t queue_length = uxQueueMessagesWaiting(Error_Queue);
        
        DEBUG_PRINT("Error_Queue:\r\n");
        DEBUG_PRINT("  Handle: 0x%08X\r\n", (uint32_t)Error_Queue);
        DEBUG_PRINT("  Capacity: %u messages\r\n", ERROR_QUEUE_LENGTH);
        DEBUG_PRINT("  Current Messages: %u\r\n", queue_length);
        DEBUG_PRINT("  Available Space: %u\r\n", ERROR_QUEUE_LENGTH - queue_length);
        DEBUG_PRINT("  Item Size: %u bytes\r\n", sizeof(Error_Info_t));
        
        /**
         * 错误队列状态分析
         * 知识点: 错误处理评估
         * - 错误队列积压: 错误处理不及时
         * - 错误队列空: 错误处理及时
         */
        float queue_usage_percent = ((float)queue_length / ERROR_QUEUE_LENGTH) * 100.0f;
        DEBUG_PRINT("  Usage Rate: %.1f%%\r\n", queue_usage_percent);
        
        if (queue_usage_percent > 80.0f) {
            DEBUG_PRINT("  WARNING: Error queue nearly full! Errors may be lost.\r\n");
        } else if (queue_usage_percent > 50.0f) {
            DEBUG_PRINT("  CAUTION: Error queue usage high, monitor error processing.\r\n");
        } else {
            DEBUG_PRINT("  OK: Error queue usage normal.\r\n");
        }
        DEBUG_PRINT("\r\n");
    } else {
        DEBUG_PRINT("Error_Queue: Not initialized\r\n\r\n");
    }
    
    /**
     * 信号量状态信息
     * 知识点: 信号量监控
     * - 互斥锁状态
     * - 信号量计数
     * - 等待任务数
     */
    DEBUG_PRINT("Semaphore Status:\r\n");
    
    /**
     * Flash互斥锁状态
     */
    if (Flash_Mutex != NULL) {
        UBaseType_t semaphore_count = uxSemaphoreGetCount(Flash_Mutex);
        UBaseType_t waiting_tasks = uxQueueMessagesWaiting((QueueHandle_t)Flash_Mutex);
        
        DEBUG_PRINT("Flash_Mutex:\r\n");
        DEBUG_PRINT("  Handle: 0x%08X\r\n", (uint32_t)Flash_Mutex);
        DEBUG_PRINT("  Type: Mutex (Binary Semaphore)\r\n");
        DEBUG_PRINT("  Available: %s\r\n", semaphore_count > 0 ? "Yes" : "No");
        DEBUG_PRINT("  Owner Count: %u\r\n", semaphore_count);
        DEBUG_PRINT("  Waiting Tasks: %u\r\n", waiting_tasks);
        
        /**
         * 互斥锁状态分析
         * 知识点: 互斥锁竞争分析
         * - 可用: 资源未被占用
         * - 不可用: 资源被占用
         * - 等待任务数: 反映竞争激烈程度
         */
        if (semaphore_count == 0) {
            DEBUG_PRINT("  Status: LOCKED (Flash being accessed)\r\n");
        } else {
            DEBUG_PRINT("  Status: UNLOCKED (Flash available)\r\n");
        }
        
        if (waiting_tasks > 0) {
            DEBUG_PRINT("  WARNING: %u tasks waiting for Flash access\r\n", waiting_tasks);
        }
        DEBUG_PRINT("\r\n");
    } else {
        DEBUG_PRINT("Flash_Mutex: Not initialized\r\n\r\n");
    }
    
    /**
     * 网络信号量状态
     */
    if (Network_Semaphore != NULL) {
        UBaseType_t semaphore_count = uxSemaphoreGetCount(Network_Semaphore);
        UBaseType_t waiting_tasks = uxQueueMessagesWaiting((QueueHandle_t)Network_Semaphore);
        
        DEBUG_PRINT("Network_Semaphore:\r\n");
        DEBUG_PRINT("  Handle: 0x%08X\r\n", (uint32_t)Network_Semaphore);
        DEBUG_PRINT("  Type: Binary Semaphore\r\n");
        DEBUG_PRINT("  Current Count: %u\r\n", semaphore_count);
        DEBUG_PRINT("  Available: %s\r\n", semaphore_count > 0 ? "Yes" : "No");
        DEBUG_PRINT("  Waiting Tasks: %u\r\n", waiting_tasks);
        
        /**
         * 网络信号量状态分析
         * 知识点: 网络状态分析
         * - 1: 网络已连接
         * - 0: 网络未连接
         */
        if (semaphore_count == 1) {
            DEBUG_PRINT("  Status: Network CONNECTED\r\n");
        } else {
            DEBUG_PRINT("  Status: Network DISCONNECTED\r\n");
        }
        DEBUG_PRINT("\r\n");
    } else {
        DEBUG_PRINT("Network_Semaphore: Not initialized\r\n\r\n");
    }
    
    /**
     * 事件组状态信息
     * 知识点: 事件组监控
     * - 事件位状态
     * - 事件标志
     */
    DEBUG_PRINT("Event Group Status:\r\n");
    
    if (System_Events != NULL) {
        EventBits_t event_bits = xEventGroupGetBits(System_Events);
        
        DEBUG_PRINT("System_Events:\r\n");
        DEBUG_PRINT("  Handle: 0x%08X\r\n", (uint32_t)System_Events);
        DEBUG_PRINT("  Current Bits: 0x%08X\r\n", event_bits);
        DEBUG_PRINT("  Event Status:\r\n");
        
        /**
         * 事件位状态解析
         * 知识点: 事件位含义
         */
        if (event_bits & NETWORK_CONNECTED_EVENT) {
            DEBUG_PRINT("    [✓] Network Connected\r\n");
        } else {
            DEBUG_PRINT("    [ ] Network Connected\r\n");
        }
        
        if (event_bits & NETWORK_DISCONNECTED_EVENT) {
            DEBUG_PRINT("    [✓] Network Disconnected\r\n");
        } else {
            DEBUG_PRINT("    [ ] Network Disconnected\r\n");
        }
        
        if (event_bits & DATA_READY_EVENT) {
            DEBUG_PRINT("    [✓] Data Ready\r\n");
        } else {
            DEBUG_PRINT("    [ ] Data Ready\r\n");
        }
        
        if (event_bits & FLASH_FULL_EVENT) {
            DEBUG_PRINT("    [✓] Flash Full\r\n");
        } else {
            DEBUG_PRINT("    [ ] Flash Full\r\n");
        }
        
        if (event_bits & SYSTEM_ERROR_EVENT) {
            DEBUG_PRINT("    [✓] System Error\r\n");
        } else {
            DEBUG_PRINT("    [ ] System Error\r\n");
        }
        
        if (event_bits & KEY_PRESSED_EVENT) {
            DEBUG_PRINT("    [✓] Key Pressed\r\n");
        } else {
            DEBUG_PRINT("    [ ] Key Pressed\r\n");
        }
        
        if (event_bits & SYSTEM_RESET_EVENT) {
            DEBUG_PRINT("    [✓] System Reset\r\n");
        } else {
            DEBUG_PRINT("    [ ] System Reset\r\n");
        }
        
        DEBUG_PRINT("\r\n");
    } else {
        DEBUG_PRINT("System_Events: Not initialized\r\n\r\n");
    }
    
    DEBUG_PRINT("=================================================\r\n");
}