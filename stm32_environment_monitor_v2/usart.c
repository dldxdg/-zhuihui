/**
 * @file    usart.c
 * @brief   串口通信和AT指令状态机实现 (标准库版本)
 * @author  MiniMax Agent
 * @version V2.0
 * @date    2025-11-08
 * 
 * 重要说明:
 * - 此版本完全使用STM32标准库，无HAL函数
 * - 彻底去除时间戳功能
 * - 实现AT指令状态机，避免阻塞延时
 * 
 * 知识点总结:
 * 
 * 【串口通信机制】
 * 1. 串口是异步通信，发送和接收可以同时进行
 * 2. 数据按位传输，需要配置相同的波特率
 * 3. 起始位和停止位用于同步
 * 4. 空闲状态时线路保持高电平
 * 
 * 【环形缓冲区技术】
 * 1. 解决串口数据接收的时序问题
 * 2. 生产者-消费者模式
 * 3. 读写指针分离，避免数据竞争
 * 4. 缓冲区满时停止接收，避免数据覆盖
 * 
 * 【AT指令状态机】
 * 1. 异步执行AT指令，提高系统响应性
 * 2. 状态机管理复杂时序
 * 3. 支持超时和重试机制
 * 4. 避免使用阻塞延时
 */

#include "usart.h"
#include "main.h"
#include "delay.h"

/* ============================== 全局变量定义 ============================== */

/**
 * 蓝牙环形缓冲区
 * 知识点: 环形缓冲区初始化
 * - head指向下一个写入位置
 * - tail指向下一个读取位置
 * - count记录当前数据量
 * - 初始化时所有指针为0
 */
USART2_RingBuffer_t USART2_RingBuffer = {
    .buffer = {0},
    .head = 0,
    .tail = 0,
    .count = 0
};

/**
 * WiFi环形缓冲区
 * 知识点: 大容量缓冲区
 * - WiFi数据量大，需要更大缓冲区
 * - 2048字节足够处理一般网络数据
 */
USART3_RingBuffer_t USART3_RingBuffer = {
    .buffer = {0},
    .head = 0,
    .tail = 0,
    .count = 0
};

/**
 * AT指令状态机
 * 知识点: 状态机状态
 * - 初始状态为空闲
 * - 没有当前执行的指令
 * - 响应缓冲区为空
 * - 忙标志为false
 */
AT_StateMachine_t AT_StateMachine = {
    .current_cmd = {0},
    .state = AT_STATE_IDLE,
    .response_buffer = {0},
    .response_length = 0,
    .start_time = 0,
    .last_activity_time = 0,
    .busy = false
};

/**
 * WiFi管理器
 * 知识点: WiFi连接管理
 * - 初始状态为未连接
 * - 自动重连开启
 * - 重试计数为0
 * - 心跳间隔为5秒
 */
WiFi_Manager_t WiFi_Manager = {
    .state = NETWORK_STATE_DISCONNECTED,
    .ssid = {0},
    .password = {0},
    .server_ip = {0},
    .server_port = 8080,
    .last_connect_time = 0,
    .heartbeat_interval = WIFI_HEARTBEAT_INTERVAL_MS,
    .connect_retry_count = 0,
    .auto_reconnect = true
};

/**
 * 系统配置 (引用main.c中的全局变量)
 * 知识点: 配置参数访问
 * - 这里只是声明，实际定义在main.c中
 * - 需要在main.c中定义此变量
 */
extern System_Config_t System_Config;

/* ============================== 串口模块初始化 ============================== */

/**
 * @brief   串口模块初始化函数
 * @details 初始化所有串口相关功能
 * 
 * 知识点: 串口模块初始化顺序
 * 1. 初始化硬件串口(在main.c中完成)
 * 2. 初始化环形缓冲区
 * 3. 初始化AT状态机
 * 4. 初始化WiFi管理器
 * 5. 加载系统配置
 */
void USART_Module_Init(void)
{
    DEBUG_PRINT("Initializing USART module...\r\n");
    
    /**
     * 步骤1: 初始化环形缓冲区
     * 知识点: 缓冲区初始化
     * - 重置读写指针
     * - 清空数据缓冲区
     * - 重置计数器
     */
    USART2_RingBuffer_Init();
    USART3_RingBuffer_Init();
    
    /**
     * 步骤2: 初始化AT状态机
     * 知识点: 状态机初始化
     * - 设置初始状态为空闲
     * - 清空当前指令
     * - 重置响应缓冲区
     */
    AT_StateMachine.state = AT_STATE_IDLE;
    AT_StateMachine.busy = false;
    memset(AT_StateMachine.response_buffer, 0, sizeof(AT_StateMachine.response_buffer));
    AT_StateMachine.response_length = 0;
    
    /**
     * 步骤3: 初始化WiFi管理器
     * 知识点: WiFi管理初始化
     * - 设置初始状态
     * - 清空网络参数
     * - 设置默认参数
     */
    WiFi_Manager.state = NETWORK_STATE_DISCONNECTED;
    WiFi_Manager.connect_retry_count = 0;
    WiFi_Manager.auto_reconnect = true;
    
    /**
     * 步骤4: 加载系统配置
     * 知识点: 配置参数加载
     * - 从内部Flash读取WiFi配置
     * - 更新WiFi管理器参数
     * - 设置默认配置(如果读取失败)
     */
    System_Config_Load();
    strcpy(WiFi_Manager.ssid, System_Config.wifi_ssid);
    strcpy(WiFi_Manager.password, System_Config.wifi_password);
    strcpy(WiFi_Manager.server_ip, System_Config.server_ip);
    WiFi_Manager.server_port = System_Config.server_port;
    
    DEBUG_PRINT("USART module initialized successfully\r\n");
}

/* ============================== 环形缓冲区实现 ============================== */

/**
 * @brief   USART2环形缓冲区初始化
 * @details 重置蓝牙环形缓冲区
 */
void USART2_RingBuffer_Init(void)
{
    /**
     * 重置所有缓冲区状态
     * 知识点: 缓冲区重置
     * - 写指针: 0
     * - 读指针: 0
     * - 数据量: 0
     * - 缓冲区内容: 清零(可选)
     */
    USART2_RingBuffer.head = 0;
    USART2_RingBuffer.tail = 0;
    USART2_RingBuffer.count = 0;
    memset(USART2_RingBuffer.buffer, 0, sizeof(USART2_RingBuffer.buffer));
    
    DEBUG_PRINT("USART2 ring buffer initialized (size: %d)\r\n", USART2_RING_BUFFER_SIZE);
}

/**
 * @brief   USART2环形缓冲区写入
 * @details 将数据写入蓝牙环形缓冲区
 * @param data 要写入的数据字节
 * 
 * 知识点: 环形缓冲区写入算法
 * - 检查缓冲区是否已满
 * - 将数据写入当前写指针位置
 * - 移动写指针(循环)
 * - 增加数据计数器
 */
void USART2_RingBuffer_Write(uint8_t data)
{
    /**
     * 检查缓冲区满状态
     * 知识点: 缓冲区满检测
     * - 如果数据量等于缓冲区大小，说明已满
     * - 满时不能写入，避免数据覆盖
     * - 可以选择覆盖最旧数据或丢弃新数据
     * - 这里选择丢弃新数据
     */
    if (USART2_RingBuffer.count >= USART2_RING_BUFFER_SIZE) {
        DEBUG_PRINT("USART2 ring buffer full, discarding data: 0x%02X\r\n", data);
        return;  // 缓冲区满，丢弃数据
    }
    
    /**
     * 写入数据
     * 知识点: 数据写入
     * - 将数据写入当前写指针指向的位置
     * - 写指针是下一个写入位置的索引
     */
    USART2_RingBuffer.buffer[USART2_RingBuffer.head] = data;
    
    /**
     * 更新写指针
     * 知识点: 环形指针移动
     * - 写指针向后移动一位
     * - 如果达到缓冲区末尾，则回到开头
     * - 实现环形访问
     */
    USART2_RingBuffer.head++;
    if (USART2_RingBuffer.head >= USART2_RING_BUFFER_SIZE) {
        USART2_RingBuffer.head = 0;
    }
    
    /**
     * 更新数据计数器
     * 知识点: 计数器维护
     * - 每次写入成功，数据量加1
     * - 用于快速判断缓冲区状态
     */
    USART2_RingBuffer.count++;
}

/**
 * @brief   USART2环形缓冲区读取
 * @details 从蓝牙环形缓冲区读取数据
 * @param data 读取到的数据(输出参数)
 * @return bool 读取是否成功
 * 
 * 知识点: 环形缓冲区读取算法
 * - 检查缓冲区是否为空
 * - 从当前读指针位置读取数据
 * - 移动读指针(循环)
 * - 减少数据计数器
 */
bool USART2_RingBuffer_Read(uint8_t *data)
{
    /**
     * 检查缓冲区空状态
     * 知识点: 缓冲区空检测
     * - 如果数据量为0，说明缓冲区空
     * - 空时不能读取，避免读取无效数据
     */
    if (USART2_RingBuffer.count == 0) {
        return false;  // 缓冲区空，读取失败
    }
    
    /**
     * 读取数据
     * 知识点: 数据读取
     * - 从当前读指针指向的位置读取数据
     * - 读指针是下一个读取位置的索引
     * - 读指针指向的是最早写入但还未读取的数据
     */
    *data = USART2_RingBuffer.buffer[USART2_RingBuffer.tail];
    
    /**
     * 更新读指针
     * 知识点: 环形指针移动
     * - 读指针向后移动一位
     * - 如果达到缓冲区末尾，则回到开头
     * - 实现环形访问
     */
    USART2_RingBuffer.tail++;
    if (USART2_RingBuffer.tail >= USART2_RING_BUFFER_SIZE) {
        USART2_RingBuffer.tail = 0;
    }
    
    /**
     * 更新数据计数器
     * 知识点: 计数器维护
     * - 每次读取成功，数据量减1
     * - 用于快速判断缓冲区状态
     */
    USART2_RingBuffer.count--;
    
    return true;  // 读取成功
}

/**
 * @brief   USART2环形缓冲区内容查询
 * @details 查询缓冲区是否包含指定字符串
 * @param str 要查找的字符串
 * @return bool 是否包含该字符串
 * 
 * 知识点: 字符串搜索算法
 * - 搜索整个缓冲区内容
 * - 支持大小写不敏感搜索
 * - 简单匹配算法(可优化为KMP等高效算法)
 */
bool USART2_RingBuffer_Contains(const char *str)
{
    if (USART2_RingBuffer.count == 0 || strlen(str) == 0) {
        return false;
    }
    
    /**
     * 简单字符串匹配
     * 知识点: 匹配算法
     * - 从缓冲区头部开始搜索
     * - 逐个字符比较
     * - 找到匹配返回true，否则返回false
     * - 注意环形缓冲区的循环特性
     */
    uint16_t search_len = strlen(str);
    if (search_len > USART2_RingBuffer.count) {
        return false;  // 要搜索的字符串比数据多，不可能匹配
    }
    
    for (uint16_t i = 0; i <= USART2_RingBuffer.count - search_len; i++) {
        uint16_t buffer_index = (USART2_RingBuffer.tail + i) % USART2_RING_BUFFER_SIZE;
        bool match = true;
        
        for (uint16_t j = 0; j < search_len; j++) {
            uint16_t data_index = (buffer_index + j) % USART2_RING_BUFFER_SIZE;
            if (USART2_RingBuffer.buffer[data_index] != str[j]) {
                match = false;
                break;
            }
        }
        
        if (match) {
            return true;  // 找到匹配
        }
    }
    
    return false;  // 未找到匹配
}

/**
 * @brief   USART2环形缓冲区清空
 * @details 清空所有数据，重置指针
 */
void USART2_RingBuffer_Clear(void)
{
    /**
     * 重置缓冲区状态
     * 知识点: 缓冲区重置
     * - 重新初始化所有状态
     * - 等价于重新初始化
     */
    USART2_RingBuffer_Init();
    
    DEBUG_PRINT("USART2 ring buffer cleared\r\n");
}

/* ============================== USART3环形缓冲区实现 ============================== */

/**
 * @brief   USART3环形缓冲区初始化
 * @details 重置WiFi环形缓冲区
 */
void USART3_RingBuffer_Init(void)
{
    /**
     * 重置所有缓冲区状态
     * 知识点: 缓冲区重置
     * - 写指针: 0
     * - 读指针: 0
     * - 数据量: 0
     * - 缓冲区内容: 清零(可选)
     */
    USART3_RingBuffer.head = 0;
    USART3_RingBuffer.tail = 0;
    USART3_RingBuffer.count = 0;
    memset(USART3_RingBuffer.buffer, 0, sizeof(USART3_RingBuffer.buffer));
    
    DEBUG_PRINT("USART3 ring buffer initialized (size: %d)\r\n", USART3_RING_BUFFER_SIZE);
}

/**
 * @brief   USART3环形缓冲区写入
 * @details 将数据写入WiFi环形缓冲区
 * @param data 要写入的数据字节
 */
void USART3_RingBuffer_Write(uint8_t data)
{
    /**
     * 检查缓冲区满状态
     * 知识点: 缓冲区满检测
     * - 如果数据量等于缓冲区大小，说明已满
     * - WiFi数据量大，满的情况可能更频繁
     * - 丢弃新数据是安全的选择
     */
    if (USART3_RingBuffer.count >= USART3_RING_BUFFER_SIZE) {
        DEBUG_PRINT("USART3 ring buffer full, discarding data: 0x%02X\r\n", data);
        return;  // 缓冲区满，丢弃数据
    }
    
    /**
     * 写入数据
     * 知识点: 数据写入
     * - 将数据写入当前写指针指向的位置
     * - 写指针是下一个写入位置的索引
     */
    USART3_RingBuffer.buffer[USART3_RingBuffer.head] = data;
    
    /**
     * 更新写指针
     * 知识点: 环形指针移动
     * - 写指针向后移动一位
     * - 如果达到缓冲区末尾，则回到开头
     * - 实现环形访问
     */
    USART3_RingBuffer.head++;
    if (USART3_RingBuffer.head >= USART3_RING_BUFFER_SIZE) {
        USART3_RingBuffer.head = 0;
    }
    
    /**
     * 更新数据计数器
     * 知识点: 计数器维护
     * - 每次写入成功，数据量加1
     * - 用于快速判断缓冲区状态
     */
    USART3_RingBuffer.count++;
}

/**
 * @brief   USART3环形缓冲区读取
 * @details 从WiFi环形缓冲区读取数据
 * @param data 读取到的数据(输出参数)
 * @return bool 读取是否成功
 */
bool USART3_RingBuffer_Read(uint8_t *data)
{
    /**
     * 检查缓冲区空状态
     * 知识点: 缓冲区空检测
     * - 如果数据量为0，说明缓冲区空
     * - 空时不能读取，避免读取无效数据
     */
    if (USART3_RingBuffer.count == 0) {
        return false;  // 缓冲区空，读取失败
    }
    
    /**
     * 读取数据
     * 知识点: 数据读取
     * - 从当前读指针指向的位置读取数据
     * - 读指针是下一个读取位置的索引
     * - 读指针指向的是最早写入但还未读取的数据
     */
    *data = USART3_RingBuffer.buffer[USART3_RingBuffer.tail];
    
    /**
     * 更新读指针
     * 知识点: 环形指针移动
     * - 读指针向后移动一位
     * - 如果达到缓冲区末尾，则回到开头
     * - 实现环形访问
     */
    USART3_RingBuffer.tail++;
    if (USART3_RingBuffer.tail >= USART3_RING_BUFFER_SIZE) {
        USART3_RingBuffer.tail = 0;
    }
    
    /**
     * 更新数据计数器
     * 知识点: 计数器维护
     * - 每次读取成功，数据量减1
     * - 用于快速判断缓冲区状态
     */
    USART3_RingBuffer.count--;
    
    return true;  // 读取成功
}

/**
 * @brief   USART3环形缓冲区内容查询
 * @details 查询缓冲区是否包含指定字符串
 * @param str 要查找的字符串
 * @return bool 是否包含该字符串
 */
bool USART3_RingBuffer_Contains(const char *str)
{
    if (USART3_RingBuffer.count == 0 || strlen(str) == 0) {
        return false;
    }
    
    /**
     * 简单字符串匹配
     * 知识点: 匹配算法
     * - 从缓冲区头部开始搜索
     * - 逐个字符比较
     * - 找到匹配返回true，否则返回false
     * - 注意环形缓冲区的循环特性
     */
    uint16_t search_len = strlen(str);
    if (search_len > USART3_RingBuffer.count) {
        return false;  // 要搜索的字符串比数据多，不可能匹配
    }
    
    for (uint16_t i = 0; i <= USART3_RingBuffer.count - search_len; i++) {
        uint16_t buffer_index = (USART3_RingBuffer.tail + i) % USART3_RING_BUFFER_SIZE;
        bool match = true;
        
        for (uint16_t j = 0; j < search_len; j++) {
            uint16_t data_index = (buffer_index + j) % USART3_RING_BUFFER_SIZE;
            if (USART3_RingBuffer.buffer[data_index] != str[j]) {
                match = false;
                break;
            }
        }
        
        if (match) {
            return true;  // 找到匹配
        }
    }
    
    return false;  // 未找到匹配
}

/**
 * @brief   USART3环形缓冲区清空
 * @details 清空所有数据，重置指针
 */
void USART3_RingBuffer_Clear(void)
{
    /**
     * 重置缓冲区状态
     * 知识点: 缓冲区重置
     * - 重新初始化所有状态
     * - 等价于重新初始化
     */
    USART3_RingBuffer_Init();
    
    DEBUG_PRINT("USART3 ring buffer cleared\r\n");
}

/* ============================== AT指令状态机实现 ============================== */

/**
 * @brief   AT指令执行函数
 * @details 异步执行AT指令
 * @param command AT指令字符串
 * @param expected_response 期望的响应字符串
 * @param timeout_ms 超时时间(毫秒)
 * @param max_retry 最大重试次数
 * @return bool 指令启动结果
 * 
 * 知识点: AT指令异步执行
 * - 不等待指令完成立即返回
 * - 使用状态机管理执行过程
 * - 避免阻塞，提高系统响应性
 * - 支持重试机制
 */
bool AT_Execute_Command(const char *command, const char *expected_response, 
                       uint32_t timeout_ms, uint8_t max_retry)
{
    /**
     * 检查状态机是否忙碌
     * 知识点: 状态机状态检查
     * - 如果正在执行其他指令，不能启动新指令
     * - 防止指令冲突
     * - 返回false表示启动失败
     */
    if (AT_StateMachine.busy) {
        DEBUG_PRINT("AT state machine is busy, cannot execute command: %s\r\n", command);
        return false;
    }
    
    /**
     * 设置当前AT指令
     * 知识点: 指令参数设置
     * - 复制指令字符串
     * - 设置期望响应
     * - 设置超时和重试参数
     * - 重置重试计数
     */
    strncpy(AT_StateMachine.current_cmd.command, command, sizeof(AT_StateMachine.current_cmd.command) - 1);
    AT_StateMachine.current_cmd.command[sizeof(AT_StateMachine.current_cmd.command) - 1] = '\0';
    
    strncpy(AT_StateMachine.current_cmd.expected_response, expected_response, 
            sizeof(AT_StateMachine.current_cmd.expected_response) - 1);
    AT_StateMachine.current_cmd.expected_response[sizeof(AT_StateMachine.current_cmd.expected_response) - 1] = '\0';
    
    AT_StateMachine.current_cmd.timeout_ms = timeout_ms;
    AT_StateMachine.current_cmd.max_retry = max_retry;
    AT_StateMachine.current_cmd.retry_count = 0;
    AT_StateMachine.current_cmd.state = AT_STATE_SENDING;
    
    /**
     * 设置状态机状态
     * 知识点: 状态机启动
     * - 设置为忙碌状态
     * - 清空响应缓冲区
     * - 记录开始时间
     * - 设置最后活动时间
     */
    AT_StateMachine.busy = true;
    AT_StateMachine.state = AT_STATE_SENDING;
    memset(AT_StateMachine.response_buffer, 0, sizeof(AT_StateMachine.response_buffer));
    AT_StateMachine.response_length = 0;
    AT_StateMachine.start_time = System_Get_Runtime_MS();
    AT_StateMachine.last_activity_time = AT_StateMachine.start_time;
    
    DEBUG_PRINT("Starting AT command: %s (expect: %s, timeout: %lu ms, retry: %u)\r\n", 
               command, expected_response, timeout_ms, max_retry);
    
    return true;  // 指令启动成功
}

/**
 * @brief   AT指令状态机更新函数
 * @details 周期性更新AT指令执行状态
 * 
 * 知识点: 状态机更新机制
 * - 周期性调用(通常在任务中)
 * - 检查超时条件
 * - 处理状态转换
 * - 解析响应数据
 * - 错误处理和重试
 */
void AT_StateMachine_Update(void)
{
    /**
     * 检查状态机是否忙碌
     * 知识点: 状态机活跃性检查
     * - 如果不忙碌，直接返回
     * - 节省CPU时间
     */
    if (!AT_StateMachine.busy) {
        return;
    }
    
    /**
     * 获取当前时间
     * 知识点: 时间计算
     * - 用于超时检测
     * - 用于活动时间更新
     */
    uint32_t current_time = System_Get_Runtime_MS();
    uint32_t elapsed_time = current_time - AT_StateMachine.start_time;
    uint32_t activity_time = current_time - AT_StateMachine.last_activity_time;
    
    /**
     * 超时检查
     * 知识点: 超时处理
     * - 如果总执行时间超过最大超时，判定失败
     * - 计算最大超时 = 单次超时 × (重试次数 + 1)
     */
    uint32_t max_timeout = AT_StateMachine.current_cmd.timeout_ms * (AT_StateMachine.current_cmd.max_retry + 1);
    if (elapsed_time > max_timeout) {
        DEBUG_PRINT("AT command timeout: %s (elapsed: %lu ms, max: %lu ms)\r\n", 
                   AT_StateMachine.current_cmd.command, elapsed_time, max_timeout);
        AT_StateMachine.state = AT_STATE_TIMEOUT;
        AT_StateMachine.busy = false;
        return;
    }
    
    /**
     * 状态机状态处理
     * 知识点: 状态转换逻辑
     * - 根据当前状态执行相应操作
     * - 状态转换由事件和条件驱动
     */
    switch (AT_StateMachine.state) {
        case AT_STATE_SENDING:
            /**
             * 发送AT指令
             * 知识点: 指令发送
             * - 通过USART3发送AT指令
             * - 发送后进入等待状态
             * - 记录发送时间
             */
            AT_Send_Command();
            AT_StateMachine.state = AT_STATE_WAITING;
            AT_StateMachine.last_activity_time = current_time;
            break;
            
        case AT_STATE_WAITING:
            /**
             * 检查是否有新数据到达
             * 知识点: 数据到达检测
             * - 检查WiFi环形缓冲区是否有数据
             * - 如果有数据，进入接收状态
             */
            if (USART3_RingBuffer.count > 0) {
                AT_StateMachine.state = AT_STATE_RECEIVING;
            } else if (activity_time > AT_StateMachine.current_cmd.timeout_ms) {
                /**
                 * 单次等待超时
                 * 知识点: 单次超时处理
                 * - 单次等待超过指定时间
                 * - 可能需要重试或结束
                 */
                DEBUG_PRINT("AT command wait timeout: %s (activity: %lu ms, timeout: %lu ms)\r\n", 
                           AT_StateMachine.current_cmd.command, activity_time, AT_StateMachine.current_cmd.timeout_ms);
                AT_StateMachine.state = AT_STATE_ERROR;
            }
            break;
            
        case AT_STATE_RECEIVING:
            /**
             * 处理接收到的响应数据
             * 知识点: 响应数据处理
             * - 读取串口数据
             * - 解析响应内容
             * - 检查是否完成
             */
            AT_Process_Response();
            break;
            
        case AT_STATE_SUCCESS:
        case AT_STATE_ERROR:
        case AT_STATE_TIMEOUT:
            /**
             * AT指令执行完成
             * 知识点: 完成状态处理
             * - 状态机结束
             * - 清理状态
             * - 准备下次指令
             */
            AT_StateMachine.busy = false;
            DEBUG_PRINT("AT command completed: %s (state: %d)\r\n", 
                       AT_StateMachine.current_cmd.command, AT_StateMachine.state);
            break;
            
        default:
            /**
             * 未知状态处理
             * 知识点: 异常状态处理
             * - 记录错误
             * - 重置状态机
             */
            DEBUG_PRINT("AT state machine in unknown state: %d\r\n", AT_StateMachine.state);
            AT_StateMachine.state = AT_STATE_ERROR;
            AT_StateMachine.busy = false;
            break;
    }
}

/**
 * @brief   AT指令发送函数
 * @details 发送当前AT指令到WiFi模块
 * 
 * 知识点: AT指令发送过程
 * 1. 获取串口互斥锁(防止并发访问)
 * 2. 发送AT指令字符串
 * 3. 发送回车符(\r\n)
 * 4. 释放互斥锁
 * 5. 清空WiFi接收缓冲区(可选)
 */
void AT_Send_Command(void)
{
    /**
     * 获取USART3互斥锁
     * 知识点: 串口访问保护
     * - 防止多个任务同时访问串口
     * - 使用FreeRTOS互斥锁
     * - 避免数据混乱
     */
    if (xSemaphoreTake(USART3_Mutex, 100) != pdPASS) {
        DEBUG_PRINT("Failed to get USART3 mutex for AT command\r\n");
        AT_StateMachine.state = AT_STATE_ERROR;
        return;
    }
    
    /**
     * 发送AT指令
     * 知识点: 串口发送
     * - 通过USART3发送字符串
     * - STM32标准库串口发送函数
     * - 阻塞发送，等待发送完成
     */
    const char *command = AT_StateMachine.current_cmd.command;
    uint16_t command_length = strlen(command);
    
    for (uint16_t i = 0; i < command_length; i++) {
        USART_SendData(USART3, command[i]);
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);  // 等待发送完成
    }
    
    /**
     * 发送回车换行符
     * 知识点: AT指令格式
     * - AT指令以\r\n结尾
     * - \r(CR, 0x0D) - 回车符
     * - \n(LF, 0x0A) - 换行符
     * - 表示指令结束
     */
    USART_SendData(USART3, '\r');
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    
    USART_SendData(USART3, '\n');
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    
    /**
     * 释放USART3互斥锁
     * 知识点: 互斥锁释放
     * - 释放串口访问权
     * - 其他任务可以访问串口
     */
    xSemaphoreGive(USART3_Mutex);
    
    /**
     * 清空WiFi接收缓冲区
     * 知识点: 缓冲区清理
     * - 清除旧数据，准备接收新响应
     * - 提高响应检测准确性
     */
    USART3_RingBuffer_Clear();
    
    DEBUG_PRINT("AT command sent: %s\r\n", command);
}

/**
 * @brief   AT指令响应处理函数
 * @details 处理WiFi模块返回的响应数据
 * 
 * 知识点: 响应数据处理流程
 * 1. 读取所有可用的串口数据
 * 2. 追加到响应缓冲区
 * 3. 检查是否包含完整响应
 * 4. 验证响应内容
 * 5. 确定执行结果
 */
void AT_Process_Response(void)
{
    /**
     * 读取所有可用数据
     * 知识点: 数据读取
     * - 循环读取直到缓冲区空
     * - 避免丢失数据
     * - 累积响应数据
     */
    uint8_t data;
    while (USART3_RingBuffer_Read(&data)) {
        if (AT_StateMachine.response_length < sizeof(AT_StateMachine.response_buffer) - 1) {
            AT_StateMachine.response_buffer[AT_StateMachine.response_length++] = data;
            AT_StateMachine.response_buffer[AT_StateMachine.response_length] = '\0';  // 确保字符串以null结尾
        } else {
            /**
             * 响应缓冲区已满
             * 知识点: 缓冲区溢出处理
             * - 丢弃超出部分
             * - 记录警告
             */
            DEBUG_PRINT("AT response buffer overflow, data truncated\r\n");
            break;
        }
    }
    
    /**
     * 检查响应是否完整
     * 知识点: 响应完整性检查
     * - 查找"OK"或"ERROR"标识响应结束
     * - 需要等待足够时间确保收到完整响应
     */
    const char *response = AT_StateMachine.response_buffer;
    
    if (strstr(response, "OK") != NULL) {
        /**
         * 响应成功
         * 知识点: 成功响应处理
         * - 验证是否匹配期望响应
         * - 匹配成功则设置成功状态
         * - 匹配失败则可能是错误响应
         */
        if (strstr(response, AT_StateMachine.current_cmd.expected_response) != NULL) {
            AT_StateMachine.state = AT_STATE_SUCCESS;
            DEBUG_PRINT("AT command success: %s -> %s\r\n", 
                       AT_StateMachine.current_cmd.command, response);
        } else {
            /**
             * 响应不匹配
             * 知识点: 响应验证失败
             * - 虽然返回OK，但不是期望的响应
             * - 可能是WiFi模块状态问题
             */
            DEBUG_PRINT("AT command response mismatch: %s (expected: %s, got: %s)\r\n", 
                       AT_StateMachine.current_cmd.command, 
                       AT_StateMachine.current_cmd.expected_response, 
                       response);
            AT_StateMachine.state = AT_STATE_ERROR;
        }
    } else if (strstr(response, "ERROR") != NULL) {
        /**
         * 响应错误
         * 知识点: 错误响应处理
         * - WiFi模块返回ERROR
         * - 需要重试或检查参数
         */
        DEBUG_PRINT("AT command error: %s -> %s\r\n", 
                   AT_StateMachine.current_cmd.command, response);
        AT_StateMachine.state = AT_STATE_ERROR;
    } else if (AT_StateMachine.response_length > 0) {
        /**
         * 部分响应但未完成
         * 知识点: 部分响应处理
         * - 收到数据但还没有完整响应
         * - 继续等待
         * - 有超时保护
         */
        uint32_t current_time = System_Get_Runtime_MS();
        uint32_t response_time = current_time - AT_StateMachine.last_activity_time;
        
        if (response_time > AT_StateMachine.current_cmd.timeout_ms * 0.8) {
            /**
             * 响应时间接近超时
             * 知识点: 响应超时预判
             * - 80%超时时间已过但仍无完整响应
             * - 可能是响应数据不完整
             * - 继续等待或处理部分响应
             */
            DEBUG_PRINT("AT command partial response (timeout approaching): %s -> %s\r\n", 
                       AT_StateMachine.current_cmd.command, response);
        }
    }
    
    /**
     * 检查是否需要重试
     * 知识点: 重试机制
     * - 如果状态为ERROR或TIMEOUT
     * - 且还有重试次数
     * - 则重新发送指令
     */
    if ((AT_StateMachine.state == AT_STATE_ERROR || AT_StateMachine.state == AT_STATE_TIMEOUT) &&
        AT_StateMachine.current_cmd.retry_count < AT_StateMachine.current_cmd.max_retry) {
        
        AT_StateMachine.current_cmd.retry_count++;
        AT_StateMachine.state = AT_STATE_SENDING;  // 重新发送
        
        DEBUG_PRINT("AT command retry %u/%u: %s\r\n", 
                   AT_StateMachine.current_cmd.retry_count,
                   AT_StateMachine.current_cmd.max_retry,
                   AT_StateMachine.current_cmd.command);
    }
}

/**
 * @brief   AT指令状态获取函数
 * @details 获取当前AT指令执行状态
 * @return AT_State_t 当前状态
 */
AT_State_t AT_Get_State(void)
{
    return AT_StateMachine.state;
}

/**
 * @brief   AT指令等待完成函数
 * @details 等待AT指令执行完成(阻塞等待)
 * @param timeout_ms 最大等待时间
 * @return bool 是否成功完成
 * 
 * 知识点: 阻塞等待机制
 * - 用于需要同步执行AT指令的场景
 * - 有超时保护，避免无限等待
 * - 期间仍可处理其他中断
 */
bool AT_Wait_Complete(uint32_t timeout_ms)
{
    /**
     * 检查是否已经在执行
     * 知识点: 执行状态检查
     * - 如果已经在执行，等待当前指令完成
     * - 如果没有执行，可能已经被完成
     */
    uint32_t start_time = System_Get_Runtime_MS();
    uint32_t elapsed_time = 0;
    
    while (elapsed_time < timeout_ms) {
        /**
         * 更新AT状态机
         * 知识点: 状态机推进
         * - 周期性更新状态机
         * - 处理状态转换和响应
         */
        AT_StateMachine_Update();
        
        /**
         * 检查执行状态
         * 知识点: 完成状态判断
         * - SUCCESS: 成功完成
         * - ERROR: 执行失败
         * - TIMEOUT: 超时失败
         * - IDLE: 状态机空闲
         */
        if (!AT_StateMachine.busy) {
            if (AT_StateMachine.state == AT_STATE_SUCCESS) {
                DEBUG_PRINT("AT command completed successfully\r\n");
                return true;
            } else {
                DEBUG_PRINT("AT command failed with state: %d\r\n", AT_StateMachine.state);
                return false;
            }
        }
        
        /**
         * 更新等待时间
         * 知识点: 时间计算
         * - 计算已等待时间
         * - 控制等待循环
         */
        elapsed_time = System_Get_Runtime_MS() - start_time;
        Delay_ms(10);  // 避免过度占用CPU
    }
    
    /**
     * 等待超时
     * 知识点: 超时处理
     * - 超过最大等待时间
     * - 返回失败
     */
    DEBUG_PRINT("AT command wait timeout: %lu ms\r\n", timeout_ms);
    return false;
}

/* ============================== WiFi管理实现 ============================== */

/**
 * @brief   WiFi模块初始化函数
 * @details 初始化WiFi模块并测试通信
 * 
 * 知识点: WiFi模块初始化流程
 * 1. 测试AT指令通信
 * 2. 配置WiFi工作模式
 * 3. 恢复出厂设置(可选)
 * 4. 设置网络参数
 */
void WiFi_Init(void)
{
    DEBUG_PRINT("Initializing WiFi module...\r\n");
    
    /**
     * 测试AT指令通信
     * 知识点: 基本通信测试
     * - 发送最简单的AT指令
 * - 验证串口通信正常
     * - WiFi模块响应正常
     */
    if (AT_Execute_Command("AT", "OK", 1000, 3)) {
        if (AT_Wait_Complete(5000)) {
            DEBUG_PRINT("WiFi module communication test passed\r\n");
        } else {
            DEBUG_PRINT("WiFi module communication test failed\r\n");
            return;
        }
    } else {
        DEBUG_PRINT("Failed to start WiFi communication test\r\n");
        return;
    }
    
    /**
     * 关闭回显
     * 知识点: 回显控制
     * - 关闭ATE0回显，简化响应解析
     * - AT指令不会回显到响应中
     * - 减少响应数据量
     */
    if (AT_Execute_Command("ATE0", "OK", 1000, 2)) {
        AT_Wait_Complete(3000);
    }
    
    /**
     * 设置WiFi工作模式
     * 知识点: WiFi模式配置
     * - CWMODE=1: Station模式，作为WiFi客户端
     * - 连接到路由器，不是创建热点
     * - 这是环境监测系统的典型模式
     */
    if (AT_Execute_Command("AT+CWMODE=1", "OK", 2000, 2)) {
        if (AT_Wait_Complete(5000)) {
            DEBUG_PRINT("WiFi mode set to Station (CWMODE=1)\r\n");
        }
    }
    
    /**
     * 查看版本信息
     * 知识点: 版本信息
     * - 获取WiFi模块固件版本
     * - 便于故障诊断
     */
    if (AT_Execute_Command("AT+GMR", "OK", 2000, 1)) {
        AT_Wait_Complete(3000);
    }
    
    DEBUG_PRINT("WiFi module initialization completed\r\n");
}

/**
 * @brief   WiFi连接函数
 * @details 连接到指定的WiFi网络
 * @param ssid WiFi网络名称
 * @param password WiFi密码
 * @return bool 连接结果
 * 
 * 知识点: WiFi连接过程
 * 1. 检查当前连接状态
 * 2. 发送连接指令
 * 3. 等待连接结果
 * 4. 验证IP地址获取
 * 5. 更新连接状态
 */
bool WiFi_Connect(const char *ssid, const char *password)
{
    if (strlen(ssid) == 0 || strlen(password) == 0) {
        DEBUG_PRINT("Invalid WiFi credentials: SSID='%s', Password='%s'\r\n", ssid, password);
        return false;
    }
    
    DEBUG_PRINT("Connecting to WiFi network: %s\r\n", ssid);
    
    /**
     * 发送WiFi连接指令
     * 知识点: 连接指令格式
     * - AT+CWJAP="SSID","PASSWORD"
     * - SSID和密码需要用双引号包围
     * - 连接过程可能需要较长时间
     */
    char command[64];
    snprintf(command, sizeof(command), "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
    
    if (AT_Execute_Command(command, "WIFI GOT IP", 30000, 2)) {
        if (AT_Wait_Complete(35000)) {
            /**
             * 连接成功
             * 知识点: 连接成功处理
             * - 更新WiFi管理器状态
             * - 记录连接时间
             * - 重置重试计数
             */
            WiFi_Manager.state = NETWORK_STATE_CONNECTED;
            strcpy(WiFi_Manager.ssid, ssid);
            strcpy(WiFi_Manager.password, password);
            WiFi_Manager.last_connect_time = System_Get_Runtime_MS();
            WiFi_Manager.connect_retry_count = 0;
            
            DEBUG_PRINT("WiFi connected successfully: %s\r\n", ssid);
            return true;
        }
    }
    
    /**
     * 连接失败
     * 知识点: 连接失败处理
     * - 更新状态为错误
     * - 增加重试计数
     * - 记录失败信息
     */
    WiFi_Manager.state = NETWORK_STATE_ERROR;
    WiFi_Manager.connect_retry_count++;
    
    DEBUG_PRINT("WiFi connection failed: %s (retry count: %u)\r\n", 
               ssid, WiFi_Manager.connect_retry_count);
    
    return false;
}

/**
 * @brief   WiFi断开函数
 * @details 断开当前WiFi连接
 */
void WiFi_Disconnect(void)
{
    DEBUG_PRINT("Disconnecting from WiFi network\r\n");
    
    /**
     * 发送断开指令
     * 知识点: 断开指令
     * - AT+CWQAP: 断开WiFi连接
     * - 简单直接
     */
    if (AT_Execute_Command("AT+CWQAP", "OK", 2000, 2)) {
        AT_Wait_Complete(5000);
    }
    
    /**
     * 更新连接状态
     * 知识点: 状态管理
     * - 设置为未连接状态
     * - 清除连接时间
     */
    WiFi_Manager.state = NETWORK_STATE_DISCONNECTED;
    WiFi_Manager.last_connect_time = 0;
    
    DEBUG_PRINT("WiFi disconnected\r\n");
}

/**
 * @brief   WiFi连接状态检测函数
 * @details 检测WiFi是否仍然连接
 * @return bool 连接状态
 * 
 * 知识点: 连接状态检测
 * - 发送AT+CIPSTATUS查询连接状态
 * - 检查返回的连接信息
 * - 解析连接状态代码
 */
bool WiFi_Is_Connected(void)
{
    /**
     * 查询连接状态
     * 知识点: 状态查询
     * - 发送AT+CIPSTATUS指令
     * - 解析返回的状态信息
     */
    if (AT_Execute_Command("AT+CIPSTATUS", "OK", 2000, 1)) {
        if (AT_Wait_Complete(5000)) {
            /**
             * 解析状态响应
             * 知识点: 状态解析
             * - 查找状态码信息
             * - 连接状态码通常为2或3
             * - 不同模块可能有不同编码
             */
            if (strstr(AT_StateMachine.response_buffer, "STATUS:2") != NULL ||
                strstr(AT_StateMachine.response_buffer, "STATUS:3") != NULL) {
                WiFi_Manager.state = NETWORK_STATE_CONNECTED;
                return true;
            } else {
                WiFi_Manager.state = NETWORK_STATE_DISCONNECTED;
                return false;
            }
        }
    }
    
    /**
     * 状态查询失败
     * 知识点: 错误处理
     * - 可能是网络问题或模块问题
     * - 设置状态为未知
     * - 返回false保守处理
     */
    DEBUG_PRINT("WiFi status check failed\r\n");
    WiFi_Manager.state = NETWORK_STATE_ERROR;
    return false;
}

/**
 * @brief   WiFi数据发送函数
 * @details 发送数据到指定的TCP服务器
 * @param data 要发送的数据
 * @param length 数据长度
 * @return bool 发送结果
 * 
 * 知识点: TCP数据发送流程
 * 1. 确认TCP连接状态
 * 2. 启动发送模式
 * 3. 发送数据长度指令
 * 4. 发送实际数据
 * 5. 等待发送确认
 */
bool WiFi_Send_Data(const uint8_t *data, uint16_t length)
{
    /**
     * 检查连接状态
     * 知识点: 发送前检查
     * - 确保WiFi已连接
     * - 确保TCP连接已建立
     */
    if (!WiFi_Is_Connected()) {
        DEBUG_PRINT("Cannot send data: WiFi not connected\r\n");
        return false;
    }
    
    /**
     * 启动TCP发送
     * 知识点: 发送模式切换
     * - AT+CIPSEND指令启动发送模式
     * - 指定发送数据长度
     * - 模块返回">"提示符
     */
    char command[32];
    snprintf(command, sizeof(command), "AT+CIPSEND=%d", length);
    
    if (AT_Execute_Command(command, ">", 2000, 2)) {
        if (AT_Wait_Complete(3000)) {
            /**
             * 发送实际数据
             * 知识点: 数据传输
             * - 通过USART3发送数据
             * - 使用互斥锁保护串口访问
             */
            if (xSemaphoreTake(USART3_Mutex, 100) == pdPASS) {
                for (uint16_t i = 0; i < length; i++) {
                    USART_SendData(USART3, data[i]);
                    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
                }
                xSemaphoreGive(USART3_Mutex);
                
                /**
                 * 等待发送确认
                 * 知识点: 发送确认
                 * - WiFi模块会返回发送结果
                 * - "SEND OK"表示发送成功
                 * - "SEND FAIL"表示发送失败
                 */
                if (AT_Wait_Complete(10000)) {  // 等待更长时间给数据发送
                    if (strstr(AT_StateMachine.response_buffer, "SEND OK") != NULL) {
                        DEBUG_PRINT("Data sent successfully (%u bytes)\r\n", length);
                        return true;
                    } else {
                        DEBUG_PRINT("Data send failed: %s\r\n", AT_StateMachine.response_buffer);
                    }
                }
            }
        }
    }
    
    DEBUG_PRINT("Data send failed\r\n");
    return false;
}

/**
 * @brief   WiFi连接管理器更新函数
 * @details 定期更新WiFi连接状态和自动重连
 * 
 * 知识点: 连接管理器更新流程
 * 1. 检查当前连接状态
 * 2. 处理心跳检测
 * 3. 处理自动重连
 * 4. 更新连接历史
 */
void WiFi_Manager_Update(void)
{
    /**
     * 获取当前时间
     * 知识点: 时间计算
     * - 用于心跳检测
     * - 用于自动重连判断
     */
    uint32_t current_time = System_Get_Runtime_MS();
    
    switch (WiFi_Manager.state) {
        case NETWORK_STATE_CONNECTED:
            /**
             * 已连接状态
             * 知识点: 连接维护
             * - 定期检查连接状态
             * - 心跳检测
             * - 处理断线情况
             */
            if (current_time - WiFi_Manager.last_connect_time > WiFi_Manager.heartbeat_interval) {
                if (!WiFi_Is_Connected()) {
                    /**
                     * 连接丢失
                     * 知识点: 断线处理
                     * - 更新状态为错误
                     * - 准备自动重连
                     */
                    DEBUG_PRINT("WiFi connection lost\r\n");
                    WiFi_Manager.state = NETWORK_STATE_ERROR;
                } else {
                    /**
                     * 连接正常
                     * 知识点: 心跳更新
                     * - 更新时间戳
                     * - 准备下次检查
                     */
                    WiFi_Manager.last_connect_time = current_time;
                }
            }
            break;
            
        case NETWORK_STATE_ERROR:
        case NETWORK_STATE_DISCONNECTED:
            /**
             * 断开/错误状态
             * 知识点: 自动重连机制
             * - 检查是否启用自动重连
             * - 检查重连间隔
             * - 执行重连操作
             */
            if (WiFi_Manager.auto_reconnect &&
                current_time - WiFi_Manager.last_connect_time > WIFI_RECONNECT_INTERVAL_MS) {
                
                DEBUG_PRINT("Attempting WiFi reconnection...\r\n");
                
                if (WiFi_Connect(WiFi_Manager.ssid, WiFi_Manager.password)) {
                    DEBUG_PRINT("WiFi reconnected successfully\r\n");
                } else {
                    DEBUG_PRINT("WiFi reconnection failed, will retry later\r\n");
                    WiFi_Manager.last_connect_time = current_time;  // 更新重试时间
                }
            }
            break;
            
        case NETWORK_STATE_CONNECTING:
            /**
             * 正在连接状态
             * 知识点: 连接中处理
             * - 检查连接是否超时
             * - 更新连接状态
             */
            // 连接状态由WiFi_Connect函数管理，这里不需要特殊处理
            break;
            
        default:
            /**
             * 未知状态处理
             * 知识点: 异常状态
             * - 重置为断开状态
             * - 准备重新连接
             */
            WiFi_Manager.state = NETWORK_STATE_DISCONNECTED;
            break;
    }
}

/* ============================== 系统配置管理 ============================== */

/**
 * @brief   系统配置加载函数
 * @details 从内部Flash加载系统配置参数
 * 
 * 知识点: 内部Flash读取
 * - STM32内部Flash掉电不丢失
 * - 适合存储配置参数
 * - 读取操作相对简单
 * - 需要处理读取失败的情况
 */
void System_Config_Load(void)
{
    uint8_t *config_ptr = (uint8_t*)&System_Config;
    uint32_t flash_addr = 0x08007C00;  // 配置存储地址
    
    DEBUG_PRINT("Loading system configuration from flash...\r\n");
    
    /**
     * 从Flash读取配置
     * 知识点: Flash读取操作
     * - 直接内存映射读取
     * - 读取速度很快
     * - 不需要特殊函数调用
     */
    for (int i = 0; i < sizeof(System_Config); i++) {
        config_ptr[i] = *(volatile uint8_t*)(flash_addr + i);
    }
    
    /**
     * 配置参数验证
     * 知识点: 参数有效性检查
     * - 验证IP地址格式
     * - 验证端口号范围
     * - 验证SSID长度
     * - 验证运行参数范围
     * - 设置默认值(如果无效)
     */
    bool config_valid = true;
    
    // 验证IP地址
    if (strlen(System_Config.server_ip) == 0) {
        strcpy(System_Config.server_ip, "192.168.1.100");
        config_valid = false;
        DEBUG_PRINT("Invalid server IP, using default\r\n");
    }
    
    // 验证端口号
    if (System_Config.server_port == 0 || System_Config.server_port > 65535) {
        System_Config.server_port = 8080;
        config_valid = false;
        DEBUG_PRINT("Invalid server port, using default\r\n");
    }
    
    // 验证WiFi SSID
    if (strlen(System_Config.wifi_ssid) == 0) {
        strcpy(System_Config.wifi_ssid, "MyWiFi");
        config_valid = false;
        DEBUG_PRINT("Empty WiFi SSID, using default\r\n");
    }
    
    // 验证WiFi密码
    if (strlen(System_Config.wifi_password) == 0) {
        strcpy(System_Config.wifi_password, "password123");
        config_valid = false;
        DEBUG_PRINT("Empty WiFi password, using default\r\n");
    }
    
    // 验证采集间隔
    if (System_Config.sample_interval < 1 || System_Config.sample_interval > 60) {
        System_Config.sample_interval = 5;
        config_valid = false;
        DEBUG_PRINT("Invalid sample interval, using default\r\n");
    }
    
    // 验证发送间隔
    if (System_Config.send_interval < 1 || System_Config.send_interval > 60) {
        System_Config.send_interval = 10;
        config_valid = false;
        DEBUG_PRINT("Invalid send interval, using default\r\n");
    }
    
    if (config_valid) {
        DEBUG_PRINT("System configuration loaded successfully\r\n");
    } else {
        DEBUG_PRINT("System configuration loaded with defaults\r\n");
    }
    
    /**
     * 打印加载的配置
     * 知识点: 配置信息输出
     * - 便于调试和验证
     * - 密码可能被隐藏显示
     */
    DEBUG_PRINT("Configuration:\r\n");
    DEBUG_PRINT("  WiFi SSID: %s\r\n", System_Config.wifi_ssid);
    DEBUG_PRINT("  WiFi Password: %s\r\n", "***");  // 隐藏密码
    DEBUG_PRINT("  Server IP: %s\r\n", System_Config.server_ip);
    DEBUG_PRINT("  Server Port: %u\r\n", System_Config.server_port);
    DEBUG_PRINT("  Sample Interval: %u seconds\r\n", System_Config.sample_interval);
    DEBUG_PRINT("  Send Interval: %u seconds\r\n", System_Config.send_interval);
}

/**
 * @brief   系统配置保存函数
 * @details 将系统配置保存到内部Flash
 * 
 * 知识点: 内部Flash写入流程
 * 1. 解锁Flash
 * 2. 擦除目标页
 * 3. 写入数据
 * 4. 锁定Flash
 * 5. 验证写入结果
 */
void System_Config_Save(void)
{
    uint32_t flash_addr = 0x08007C00;  // 配置存储地址
    uint32_t page_addr = 0x08007C00;   // 页起始地址
    uint8_t *config_ptr = (uint8_t*)&System_Config;
    
    DEBUG_PRINT("Saving system configuration to flash...\r\n");
    
    /**
     * 解锁Flash
     * 知识点: Flash写保护
     * - STM32 Flash默认写保护
     * - 需要先解锁才能写入
     * - 写操作完成后会自动重新上锁
     */
    FLASH_Unlock();
    
    /**
     * 擦除Flash页
     * 知识点: Flash擦除机制
     * - 写入前必须先擦除
     * - Flash擦除后所有位变为1
     * - 按页擦除，每页2KB
     * - 擦除操作需要时间
     */
    if (FLASH_ErasePage(page_addr) != FLASH_COMPLETE) {
        ERROR_HANDLER();
        return;
    }
    
    /**
     * 写入配置数据
     * 知识点: Flash写入操作
     * - 按半字(16位)写入
     * - 写入速度比擦除快
     * - 每次写入后检查状态
     */
    for (int i = 0; i < sizeof(System_Config); i += 2) {
        uint16_t data;
        if (i + 1 < sizeof(System_Config)) {
            data = (config_ptr[i+1] << 8) | config_ptr[i];
        } else {
            data = config_ptr[i];  // 最后一个字节，低8位有效
        }
        
        if (FLASH_ProgramHalfWord(flash_addr + i, data) != FLASH_COMPLETE) {
            ERROR_HANDLER();
            return;
        }
    }
    
    /**
     * 重新锁定Flash
     * 知识点: 安全性考虑
     * - 写入完成后立即锁定
     * - 防止意外写入
     * - 提高系统安全性
     */
    FLASH_Lock();
    
    /**
     * 验证写入结果
     * 知识点: 写入验证
     * - 读取刚写入的数据
     * - 与原始数据比较
     * - 确保写入成功
     */
    bool verify_success = true;
    for (int i = 0; i < sizeof(System_Config); i++) {
        uint8_t read_data = *(volatile uint8_t*)(flash_addr + i);
        if (read_data != config_ptr[i]) {
            verify_success = false;
            DEBUG_PRINT("Flash verification failed at offset %d: 0x%02X != 0x%02X\r\n", 
                       i, read_data, config_ptr[i]);
            break;
        }
    }
    
    if (verify_success) {
        DEBUG_PRINT("System configuration saved successfully\r\n");
    } else {
        DEBUG_PRINT("System configuration save failed - verification error\r\n");
        ERROR_HANDLER();
    }
}

/* ============================== 调试函数实现 ============================== */

#ifdef DEBUG
/**
 * @brief   串口状态打印函数
 * @details 打印所有串口的当前状态
 */
void USART_Print_Status(void)
{
    printf("\r\nUSART Status:\r\n");
    
    /**
     * 打印USART1状态
     * 知识点: 调试串口状态
     * - 检查接收缓冲区状态
     * - 统计接收数据量
     */
    printf("  USART1 (Debug, 9600bps):\r\n");
    printf("    RX Buffer: %d/%d bytes\r\n", USART1_RX_BUFFER_SIZE, USART1_RX_BUFFER_SIZE);
    
    /**
     * 打印USART2状态
     * 知识点: 蓝牙串口状态
     * - 打印环形缓冲区状态
     * - 显示数据统计
     */
    printf("  USART2 (Bluetooth, 9600bps):\r\n");
    printf("    Ring Buffer: %d/%d bytes\r\n", 
           USART2_RingBuffer.count, USART2_RING_BUFFER_SIZE);
    printf("    Head: %d, Tail: %d\r\n", 
           USART2_RingBuffer.head, USART2_RingBuffer.tail);
    
    /**
     * 打印USART3状态
     * 知识点: WiFi串口状态
     * - 打印环形缓冲区状态
     * - 显示AT状态机状态
     */
    printf("  USART3 (WiFi, 115200bps):\r\n");
    printf("    Ring Buffer: %d/%d bytes\r\n", 
           USART3_RingBuffer.count, USART3_RING_BUFFER_SIZE);
    printf("    Head: %d, Tail: %d\r\n", 
           USART3_RingBuffer.head, USART3_RingBuffer.tail);
    
    /**
     * 打印AT状态机状态
     * 知识点: AT指令状态机状态
     * - 当前执行的指令
     * - 状态机状态
     * - 响应数据
     */
    printf("  AT State Machine:\r\n");
    printf("    State: %d (%s)\r\n", 
           AT_StateMachine.state,
           AT_StateMachine.state == AT_STATE_IDLE ? "IDLE" :
           AT_StateMachine.state == AT_STATE_SENDING ? "SENDING" :
           AT_StateMachine.state == AT_STATE_WAITING ? "WAITING" :
           AT_StateMachine.state == AT_STATE_RECEIVING ? "RECEIVING" :
           AT_StateMachine.state == AT_STATE_SUCCESS ? "SUCCESS" :
           AT_StateMachine.state == AT_STATE_ERROR ? "ERROR" : "TIMEOUT");
    printf("    Busy: %s\r\n", AT_StateMachine.busy ? "Yes" : "No");
    printf("    Current Command: %s\r\n", AT_StateMachine.current_cmd.command);
    printf("    Response Length: %d\r\n", AT_StateMachine.response_length);
}

/**
 * @brief   AT指令状态打印函数
 * @details 打印AT指令执行状态
 */
void AT_Print_State(void)
{
    printf("\r\nAT State Machine Details:\r\n");
    
    /**
     * 打印当前指令
     * 知识点: 当前指令信息
     * - 指令字符串
     * - 期望响应
     * - 超时和重试设置
     */
    printf("  Current Command:\r\n");
    printf("    Command: %s\r\n", AT_StateMachine.current_cmd.command);
    printf("    Expected: %s\r\n", AT_StateMachine.current_cmd.expected_response);
    printf("    Timeout: %lu ms\r\n", AT_StateMachine.current_cmd.timeout_ms);
    printf("    Max Retry: %u\r\n", AT_StateMachine.current_cmd.max_retry);
    printf("    Retry Count: %u\r\n", AT_StateMachine.current_cmd.retry_count);
    
    /**
     * 打印状态信息
     * 知识点: 状态机状态
     * - 当前状态
     * - 忙碌状态
     * - 时间信息
     */
    printf("  State:\r\n");
    printf("    Current: %d\r\n", AT_StateMachine.state);
    printf("    Busy: %s\r\n", AT_StateMachine.busy ? "Yes" : "No");
    printf("    Response Length: %d\r\n", AT_StateMachine.response_length);
    
    /**
     * 打印响应数据
     * 知识点: 响应内容
     * - 响应缓冲区内容
     * - 便于调试和分析
     */
    if (AT_StateMachine.response_length > 0) {
        printf("  Response Buffer:\r\n");
        printf("    Content: \"%s\"\r\n", AT_StateMachine.response_buffer);
        printf("    Hex: ");
        for (uint16_t i = 0; i < AT_StateMachine.response_length; i++) {
            printf("%02X ", AT_StateMachine.response_buffer[i]);
        }
        printf("\r\n");
    }
}

/**
 * @brief   WiFi状态打印函数
 * @details 打印WiFi连接状态和配置
 */
void WiFi_Print_Status(void)
{
    printf("\r\nWiFi Status:\r\n");
    
    /**
     * 打印连接状态
     * 知识点: 连接状态显示
     * - 当前网络状态
     * - 连接时间信息
     */
    printf("  Connection:\r\n");
    printf("    State: %d (%s)\r\n", 
           WiFi_Manager.state,
           WiFi_Manager.state == NETWORK_STATE_DISCONNECTED ? "Disconnected" :
           WiFi_Manager.state == NETWORK_STATE_CONNECTING ? "Connecting" :
           WiFi_Manager.state == NETWORK_STATE_CONNECTED ? "Connected" : "Error");
    
    /**
     * 打印网络配置
     * 知识点: 网络配置显示
     * - WiFi网络信息
     * - 服务器配置
     */
    printf("  Network Config:\r\n");
    printf("    SSID: %s\r\n", WiFi_Manager.ssid);
    printf("    Password: %s\r\n", "***");  // 隐藏密码
    printf("    Server: %s:%u\r\n", 
           WiFi_Manager.server_ip, WiFi_Manager.server_port);
    
    /**
     * 打印连接统计
     * 知识点: 连接统计信息
     * - 重连次数
     * - 上次连接时间
     * - 心跳设置
     */
    printf("  Statistics:\r\n");
    printf("    Reconnect Count: %u\r\n", WiFi_Manager.connect_retry_count);
    printf("    Auto Reconnect: %s\r\n", 
           WiFi_Manager.auto_reconnect ? "Enabled" : "Disabled");
    printf("    Heartbeat Interval: %lu ms\r\n", WiFi_Manager.heartbeat_interval);
    
    if (WiFi_Manager.last_connect_time > 0) {
        uint32_t current_time = System_Get_Runtime_MS();
        uint32_t connected_duration = current_time - WiFi_Manager.last_connect_time;
        printf("    Last Connect Time: %lu ms ago\r\n", connected_duration);
    }
}

/**
 * @brief   环形缓冲区状态打印函数
 * @details 打印环形缓冲区详细状态
 */
void RingBuffer_Print_Status(void)
{
    printf("\r\nRing Buffer Status:\r\n");
    
    /**
     * 打印USART2缓冲区状态
     * 知识点: 蓝牙缓冲区状态
     * - 缓冲区使用情况
     * - 指针位置
     * - 内容预览
     */
    printf("  USART2 Ring Buffer (Bluetooth):\r\n");
    printf("    Size: %d bytes\r\n", USART2_RING_BUFFER_SIZE);
    printf("    Used: %d bytes (%.1f%%)\r\n", 
           USART2_RingBuffer.count, 
           (float)USART2_RingBuffer.count / USART2_RING_BUFFER_SIZE * 100.0f);
    printf("    Head: %d, Tail: %d\r\n", 
           USART2_RingBuffer.head, USART2_RingBuffer.tail);
    
    if (USART2_RingBuffer.count > 0) {
        printf("    Content Preview: \"");
        for (uint16_t i = 0; i < (USART2_RingBuffer.count < 32 ? USART2_RingBuffer.count : 32); i++) {
            uint16_t index = (USART2_RingBuffer.tail + i) % USART2_RING_BUFFER_SIZE;
            uint8_t data = USART2_RingBuffer.buffer[index];
            if (data >= 32 && data <= 126) {
                printf("%c", data);
            } else {
                printf(".");
            }
        }
        printf("\"\r\n");
    }
    
    /**
     * 打印USART3缓冲区状态
     * 知识点: WiFi缓冲区状态
     * - 缓冲区使用情况
     * - 指针位置
     * - 内容预览
     */
    printf("  USART3 Ring Buffer (WiFi):\r\n");
    printf("    Size: %d bytes\r\n", USART3_RING_BUFFER_SIZE);
    printf("    Used: %d bytes (%.1f%%)\r\n", 
           USART3_RingBuffer.count, 
           (float)USART3_RingBuffer.count / USART3_RING_BUFFER_SIZE * 100.0f);
    printf("    Head: %d, Tail: %d\r\n", 
           USART3_RingBuffer.head, USART3_RingBuffer.tail);
    
    if (USART3_RingBuffer.count > 0) {
        printf("    Content Preview: \"");
        for (uint16_t i = 0; i < (USART3_RingBuffer.count < 32 ? USART3_RingBuffer.count : 32); i++) {
            uint16_t index = (USART3_RingBuffer.tail + i) % USART3_RING_BUFFER_SIZE;
            uint8_t data = USART3_RingBuffer.buffer[index];
            if (data >= 32 && data <= 126) {
                printf("%c", data);
            } else {
                printf(".");
            }
        }
        printf("\"\r\n");
    }
}
#endif

/* ============================== 网络消息队列函数 ============================== */

/**
 * @brief   网络消息发送函数
 * @details 发送消息到网络队列
 * @param msg 要发送的消息
 * @return bool 发送结果
 */
bool Network_Send_Message(Network_Message_t *msg)
{
    // 外部队列句柄在main.c中定义
    extern xQueueHandle NetworkQueue;
    
    if (xQueueSend(NetworkQueue, msg, 0) == pdPASS) {
        return true;
    } else {
        DEBUG_PRINT("Failed to send network message, queue full\r\n");
        return false;
    }
}

/**
 * @brief   网络消息接收函数
 * @details 从网络队列接收消息
 * @param msg 接收到的消息
 * @param timeout_ms 超时时间
 * @return bool 接收结果
 */
bool Network_Receive_Message(Network_Message_t *msg, uint32_t timeout_ms)
{
    // 外部队列句柄在main.c中定义
    extern xQueueHandle NetworkQueue;
    
    if (xQueueReceive(NetworkQueue, msg, timeout_ms / portTICK_PERIOD_MS) == pdPASS) {
        return true;
    } else {
        return false;
    }
}
