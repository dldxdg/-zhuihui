/**
 * @file    usart.h
 * @brief   串口通信和AT指令状态机头文件 (标准库版本)
 * @author  MiniMax Agent
 * @version V2.0
 * @date    2025-11-08
 * 
 * 知识点总结:
 * 
 * 【串口通信基础】
 * 1. 串口(USART/UART)是一种异步串行通信接口
 * 2. 数据按位依次传输，接收端需要按相同速率接收
 * 3. 需要配置波特率、数据位、停止位、校验位
 * 4. 支持全双工通信(同时发送和接收)
 * 
 * 【AT指令集】
 * 1. AT指令是调制解调器的命令语言
 * 2. 以"AT"开头，后跟命令和参数
 * 3. 返回"OK"表示成功，"ERROR"表示失败
 * 4. ESP-01s WiFi模块使用AT指令进行网络配置
 * 
 * 【状态机设计】
 * 1. 状态机用于管理复杂的时序操作
 * 2. 避免使用阻塞延时，提高系统响应性
 * 3. 适合处理AT指令等需要等待响应的操作
 * 4. 状态转换由事件触发，清晰可控
 */

#ifndef __USART_H
#define __USART_H

/* ============================== 包含头文件 ============================== */

#include "stm32f1xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ============================== 宏定义 ============================== */

/**
 * 串口缓冲区大小
 * 知识点: 缓冲区大小设计
 * - 需要足够大以避免数据丢失
 * - 考虑最长AT指令响应长度
 * - 合理平衡内存使用和可靠性
 */
#define USART1_RX_BUFFER_SIZE      256  // USART1接收缓冲区(调试)
#define USART2_RX_BUFFER_SIZE      256  // USART2接收缓冲区(蓝牙)
#define USART3_RX_BUFFER_SIZE      512  // USART3接收缓冲区(WiFi - 需要更大)

/**
 * 环形缓冲区大小
 * 知识点: 环形缓冲区设计
 * - 避免数据丢失
 * - 支持连续接收大量数据
 * - 读写指针分离
 */
#define USART2_RING_BUFFER_SIZE    1024  // 蓝牙环形缓冲区
#define USART3_RING_BUFFER_SIZE    2048  // WiFi环形缓冲区(更大)

/**
 * AT指令相关定义
 * 知识点: AT指令处理参数
 * - 超时时间: 等待响应的最大时间
 * - 重试次数: 失败后重试的次数
 * - 响应最大长度: AT指令响应的最大长度
 */
#define AT_CMD_TIMEOUT_MS          5000   // AT指令超时时间5秒
#define AT_CMD_MAX_RETRY           3      // 最大重试次数
#define AT_RESPONSE_MAX_LENGTH     128    // 响应最大长度

/**
 * WiFi连接参数
 * 知识点: WiFi网络参数
 * - 连接超时时间: 等待WiFi连接成功的最长时间
 * - 心跳间隔: 检测WiFi连接状态的时间间隔
 * - 断线重连间隔: WiFi断开后重新连接的间隔
 */
#define WIFI_CONNECT_TIMEOUT_MS    30000  // WiFi连接超时30秒
#define WIFI_HEARTBEAT_INTERVAL_MS 5000   // WiFi心跳5秒
#define WIFI_RECONNECT_INTERVAL_MS 10000  // 断线重连间隔10秒

/* ============================== 数据结构定义 ============================== */

/**
 * 串口环形缓冲区结构
 * 知识点: 环形缓冲区设计
 * - 避免数据覆盖
 * - 读写指针分离
 * - 支持连续接收数据
 * - 空间满时停止接收
 */
typedef struct {
    uint8_t buffer[USART2_RING_BUFFER_SIZE];  // 环形缓冲区
    uint16_t head;                            // 写指针
    uint16_t tail;                            // 读指针
    uint16_t count;                           // 当前数据量
} USART2_RingBuffer_t;

/**
 * WiFi环形缓冲区结构
 * 知识点: 大容量环形缓冲区
 * - WiFi通信数据量大，需要更大缓冲区
 * - 可能接收完整的网页响应
 * - 需要更精细的解析
 */
typedef struct {
    uint8_t buffer[USART3_RING_BUFFER_SIZE];  // 环形缓冲区
    uint16_t head;                            // 写指针
    uint16_t tail;                            // 读指针
    uint16_t count;                           // 当前数据量
} USART3_RingBuffer_t;

/**
 * AT指令状态枚举
 * 知识点: AT指令状态机
 * - 每个状态代表AT指令执行的不同阶段
 * - 状态转换由事件驱动
 * - 超时和错误处理机制
 */
typedef enum {
    AT_STATE_IDLE,        // 空闲状态，等待指令
    AT_STATE_SENDING,     // 正在发送AT指令
    AT_STATE_WAITING,     // 等待响应
    AT_STATE_RECEIVING,   // 接收响应数据
    AT_STATE_SUCCESS,     // 指令执行成功
    AT_STATE_ERROR,       // 指令执行失败
    AT_STATE_TIMEOUT      // 指令执行超时
} AT_State_t;

/**
 * 网络连接状态枚举
 * 知识点: 网络连接状态机
 * - 表示WiFi模块的网络连接状态
 * - 用于自动重连机制
 * - 指导数据发送策略
 */
typedef enum {
    NETWORK_STATE_DISCONNECTED,  // 未连接WiFi
    NETWORK_STATE_CONNECTING,    // 正在连接WiFi
    NETWORK_STATE_CONNECTED,     // 已连接WiFi
    NETWORK_STATE_ERROR          // 网络错误
} Network_State_t;

/**
 * 网络消息类型枚举
 * 知识点: 消息类型定义
 * - 区分不同类型的网络消息
 * - 便于消息路由和处理
 */
typedef enum {
    NETWORK_MSG_NONE,            // 无消息
    NETWORK_MSG_AT_RESPONSE,     // AT指令响应
    NETWORK_MSG_WIFI_DATA,       // WiFi数据
    NETWORK_MSG_BLUETOOTH_DATA,  // 蓝牙数据
    NETWORK_MSG_NETWORK_STATUS,  // 网络状态消息
    NETWORK_MSG_ERROR            // 网络错误消息
} Network_Message_Type_t;

/**
 * AT指令结构
 * 知识点: AT指令定义
 * - 包含指令字符串、期望响应、超时参数
 * - 支持参数化的AT指令
 * - 重试机制配置
 */
typedef struct {
    char command[32];            // AT指令字符串
    char expected_response[64];  // 期望的响应字符串
    uint32_t timeout_ms;         // 超时时间(毫秒)
    uint8_t max_retry;           // 最大重试次数
    uint8_t retry_count;         // 当前重试次数
    AT_State_t state;            // 当前状态
} AT_Command_t;

/**
 * 网络消息结构
 * 知识点: 消息传递结构
 * - 统一的消息格式
 * - 包含消息类型和具体数据
 * - 便于队列传递
 */
typedef struct {
    Network_Message_Type_t type; // 消息类型
    union {
        char at_response[AT_RESPONSE_MAX_LENGTH];  // AT指令响应
        uint8_t wifi_byte;                         // WiFi单字节数据
        uint8_t bluetooth_byte;                    // 蓝牙单字节数据
        Network_State_t network_state;             // 网络状态
        int8_t error_code;                         // 错误代码
    } data;
} Network_Message_t;

/**
 * AT指令状态机结构
 * 知识点: 状态机状态保存
 * - 保存当前执行的AT指令
 * - 记录状态机当前状态
 * - 保存响应数据缓冲区
 */
typedef struct {
    AT_Command_t current_cmd;      // 当前执行的指令
    AT_State_t state;              // 当前状态
    char response_buffer[AT_RESPONSE_MAX_LENGTH];  // 响应缓冲区
    uint16_t response_length;      // 响应长度
    uint32_t start_time;           // 开始时间
    uint32_t last_activity_time;   // 最后活动时间
    bool busy;                     // 是否正在执行指令
} AT_StateMachine_t;

/**
 * WiFi管理结构
 * 知识点: WiFi连接管理
 * - 管理WiFi连接状态和配置
 * - 存储网络参数
 * - 跟踪连接历史
 */
typedef struct {
    Network_State_t state;         // 当前网络状态
    char ssid[32];                 // WiFi网络名称
    char password[32];             // WiFi密码
    char server_ip[16];            // 服务器IP地址
    uint16_t server_port;          // 服务器端口号
    uint32_t last_connect_time;    // 最后连接时间
    uint32_t heartbeat_interval;   // 心跳间隔
    uint8_t connect_retry_count;   // 连接重试次数
    bool auto_reconnect;           // 自动重连标志
} WiFi_Manager_t;

/**
 * 系统配置结构
 * 知识点: 系统配置参数
 * - 存储在内部Flash的系统配置
 * - WiFi网络参数
 * - 服务器连接参数
 * - 运行参数
 */
typedef struct {
    char wifi_ssid[32];            // WiFi网络名称
    char wifi_password[32];        // WiFi密码
    char server_ip[16];            // 服务器IP地址
    uint16_t server_port;          // 服务器端口号
    uint8_t sample_interval;       // 采集间隔(秒)
    uint8_t send_interval;         // 发送间隔(秒)
} System_Config_t;

/* ============================== 全局变量声明 ============================== */

// 环形缓冲区
extern USART2_RingBuffer_t USART2_RingBuffer;  // 蓝牙环形缓冲区
extern USART3_RingBuffer_t USART3_RingBuffer;  // WiFi环形缓冲区

// 状态机
extern AT_StateMachine_t AT_StateMachine;      // AT指令状态机
extern WiFi_Manager_t WiFi_Manager;            // WiFi管理器

// 系统配置
extern System_Config_t System_Config;          // 系统配置

/* ============================== 函数声明 ============================== */

/**
 * 串口模块初始化函数
 * 知识点: 串口初始化流程
 * 1. 初始化串口硬件
 * 2. 初始化环形缓冲区
 * 3. 初始化状态机
 * 4. 配置中断处理
 */
void USART_Module_Init(void);

/**
 * 环形缓冲区初始化函数
 * 知识点: 环形缓冲区初始化
 * - 重置读写指针
 * - 清空数据缓冲区
 * - 初始化计数器
 */
void USART2_RingBuffer_Init(void);
void USART3_RingBuffer_Init(void);

/**
 * 环形缓冲区写入函数
 * 知识点: 缓冲区写入
 * - 将数据写入环形缓冲区
 * - 处理缓冲区满的情况
 * - 更新写指针和计数器
 */
void USART2_RingBuffer_Write(uint8_t data);
void USART3_RingBuffer_Write(uint8_t data);

/**
 * 环形缓冲区读取函数
 * 知识点: 缓冲区读取
 * - 从环形缓冲区读取数据
 * - 处理缓冲区空的情况
 * - 更新读指针和计数器
 */
bool USART2_RingBuffer_Read(uint8_t *data);
bool USART3_RingBuffer_Read(uint8_t *data);

/**
 * 环形缓冲区查询函数
 * 知识点: 缓冲区查询
 * - 查看缓冲区中是否有指定数据
 * - 不从缓冲区中移除数据
 * - 支持字符串搜索
 */
bool USART2_RingBuffer_Contains(const char *str);
bool USART3_RingBuffer_Contains(const char *str);

/**
 * 环形缓冲区清空函数
 * 知识点: 缓冲区清空
 * - 清空所有数据
 * - 重置指针和计数器
 * - 用于重新开始接收
 */
void USART2_RingBuffer_Clear(void);
void USART3_RingBuffer_Clear(void);

/**
 * AT指令执行函数
 * 知识点: AT指令异步执行
 * - 设置AT指令参数
 * - 启动状态机
 * - 非阻塞执行
 * @param command AT指令字符串
 * @param expected_response 期望响应
 * @param timeout_ms 超时时间
 * @param max_retry 最大重试次数
 * @return bool 执行结果
 */
bool AT_Execute_Command(const char *command, const char *expected_response, 
                       uint32_t timeout_ms, uint8_t max_retry);

/**
 * AT指令状态机更新函数
 * 知识点: 状态机更新
 * - 周期性调用更新状态机
 * - 处理状态转换
 * - 超时检测
 * - 响应解析
 */
void AT_StateMachine_Update(void);

/**
 * AT指令发送函数
 * 知识点: AT指令发送
 * - 发送AT指令到WiFi模块
 * - 记录发送时间
 * - 设置等待状态
 */
void AT_Send_Command(void);

/**
 * AT指令响应处理函数
 * 知识点: 响应数据处理
 * - 读取串口数据
 * - 解析响应内容
 * - 验证响应是否匹配
 */
void AT_Process_Response(void);

/**
 * AT指令状态获取函数
 * 知识点: 状态查询
 * - 获取当前AT指令执行状态
 * - 用于任务间通信
 * @return AT_State_t 当前状态
 */
AT_State_t AT_Get_State(void);

/**
 * AT指令等待完成函数
 * 知识点: 阻塞等待
 * - 等待AT指令执行完成
 * - 避免死等，有超时保护
 * @param timeout_ms 最大等待时间
 * @return bool 是否成功完成
 */
bool AT_Wait_Complete(uint32_t timeout_ms);

/**
 * WiFi模块初始化函数
 * 知识点: WiFi模块初始化
 * 1. 测试AT指令通信
 * 2. 恢复出厂设置(可选)
 * 3. 配置工作模式
 * 4. 设置网络参数
 */
void WiFi_Init(void);

/**
 * WiFi连接函数
 * 知识点: WiFi网络连接
 * - 连接到指定的WiFi网络
 * - 自动重连机制
 * - 连接状态检测
 * @param ssid WiFi网络名称
 * @param password WiFi密码
 * @return bool 连接结果
 */
bool WiFi_Connect(const char *ssid, const char *password);

/**
 * WiFi断开函数
 * 知识点: WiFi网络断开
 * - 主动断开WiFi连接
 * - 清理连接状态
 * - 准备重新连接
 */
void WiFi_Disconnect(void);

/**
 * WiFi连接状态检测函数
 * 知识点: 连接状态检测
 * - 检测WiFi是否仍然连接
 * - 心跳机制
 * - 自动重连逻辑
 * @return bool 连接状态
 */
bool WiFi_Is_Connected(void);

/**
 * WiFi数据发送函数
 * 知识点: 数据发送
 * - 发送数据到指定服务器
 * - TCP连接管理
 * - 发送失败重试
 * @param data 要发送的数据
 * @param length 数据长度
 * @return bool 发送结果
 */
bool WiFi_Send_Data(const uint8_t *data, uint16_t length);

/**
 * WiFi连接管理器更新函数
 * 知识点: 连接管理更新
 * - 更新WiFi连接状态
 * - 处理自动重连
 * - 心跳检测
 */
void WiFi_Manager_Update(void);

/**
 * 网络消息发送函数
 * 知识点: 消息队列发送
 * - 将消息发送到网络队列
 * - 用于任务间通信
 * @param msg 要发送的消息
 * @return bool 发送结果
 */
bool Network_Send_Message(Network_Message_t *msg);

/**
 * 网络消息接收函数
 * 知识点: 消息队列接收
 * - 从网络队列接收消息
 * - 阻塞接收
 * @param msg 接收到的消息
 * @param timeout_ms 超时时间
 * @return bool 接收结果
 */
bool Network_Receive_Message(Network_Message_t *msg, uint32_t timeout_ms);

/**
 * 系统配置加载函数
 * 知识点: 配置参数加载
 * - 从内部Flash读取系统配置
 * - 加载WiFi网络参数
 * - 参数验证和默认值
 */
void System_Config_Load(void);

/**
 * 系统配置保存函数
 * 知识点: 配置参数保存
 * - 将系统配置保存到内部Flash
 * - 页擦除和写入操作
 * - 写入后验证
 */
void System_Config_Save(void);

/* ============================== 调试函数声明 ============================== */

#ifdef DEBUG
/**
 * 串口状态打印函数 (调试模式)
 * 知识点: 串口状态监控
 * - 打印串口缓冲区状态
 * - 打印AT状态机状态
 * - 打印WiFi连接状态
 */
void USART_Print_Status(void);

/**
 * AT指令状态打印函数 (调试模式)
 * 知识点: AT指令调试
 * - 打印当前执行的AT指令
 * - 打印状态机状态
 * - 打印响应数据
 */
void AT_Print_State(void);

/**
 * WiFi状态打印函数 (调试模式)
 * 知识点: WiFi状态监控
 * - 打印WiFi连接状态
 * - 打印网络配置
 * - 打印连接历史
 */
void WiFi_Print_Status(void);

/**
 * 环形缓冲区状态打印函数 (调试模式)
 * 知识点: 缓冲区状态监控
 * - 打印缓冲区使用情况
 * - 打印内容预览
 * - 诊断缓冲区问题
 */
void RingBuffer_Print_Status(void);
#endif

/* ============================== 常用AT指令宏定义 ============================== */

/**
 * 常用AT指令定义
 * 知识点: AT指令标准化
 * - 将常用AT指令定义为宏
 * - 便于使用和维护
 * - 减少字符串错误
 */

// 基本AT指令
#define AT_CMD_TEST             "AT"
#define AT_CMD_RESET            "AT+RST"
#define AT_CMD_VERSION          "AT+GMR"
#define AT_CMD_ECHO_OFF         "ATE0"
#define AT_CMD_ECHO_ON          "ATE1"

// WiFi模式设置
#define AT_CMD_WIFI_MODE_STA    "AT+CWMODE=1"  // Station模式
#define AT_CMD_WIFI_MODE_AP     "AT+CWMODE=2"  // AP模式
#define AT_CMD_WIFI_MODE_BOTH   "AT+CWMODE=3"  // 混合模式

// WiFi连接
#define AT_CMD_WIFI_CONNECT(ssid, password) "AT+CWJAP=\"" sssid "\",\"" password "\""
#define AT_CMD_WIFI_DISCONNECT    "AT+CWQAP"
#define AT_CMD_WIFI_STATUS        "AT+CIPSTATUS"

// TCP连接
#define AT_CMD_TCP_START(ip, port) "AT+CIPSTART=\"TCP\",\"" ip "\"," port
#define AT_CMD_TCP_CLOSE          "AT+CIPCLOSE"
#define AT_CMD_TCP_STATUS         "AT+CIPSTATUS"

// 数据发送
#define AT_CMD_TCP_SEND(length)   "AT+CIPSEND=" length

// 常用响应
#define AT_RESPONSE_OK            "OK"
#define AT_RESPONSE_ERROR         "ERROR"
#define AT_RESPONSE_FAIL          "FAIL"
#define AT_RESPONSE_READY         "ready"
#define AT_RESPONSE_WIFI_CONNECTED "WIFI CONNECTED"
#define AT_RESPONSE_WIFI_GOT_IP   "WIFI GOT IP"

#endif /* __USART_H */
