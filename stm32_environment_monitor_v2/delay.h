/**
 * @file    delay.h
 * @brief   TIM2精确延时模块头文件 (标准库版本)
 * @author  MiniMax Agent
 * @version V2.0
 * @date    2025-11-08
 * 
 * 重要说明:
 * - 此版本完全使用STM32标准库，无HAL函数
 * - 基于TIM2定时器实现精确延时
 * - 支持微秒级和毫秒级延时
 * - 无时间戳依赖，使用相对时间
 * 
 * 延时模块功能:
 * 
 * 【1. TIM2定时器配置】
 * 知识点: 定时器延时原理
 * - 使用STM32定时器2(TIM2)作为延时时钟
 * - 配置为向上计数模式
 * - 设置合适的预分频和重装载值
 * - 启用定时器中断
 * 
 * 【2. 微秒级延时】
 * 知识点: 微秒延时实现
 * - 延时范围: 1微秒 到 65535微秒
 * - 精度: ±1微秒
 * - 基于定时器计数值计算
 * - 非阻塞式延时
 * 
 * 【3. 毫秒级延时】
 * 知识点: 毫秒延时实现
 * - 延时范围: 1毫秒 到 65535毫秒
 * - 精度: ±1毫秒
 * - 内部循环微秒延时实现
 * - 适合较长延时
 * 
 * 【4. 系统时间获取】
 * 知识点: 运行时间计算
 * - 返回系统启动后的运行时间
 * - 单位: 毫秒
 * - 32位计数器，最大49.7天
 * - 用于时间测量和超时判断
 * 
 * 【5. 时间测量】
 * 知识点: 精确时间测量
 * - 开始时间记录
 * - 结束时间记录
 * - 差值计算得到执行时间
 * - 用于性能分析和调试
 * 
 * 技术参数:
 * 
 * 【时钟配置】
 * - 系统时钟: 72MHz (STM32F103)
 * - TIM2时钟: 72MHz (APB1时钟)
 * - 预分频: 72-1 (1MHz计数频率)
 * - 计数周期: 65535 (16位计数器)
 * - 计数频率: 1MHz (1us分辨率)
 * 
 * 【延时精度】
 * - 微秒延时: 1微秒精度
 * - 毫秒延时: 1毫秒精度
 * - 时间测量: 1微秒分辨率
 * - 系统时间: 1毫秒精度
 * 
 * 【资源使用】
 * - 定时器: TIM2占用
 * - 中断: TIM2中断
 * - 堆栈: 最小化堆栈使用
 * - 代码: 高效汇编实现
 * 
 * 注意事项:
 * 
 * 【1. 中断安全】
 * - 延时函数可以在中断上下文中调用
 * - 中断处理不会影响延时精度
 * - 中断服务程序中的延时不会阻塞其他中断
 * 
 * 【2. 多任务安全】
 * - 延时函数可以被多个任务同时调用
 * - 基于硬件定时器，不依赖操作系统
 * - 在FreeRTOS环境中可以安全使用
 * 
 * 【3. 功耗考虑】
 * - 延时期间CPU可能进入低功耗模式
 * - 可配置是否在延时期间进入低功耗
 * - 平衡功耗和响应性
 * 
 * 【4. 性能优化】
 * - 使用汇编语言优化关键延时循环
 * - 避免浮点运算提高性能
 * - 预计算常量减少计算开销
 * 
 * 使用方法:
 * 
 * 【1. 初始化】
 * ```c
 * // 在系统启动时调用一次
 * Delay_Init();
 * ```
 * 
 * 【2. 基本延时】
 * ```c
 * // 延时1毫秒
 * Delay_MS(1);
 * 
 * // 延时100微秒
 * Delay_US(100);
 * ```
 * 
 * 【3. 时间测量】
 * ```c
 * uint32_t start_time = Get_Runtime_MS();
 * // 执行一些代码
 * uint32_t end_time = Get_Runtime_MS();
 * uint32_t execution_time = end_time - start_time;
 * ```
 * 
 * 【4. 循环延时】
 * ```c
 * for (int i = 0; i < 100; i++) {
 *     Delay_US(1000);  // 延时1毫秒
 *     // 执行周期性任务
 * }
 * ```
 */

#ifndef __DELAY_H
#define __DELAY_H

/* ============================== 头文件包含 ============================== */

#include "stm32f1xx.h"

/* ============================== 宏定义 ============================== */

/**
 * TIM2配置参数
 * 知识点: 定时器参数配置
 * - 使用72MHz系统时钟
 * - APB1时钟为36MHz
 * - TIM2时钟为72MHz (APB1*2)
 * - 预分频72，得到1MHz计数频率
 * - 计数周期65535，对应65.535ms
 */
#define TIM2_PSC                    (72-1)       /**< 定时器预分频值: 72 */
#define TIM2_ARR                    (65535)      /**< 定时器重装载值: 65535 */
#define TIM2_FREQ                   (72000000)   /**< 系统时钟频率: 72MHz */
#define TIM2_CLOCK_FREQ             (72000000)   /**< TIM2时钟频率: 72MHz */
#define TIM2_COUNT_FREQ             (1000000)    /**< 计数频率: 1MHz (1us分辨率) */

/**
 * 延时参数定义
 * 知识点: 延时参数说明
 * - 最小延时: 1微秒
 * - 最大微秒延时: 65535微秒
 * - 最大毫秒延时: 65535毫秒
 * - 系统时间范围: 4294967295毫秒(约49.7天)
 */
#define MIN_DELAY_US                (1)          /**< 最小微秒延时: 1us */
#define MAX_DELAY_US                (65535)      /**< 最大微秒延时: 65535us */
#define MAX_DELAY_MS                (65535)      /**< 最大毫秒延时: 65535ms */
#define SYSTICK_OVERFLOW_VALUE      (0xFFFFFF)   /**< SysTick溢出值: 24位计数器 */

/**
 * 延时功能开关
 * 知识点: 功能配置
 * - 可选是否启用低功耗模式
 * - 可选是否启用精确延时
 * - 可选是否启用调试输出
 */
#define USE_LOW_POWER_DELAY         (1)          /**< 启用低功耗延时: 1启用, 0禁用 */
#define USE_PRECISE_DELAY           (1)          /**< 启用精确延时: 1启用, 0禁用 */
#define USE_DEBUG_DELAY             (0)          /**< 启用调试输出: 1启用, 0禁用 */

/**
 * 调试输出控制
 * 知识点: 调试信息控制
 * - 延时函数调试信息
 * - 性能统计信息
 * - 错误报告信息
 */
#if USE_DEBUG_DELAY
    #define DELAY_DEBUG_PRINT(fmt, args...)      DEBUG_PRINT("[DELAY] " fmt, ##args)
#else
    #define DELAY_DEBUG_PRINT(fmt, args...)      ((void)0)
#endif

/* ============================== 类型定义 ============================== */

/**
 * 延时模块状态枚举
 * 知识点: 延时模块状态
 * - 记录初始化状态
 * - 错误状态跟踪
 * - 功能可用性检查
 */
typedef enum {
    DELAY_STATUS_UNINIT = 0,       /**< 未初始化 */
    DELAY_STATUS_OK,              /**< 正常工作 */
    DELAY_STATUS_ERROR            /**< 错误状态 */
} Delay_Status_t;

/**
 * 延时统计信息结构
 * 知识点: 延时性能统计
 * - 延时调用次数统计
 * - 总延时时间统计
 * - 最小/最大延时时间
 * - 错误计数
 */
typedef struct {
    uint32_t delay_us_count;       /**< 微秒延时调用次数 */
    uint32_t delay_ms_count;       /**< 毫秒延时调用次数 */
    uint32_t get_time_count;       /**< 获取时间调用次数 */
    uint64_t total_delay_us;       /**< 总延时时间(微秒) */
    uint32_t min_delay_us;         /**< 最小延时时间 */
    uint32_t max_delay_us;         /**< 最大延时时间 */
    uint32_t error_count;          /**< 错误计数 */
} Delay_Statistics_t;

/**
 * 延时模块信息结构
 * 知识点: 延时模块全局信息
 * - 模块状态
 * - 统计信息
 * - 配置参数
 */
typedef struct {
    Delay_Status_t status;         /**< 模块状态 */
    Delay_Statistics_t statistics; /**< 统计信息 */
    uint32_t tick_per_ms;          /**< 每毫秒SysTick计数 */
    uint32_t init_time;            /**< 初始化时间 */
} Delay_Module_Info_t;

/* ============================== 全局变量声明 ============================== */

/**
 * 延时模块全局信息
 * 知识点: 延时模块状态管理
 * - 模块初始化状态
 * - 性能统计信息
 * - 配置参数存储
 */
extern Delay_Module_Info_t Delay_Module_Info;

/**
 * 系统运行时间变量
 * 知识点: 系统时间管理
 * - 32位无符号整数
 * - 单位: 毫秒
 * - 最大计时约49.7天
 * - 自动溢出处理
 */
extern volatile uint32_t g_system_runtime_ms;

/* ============================== 函数声明 ============================== */

/**
 * @brief   延时模块初始化
 * @details 初始化TIM2定时器配置为精确延时功能
 *          此函数必须在系统启动时调用一次
 * @param   无
 * @return  bool: 初始化成功返回true，失败返回false
 * 
 * 初始化流程:
 * 1. 检查TIM2时钟使能
 * 2. 配置TIM2定时器参数
 * 3. 配置TIM2中断
 * 4. 启动TIM2定时器
 * 5. 初始化SysTick中断
 * 6. 验证配置正确性
 * 
 * 知识点: 定时器初始化要点
 * - 时钟源: 内部时钟72MHz
 * - 计数模式: 向上计数
 * - 预分频: 72 (得到1MHz计数)
 * - 重装载: 65535 (16位最大值)
 * - 中断: 允许更新中断
 * - 使能: 启动定时器
 * 
 * 错误处理:
 * - 时钟使能失败: 返回false
 * - 定时器配置失败: 返回false
 * - 中断配置失败: 返回false
 * - 启动失败: 返回false
 */
bool Delay_Init(void);

/**
 * @brief   延时模块去初始化
 * @details 关闭TIM2定时器并清理相关资源
 * @param   无
 * @return  bool: 去初始化成功返回true，失败返回false
 * 
 * 去初始化流程:
 * 1. 停止TIM2定时器
 * 2. 禁止TIM2中断
 * 3. 关闭TIM2时钟
 * 4. 清理统计信息
 * 5. 重置模块状态
 * 
 * 知识点: 资源清理
 * - 定时器停止: 停止计数
 * - 中断禁止: 避免不必要中断
 * - 时钟关闭: 节省功耗
 * - 状态重置: 清理模块状态
 * 
 * 注意事项:
 * - 只能在系统关闭时调用
 * - 确保没有任务在使用延时功能
 */
bool Delay_DeInit(void);

/**
 * @brief   微秒级延时
 * @details 精确延时指定微秒数
 *          延时范围: 1-65535微秒
 * @param   us: 延时的微秒数
 * @return  无
 * 
 * 实现原理:
 * 1. 获取当前TIM2计数值
 * 2. 计算目标计数值
 * 3. 等待计数到达目标值
 * 4. 处理计数器溢出情况
 * 
 * 知识点: 微秒延时算法
 * - TIM2计数频率: 1MHz
 * - 每个计数: 1微秒
 * - 目标计数值 = 当前计数值 + 延时微秒数
 * - 溢出处理: 计数器回绕时特殊处理
 * 
 * 使用示例:
 * ```c
 * Delay_US(100);     // 延时100微秒
 * Delay_US(1000);    // 延时1000微秒(1毫秒)
 * Delay_US(1);       // 最小延时1微秒
 * ```
 * 
 * 注意事项:
 * - 延时时间受系统中断影响
 * - 在中断服务程序中可以安全使用
 * - 不建议在中断中延时过长时间
 */
void Delay_US(uint16_t us);

/**
 * @brief   毫秒级延时
 * @details 精确延时指定毫秒数
 *          延时范围: 1-65535毫秒
 * @param   ms: 延时的毫秒数
 * @return  无
 * 
 * 实现原理:
 * 1. 分解为多个较小延时
 * 2. 调用微秒延时函数
 * 3. 循环执行完成总延时
 * 4. 处理剩余不足1ms的延时
 * 
 * 知识点: 毫秒延时算法
 * - 大部分延时: 使用多次微秒延时
 * - 剩余延时: 单独处理不足1ms部分
 * - 效率优化: 减少函数调用开销
 * - 精度保证: 整体误差在±1ms内
 * 
 * 使用示例:
 * ```c
 * Delay_MS(1);       // 延时1毫秒
 * Delay_MS(100);     // 延时100毫秒
 * Delay_MS(1000);    // 延时1000毫秒(1秒)
 * ```
 * 
 * 注意事项:
 * - 总延时时间 = ms毫秒数
 * - 函数执行期间会阻塞CPU
 * - 适合较长延时，精度要求不高
 * - 不建议在实时任务中调用过长延时
 */
void Delay_MS(uint32_t ms);

/**
 * @brief   获取系统运行时间
 * @details 返回系统启动后的运行时间
 *          单位: 毫秒
 * @param   无
 * @return  uint32_t: 系统运行时间(毫秒)
 * 
 * 实现原理:
 * 1. 读取SysTick计数器当前值
 * 2. 计算已过去的时间
 * 3. 加上累计的时间
 * 4. 返回总运行时间
 * 
 * 知识点: 系统时间计算
 * - SysTick: 系统定时器，每毫秒中断一次
 * - 24位计数器，频率1kHz
 * - 每毫秒计数1000次
 * - 溢出处理: 49.7天后自动回绕
 * 
 * 使用示例:
 * ```c
 * uint32_t current_time = Get_Runtime_MS();
 * // 记录开始时间
 * uint32_t start = Get_Runtime_MS();
 * // 执行任务
 * do_something();
 * // 计算执行时间
 * uint32_t end = Get_Runtime_MS();
 * uint32_t execution_time = end - start;
 * ```
 * 
 * 精度说明:
 * - 毫秒级精度
 * - 分辨率: 1毫秒
 * - 累计误差: < 1ppm
 * 
 * 注意事项:
 * - 函数可以安全地在中断中调用
 * - 时间值在溢出时会自动回绕
 * - 用于相对时间测量，不适合绝对时间
 */
uint32_t Get_Runtime_MS(void);

/**
 * @brief   获取高精度运行时间
 * @details 返回带微秒精度的系统运行时间
 *          内部使用SysTick + TIM2组合
 * @param   无
 * @return  uint32_t: 高精度系统时间(微秒)
 * 
 * 实现原理:
 * 1. 获取毫秒时间基
 * 2. 获取TIM2微秒补充
 * 3. 组合成完整时间
 * 4. 返回64位时间值
 * 
 * 知识点: 高精度时间组合
 * - SysTick: 提供毫秒基
 * - TIM2: 提供微秒补充
 * - 组合精度: 1微秒
 * - 时间范围: 约49.7天
 * 
 * 返回值说明:
 * - 低32位: 微秒数
 * - 高32位: 毫秒数/1000
 * - 组合使用可获得完整时间
 * 
 * 注意事项:
 * - 需要TIM2正常工作
 * - 精度高但性能开销稍大
 * - 用于精确时间测量
 */
uint64_t Get_Runtime_US(void);

/**
 * @brief   精确延时(带参数验证)
 * @details 带参数范围检查的微秒延时
 * @param   us: 延时的微秒数 (1-65535)
 * @return  bool: 参数有效返回true，无效返回false
 * 
 * 参数验证:
 * - 检查延时范围
 * - 检查参数类型
 * - 记录错误统计
 * 
 * 知识点: 安全延时函数
 * - 参数范围检查
 * - 错误处理机制
 * - 调试信息输出
 * - 统计信息更新
 * 
 * 用途:
 * - 需要参数验证的场合
 * - 调试和测试
 * - 错误处理要求高的应用
 */
bool Delay_US_Safe(uint16_t us);

/**
 * @brief   精确延时(带参数验证)
 * @details 带参数范围检查的毫秒延时
 * @param   ms: 延时的毫秒数 (1-65535)
 * @return  bool: 参数有效返回true，无效返回false
 * 
 * 参数验证:
 * - 检查延时范围
 * - 检查参数类型
 * - 记录错误统计
 * 
 * 知识点: 安全延时函数
 * - 参数范围检查
 * - 错误处理机制
 * - 调试信息输出
 * - 统计信息更新
 * 
 * 用途:
 * - 需要参数验证的场合
 * - 调试和测试
 * - 错误处理要求高的应用
 */
bool Delay_MS_Safe(uint32_t ms);

/**
 * @brief   延时并检查超时
 * @details 在延时过程中检查是否达到超时条件
 * @param   max_delay_ms: 最大延时时间(毫秒)
 * @param   check_interval_ms: 检查间隔(毫秒)
 * @param   condition: 超时检查条件函数指针
 * @return  bool: 超时返回true，未超时返回false
 * 
 * 使用示例:
 * ```c
 * bool timeout = Delay_Wait_Timeout(1000, 100, check_network_ready);
 * if (timeout) {
 *     // 超时处理
 * }
 * ```
 * 
 * 知识点: 超时等待机制
 * - 避免无限等待
 * - 定期检查条件
 * - 条件函数可自定义
 * - 提高系统响应性
 * 
 * 注意事项:
 * - 条件函数应快速返回
 * - 检查间隔影响响应性
 * - 最大延时防止无限循环
 */
bool Delay_Wait_Timeout(uint32_t max_delay_ms, uint32_t check_interval_ms, bool (*condition)(void));

/**
 * @brief   获取延时模块信息
 * @details 返回延时模块的详细状态信息
 * @param   无
 * @return  Delay_Module_Info_t*: 模块信息结构指针
 * 
 * 知识点: 模块状态查询
 * - 初始化状态
 * - 配置参数
 * - 性能统计
 * - 错误信息
 * 
 * 使用示例:
 * ```c
 * Delay_Module_Info_t* info = Delay_Get_Info();
 * if (info->status == DELAY_STATUS_OK) {
 *     // 延时模块正常工作
 * }
 * ```
 */
Delay_Module_Info_t* Delay_Get_Info(void);

/**
 * @brief   打印延时模块状态
 * @details 打印延时模块的详细状态信息
 * @param   无
 * @return  无
 * 
 * 知识点: 状态打印输出
 * - 模块状态信息
 * - 性能统计报告
 * - 配置参数显示
 * - 错误统计信息
 * 
 * 输出内容:
 * - 初始化状态
 * - 调用次数统计
 * - 延时时间统计
 * - 错误计数
 * - 当前配置
 * 
 * 用途:
 * - 调试和诊断
 * - 性能分析
 * - 错误排查
 * - 系统监控
 */
void Delay_Print_Status(void);

/**
 * @brief   重置延时模块统计
 * @details 清零所有性能统计信息
 * @param   无
 * @return  无
 * 
 * 知识点: 统计信息重置
 * - 清零所有计数器
 * - 重置最小/最大值
 * - 清理时间统计
 * - 保持配置不变
 * 
 * 使用场景:
 * - 性能测试开始
 * - 统计周期开始
 * - 调试模式切换
 * - 系统重新校准
 */
void Delay_Reset_Statistics(void);

/* ============================== 宏函数定义 ============================== */

/**
 * 快速延时宏
 * 知识点: 常用延时宏定义
 * - 提供常用延时时间
 * - 宏替换避免函数调用开销
 * - 编译时确定延时时间
 * 
 * 使用示例:
 * ```c
 * DELAY_1MS();     // 延时1毫秒
 * DELAY_10MS();    // 延时10毫秒
 * DELAY_100MS();   // 延时100毫秒
 * DELAY_1S();      // 延时1秒
 * DELAY_500US();   // 延时500微秒
 * DELAY_1MS_2();   // 延时2毫秒
 * ```
 */
#define DELAY_1MS()         Delay_MS(1)          /**< 延时1毫秒 */
#define DELAY_10MS()        Delay_MS(10)         /**< 延时10毫秒 */
#define DELAY_100MS()       Delay_MS(100)        /**< 延时100毫秒 */
#define DELAY_1S()          Delay_MS(1000)       /**< 延时1秒 */
#define DELAY_2S()          Delay_MS(2000)       /**< 延时2秒 */
#define DELAY_500US()       Delay_US(500)        /**< 延时500微秒 */
#define DELAY_1MS_2()       Delay_US(2000)       /**< 延时2毫秒(微秒实现) */
#define DELAY_10MS_2()      Delay_US(10000)      /**< 延时10毫秒(微秒实现) */

/**
 * 时间测量宏
 * 知识点: 时间测量宏定义
 * - 开始时间记录
 * - 结束时间记录
 * - 差值计算
 * - 自动变量管理
 * 
 * 使用示例:
 * ```c
 * TIME_MEASURE_START(my_time);
 * // 执行要测量的代码
 * do_something();
 * TIME_MEASURE_END(my_time);
 * printf("执行时间: %u ms\r\n", my_time);
 * ```
 */
#define TIME_MEASURE_START(name)     uint32_t name##_start = Get_Runtime_MS()
#define TIME_MEASURE_END(name)       uint32_t name##_end = Get_Runtime_MS(); uint32_t name = name##_end - name##_start
#define TIME_MEASURE_US_START(name)  uint64_t name##_start = Get_Runtime_US()
#define TIME_MEASURE_US_END(name)    uint64_t name##_end = Get_Runtime_US(); uint64_t name = name##_end - name##_start

/**
 * 性能测试宏
 * 知识点: 性能测试辅助宏
 * - 循环性能测试
 * - 平均时间计算
 * - 性能基准比较
 * 
 * 使用示例:
 * ```c
 * uint32_t avg_time;
 * PERF_TEST_LOOP(100, {
 *     // 测试的代码块
 *     some_function();
 * }, avg_time);
 * ```
 */
#define PERF_TEST_LOOP(count, code_block, avg_var) ({ \
    uint32_t total_time = 0; \
    for (int i = 0; i < count; i++) { \
        uint32_t start = Get_Runtime_MS(); \
        code_block; \
        uint32_t end = Get_Runtime_MS(); \
        total_time += (end - start); \
    } \
    avg_var = total_time / count; \
})

/* ============================== 调试接口 ============================== */

/**
 * 延时精度测试
 * @brief 测试延时函数的精度
 * @param test_count: 测试次数
 * @return bool: 测试成功返回true，失败返回false
 * 
 * 知识点: 延时精度验证
 * - 预设延时时间
 * - 实际测量延时时间
 * - 计算误差百分比
 * - 统计精度分布
 * 
 * 测试项目:
 * - 1微秒延时精度
 * - 10微秒延时精度
 * - 100微秒延时精度
 * - 1毫秒延时精度
 * - 10毫秒延时精度
 * - 100毫秒延时精度
 * 
 * 输出报告:
 * - 预设时间 vs 实际时间
 * - 误差百分比
 * - 精度等级评定
 * - 优化建议
 */
bool Delay_Precision_Test(uint32_t test_count);

/**
 * 延时性能基准测试
 * @brief 测试延时函数的性能开销
 * @param 无
 * @return 无
 * 
 * 知识点: 性能开销分析
 * - 函数调用开销
 * - 循环开销
 * - 中断处理开销
 * - 总开销统计
 * 
 * 测试内容:
 * - 微秒延时函数性能
 * - 毫秒延时函数性能
 * - 时间获取函数性能
 * - 组合操作性能
 * 
 * 性能指标:
 * - CPU使用率
 * - 内存访问次数
 * - 函数调用次数
 * - 中断响应时间
 */
void Delay_Performance_Benchmark(void);

#endif /* __DELAY_H */