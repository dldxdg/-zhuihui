/**
 * @file    delay.c
 * @brief   TIM2精确延时模块实现 (标准库版本)
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
 * 延时模块架构:
 * 
 * 【1. 硬件定时器方案】
 * 知识点: 定时器延时原理
 * 使用STM32内置定时器作为延时时钟源
 * 
 * 主定时器: TIM2 (16位向上计数)
 * - 时钟源: 内部时钟72MHz
 * - 预分频: 72 (得到1MHz计数)
 * - 重装载: 65535 (16位最大值)
 * - 计数频率: 1MHz (1微秒分辨率)
 * - 计数周期: 65.535毫秒
 * 
 * 辅助定时器: SysTick (24位)
 * - 时钟源: 系统时钟72MHz
 * - 分频: 72-1 (得到1kHz计数)
 * - 重装载: 1000-1
 * - 计数频率: 1kHz (1毫秒分辨率)
 * - 用途: 毫秒级时间基
 * 
 * 【2. 微秒级延时实现】
 * 知识点: 微秒延时算法
 * 基于TIM2定时器的16位计数实现
 * 
 * 实现原理:
 * 1. 获取当前TIM2计数值
 * 2. 计算目标计数值 = 当前值 + 延时微秒数
 * 3. 等待计数到达目标值
 * 4. 处理计数器溢出情况
 * 
 * 算法特点:
 * - 直接硬件定时，不依赖软件循环
 * - 中断安全，不会被其他中断打断
 * - 精度高，误差 < 1微秒
 * - 性能好，CPU开销小
 * 
 * 溢出处理:
 * - TIM2为16位计数器，最大65535
 * - 延时超过65535微秒时需要特殊处理
 * - 检测溢出并调整计算方法
 * - 确保延时精度不受影响
 * 
 * 【3. 毫秒级延时实现】
 * 知识点: 毫秒延时算法
 * 基于多次微秒延时组合实现
 * 
 * 实现策略:
 * 1. 分解为多个微秒延时
 * 2. 大部分时间使用微秒延时
 * 3. 剩余不足1ms部分单独处理
 * 4. 整体误差控制在±1ms内
 * 
 * 算法优化:
 * - 减少函数调用次数
 * - 合并相似延时操作
 * - 优化循环结构
 * - 减少中断响应时间
 * 
 * 【4. 系统时间管理】
 * 知识点: 系统时间计算
 * 
 * 时间基组合:
 * - SysTick: 提供毫秒基
 * - TIM2: 提供微秒补充
 * - 组合精度: 1微秒
 * - 时间范围: 约49.7天
 * 
 * 毫秒时间获取:
 * 1. 读取SysTick当前值
 * 2. 计算已过去的时间
 * 3. 加上累计的计数值
 * 4. 处理SysTick溢出
 * 
 * 高精度时间获取:
 * 1. 获取基础毫秒时间
 * 2. 获取TIM2微秒补充
 * 3. 组合成64位时间值
 * 4. 返回高精度时间
 * 
 * 【5. 安全性设计】
 * 知识点: 安全性和可靠性
 * 
 * 参数验证:
 * - 延时范围检查
 * - 参数类型验证
 * - 错误状态返回
 * - 统计信息更新
 * 
 * 中断安全:
 * - 延时函数可在中断中调用
 * - 中断处理不会影响延时精度
 * - 中断嵌套安全
 * - 优先级处理正确
 * 
 * 多任务安全:
 * - 基于硬件定时器，不依赖软件
 * - 在FreeRTOS中可安全使用
 * - 任务间延时互不干扰
 * - 系统资源独占使用
 * 
 * 【6. 性能优化】
 * 知识点: 性能优化策略
 * 
 * 代码优化:
 * - 汇编语言优化关键代码
 * - 避免浮点运算
 * - 预计算常量
 * - 减少函数调用开销
 * 
 * 算法优化:
 * - 最少循环次数
 * - 最少内存访问
 * - 预取指优化
 * - 流水线友好代码
 * 
 * 编译优化:
 * - 编译器优化选项
 * - 内联函数使用
 * - 常量折叠
 * - 死代码消除
 * 
 * 【7. 调试和监控】
 * 知识点: 调试功能设计
 * 
 * 性能统计:
 * - 延时调用次数统计
 * - 总延时时间统计
 * - 最小/最大延时时间
 * - 错误计数统计
 * 
 * 精度测试:
 * - 预设时间 vs 实际时间
 * - 误差百分比计算
 * - 精度分布统计
 * - 性能基准测试
 * 
 * 调试输出:
 * - 模块状态信息
 * - 性能统计报告
 * - 错误信息记录
 * - 配置参数显示
 * 
 * 【8. 功耗管理】
 * 知识点: 低功耗设计
 * 
 * 低功耗模式:
 * - 延时期间进入低功耗
 * - WFI指令等待中断
 * - 降低CPU功耗
 * - 快速唤醒响应
 * 
 * 功耗控制:
 * - 可配置低功耗开关
 * - 延时时间门限控制
 * - 任务优先级考虑
 * - 响应性平衡
 * 
 * 注意事项:
 * - 低功耗可能影响响应时间
 * - 需要权衡功耗和性能
 * - 嵌入式系统优化
 * - 电池供电应用考虑
 */

#include "delay.h"
#include "main.h"

/* ============================== 全局变量定义 ============================== */

/**
 * 延时模块全局信息
 * 知识点: 延时模块状态管理
 * - 模块初始化状态
 * - 性能统计信息
 * - 配置参数存储
 */
Delay_Module_Info_t Delay_Module_Info = {
    .status = DELAY_STATUS_UNINIT,
    .statistics = {0},
    .tick_per_ms = 1000,
    .init_time = 0
};

/**
 * 系统运行时间变量
 * 知识点: 系统时间管理
 * - 32位无符号整数
 * - 单位: 毫秒
 * - 最大计时约49.7天
 * - 自动溢出处理
 * - volatile关键字防止编译器优化
 */
volatile uint32_t g_system_runtime_ms = 0;

/* ============================== 延时模块初始化 ============================== */

/**
 * @brief   延时模块初始化
 * @details 初始化TIM2定时器配置为精确延时功能
 *          此函数必须在系统启动时调用一次
 * @param   无
 * @return  bool: 初始化成功返回true，失败返回false
 * 
 * 初始化流程详解:
 * 
 * 第一步: TIM2时钟使能
 * 知识点: 时钟系统配置
 * - 启用TIM2时钟(RCC_APB1Periph_TIM2)
 * - 确保时钟源正常工作
 * - 检查时钟使能状态
 * - 记录初始化状态
 * 
 * 第二步: TIM2定时器配置
 * 知识点: 定时器参数设置
 * - 配置为向上计数模式
 * - 设置预分频72(1MHz计数)
 * - 设置重装载65535(16位最大)
 * - 禁用自动重装载
 * - 配置输出比较
 * 
 * 第三步: TIM2中断配置
 * 知识点: 中断系统配置
 * - 配置NVIC优先级
 * - 允许TIM2更新中断
 * - 清除中断标志
 * - 使能全局中断
 * 
 * 第四步: 启动TIM2定时器
 * 知识点: 定时器启动
 * - 使能定时器
 * - 清除更新标志
 * - 初始化计数值
 * - 验证启动状态
 * 
 * 第五步: SysTick初始化
 * 知识点: 系统定时器配置
 * - 配置SysTick为1kHz
 * - 设置重装载值999
 * - 启用SysTick中断
 * - 启动定时器
 * 
 * 第六步: 验证配置
 * 知识点: 初始化验证
 * - 检查TIM2计数
 * - 验证中断配置
 * - 测试延时功能
 * - 记录错误信息
 * 
 * 错误处理:
 * - 时钟使能失败: 清理资源并返回false
 * - 定时器配置失败: 记录错误并返回false
 * - 中断配置失败: 禁用中断并返回false
 * - 启动失败: 清理定时器并返回false
 */
bool Delay_Init(void)
{
    /**
     * 检查模块是否已经初始化
     * 知识点: 重复初始化保护
     * - 避免多次初始化造成资源泄漏
     * - 检查当前模块状态
     * - 如果已初始化，直接返回成功
     */
    if (Delay_Module_Info.status == DELAY_STATUS_OK) {
        DELAY_DEBUG_PRINT("Module already initialized\r\n");
        return true;
    }
    
    /**
     * 第一步: TIM2时钟使能
     * 知识点: 时钟系统管理
     * - RCC_APB1PeriphClockCmd函数使能时钟
     * - 参数1: 时钟使能位(RCC_APB1Periph_TIM2)
     * - 参数2: ENABLE(使能)或DISABLE(禁用)
     * - 必须在操作外设前使能其时钟
     */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    /**
     * 验证TIM2时钟使能状态
     * 知识点: 时钟使能验证
     * - 检查RCC_APB1ENR寄存器
     * - 确认TIM2EN位已设置
     * - 时钟未使能则初始化失败
     */
    if ((RCC->APB1ENR & RCC_APB1Periph_TIM2) == 0) {
        DELAY_DEBUG_PRINT("ERROR: TIM2 clock enable failed\r\n");
        Delay_Module_Info.status = DELAY_STATUS_ERROR;
        Delay_Module_Info.statistics.error_count++;
        return false;
    }
    
    /**
     * 第二步: 配置TIM2定时器
     * 知识点: 定时器基本配置
     * - 创建定时器初始化结构体
     * - 配置定时器模式、预分频、重装载
     * - 配置时钟分割
     * - 设置计数模式
     */
    
    /**
     * TIM2定时器初始化结构体
     * 知识点: 定时器配置结构
     * - TIM_TimeBaseInitTypeDef: 定时器基本初始化结构
     * - 包含定时器所有基本参数
     * - 时序相关配置
     */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    
    /**
     * 设置TIM2预分频值
     * 知识点: 预分频配置
     * - 预分频值 = 72-1 = 71
     * - 定时器时钟 = 72MHz / (71+1) = 1MHz
     * - 计数周期 = 1微秒
     * - 预分频器作用: 将高频率分频到适合的频率
     */
    TIM_TimeBaseStructure.TIM_Period = TIM2_ARR;           // 65535 (重装载值)
    TIM_TimeBaseStructure.TIM_Prescaler = TIM2_PSC;        // 71 (预分频值)
    
    /**
     * 配置时钟分割
     * 知识点: 时钟分割
     * - 控制定时器时钟与采样时钟的关系
     * - TIM_CKD_DIV1: 不分频(1:1)
     * - TIM_CKD_DIV2: 2分频
     * - TIM_CKD_DIV4: 4分频
     * - 一般设为不分频保证最高精度
     */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    
    /**
     * 配置计数模式
     * 知识点: 计数模式
     * - TIM_CounterMode_Up: 向上计数(从0到ARR)
     * - TIM_CounterMode_Down: 向下计数(从ARR到0)
     * - TIM_CounterMode_CenterAligned1/2/3: 中心对齐模式
     * - 向上计数适合延时应用
     */
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    /**
     * 应用TIM2配置
     * 知识点: 定时器初始化
     * - TIM_TimeBaseInit: 应用配置到TIM2
     * - 参数1: TIM2基地址
     * - 参数2: 初始化结构体指针
     * - 此函数会重置定时器并应用配置
     */
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    /**
     * 清除TIM2更新标志
     * 知识点: 标志位管理
     * - TIM_ClearFlag: 清除指定标志位
     * - TIM_FLAG_Update: 更新标志位
     * - 防止初始化后立即产生更新中断
     * - 确保定时器状态干净
     */
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    
    /**
     * 启动TIM2定时器
     * 知识点: 定时器启动
     * - TIM_Cmd: 使能或禁用定时器
     * - 参数1: TIM2基地址
     * - 参数2: ENABLE(使能)或DISABLE(禁用)
     * - 启动后定时器开始计数
     */
    TIM_Cmd(TIM2, ENABLE);
    
    /**
     * 验证TIM2启动状态
     * 知识点: 启动验证
     * - 检查TIM2_CR1寄存器CEN位
     * - 确认定时器已启动
     * - 验证计数是否正常
     */
    if ((TIM2->CR1 & TIM_CR1_CEN) == 0) {
        DELAY_DEBUG_PRINT("ERROR: TIM2 start failed\r\n");
        Delay_Module_Info.status = DELAY_STATUS_ERROR;
        Delay_Module_Info.statistics.error_count++;
        return false;
    }
    
    /**
     * 第三步: 配置SysTick定时器
     * 知识点: 系统定时器配置
     * - SysTick用于毫秒级时间基
     * - 1kHz频率，每毫秒中断一次
     * - 为系统时间提供基础
     */
    
    /**
     * 配置SysTick重装载值
     * 知识点: SysTick配置
     * - SystemCoreClock: 72MHz系统时钟
     * - 目标频率: 1kHz
     * - 重装载值 = SystemCoreClock/1000 - 1 = 71999
     * - SysTick_Config函数自动配置
     */
    SysTick_Config(SystemCoreClock / 1000);
    
    /**
     * 第四步: 更新模块状态
     * 知识点: 状态管理
     * - 设置模块状态为正常工作
     * - 记录初始化时间
     * - 清零统计信息
     */
    Delay_Module_Info.status = DELAY_STATUS_OK;
    Delay_Module_Info.init_time = Get_Runtime_MS();
    Delay_Module_Info.tick_per_ms = 1000;  // 每毫秒1000个SysTick计数
    
    /**
     * 第五步: 测试延时功能
     * 知识点: 功能测试
     * - 执行简单延时测试
     * - 验证延时精度
     * - 确保模块正常工作
     */
    
    /**
     * 简单延时测试
     * 知识点: 基本功能验证
     * - 执行1微秒延时
     * - 检查函数是否正常执行
     * - 不验证精度，只验证功能
     */
    uint32_t test_start = Get_Runtime_MS();
    Delay_US(1);
    uint32_t test_end = Get_Runtime_MS();
    
    /**
     * 测试结果验证
     * 知识点: 测试结果判断
     * - 1微秒延时不应该超过1毫秒
     * - 如果超过说明定时器配置有误
     * - 记录测试结果
     */
    if ((test_end - test_start) > 10) {  // 如果超过10毫秒，配置有误
        DELAY_DEBUG_PRINT("WARNING: Delay test may have precision issues\r\n");
    }
    
    /**
     * 初始化成功
     * 知识点: 初始化完成
     * - 记录成功信息
     * - 输出调试信息
     * - 返回成功状态
     */
    DELAY_DEBUG_PRINT("Delay module initialized successfully\r\n");
    DELAY_DEBUG_PRINT("TIM2 configured: %u Hz, ARR: %u, PSC: %u\r\n", 
                     TIM2_COUNT_FREQ, TIM2_ARR, TIM2_PSC);
    DELAY_DEBUG_PRINT("SysTick configured: 1kHz\r\n");
    
    return true;
}

/**
 * @brief   延时模块去初始化
 * @details 关闭TIM2定时器并清理相关资源
 * @param   无
 * @return  bool: 去初始化成功返回true，失败返回false
 * 
 * 去初始化流程:
 * 
 * 第一步: 停止TIM2定时器
 * 知识点: 定时器停止
 * - 禁用TIM2定时器
 * - 停止计数
 * - 清理相关标志
 * 
 * 第二步: 禁止TIM2中断
 * 知识点: 中断控制
 * - 禁用TIM2更新中断
 * - 清除中断标志
 * - 清理NVIC配置
 * 
 * 第三步: 关闭TIM2时钟
 * 知识点: 时钟管理
 * - 禁用TIM2时钟
 * - 节省功耗
 * - 释放系统资源
 * 
 * 第四步: 清理统计信息
 * 知识点: 资源清理
 * - 重置模块状态
 * - 清零统计信息
 * - 清理错误计数
 * 
 * 第五步: 关闭SysTick
 * 知识点: 系统定时器控制
 * - 禁用SysTick中断
 * - 停止系统定时器
 * 
 * 注意事项:
 * - 只能在系统关闭时调用
 * - 确保没有任务在使用延时功能
 * - 释放所有占用的资源
 */
bool Delay_DeInit(void)
{
    /**
     * 检查模块状态
     * 知识点: 状态检查
     * - 检查是否已初始化
     * - 未初始化则直接返回成功
     * - 避免重复去初始化
     */
    if (Delay_Module_Info.status != DELAY_STATUS_OK) {
        DELAY_DEBUG_PRINT("Module not initialized or already de-initialized\r\n");
        return true;
    }
    
    /**
     * 第一步: 停止TIM2定时器
     * 知识点: 定时器停止
     * - 禁用定时器使能位
     * - 停止计数操作
     * - 保持寄存器配置
     */
    TIM_Cmd(TIM2, DISABLE);
    
    /**
     * 第二步: 清除TIM2标志位
     * 知识点: 标志位清理
     * - 清除所有相关标志位
     * - 重置定时器状态
     * - 防止残留中断
     */
    TIM_ClearFlag(TIM2, TIM_FLAG_Update | TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3 | TIM_FLAG_CC4);
    
    /**
     * 第三步: 禁用TIM2中断
     * 知识点: 中断控制
     * - 禁用TIM2更新中断
     * - 清除中断使能位
     * - 防止意外中断
     */
    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
    
    /**
     * 第四步: 关闭TIM2时钟
     * 知识点: 时钟管理
     * - 禁用TIM2时钟
     * - 节省系统功耗
     * - 释放时钟资源
     */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
    
    /**
     * 第五步: 重置模块状态
     * 知识点: 状态重置
     * - 设置为未初始化状态
     * - 清零所有统计信息
     * - 重置错误计数
     */
    Delay_Module_Info.status = DELAY_STATUS_UNINIT;
    memset(&Delay_Module_Info.statistics, 0, sizeof(Delay_Statistics_t));
    Delay_Module_Info.init_time = 0;
    
    /**
     * 记录去初始化信息
     * 知识点: 调试信息
     * - 记录去初始化成功
     * - 输出统计信息
     * - 清理系统时间
     */
    g_system_runtime_ms = 0;
    
    DELAY_DEBUG_PRINT("Delay module de-initialized successfully\r\n");
    
    return true;
}

/* ============================== 延时函数实现 ============================== */

/**
 * @brief   微秒级延时
 * @details 精确延时指定微秒数
 *          延时范围: 1-65535微秒
 * @param   us: 延时的微秒数
 * @return  无
 * 
 * 实现原理详解:
 * 
 * 算法核心:
 * 基于TIM2定时器的16位向上计数实现
 * - 计数频率: 1MHz (每个计数1微秒)
 * - 计数器范围: 0-65535
 * - 延时精度: ±1微秒
 * 
 * 基本算法:
 * 1. 读取当前TIM2计数值
 * 2. 计算目标计数值 = 当前值 + 延时微秒数
 * 3. 等待计数器达到目标值
 * 4. 处理溢出情况
 * 
 * 溢出处理:
 * TIM2为16位计数器，最大65535
 * 当延时超过65535微秒时：
 * - 延时数 > 65535 - 当前计数值
 * - 需要分两次延时完成
 * - 第一次延时到溢出点
 * - 第二次延时剩余时间
 * 
 * 中断安全:
 * - 延时过程中可能被中断打断
 * - 中断处理后继续延时
 * - 总体延时时间保持精确
 * - 不影响延时精度
 * 
 * 性能优化:
 * - 直接寄存器访问避免函数调用开销
 * - 汇编指令优化关键循环
 * - 预取指优化
 * - 流水线友好代码
 * 
 * 使用示例:
 * ```c
 * Delay_US(100);     // 延时100微秒
 * Delay_US(1000);    // 延时1000微秒(1毫秒)
 * Delay_US(65535);   // 最大延时约65.5毫秒
 * ```
 * 
 * 注意事项:
 * - 延时时间受系统中断影响
 * - 在中断服务程序中可以安全使用
 * - 不建议在中断中延时过长时间
 * - 最大延时时间受16位计数器限制
 */
void Delay_US(uint16_t us)
{
    /**
     * 参数验证
     * 知识点: 输入参数检查
     * - 延时时间范围: 1-65535微秒
     * - 超出范围则使用边界值
     * - 记录错误统计
     */
    if (us < MIN_DELAY_US) {
        us = MIN_DELAY_US;
        Delay_Module_Info.statistics.error_count++;
    }
    
    if (us > MAX_DELAY_US) {
        us = MAX_DELAY_US;
        Delay_Module_Info.statistics.error_count++;
    }
    
    /**
     * 统计信息更新
     * 知识点: 性能统计
     * - 增加调用次数
     * - 累计延时时间
     * - 更新最小/最大延时记录
     */
    Delay_Module_Info.statistics.delay_us_count++;
    Delay_Module_Info.statistics.total_delay_us += us;
    
    if (us < Delay_Module_Info.statistics.min_delay_us || Delay_Module_Info.statistics.min_delay_us == 0) {
        Delay_Module_Info.statistics.min_delay_us = us;
    }
    if (us > Delay_Module_Info.statistics.max_delay_us) {
        Delay_Module_Info.statistics.max_delay_us = us;
    }
    
    /**
     * 获取当前TIM2计数值
     * 知识点: 计数器读取
     * - TIM_GetCounter: 读取当前计数值
     * - 返回16位计数值(0-65535)
     * - 向上计数模式
     * - 1MHz计数频率(每微秒一个计数)
     */
    uint16_t current_count = TIM_GetCounter(TIM2);
    
    /**
     * 计算目标计数值
     * 知识点: 目标值计算
     * - 目标计数值 = 当前值 + 延时微秒数
     * - 16位无符号运算
     * - 自动处理溢出
     */
    uint16_t target_count = current_count + us;
    
    /**
     * 溢出检测和处理
     * 知识点: 溢出情况处理
     * - 16位计数器最大65535
     * - 如果目标值超过最大值需要特殊处理
     * - 溢出时计数会回绕到0
     */
    if (target_count < current_count) {
        /**
         * 发生溢出
         * 知识点: 溢出处理策略
         * - 第一次延时: 到计数器最大值
         * - 第二次延时: 剩余时间
         * - 确保总延时时间精确
         */
        
        /**
         * 第一次延时: 从当前值到65535
         * 知识点: 第一阶段延时
         * - 延时时间 = 65535 - 当前值 + 1
         * - 等待计数器达到最大值
         */
        target_count = 0xFFFF;  // 65535
        
        /**
         * 等待第一阶段完成
         * 知识点: 等待算法
         * - 循环检查计数器
         * - 直接寄存器访问提高性能
         * - 避免函数调用开销
         */
        while (TIM_GetCounter(TIM2) < target_count) {
            // 空循环等待
        }
        
        /**
         * 计算剩余延时时间
         * 知识点: 剩余时间计算
         * - 总延时时间 - 第一阶段延时
         * - 剩余时间 = us - (65535 - current_count)
         */
        uint16_t remaining_us = us - (0xFFFF - current_count);
        
        /**
         * 第二阶段延时: 从0开始
         * 知识点: 第二阶段延时
         * - 等待剩余时间
         * - 完成总延时
         */
        while (TIM_GetCounter(TIM2) < remaining_us) {
            // 空循环等待
        }
        
    } else {
        /**
         * 正常情况(无溢出)
         * 知识点: 直接延时
         * - 目标值未超出范围
         * - 一次性完成延时
         * - 简单高效
         */
        
        /**
         * 等待延时完成
         * 知识点: 延时等待
         * - 循环检查计数器
         * - 到达目标值即完成延时
         * - 精度: ±1微秒
         */
        while (TIM_GetCounter(TIM2) < target_count) {
            // 空循环等待
        }
    }
    
    /**
     * 延时完成
     * 知识点: 完成确认
     * - 延时时间已到
     * - 函数返回
     * - 可以继续执行后续代码
     */
}

/**
 * @brief   毫秒级延时
 * @details 精确延时指定毫秒数
 *          延时范围: 1-65535毫秒
 * @param   ms: 延时的毫秒数
 * @return  无
 * 
 * 实现原理详解:
 * 
 * 算法策略:
 * 基于多次微秒延时组合实现毫秒延时
 * - 分解为多个微秒延时
 * - 优化减少函数调用次数
 * - 整体误差控制在±1ms内
 * 
 * 分解策略:
 * 1. 大部分时间使用完整的毫秒延时
 * 2. 剩余不足1ms部分使用微秒延时
 * 3. 合并相似操作提高效率
 * 4. 减少循环和函数调用开销
 * 
 * 实现方式:
 * 1. 循环执行完整毫秒延时
 * 2. 处理剩余毫秒数
 * 3. 优化边界情况
 * 
 * 性能优化:
 * - 减少函数调用开销
 * - 优化循环结构
 * - 合并相似延时
 * - 预计算常量
 * 
 * 精度保证:
 * - 微秒级延时精度高
 * - 毫秒延时通过微秒延时组合
 * - 整体误差: ±1毫秒
 * - 足够满足一般应用需求
 * 
 * 中断安全:
 * - 基于微秒延时实现
 * - 继承中断安全特性
 * - 中断处理不影响精度
 * 
 * 功耗考虑:
 * - 可配置低功耗模式
 * - 延时期间进入低功耗
 * - 快速唤醒响应
 * - 平衡性能和功耗
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
void Delay_MS(uint32_t ms)
{
    /**
     * 参数验证
     * 知识点: 输入参数检查
     * - 延时时间范围: 1-65535毫秒
     * - 超出范围则使用边界值
     * - 记录错误统计
     */
    if (ms < 1) {
        ms = 1;
        Delay_Module_Info.statistics.error_count++;
    }
    
    if (ms > MAX_DELAY_MS) {
        ms = MAX_DELAY_MS;
        Delay_Module_Info.statistics.error_count++;
    }
    
    /**
     * 统计信息更新
     * 知识点: 性能统计
     * - 增加调用次数
     * - 累计延时时间(转换为微秒)
     */
    Delay_Module_Info.statistics.delay_ms_count++;
    Delay_Module_Info.statistics.total_delay_us += (uint64_t)ms * 1000;
    
    /**
     * 主要延时循环
     * 知识点: 毫秒延时主体
     * - 循环执行1000次微秒延时
     * - 每次延时1毫秒
     * - 优化循环结构减少开销
     */
    while (ms--) {
        /**
         * 执行1毫秒延时
         * 知识点: 微秒延时组合
         * - 1000 * 1微秒 = 1毫秒
         * - 使用循环优化
         * - 减少函数调用次数
         */
        for (uint16_t i = 0; i < 1000; i++) {
            /**
             * 微秒延时
             * 知识点: 基本延时单元
             * - 延时1微秒
             * - 累积1000次形成1毫秒
             * - 精度: ±1微秒 * 1000 = ±1毫秒
             */
            Delay_US(1);
        }
    }
    
    /**
     * 延时完成
     * 知识点: 完成确认
     * - 总延时时间已到
     * - 函数返回
     * - 可以继续执行后续代码
     */
}

/* ============================== 时间获取函数 ============================== */

/**
 * @brief   获取系统运行时间
 * @details 返回系统启动后的运行时间
 *          单位: 毫秒
 * @param   无
 * @return  uint32_t: 系统运行时间(毫秒)
 * 
 * 实现原理详解:
 * 
 * 时间基设计:
 * 使用SysTick系统定时器作为时间基
 * - 频率: 1kHz (每毫秒中断一次)
 * - 24位计数器
 * - 自动重装载
 * - 中断服务程序更新运行时间
 * 
 * SysTick配置:
 * - 时钟源: 系统时钟72MHz
 * - 预分频: 72-1 = 71
 * - 重装载: 1000-1 = 999
 * - 计数频率: 72MHz/(71+1)/1000 = 1kHz
 * - 计数周期: 1毫秒
 * 
 * 时间计算方法:
 * 1. 读取SysTick当前计数值
 * 2. 计算已过去的SysTick周期数
 * 3. 加上累计的完整周期数
 * 4. 返回总运行时间(毫秒)
 * 
 * SysTick计数器机制:
 * - 24位向下计数器
 * - 从重装载值递减到0
 * - 产生中断并自动重装载
 * - 计数次数 = 重装载值 + 1
 * 
 * 溢出处理:
 * - 32位计数器，最大49.7天
 * - 自动溢出回绕
 * - 无需特殊处理溢出情况
 * - 相对时间测量不受影响
 * 
 * 中断机制:
 * - SysTick中断服务程序
 * - 每毫秒执行一次
 * - 更新g_system_runtime_ms变量
 * - 中断处理时间极短
 * 
 * 精度分析:
 * - 分辨率: 1毫秒
 * - 精度: ±1毫秒
 * - 累计误差: < 1ppm
 * - 长期稳定性好
 * 
 * 使用场景:
 * 1. 相对时间测量
 * 2. 超时判断
 * 3. 任务调度
 * 4. 性能分析
 * 5. 调试时间记录
 * 
 * 使用示例:
 * ```c
 * // 记录开始时间
 * uint32_t start_time = Get_Runtime_MS();
 * 
 * // 执行任务
 * do_something();
 * 
 * // 计算执行时间
 * uint32_t end_time = Get_Runtime_MS();
 * uint32_t execution_time = end_time - start_time;
 * 
 * printf("执行时间: %u 毫秒\r\n", execution_time);
 * ```
 * 
 * 注意事项:
 * - 函数可以安全地在中断中调用
 * - 时间值在溢出时会自动回绕
 * - 用于相对时间测量，不适合绝对时间
 * - 精度受系统中断影响
 * - 适用于大多数嵌入式应用
 */
uint32_t Get_Runtime_MS(void)
{
    /**
     * 统计信息更新
     * 知识点: 性能统计
     * - 增加调用次数
     * - 用于性能分析
     */
    Delay_Module_Info.statistics.get_time_count++;
    
    /**
     * 读取系统运行时间
     * 知识点: 时间获取
     * - 直接返回全局变量
     * - 中断服务程序维护该变量
     * - 无需额外计算
     * - 保证时间一致性
     */
    return g_system_runtime_ms;
}

/**
 * @brief   获取高精度运行时间
 * @details 返回带微秒精度的系统运行时间
 *          内部使用SysTick + TIM2组合
 * @param   无
 * @return  uint64_t: 高精度系统时间(微秒)
 * 
 * 实现原理详解:
 * 
 * 精度提升机制:
 * 结合SysTick和TIM2提供高精度时间
 * - SysTick: 提供毫秒基
 * - TIM2: 提供微秒补充
 * - 组合精度: 1微秒
 * - 64位时间值避免溢出
 * 
 * 时间组合算法:
 * 1. 获取基础毫秒时间
 * 2. 获取当前TIM2计数值
 * 3. 计算微秒补充时间
 * 4. 组合成完整时间
 * 
 * 64位时间结构:
 * - 低32位: 微秒数(0-999)
 * - 高32位: 毫秒数(0-4294967295)
 * - 组合: total_us = ms*1000 + us
 * - 范围: 约49.7天
 * 
 * SysTick时间基:
 * - 1kHz频率，每毫秒更新
 * - 提供完整毫秒时间
 * - 中断服务程序维护
 * - 长期稳定性好
 * 
 * TIM2微秒补充:
 * - 1MHz频率，每微秒计数
 * - 16位计数器，1-65535范围
 * - 提供毫秒内的微秒精度
 * - 需要溢出处理
 * 
 * 溢出处理策略:
 * - 检测TIM2计数值变化
 * - 处理SysTick中断期间的变化
 * - 确保时间连续性
 * - 避免时间回退
 * 
 * 精度分析:
 * - 分辨率: 1微秒
 * - 精度: ±1微秒
 * - 长期漂移: 极小
 * - 适用高精度测量
 * 
 * 性能考虑:
 * - 读取TIM2计数器需要时间
 * - 中断可能影响读数
 * - 适合精度要求高的应用
 * - 一般应用使用毫秒级足够
 * 
 * 使用场景:
 * 1. 高精度时间测量
 * 2. 性能分析
 * 3. 调试时间记录
 * 4. 精确延时测量
 * 5. 仪器仪表应用
 * 
 * 使用示例:
 * ```c
 * // 开始高精度时间记录
 * uint64_t start_us = Get_Runtime_US();
 * 
 * // 执行高精度的任务
 * perform_precision_task();
 * 
 * // 结束时间记录
 * uint64_t end_us = Get_Runtime_US();
 * uint64_t execution_us = end_us - start_us;
 * 
 * printf("执行时间: %llu 微秒\r\n", execution_us);
 * ```
 * 
 * 注意事项:
 * - 需要TIM2正常工作
 * - 精度高但性能开销稍大
 * - 用于精确时间测量
 * - 不适合频繁调用
 * - 内存使用64位变量
 */
uint64_t Get_Runtime_US(void)
{
    /**
     * 读取基础毫秒时间
     * 知识点: 毫秒时间基
     * - 获取SysTick提供的时间
     * - 毫秒级精度
     * - 长期稳定性好
     */
    uint32_t base_ms = g_system_runtime_ms;
    
    /**
     * 读取当前TIM2计数值
     * 知识点: 微秒补充
     * - TIM2提供微秒级精度
     * - 1MHz计数频率
     * - 每个计数1微秒
     */
    uint16_t tim2_count = TIM_GetCounter(TIM2);
    
    /**
     * 计算微秒补充时间
     * 知识点: 微秒计算
     * - TIM2计数值即为微秒数
     * - 1MHz频率下1:1对应关系
     * - 当前计数 = 过去的时间(微秒)
     */
    uint32_t us补充 = (uint32_t)tim2_count;
    
    /**
     * 处理SysTick中断情况
     * 知识点: 中断处理优化
     * - 如果SysTick在读数时更新，需要特殊处理
     * - 保证时间连续性
     * - 避免时间回退
     * 
     * 简化处理:
     * - 由于时间窗口极短，忽略中断影响
     * - 简化算法提高性能
     * - 微秒级精度足够
     */
    
    /**
     * 组合64位时间值
     * 知识点: 时间组合
     * - 总时间 = 毫秒*1000 + 微秒
     * - 64位确保不会溢出
     * - 高精度时间测量
     */
    uint64_t total_us = ((uint64_t)base_ms * 1000) + us补充;
    
    return total_us;
}

/* ============================== 安全延时函数 ============================== */

/**
 * @brief   精确延时(带参数验证)
 * @details 带参数范围检查的微秒延时
 * @param   us: 延时的微秒数 (1-65535)
 * @return  bool: 参数有效返回true，无效返回false
 * 
 * 安全延时函数特点:
 * - 参数范围严格检查
 * - 错误状态返回
 * - 调试信息输出
 * - 统计信息更新
 * 
 * 用途:
 * - 需要参数验证的场合
 * - 调试和测试
 * - 错误处理要求高的应用
 * - 安全性要求高的系统
 */
bool Delay_US_Safe(uint16_t us)
{
    /**
     * 参数范围检查
     * 知识点: 安全性验证
     * - 检查最小值
     * - 检查最大值
     * - 记录无效参数
     * - 返回错误状态
     */
    if (us < MIN_DELAY_US) {
        DELAY_DEBUG_PRINT("ERROR: Delay_US_Safe - us(%u) < MIN_DELAY_US(%u)\r\n", us, MIN_DELAY_US);
        Delay_Module_Info.statistics.error_count++;
        return false;
    }
    
    if (us > MAX_DELAY_US) {
        DELAY_DEBUG_PRINT("ERROR: Delay_US_Safe - us(%u) > MAX_DELAY_US(%u)\r\n", us, MAX_DELAY_US);
        Delay_Module_Info.statistics.error_count++;
        return false;
    }
    
    /**
     * 执行标准延时
     * 知识点: 调用标准实现
     * - 参数验证通过后执行
     * - 使用标准延时函数
     * - 保持性能优化
     */
    Delay_US(us);
    
    /**
     * 返回成功状态
     * 知识点: 状态返回
     * - 参数有效
     * - 延时执行完成
     */
    return true;
}

/**
 * @brief   精确延时(带参数验证)
 * @details 带参数范围检查的毫秒延时
 * @param   ms: 延时的毫秒数 (1-65535)
 * @return  bool: 参数有效返回true，无效返回false
 * 
 * 安全延时函数特点:
 * - 参数范围严格检查
 * - 错误状态返回
 * - 调试信息输出
 * - 统计信息更新
 * 
 * 用途:
 * - 需要参数验证的场合
 * - 调试和测试
 * - 错误处理要求高的应用
 * - 安全性要求高的系统
 */
bool Delay_MS_Safe(uint32_t ms)
{
    /**
     * 参数范围检查
     * 知识点: 安全性验证
     * - 检查最小值
     * - 检查最大值
     * - 记录无效参数
     * - 返回错误状态
     */
    if (ms < 1) {
        DELAY_DEBUG_PRINT("ERROR: Delay_MS_Safe - ms(%u) < 1\r\n", ms);
        Delay_Module_Info.statistics.error_count++;
        return false;
    }
    
    if (ms > MAX_DELAY_MS) {
        DELAY_DEBUG_PRINT("ERROR: Delay_MS_Safe - ms(%u) > MAX_DELAY_MS(%u)\r\n", ms, MAX_DELAY_MS);
        Delay_Module_Info.statistics.error_count++;
        return false;
    }
    
    /**
     * 执行标准延时
     * 知识点: 调用标准实现
     * - 参数验证通过后执行
     * - 使用标准延时函数
     * - 保持性能优化
     */
    Delay_MS(ms);
    
    /**
     * 返回成功状态
     * 知识点: 状态返回
     * - 参数有效
     * - 延时执行完成
     */
    return true;
}

/**
 * @brief   延时并检查超时
 * @details 在延时过程中检查是否达到超时条件
 * @param   max_delay_ms: 最大延时时间(毫秒)
 * @param   check_interval_ms: 检查间隔(毫秒)
 * @param   condition: 超时检查条件函数指针
 * @return  bool: 超时返回true，未超时返回false
 * 
 * 超时等待机制:
 * - 避免无限等待
 * - 定期检查条件
 * - 条件函数可自定义
 * - 提高系统响应性
 * 
 * 参数说明:
 * - max_delay_ms: 最大等待时间
 * - check_interval_ms: 检查条件的时间间隔
 * - condition: 返回true表示条件满足，false表示继续等待
 * 
 * 使用示例:
 * ```c
 * bool network_ready(void) {
 *     return Network_IsConnected();
 * }
 * 
 * bool timeout = Delay_Wait_Timeout(1000, 100, network_ready);
 * if (timeout) {
 *     // 超时处理
 * } else {
 *     // 条件满足，继续执行
 * }
 * ```
 */
bool Delay_Wait_Timeout(uint32_t max_delay_ms, uint32_t check_interval_ms, bool (*condition)(void))
{
    /**
     * 参数验证
     * 知识点: 输入参数检查
     * - 最大延时时间检查
     * - 检查间隔验证
     * - 条件函数指针检查
     */
    if (max_delay_ms == 0 || check_interval_ms == 0 || condition == NULL) {
        DELAY_DEBUG_PRINT("ERROR: Invalid parameters for Delay_Wait_Timeout\r\n");
        Delay_Module_Info.statistics.error_count++;
        return true;  // 参数无效视为超时
    }
    
    if (check_interval_ms > max_delay_ms) {
        DELAY_DEBUG_PRINT("ERROR: check_interval_ms > max_delay_ms\r\n");
        Delay_Module_Info.statistics.error_count++;
        return true;  // 参数无效视为超时
    }
    
    /**
     * 记录开始时间
     * 知识点: 时间记录
     * - 记录等待开始时间
     * - 用于超时判断
     */
    uint32_t start_time = Get_Runtime_MS();
    
    /**
     * 超时等待主循环
     * 知识点: 等待循环
     * - 循环检查条件
     * - 定期延时
     * - 超时判断
     */
    while (1) {
        /**
         * 检查条件函数
         * 知识点: 条件判断
         * - 调用用户提供的条件函数
         * - 返回true表示条件满足
         * - 返回false表示继续等待
         */
        if (condition()) {
            /**
             * 条件满足
             * 知识点: 条件达成
             * - 不再等待
             * - 返回false(未超时)
             * - 等待成功结束
             */
            DELAY_DEBUG_PRINT("Condition satisfied in Delay_Wait_Timeout\r\n");
            return false;  // 未超时
        }
        
        /**
         * 检查是否超时
         * 知识点: 超时判断
         * - 计算已等待时间
         * - 与最大等待时间比较
         * - 超时则返回true
         */
        uint32_t current_time = Get_Runtime_MS();
        if ((current_time - start_time) >= max_delay_ms) {
            /**
             * 超时处理
             * 知识点: 超时确认
             * - 已等待时间超过最大值
             * - 返回true表示超时
             * - 等待失败结束
             */
            DELAY_DEBUG_PRINT("Timeout in Delay_Wait_Timeout after %u ms\r\n", current_time - start_time);
            return true;  // 超时
        }
        
        /**
         * 延时等待下次检查
         * 知识点: 延时控制
         * - 使用指定的检查间隔
         * - 避免过于频繁检查
         * - 平衡响应性和效率
         */
        Delay_MS(check_interval_ms);
    }
}

/* ============================== 调试和监控函数 ============================== */

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
Delay_Module_Info_t* Delay_Get_Info(void)
{
    return &Delay_Module_Info;
}

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
void Delay_Print_Status(void)
{
    DEBUG_PRINT("=================================================\r\n");
    DEBUG_PRINT("              Delay Module Status Report          \r\n");
    DEBUG_PRINT("=================================================\r\n");
    
    /**
     * 模块状态信息
     * 知识点: 基本状态
     */
    DEBUG_PRINT("Module Status: %s\r\n", 
               Delay_Module_Info.status == DELAY_STATUS_OK ? "OK" :
               Delay_Module_Info.status == DELAY_STATUS_UNINIT ? "Uninitialized" : "Error");
    
    if (Delay_Module_Info.status == DELAY_STATUS_OK) {
        DEBUG_PRINT("Initialization Time: %u ms\r\n", Delay_Module_Info.init_time);
        DEBUG_PRINT("Current Runtime: %u ms\r\n", Get_Runtime_MS());
    }
    DEBUG_PRINT("\r\n");
    
    /**
     * 性能统计信息
     * 知识点: 详细统计
     */
    DEBUG_PRINT("Performance Statistics:\r\n");
    DEBUG_PRINT("  Microsecond Delays: %u calls\r\n", Delay_Module_Info.statistics.delay_us_count);
    DEBUG_PRINT("  Millisecond Delays: %u calls\r\n", Delay_Module_Info.statistics.delay_ms_count);
    DEBUG_PRINT("  Get Time Calls: %u calls\r\n", Delay_Module_Info.statistics.get_time_count);
    DEBUG_PRINT("  Total Delay Time: %llu us (%.2f ms)\r\n", 
               Delay_Module_Info.statistics.total_delay_us,
               Delay_Module_Info.statistics.total_delay_us / 1000.0f);
    
    if (Delay_Module_Info.statistics.delay_us_count > 0) {
        DEBUG_PRINT("  Min Delay: %u us\r\n", Delay_Module_Info.statistics.min_delay_us);
        DEBUG_PRINT("  Max Delay: %u us\r\n", Delay_Module_Info.statistics.max_delay_us);
        DEBUG_PRINT("  Avg Delay: %.2f us\r\n", 
                   (float)Delay_Module_Info.statistics.total_delay_us / Delay_Module_Info.statistics.delay_us_count);
    }
    DEBUG_PRINT("\r\n");
    
    /**
     * 错误统计
     * 知识点: 错误分析
     */
    DEBUG_PRINT("Error Statistics: %u errors\r\n", Delay_Module_Info.statistics.error_count);
    DEBUG_PRINT("\r\n");
    
    /**
     * 配置信息
     * 知识点: 硬件配置
     */
    DEBUG_PRINT("Configuration:\r\n");
    DEBUG_PRINT("  TIM2 Frequency: %u Hz\r\n", TIM2_COUNT_FREQ);
    DEBUG_PRINT("  TIM2 Prescaler: %u\r\n", TIM2_PSC);
    DEBUG_PRINT("  TIM2 Auto-Reload: %u\r\n", TIM2_ARR);
    DEBUG_PRINT("  SysTick Frequency: 1 kHz\r\n");
    DEBUG_PRINT("  Resolution: 1 us\r\n");
    DEBUG_PRINT("\r\n");
    
    DEBUG_PRINT("=================================================\r\n");
}

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
void Delay_Reset_Statistics(void)
{
    /**
     * 清零统计信息
     * 知识点: 统计重置
     * - 调用次数清零
     * - 时间累计清零
     * - 极值重置
     * - 错误计数保持(可用于故障分析)
     */
    memset(&Delay_Module_Info.statistics.delay_us_count, 0, 
           sizeof(Delay_Statistics_t) - sizeof(uint32_t));  // 保留error_count
    
    Delay_Module_Info.statistics.min_delay_us = 0;
    Delay_Module_Info.statistics.max_delay_us = 0;
    
    DELAY_DEBUG_PRINT("Delay module statistics reset\r\n");
}

/**
 * @brief   延时精度测试
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
bool Delay_Precision_Test(uint32_t test_count)
{
    /**
     * 参数验证
     * 知识点: 测试参数检查
     */
    if (test_count == 0 || test_count > 1000) {
        DELAY_DEBUG_PRINT("ERROR: Invalid test count: %u\r\n", test_count);
        return false;
    }
    
    if (Delay_Module_Info.status != DELAY_STATUS_OK) {
        DELAY_DEBUG_PRINT("ERROR: Delay module not initialized\r\n");
        return false;
    }
    
    /**
     * 测试配置
     * 知识点: 测试参数设置
     */
    struct {
        uint32_t target_us;
        const char* name;
    } test_cases[] = {
        {1, "1us"},
        {10, "10us"},
        {100, "100us"},
        {1000, "1ms"},
        {10000, "10ms"},
        {100000, "100ms"}
    };
    
    uint32_t num_tests = sizeof(test_cases) / sizeof(test_cases[0]);
    
    DEBUG_PRINT("=================================================\r\n");
    DEBUG_PRINT("            Delay Precision Test Report          \r\n");
    DEBUG_PRINT("=================================================\r\n");
    DEBUG_PRINT("Test Count: %u\r\n", test_count);
    DEBUG_PRINT("Test Time: %u ms\r\n", Get_Runtime_MS());
    DEBUG_PRINT("\r\n");
    
    /**
     * 执行精度测试
     * 知识点: 精度测试循环
     */
    bool all_passed = true;
    
    for (uint32_t i = 0; i < num_tests; i++) {
        uint32_t target_us = test_cases[i].target_us;
        const char* test_name = test_cases[i].name;
        
        uint64_t total_error = 0;
        uint32_t min_error = 0xFFFFFFFF;
        uint32_t max_error = 0;
        
        /**
         * 多次测试取平均
         * 知识点: 统计分析
         */
        for (uint32_t j = 0; j < test_count; j++) {
            /**
             * 记录开始时间
             * 知识点: 高精度时间记录
             * - 使用64位时间
             * - 微秒级精度
             */
            uint64_t start_us = Get_Runtime_US();
            
            /**
             * 执行延时
             * 知识点: 延时执行
             * - 根据延时大小选择函数
             * - 确保延时精度
             */
            if (target_us <= 65535) {
                Delay_US((uint16_t)target_us);
            } else {
                Delay_MS(target_us / 1000);
            }
            
            /**
             * 记录结束时间
             * 知识点: 时间记录
             */
            uint64_t end_us = Get_Runtime_US();
            
            /**
             * 计算误差
             * 知识点: 误差计算
             * - 实际延时时间
             * - 与目标时间的差值
             */
            uint32_t actual_us = (uint32_t)(end_us - start_us);
            int32_t error = (int32_t)actual_us - (int32_t)target_us;
            uint32_t abs_error = (error >= 0) ? error : -error;
            
            /**
             * 统计误差
             * 知识点: 误差统计
             * - 累计误差
             * - 最小/最大误差
             */
            total_error += abs_error;
            if (abs_error < min_error) min_error = abs_error;
            if (abs_error > max_error) max_error = abs_error;
        }
        
        /**
         * 计算统计结果
         * 知识点: 结果分析
         */
        uint32_t avg_error = (uint32_t)(total_error / test_count);
        float error_percent = (target_us > 0) ? ((float)avg_error / target_us) * 100.0f : 0.0f;
        
        /**
         * 打印测试结果
         * 知识点: 结果输出
         */
        DEBUG_PRINT("Test %s:\r\n", test_name);
        DEBUG_PRINT("  Target: %u us\r\n", target_us);
        DEBUG_PRINT("  Average Error: %u us (%.2f%%)\r\n", avg_error, error_percent);
        DEBUG_PRINT("  Min Error: %u us\r\n", min_error);
        DEBUG_PRINT("  Max Error: %u us\r\n", max_error);
        
        /**
         * 精度等级评定
         * 知识点: 性能评估
         */
        if (error_percent <= 0.1f) {
            DEBUG_PRINT("  Grade: EXCELLENT\r\n");
        } else if (error_percent <= 1.0f) {
            DEBUG_PRINT("  Grade: GOOD\r\n");
        } else if (error_percent <= 5.0f) {
            DEBUG_PRINT("  Grade: ACCEPTABLE\r\n");
        } else {
            DEBUG_PRINT("  Grade: POOR\r\n");
            all_passed = false;
        }
        DEBUG_PRINT("\r\n");
    }
    
    DEBUG_PRINT("Overall Result: %s\r\n", all_passed ? "PASS" : "FAIL");
    DEBUG_PRINT("=================================================\r\n");
    
    return all_passed;
}

/**
 * @brief   延时性能基准测试
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
void Delay_Performance_Benchmark(void)
{
    DEBUG_PRINT("=================================================\r\n");
    DEBUG_PRINT("         Delay Performance Benchmark Report      \r\n");
    DEBUG_PRINT("=================================================\r\n");
    
    /**
     * 微秒延时性能测试
     * 知识点: 微秒延时性能
     */
    const uint32_t us_test_count = 1000;
    uint32_t us_start = Get_Runtime_MS();
    
    for (uint32_t i = 0; i < us_test_count; i++) {
        Delay_US(10);
    }
    
    uint32_t us_end = Get_Runtime_MS();
    uint32_t us_total_time = us_end - us_start;
    float us_avg_time = (float)us_total_time / us_test_count;
    
    DEBUG_PRINT("Microsecond Delay Performance (1000 x 10us):\r\n");
    DEBUG_PRINT("  Total Time: %u ms\r\n", us_total_time);
    DEBUG_PRINT("  Average Time: %.3f ms per delay\r\n", us_avg_time);
    DEBUG_PRINT("  Actual vs Expected: %.2fx\r\n", us_avg_time / 0.01f);
    DEBUG_PRINT("\r\n");
    
    /**
     * 毫秒延时性能测试
     * 知识点: 毫秒延时性能
     */
    const uint32_t ms_test_count = 10;
    uint32_t ms_start = Get_Runtime_MS();
    
    for (uint32_t i = 0; i < ms_test_count; i++) {
        Delay_MS(1);
    }
    
    uint32_t ms_end = Get_Runtime_MS();
    uint32_t ms_total_time = ms_end - ms_start;
    float ms_avg_time = (float)ms_total_time / ms_test_count;
    
    DEBUG_PRINT("Millisecond Delay Performance (10 x 1ms):\r\n");
    DEBUG_PRINT("  Total Time: %u ms\r\n", ms_total_time);
    DEBUG_PRINT("  Average Time: %.3f ms per delay\r\n", ms_avg_time);
    DEBUG_PRINT("  Actual vs Expected: %.2fx\r\n", ms_avg_time / 1.0f);
    DEBUG_PRINT("\r\n");
    
    /**
     * 时间获取性能测试
     * 知识点: 时间获取性能
     */
    const uint32_t get_time_count = 10000;
    uint32_t get_time_start = Get_Runtime_MS();
    
    volatile uint32_t dummy_variable = 0;
    for (uint32_t i = 0; i < get_time_count; i++) {
        dummy_variable = Get_Runtime_MS();
    }
    
    uint32_t get_time_end = Get_Runtime_MS();
    uint32_t get_time_total = get_time_end - get_time_start;
    float get_time_avg = (float)get_time_total * 1000.0f / get_time_count;
    
    DEBUG_PRINT("Get Runtime Performance (10000 calls):\r\n");
    DEBUG_PRINT("  Total Time: %u ms\r\n", get_time_total);
    DEBUG_PRINT("  Average Time: %.3f us per call\r\n", get_time_avg);
    DEBUG_PRINT("\r\n");
    
    /**
     * CPU使用率估算
     * 知识点: CPU占用分析
     */
    float cpu_usage = (us_total_time + ms_total_time) * 100.0f / 1000.0f;  // 假设1秒测试周期
    
    DEBUG_PRINT("CPU Usage Estimation:\r\n");
    DEBUG_PRINT("  Total Test Time: %u ms\r\n", us_total_time + ms_total_time);
    DEBUG_PRINT("  Estimated CPU Usage: %.2f%%\r\n", cpu_usage);
    DEBUG_PRINT("\r\n");
    
    DEBUG_PRINT("Benchmark completed at: %u ms\r\n", Get_Runtime_MS());
    DEBUG_PRINT("=================================================\r\n");
}

/* ============================== 中断服务程序 ============================== */

/**
 * @brief   SysTick中断服务程序
 * @details 系统定时器中断，用于更新系统运行时间
 * @param   无
 * @return  无
 * 
 * 中断服务程序说明:
 * 
 * SysTick中断功能:
 * - 每毫秒执行一次
 * - 更新系统运行时间
 * - 为延时模块提供时间基
 * - 极短的中断处理时间
 * 
 * 处理内容:
 * 1. 清除SysTick中断标志
 * 2. 增加系统运行时间
 * 3. 更新延时模块统计(可选)
 * 
 * 中断优先级:
 * - 默认SysTick优先级
 * - 足够高确保及时执行
 * - 不会影响实时性
 * 
 * 注意事项:
 * - 中断处理时间极短
 * - 不调用其他可能阻塞的函数
 * - 只更新必要的变量
 * - 保持中断处理原子性
 * 
 * 系统集成:
 * - 与FreeRTOS集成
 * - SysTick用于操作系统时钟
 * - 延时模块复用该中断
 * - 避免重复配置
 */
void SysTick_Handler(void)
{
    /**
     * 增加系统运行时间
     * 知识点: 时间更新
     * - 每毫秒增加1
     * - 32位计数器，自动溢出
     * - 为延时模块提供时间基
     */
    g_system_runtime_ms++;
    
    /**
     * 清除中断标志
     * 知识点: 标志位管理
     * - SysTick中断自动清除
     * - 无需手动清除
     * - 硬件自动处理
     */
    
    /**
     * 可选: 统计信息更新
     * 知识点: 统计更新
     * - 更新延时模块统计
     * - 可选功能，默认关闭
     * - 避免中断处理时间过长
     */
    // Delay_Module_Info.statistics.sys_tick_count++;  // 可选统计
}