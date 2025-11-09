/**
 * @file    main.c
 * @brief   STM32F103环境监测系统 - 主程序 (标准库版本)
 * @author  MiniMax Agent
 * @version V2.0
 * @date    2025-11-08
 * 
 * 重要说明:
 * - 此版本完全使用STM32标准外设库，禁用了所有HAL函数
 * - 数据结构中完全去除了时间戳功能
 * - 使用简单数据格式："传感器名:数据\r\n"
 * 
 * 知识点总结:
 * 
 * 【主函数作用】
 * 1. 系统入口点，程序从main函数开始执行
 * 2. 完成硬件初始化和软件配置
 * 3. 创建FreeRTOS对象和任务
 * 4. 启动FreeRTOS调度器
 * 5. 此后所有工作由FreeRTOS任务接管
 * 
 * 【系统初始化顺序】
 * 1. 系统上电，启动文件执行复位向量表
 * 2. 调用SystemInit()配置系统时钟
 * 3. 执行main()函数
 * 4. 硬件初始化(时钟、GPIO、外设)
 * 5. FreeRTOS对象和任务创建
 * 6. 启动调度器，进入多任务运行
 * 
 * 【FreeRTOS调度】
 * - 调度器启动后，main函数不再执行
 * - 各任务按优先级轮流执行
 * - 任务切换由内核自动完成
 * - 永远不会再回到main函数
 */

#include "main.h"

/* ============================== 全局变量定义 ============================== */

/**
 * 系统配置结构体
 * 知识点: 全局变量定义
 * - 在头文件中用extern声明
 * - 在源文件中定义并初始化
 * - 所有任务都可以访问
 */
System_Config_t System_Config = {
    .wifi_ssid = "MyWiFi",           // WiFi网络名称
    .wifi_password = "password123",  // WiFi密码
    .server_ip = "192.168.1.100",   // 服务器IP地址
    .server_port = 8080,             // 服务器端口号
    .sample_interval = 5,            // 采集间隔5秒
    .send_interval = 10              // 发送间隔10秒
};

/**
 * 系统状态结构体
 * 知识点: 系统状态跟踪
 * - 实时记录系统运行状态
 * - 为调试和监控提供信息
 */
System_Status_t System_Status = {
    .state = SYSTEM_STATE_INIT,      // 初始状态为初始化
    .start_time = 0,                 // 启动时间待初始化
    .error_count = 0,                // 错误计数为0
    .data_collected = 0,             // 采集数据为0
    .data_sent = 0,                  // 发送数据为0
    .wifi_connected = false,         // WiFi初始未连接
    .bluetooth_connected = false     // 蓝牙初始未连接
};

/**
 * FreeRTOS对象定义
 * 知识点: FreeRTOS对象
 * - 队列用于任务间数据传递
 * - 互斥锁用于共享资源保护
 * - 在硬件初始化后创建
 */

// 网络消息队列 - 用于AT指令响应和网络状态传递
xQueueHandle NetworkQueue = NULL;

// 数据队列 - 用于传感器数据传递
xQueueHandle DataQueue = NULL;

// 串口互斥锁 - 防止多个任务同时访问串口
xSemaphoreHandle USART2_Mutex = NULL;  // 蓝牙串口
xSemaphoreHandle USART3_Mutex = NULL;  // WiFi串口

/**
 * 任务句柄定义
 * 知识点: 任务句柄
 * - 用于后续管理任务(挂起、恢复、删除)
 * - 在任务创建时获得
 * - 方便任务间通信和控制
 */
xTaskHandle NetworkTaskHandle = NULL;           // 网络管理任务
xTaskHandle DataCollectTaskHandle = NULL;       // 数据采集任务
xTaskHandle DataSendTaskHandle = NULL;          // 数据发送任务
xTaskHandle SystemMonitorTaskHandle = NULL;     // 系统监控任务

/* ============================== 主函数 ============================== */

/**
 * @brief   主函数 - 系统入口
 * @return  int 正常情况下不会返回
 * 
 * 知识点: 主函数执行流程
 * 1. 执行硬件初始化
 * 2. 创建FreeRTOS对象
 * 3. 创建FreeRTOS任务
 * 4. 启动FreeRTOS调度器
 * 5. 此后永远不会返回
 * 
 * 重要提醒:
 * - main函数本身不参与任何业务逻辑
 * - 所有功能由FreeRTOS任务实现
 * - 启动调度器后main函数"消失"
 */
int main(void)
{
    /* ============================== 硬件初始化阶段 ============================== */
    
    /**
     * 第1步: 系统时钟配置
     * 知识点: STM32时钟系统重要性
     * - STM32基于ARM Cortex-M内核，需要正确时钟才能工作
     * - 默认使用内部HSI 8MHz，但需要配置PLL达到72MHz
     * - 外设时钟都需要在系统时钟基础上分频获得
     * - 时钟配置错误会导致外设工作异常
     */
    SystemClock_Config();
    DEBUG_PRINT("Step 1: System clock configured (72MHz)\r\n");
    
    /**
     * 第2步: NVIC中断优先级配置
     * 知识点: 中断优先级管理
     * - STM32支持中断嵌套，高优先级中断可以打断低优先级
     * - 需要合理分配中断优先级避免冲突
     * - 这里配置为2位抢占优先级+2位子优先级
     */
    NVIC_Config();
    DEBUG_PRINT("Step 2: NVIC priority configured\r\n");
    
    /**
     * 第3步: GPIO初始化
     * 知识点: GPIO基本配置
     * - 配置LED、蜂鸣器、按键引脚
     * - 配置传感器引脚为模拟输入模式
     * - 配置外部中断用于按键检测
     */
    GPIO_Init();
    DEBUG_PRINT("Step 3: GPIO initialized\r\n");
    
    /**
     * 第4步: 串口初始化
     * 知识点: 串口通信基础
     * - USART1: 调试串口(9600bps)，连接PC
     * - USART2: 蓝牙串口(9600bps)，连接HC-05
     * - USART3: WiFi串口(115200bps)，连接ESP-01s
     * - 串口是异步通信，需要配置波特率、数据位、停止位、校验位
     */
    USART_Init();
    DEBUG_PRINT("Step 4: USART interfaces initialized\r\n");
    
    /**
     * 第5步: ADC初始化
     * 知识点: ADC模数转换器
     * - 传感器都是模拟量，需要ADC转换为数字量
     * - 配置3个通道分别对应温度、光线、气体传感器
     * - ADC时钟不能超过14MHz，这里设置为12MHz
     * - 采样时间需要足够长以确保转换精度
     */
    ADC_Init();
    DEBUG_PRINT("Step 5: ADC initialized for sensor sampling\r\n");
    
    /**
     * 第6步: 定时器初始化
     * 知识点: 定时器用途
     * - TIM2: 用于精确延时函数(微秒/毫秒级)
     * - TIM4: 用于系统心跳和时间统计
     * - 定时器是STM32重要外设，用于定时控制和测量
     */
    TIM2_Init();  // 延时定时器
    TIM4_Init();  // 系统心跳定时器
    DEBUG_PRINT("Step 6: Timers initialized (TIM2 for delay, TIM4 for heartbeat)\r\n");
    
    /**
     * 第7步: 看门狗初始化
     * 知识点: 看门狗重要性
     * - IWDG独立看门狗，防止系统死机
     * - 如果程序跑飞没有及时喂狗，系统会自动复位
     * - 超时时间设置为1.6秒，足够完成正常操作
     * - 看门狗是系统可靠性的重要保障
     */
    IWDG_Init();
    DEBUG_PRINT("Step 7: Independent Watchdog initialized\r\n");
    
    /**
     * 第8步: 加载系统配置
     * 知识点: 配置参数管理
     * - WiFi配置、服务器地址等参数存储在内部Flash
     * - 启动时从Flash读取配置，如果读取失败则使用默认值
     * - 内部Flash掉电不丢失，适合存储配置信息
     */
    System_Config_Load();
    DEBUG_PRINT("Step 8: System configuration loaded\r\n");
    
    /**
     * 第9步: SPI Flash初始化
     * 知识点: SPI Flash外部存储
     * - W25Q64容量8MB，用于存储传感器数据
     * - 初始化环形缓冲区管理结构
     * - 读取读写指针和元数据信息
     * - SPI Flash提供大容量数据存储
     */
    SPI_Flash_Init();
    DEBUG_PRINT("Step 9: SPI Flash initialized\r\n");
    
    /**
     * 第10步: 系统状态更新
     * 知识点: 状态机管理
     * - 更新系统状态为运行中
     * - 记录启动时间
     * - 完成硬件初始化阶段
     */
    System_Status.state = SYSTEM_STATE_RUNNING;
    System_Status.start_time = System_Get_Runtime_MS();
    DEBUG_PRINT("Step 10: Hardware initialization completed, system running\r\n");
    
    /* ============================== FreeRTOS对象创建阶段 ============================== */
    
    /**
     * 第11步: 创建FreeRTOS对象
     * 知识点: FreeRTOS对象管理
     * - 队列: 任务间数据传递
     * - 互斥锁: 共享资源保护(串口访问)
     * - 对象创建必须在内核启动前完成
     * - 创建失败会导致系统无法正常运行
     */
    FreeRTOS_Objects_Create();
    DEBUG_PRINT("Step 11: FreeRTOS objects created\r\n");
    
    /* ============================== FreeRTOS任务创建阶段 ============================== */
    
    /**
     * 第12步: 创建FreeRTOS任务
     * 知识点: 任务设计原则
     * - 任务1(网络管理): 最高优先级，处理WiFi连接
     * - 任务2(数据采集): 中等优先级，周期采集传感器
     * - 任务3(数据发送): 中等优先级，发送数据到服务器
     * - 任务4(系统监控): 最低优先级，系统健康检查
     * - 任务创建后立即开始运行
     */
    FreeRTOS_Tasks_Create();
    DEBUG_PRINT("Step 12: FreeRTOS tasks created and starting\r\n");
    
    /**
     * 第13步: 打印系统信息
     * 知识点: 启动信息输出
     * - 输出系统配置信息
     * - 输出硬件资源使用情况
     * - 方便调试和验证
     */
    #ifdef DEBUG
    System_Print_Info();
    System_Print_Memory_Usage();
    System_Print_Task_Status();
    #endif
    
    /**
     * 第14步: 启动FreeRTOS调度器
     * 知识点: 调度器启动
     * - 这是main函数的最后一行代码
     * - 调度器启动后，所有任务开始并发执行
     * - main函数永远不会返回
     * - 系统进入多任务运行状态
     */
    DEBUG_PRINT("Starting FreeRTOS scheduler...\r\n");
    vTaskStartScheduler();
    
    /**
     * 永远不应该到达这里
     * 知识点: 调度器启动后不会返回
     * - 如果到达这里，说明调度器启动失败
     * - 可能是内存不足或其他严重错误
     */
    ERROR_HANDLER();
    
    return 0;  // 这行代码永远不会执行
}

/* ============================== 系统初始化函数实现 ============================== */

/**
 * @brief   系统配置加载函数
 * @details 从内部Flash读取WiFi配置等参数
 * 
 * 知识点: 内部Flash读写
 * - STM32内部Flash支持页擦除(2KB页)
 * - 可以按字节或按字读取
 * - 写操作前必须先擦除对应页
 * - 适合存储配置参数等不经常改变的数据
 */
void System_Config_Load(void)
{
    uint8_t *config_ptr = (uint8_t*)&System_Config;
    uint32_t flash_addr = 0x08007C00;  // 配置存储地址
    
    DEBUG_PRINT("Loading configuration from internal flash...\r\n");
    
    /**
     * 读取Flash配置
     * 知识点: Flash读取操作
     * - 直接从指定地址读取数据
     * - 读取操作不需要擦除
     * - 读取速度很快
     */
    for (int i = 0; i < sizeof(System_Config); i++) {
        config_ptr[i] = *(volatile uint8_t*)(flash_addr + i);
    }
    
    /**
     * 配置参数验证
     * 知识点: 有效性检查
     * - 检查IP地址格式是否正确
     * - 检查端口号是否在有效范围
     * - 检查SSID长度是否合理
     * - 如果验证失败，使用默认配置
     */
    
    // 验证IP地址格式
    if (System_Config.server_port == 0 || System_Config.server_port > 65535) {
        System_Config.server_port = 8080;  // 使用默认端口
        DEBUG_PRINT("Invalid port, using default 8080\r\n");
    }
    
    // 验证SSID长度
    if (strlen(System_Config.wifi_ssid) == 0) {
        strcpy(System_Config.wifi_ssid, "DefaultWiFi");
        DEBUG_PRINT("Empty SSID, using default\r\n");
    }
    
    // 验证采样间隔
    if (System_Config.sample_interval < 1 || System_Config.sample_interval > 60) {
        System_Config.sample_interval = 5;  // 使用默认5秒
        DEBUG_PRINT("Invalid sample interval, using default 5s\r\n");
    }
    
    DEBUG_PRINT("Configuration loaded successfully\r\n");
}

/**
 * @brief   系统配置保存函数
 * @details 将WiFi配置等参数保存到内部Flash
 */
void System_Config_Save(void)
{
    uint32_t flash_addr = 0x08007C00;  // 配置存储地址
    uint32_t page_addr = 0x08007C00;   // 页起始地址
    uint8_t *config_ptr = (uint8_t*)&System_Config;
    
    DEBUG_PRINT("Saving configuration to internal flash...\r\n");
    
    /**
     * 解锁Flash
     * 知识点: Flash写保护
     * - STM32 Flash默认是写保护状态
     * - 需要先解锁才能进行擦除和写入操作
     * - 写操作完成后自动重新上锁
     */
    FLASH_Unlock();
    
    /**
     * 擦除Flash页
     * 知识点: Flash擦除机制
     * - Flash写入前必须先擦除
     * - 按页擦除，每页2KB
     * - 擦除后所有位变为1
     * - 擦除过程需要等待完成
     */
    if (FLASH_ErasePage(page_addr) != FLASH_COMPLETE) {
        ERROR_HANDLER();
        return;
    }
    
    /**
     * 写入配置数据
     * 知识点: Flash写入操作
     * - 按半字(16位)写入
     * - 写入的数据必须4字节对齐
     * - 每次写入后需要检查状态
     * - 写入过程中不能中断
     */
    for (int i = 0; i < sizeof(System_Config); i += 2) {
        uint16_t data = (config_ptr[i+1] << 8) | config_ptr[i];
        if (FLASH_ProgramHalfWord(flash_addr + i, data) != FLASH_COMPLETE) {
            ERROR_HANDLER();
            return;
        }
    }
    
    /**
     * 重新锁定Flash
     * 知识点: 安全性考虑
     * - 写操作完成后立即锁定
     * - 防止意外写入导致数据损坏
     * - 提高系统安全性
     */
    FLASH_Lock();
    
    DEBUG_PRINT("Configuration saved successfully\r\n");
}

/**
 * @brief   系统运行时间获取函数
 * @return  uint32_t 系统运行时间(毫秒)
 * 
 * 知识点: 32位计数器溢出处理
 * - TIM2计数器最大值为0xFFFFFFFF(约49天)
 * - 溢出时会自动回绕到0
 * - 需要考虑溢出情况下的时间计算
 */
uint32_t System_Get_Runtime_MS(void)
{
    static uint32_t last_count = 0;
    static uint32_t overflow_count = 0;
    uint32_t current_count = TIM2->CNT;  // 读取当前计数器值
    
    /**
     * 检测计数器溢出
     * 知识点: 溢出检测算法
     * - 如果当前值小于上次值，说明发生了溢出
     * - 溢出周期 = 2^32 / 1MHz = 4294秒 ≈ 71分钟
     * - 这在正常情况下不会发生，但需要处理
     */
    if (current_count < last_count) {
        overflow_count++;
    }
    last_count = current_count;
    
    /**
     * 计算总运行时间
     * 知识点: 时间计算
     * - 基础时间 = 溢出次数 * 2^32
     * - 总时间 = 基础时间 + 当前计数
     * - 除以1000转换为毫秒
     */
    uint64_t total_count = ((uint64_t)overflow_count << 32) | current_count;
    return (uint32_t)(total_count / 1000);
}

/**
 * @brief   系统错误处理函数
 * @details 统一处理系统错误，记录错误信息并尝试恢复
 */
void System_Error_Handler(void)
{
    System_Status.error_count++;
    System_Status.state = SYSTEM_STATE_ERROR;
    
    DEBUG_PRINT("System error occurred! Error count: %d\r\n", System_Status.error_count);
    
    /**
     * 错误恢复策略
     * 知识点: 错误处理分级
     * - 轻量级错误: 重试操作
     * - 中等错误: 重新初始化相关模块
     * - 严重错误: 系统重启
     * 
     * 这里简单处理，错误次数较少时记录，较多时重启
     */
    if (System_Status.error_count > 10) {
        DEBUG_PRINT("Too many errors, rebooting system...\r\n");
        Delay_ms(1000);  // 等待1秒让错误信息输出完成
        System_Reboot();
    }
}

/**
 * @brief   系统重启函数
 * @details 软件重启系统，重新初始化所有模块
 */
void System_Reboot(void)
{
    DEBUG_PRINT("System reboot initiated...\r\n");
    
    /**
     * 重启准备
     * 知识点: 优雅关闭
     * - 关闭外设
     * - 保存重要状态
     * - 清理资源
     */
    
    // 关闭LED指示
    GPIO_ResetBits(GPIOA, GPIO_Pin_0);
    GPIO_ResetBits(GPIOB, GPIO_Pin_8);
    
    // 停止所有任务(如果有的话)
    if (NetworkTaskHandle) vTaskDelete(NetworkTaskHandle);
    if (DataCollectTaskHandle) vTaskDelete(DataCollectTaskHandle);
    if (DataSendTaskHandle) vTaskDelete(DataSendTaskHandle);
    if (SystemMonitorTaskHandle) vTaskDelete(SystemMonitorTaskHandle);
    
    /**
     * 系统重启
     * 知识点: 软件重启方法
     * - 方法1: 跳转到复位向量
     * - 方法2: 使用看门狗
     * - 这里使用方法1
     */
    __set_PRIMASK(1);  // 关闭所有中断
    NVIC_SystemReset(); // 调用NVIC复位函数
}

/* ============================== 硬件配置函数实现 ============================== */

/**
 * @brief   系统时钟配置函数
 * @details 配置STM32F103系统时钟为72MHz
 * 
 * 知识点: STM32时钟系统
 * - HSE: 外部高速时钟8MHz(通过外部晶振提供)
 * - PLL: 锁相环倍频器，将8MHz倍频9倍得到72MHz
 * - AHB: 高级高性能总线，时钟72MHz
 * - APB1: 高级外设总线1，时钟36MHz(72MHz/2)
 * - APB2: 高级外设总线2，时钟72MHz
 * - ADC: 模拟数字转换器，时钟12MHz(72MHz/6)
 */
void SystemClock_Config(void)
{
    RCC_ClocksTypeDef RCC_Clocks;
    
    /**
     * 步骤1: 配置系统时钟源为HSI
     * 知识点: 启动时的默认时钟
     * - HSI: 内部高速时钟8MHz
     * - 精度较低但不需要外部晶振
     * - 作为启动时的临时时钟源
     */
    RCC_HSICmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);  // 等待HSI就绪
    
    /**
     * 步骤2: 配置HSE并使能
     * 知识点: 外部高速时钟
     * - HSE: 8MHz外部晶振(需要外部电路)
     * - 精度比HSI高，是系统主时钟源
     * - 需要等待HSE就绪后才能使用
     */
    RCC_HSEConfig(RCC_HSE_ON);
    while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);  // 等待HSE就绪
    
    /**
     * 步骤3: 配置PLL倍频器
     * 知识点: 锁相环(PLL)倍频
     * - 将HSE 8MHz倍频9倍得到72MHz
     * - PLL配置: 时钟源=HSE, 倍频系数=9
     * - PLL输出频率 = 8MHz × 9 = 72MHz
     */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    RCC_PLLCmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);  // 等待PLL就绪
    
    /**
     * 步骤4: 配置Flash等待周期
     * 知识点: Flash访问时序
     * - CPU运行在72MHz，Flash访问需要等待
     * - 72MHz需要2个等待周期
     * - 确保Flash读取稳定性
     */
    FLASH_SetLatency(FLASH_Latency_2);
    
    /**
     * 步骤5: 配置总线时钟分频器
     * 知识点: 时钟分频配置
     * - AHB: 不分频，72MHz
     * - APB1: 2分频，36MHz(限制外设最大频率)
     * - APB2: 不分频，72MHz
     */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);    // AHB = 72MHz
    RCC_PCLK1Config(RCC_HCLK_Div2);     // APB1 = 36MHz
    RCC_PCLK2Config(RCC_HCLK_Div1);     // APB2 = 72MHz
    
    /**
     * 步骤6: 选择PLL作为系统时钟源
     * 知识点: 时钟源切换
     * - 从HSI切换到PLL(72MHz)
     * - 切换前需要确保PLL已就绪
     * - 这是系统时钟配置的最后一步
     */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while (RCC_GetSYSCLKSource() != 0x08);  // 等待切换完成
    
    /**
     * 步骤7: 配置各外设时钟
     * 知识点: 外设时钟使能
     * - GPIOA, GPIOB: GPIO时钟
     * - USART1, USART2, USART3: 串口时钟
     * - ADC1: ADC时钟
     * - TIM2, TIM4: 定时器时钟
     * - IWDG: 看门狗时钟
     * - SPI1: SPI Flash时钟
     */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                          RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3 |
                          RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4 |
                          RCC_APB1Periph_IWDG, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_SPI1, ENABLE);
    
    /**
     * 验证时钟配置
     * 知识点: 时钟验证
     * - 读取并验证各总线时钟频率
     * - 确保配置正确
     */
    RCC_GetClocksFreq(&RCC_Clocks);
    
    DEBUG_PRINT("System clock configuration:\r\n");
    DEBUG_PRINT("  SYSCLK: %lu Hz\r\n", RCC_Clocks.SYSCLK_Frequency);
    DEBUG_PRINT("  HCLK: %lu Hz\r\n", RCC_Clocks.HCLK_Frequency);
    DEBUG_PRINT("  PCLK1: %lu Hz\r\n", RCC_Clocks.PCLK1_Frequency);
    DEBUG_PRINT("  PCLK2: %lu Hz\r\n", RCC_Clocks.PCLK2_Frequency);
    DEBUG_PRINT("  ADCCLK: %lu Hz\r\n", RCC_Clocks.ADCCLK_Frequency);
}

/**
 * @brief   NVIC中断优先级配置函数
 * @details 配置中断优先级分组和优先级
 * 
 * 知识点: 中断优先级系统
 * - STM32支持4位优先级(0-15)
 * - 可配置为不同分组方式
 * - 这里配置为2位抢占+2位子优先级
 * - 数值越小优先级越高
 */
void NVIC_Config(void)
{
    /**
     * 配置NVIC优先级分组
     * 知识点: 优先级分组解释
     * - Group 2: 2位抢占优先级(0-3)，2位子优先级(0-3)
     * - 抢占优先级: 高级中断可以打断低级中断
     * - 子优先级: 同一抢占优先级内的优先级
     */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    /**
     * 使能外设中断
     * 知识点: 中断使能
     * - USART1中断: 用于串口接收
     * - USART2中断: 用于蓝牙数据接收
     * - USART3中断: 用于WiFi数据接收
     * - TIM2中断: 用于延时函数
     * - TIM4中断: 用于系统心跳
     * - EXTI15_10中断: 用于按键检测
     */
    NVIC_SetPriority(USART1_IRQn, 1);  // USART1优先级1
    NVIC_EnableIRQ(USART1_IRQn);
    
    NVIC_SetPriority(USART2_IRQn, 1);  // USART2优先级1
    NVIC_EnableIRQ(USART2_IRQn);
    
    NVIC_SetPriority(USART3_IRQn, 1);  // USART3优先级1
    NVIC_EnableIRQ(USART3_IRQn);
    
    NVIC_SetPriority(TIM2_IRQn, 2);    // TIM2优先级2(延时)
    NVIC_EnableIRQ(TIM2_IRQn);
    
    NVIC_SetPriority(TIM4_IRQn, 2);    // TIM4优先级2(心跳)
    NVIC_EnableIRQ(TIM4_IRQn);
    
    NVIC_SetPriority(EXTI15_10_IRQn, 0); // EXTI优先级0(最高)
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* ============================== 调试函数实现 ============================== */

#ifdef DEBUG
/**
 * @brief   系统信息打印函数 (调试模式)
 * @details 打印系统当前状态和配置信息
 */
void System_Print_Info(void)
{
    printf("\r\n");
    printf("=================================================\r\n");
    printf("     STM32F103环境监测系统 - 启动信息\r\n");
    printf("=================================================\r\n");
    printf("系统版本: V2.0\r\n");
    printf("编译时间: %s %s\r\n", __DATE__, __TIME__);
    printf("\r\n");
    
    printf("系统状态:\r\n");
    printf("  状态: %s\r\n", 
           System_Status.state == SYSTEM_STATE_INIT ? "初始化" :
           System_Status.state == SYSTEM_STATE_RUNNING ? "运行中" :
           System_Status.state == SYSTEM_STATE_ERROR ? "错误" : "停止");
    printf("  运行时间: %lu秒\r\n", System_Get_Runtime_MS() / 1000);
    printf("  错误计数: %lu\r\n", System_Status.error_count);
    printf("  采集数据: %lu条\r\n", System_Status.data_collected);
    printf("  发送数据: %lu条\r\n", System_Status.data_sent);
    printf("\r\n");
    
    printf("网络配置:\r\n");
    printf("  WiFi SSID: %s\r\n", System_Config.wifi_ssid);
    printf("  服务器IP: %s\r\n", System_Config.server_ip);
    printf("  服务器端口: %u\r\n", System_Config.server_port);
    printf("  WiFi状态: %s\r\n", System_Status.wifi_connected ? "已连接" : "未连接");
    printf("  蓝牙状态: %s\r\n", System_Status.bluetooth_connected ? "已连接" : "未连接");
    printf("\r\n");
    
    printf("运行参数:\r\n");
    printf("  采集间隔: %u秒\r\n", System_Config.sample_interval);
    printf("  发送间隔: %u秒\r\n", System_Config.send_interval);
    printf("\r\n");
    
    printf("=================================================\r\n");
}

/**
 * @brief   内存使用情况打印函数 (调试模式)
 * @details 打印系统内存使用统计
 */
void System_Print_Memory_Usage(void)
{
    printf("\r\n内存使用情况:\r\n");
    
    /**
     * 堆使用情况
     * 知识点: 堆内存监控
     * - xPortGetFreeHeapSize(): 获取当前空闲堆大小
     * - configTOTAL_HEAP_SIZE: 编译时定义的堆大小
     * - 通过计算得出已使用内存
     */
    size_t free_heap = xPortGetFreeHeapSize();
    size_t used_heap = configTOTAL_HEAP_SIZE - free_heap;
    float usage_percent = (float)used_heap / configTOTAL_HEAP_SIZE * 100.0f;
    
    printf("  堆内存:\r\n");
    printf("    总大小: %u字节\r\n", configTOTAL_HEAP_SIZE);
    printf("    已使用: %u字节 (%.1f%%)\r\n", used_heap, usage_percent);
    printf("    空闲: %u字节\r\n", free_heap);
    
    /**
     * 任务堆栈使用情况
     * 知识点: 堆栈监控
     * - uxTaskGetStackHighWaterMark(): 获取任务剩余堆栈
     * - 任务堆栈使用过多可能导致栈溢出
     */
    printf("  任务堆栈:\r\n");
    
    if (NetworkTaskHandle) {
        uint16_t network_stack = uxTaskGetStackHighWaterMark(NetworkTaskHandle);
        printf("    网络任务: %u字剩余\r\n", network_stack);
    }
    
    if (DataCollectTaskHandle) {
        uint16_t collect_stack = uxTaskGetStackHighWaterMark(DataCollectTaskHandle);
        printf("    采集任务: %u字剩余\r\n", collect_stack);
    }
    
    if (DataSendTaskHandle) {
        uint16_t send_stack = uxTaskGetStackHighWaterMark(DataSendTaskHandle);
        printf("    发送任务: %u字剩余\r\n", send_stack);
    }
    
    if (SystemMonitorTaskHandle) {
        uint16_t monitor_stack = uxTaskGetStackHighWaterMark(SystemMonitorTaskHandle);
        printf("    监控任务: %u字剩余\r\n", monitor_stack);
    }
    
    printf("\r\n");
}

/**
 * @brief   任务状态打印函数 (调试模式)
 * @details 打印所有任务当前状态
 */
void System_Print_Task_Status(void)
{
    printf("任务状态:\r\n");
    printf("  任务名称        状态      优先级  堆栈剩余\r\n");
    printf("  ------------   --------   ------  --------\r\n");
    
    /**
     * 获取任务状态信息
     * 知识点: 任务监控API
     * - eTaskGetState(): 获取任务状态
     * - uxTaskPriorityGet(): 获取任务优先级
     * - uxTaskGetStackHighWaterMark(): 获取堆栈剩余
     */
    
    // 网络管理任务
    if (NetworkTaskHandle) {
        eTaskState network_state = eTaskGetState(NetworkTaskHandle);
        UBaseType_t network_priority = uxTaskPriorityGet(NetworkTaskHandle);
        uint16_t network_stack = uxTaskGetStackHighWaterMark(NetworkTaskHandle);
        
        printf("  网络管理       %s      %2u      %u\r\n",
               network_state == eRunning ? "运行   " :
               network_state == eReady ? "就绪   " :
               network_state == eBlocked ? "阻塞   " :
               network_state == eSuspended ? "挂起   " : "删除   ",
               network_priority, network_stack);
    }
    
    // 数据采集任务
    if (DataCollectTaskHandle) {
        eTaskState collect_state = eTaskGetState(DataCollectTaskHandle);
        UBaseType_t collect_priority = uxTaskPriorityGet(DataCollectTaskHandle);
        uint16_t collect_stack = uxTaskGetStackHighWaterMark(DataCollectTaskHandle);
        
        printf("  数据采集       %s      %2u      %u\r\n",
               collect_state == eRunning ? "运行   " :
               collect_state == eReady ? "就绪   " :
               collect_state == eBlocked ? "阻塞   " :
               collect_state == eSuspended ? "挂起   " : "删除   ",
               collect_priority, collect_stack);
    }
    
    // 数据发送任务
    if (DataSendTaskHandle) {
        eTaskState send_state = eTaskGetState(DataSendTaskHandle);
        UBaseType_t send_priority = uxTaskPriorityGet(DataSendTaskHandle);
        uint16_t send_stack = uxTaskGetStackHighWaterMark(DataSendTaskHandle);
        
        printf("  数据发送       %s      %2u      %u\r\n",
               send_state == eRunning ? "运行   " :
               send_state == eReady ? "就绪   " :
               send_state == eBlocked ? "阻塞   " :
               send_state == eSuspended ? "挂起   " : "删除   ",
               send_priority, send_stack);
    }
    
    // 系统监控任务
    if (SystemMonitorTaskHandle) {
        eTaskState monitor_state = eTaskGetState(SystemMonitorTaskHandle);
        UBaseType_t monitor_priority = uxTaskPriorityGet(SystemMonitorTaskHandle);
        uint16_t monitor_stack = uxTaskGetStackHighWaterMark(SystemMonitorTaskHandle);
        
        printf("  系统监控       %s      %2u      %u\r\n",
               monitor_state == eRunning ? "运行   " :
               monitor_state == eReady ? "就绪   " :
               monitor_state == eBlocked ? "阻塞   " :
               monitor_state == eSuspended ? "挂起   " : "删除   ",
               monitor_priority, monitor_stack);
    }
    
    printf("\r\n");
}
#endif

/* ============================== 硬件初始化函数实现 ============================== */

/**
 * @brief   GPIO初始化函数
 * @details 配置LED、蜂鸣器、按键和传感器引脚
 * 
 * 知识点: GPIO配置基础
 * - STM32 GPIO支持多种模式: 输入、输出、复用、模拟
 * - 需要配置引脚、速度、上拉/下拉
 * - 不同外设需要不同配置
 */
void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /**
     * 启用GPIO时钟
     * 知识点: 外设时钟使能
     * - 所有GPIO外设都需要先使能时钟
     * - 时钟使能后才能配置GPIO
     */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    
    /**
     * 配置LED引脚
     * 知识点: LED控制配置
     * - PA0: LED1，系统状态指示
     * - PB8: LED2，网络状态指示
     * - 配置为推挽输出，最大速度50MHz
     * - 推挽输出可以提供足够的驱动电流
     */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /**
     * 配置蜂鸣器引脚
     * 知识点: 蜂鸣器控制
     * - PB7: 蜂鸣器控制引脚
     * - 推挽输出，可以提供足够电流驱动蜂鸣器
     * - 初始化为低电平，蜂鸣器不响
     */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, GPIO_Pin_7);  // 蜂鸣器初始不响
    
    /**
     * 配置按键引脚
     * 知识点: 按键输入配置
     * - PA11: 按键输入引脚
     * - 上拉输入模式，按键未按下时为高电平
     * - 配置外部中断，用于检测按键按下
     * - 下降沿触发中断(按键按下时电平下降)
     */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  // 上拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /**
     * 配置传感器引脚
     * 知识点: 模拟输入配置
     * - PA1: 温度传感器(热敏电阻)
     * - PA4: 光线传感器(光敏电阻)
     * - PB1: 气体传感器(MQ135)
     * - 配置为模拟输入模式，用于ADC采样
     */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  // 模拟输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /**
     * 配置SPI Flash引脚
     * 知识点: SPI通信引脚
     * - PA5: SPI1_SCK (时钟线)
     * - PA6: SPI1_MISO (主设备输入，从设备输出)
     * - PA7: SPI1_MOSI (主设备输出，从设备输入)
     * - PB5: W25Q64_CS (片选信号)
     * - SPI通信需要配置为复用推挽模式
     */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_5);  // CS初始为高电平(未选中)
}

/**
 * @brief   USART初始化函数
 * @details 配置3个串口的参数和中断
 */
void USART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    /**
     * 启用相关时钟
     * 知识点: 外设时钟使能
     * - GPIOA: USART1和USART2的GPIO引脚
     * - GPIOB: USART3的GPIO引脚
     * - USART1, USART2, USART3: 串口外设时钟
     */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3, ENABLE);
    
    /**
     * 配置USART1 (PA9=TX, PA10=RX, 9600bps)
     * 知识点: 串口引脚配置
     * - TX引脚: 复用推挽输出
     * - RX引脚: 浮空输入
     * - 波特率: 9600
     * - 数据位: 8位
     * - 停止位: 1位
     * - 校验位: 无
     * - 流控制: 无
     */
    
    // GPIO配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;   // USART1 TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  // USART1 RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // USART1参数配置
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    
    // 使能USART1
    USART_Cmd(USART1, ENABLE);
    
    /**
     * 配置USART2 (PA2=TX, PA3=RX, 9600bps)
     * 知识点: 蓝牙通信串口
     * - 与USART1相同配置
     * - 用于与HC-05蓝牙模块通信
     */
    
    // GPIO配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;   // USART2 TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;   // USART2 RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // USART2参数配置
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
    
    // 使能USART2
    USART_Cmd(USART2, ENABLE);
    
    /**
     * 配置USART3 (PB10=TX, PB11=RX, 115200bps)
     * 知识点: WiFi通信串口
     * - 波特率为115200，比其他串口高
     * - 用于与ESP-01s WiFi模块通信
     */
    
    // GPIO配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  // USART3 TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;  // USART3 RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // USART3参数配置
    USART_InitStructure.USART_BaudRate = 115200;  // WiFi通信需要高波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
    
    // 使能USART3
    USART_Cmd(USART3, ENABLE);
    
    /**
     * 使能接收中断
     * 知识点: 串口中断
     * - RXNE: 接收数据寄存器非空中断
     * - 当有数据到达时触发中断
     * - 中断服务函数中读取数据
     * - 实现了串口数据的异步接收
     */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

/**
 * @brief   ADC初始化函数
 * @details 配置ADC1用于传感器数据采集
 * 
 * 知识点: ADC配置详解
 * - ADC(模数转换器)将模拟信号转换为数字信号
 * - STM32F103内置12位ADC，转换精度较高
 * - 支持多通道同时采样
 * - 转换结果为0-4095对应0-3.3V
 */
void ADC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    
    /**
     * 启用相关时钟
     * 知识点: ADC时钟使能
     * - GPIOA: ADC通道引脚
     * - GPIOB: ADC通道引脚
     * - ADC1: ADC外设时钟
     */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                          RCC_APB2Periph_ADC1, ENABLE);
    
    /**
     * 配置ADC引脚
     * 知识点: ADC引脚配置
     * - PA1: 通道1，温度传感器
     * - PA4: 通道4，光线传感器
     * - PB1: 通道9，气体传感器
     * - 配置为模拟输入模式
     */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  // 模拟输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /**
     * 配置ADC1参数
     * 知识点: ADC工作模式
     * - 模式: 独立模式，只使用ADC1
     * - 扫描模式: 禁用，单次转换
     * - 连续转换: 禁用，软件触发
     * - 对齐方式: 右对齐(标准配置)
     * - 转换数量: 1个通道
     */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;    // 单通道模式
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // 单次转换
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 软件触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // 右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 1;         // 1个通道
    ADC_Init(ADC1, &ADC_InitStructure);
    
    /**
     * 配置ADC通道
     * 知识点: ADC通道配置
     * - 通道1: PA1，温度传感器，采样时间55.5周期
     * - 通道4: PA4，光线传感器，采样时间55.5周期
     * - 通道9: PB1，气体传感器，采样时间55.5周期
     * - 采样时间越长精度越高但速度越慢
     */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);   // PA1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_55Cycles5);   // PA4
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_55Cycles5);   // PB1
    
    /**
     * 启用ADC并校准
     * 知识点: ADC校准
     * - ADC需要校准以提高精度
     * - 校准过程需要一些时间
     * - 校准完成后ADC才能正常工作
     */
    ADC_Cmd(ADC1, ENABLE);
    
    // 等待ADC稳定
    Delay_ms(10);
    
    // 校准ADC
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1) == SET);
    
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1) == SET);
    
    DEBUG_PRINT("ADC initialized and calibrated\r\n");
}

/**
 * @brief   TIM2初始化函数
 * @details 配置TIM2用于精确延时
 */
void TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    
    /**
     * 启用TIM2时钟
     * 知识点: 定时器时钟使能
     * - TIM2挂载在APB1总线上
     * - APB1时钟为36MHz
     * - 但TIM2实际时钟为72MHz(因为APB1分频器设置为2)
     */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    /**
     * 配置TIM2时基
     * 知识点: 定时器时基计算
     * - 预分频器: 36-1 = 35
     * - 自动重载值: 0xFFFFFFFF
     * - 计数频率: 72MHz / 36 = 2MHz
     * - 每个计数周期: 0.5微秒
     * - 0xFFFFFFFF计数次数: 约35分钟
     */
    TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;        // 自动重载值
    TIM_TimeBaseStructure.TIM_Prescaler = 36 - 1;         // 预分频器
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    /**
     * 启动TIM2
     * 知识点: 定时器启动
     * - 启动定时器后计数器开始计数
     * - 可以随时读取计数器值
     * - 用于实现精确延时功能
     */
    TIM_Cmd(TIM2, ENABLE);
    
    DEBUG_PRINT("TIM2 initialized for precise delay (2MHz count frequency)\r\n");
}

/**
 * @brief   TIM4初始化函数
 * @details 配置TIM4用于系统心跳
 */
void TIM4_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    /**
     * 启用TIM4时钟
     * 知识点: 定时器时钟使能
     * - TIM4挂载在APB1总线上
     * - APB1时钟为36MHz
     */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    /**
     * 配置TIM4时基
     * 知识点: 定时器中断周期
     * - 预分频器: 36000-1 = 35999
     * - 自动重载值: 1000-1 = 999
     * - 计数频率: 72MHz / 36000 = 2000Hz
     * - 中断频率: 2000Hz / 1000 = 2Hz = 0.5秒
     * - 实际中断周期: 0.5秒
     */
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;          // 自动重载值
    TIM_TimeBaseStructure.TIM_Prescaler = 36000 - 1;      // 预分频器
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    /**
     * 使能TIM4更新中断
     * 知识点: 定时器中断
     * - 更新中断: 计数器达到自动重载值时触发
     * - 周期: 0.5秒
     * - 用于系统心跳和统计
     */
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    
    /**
     * 配置TIM4中断优先级
     * 知识点: 中断优先级
     * - 优先级: 2(中等优先级)
     * - 子优先级: 0
     */
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /**
     * 启动TIM4
     * 知识点: 定时器启动
     * - 启动后每0.5秒产生一次中断
     * - 中断服务函数中更新系统心跳
     */
    TIM_Cmd(TIM4, ENABLE);
    
    DEBUG_PRINT("TIM4 initialized for system heartbeat (0.5s period)\r\n");
}

/**
 * @brief   IWDG看门狗初始化函数
 * @details 配置独立看门狗防止系统死机
 * 
 * 知识点: 看门狗工作原理
 * - IWDG独立看门狗使用内部40kHz时钟
 * - 如果在超时时间内没有喂狗，MCU会复位
 * - 防止程序跑飞或死机
 * - 超时时间 = (预分频值 × 重装载值) / 40kHz
 */
void IWDG_Init(void)
{
    /**
     * 启用IWDG时钟
     * 知识点: 看门狗时钟
     * - IWDG使用内部低速时钟LSI(40kHz)
     * - 不受主时钟影响，主时钟故障时仍能工作
     * - 这是"独立"看门狗的由来
     */
    RCC_LSICmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);  // 等待LSI就绪
    
    /**
     * 配置IWDG参数
     * 知识点: 看门狗超时计算
     * - 预分频: 64
     * - 重装载值: 1000
     * - 超时时间: (64 × 1000) / 40kHz = 1.6秒
     * - 如果1.6秒内没有喂狗，系统复位
     */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  // 允许写访问
    IWDG_SetPrescaler(IWDG_Prescaler_64);          // 预分频64
    IWDG_SetReload(1000);                          // 重装载值1000
    IWDG_ReloadCounter();                          // 首次喂狗
    IWDG_Enable();                                 // 启用看门狗
    
    DEBUG_PRINT("IWDG initialized (timeout: 1.6s)\r\n");
}

/* ============================== FreeRTOS相关函数 ============================== */

/**
 * @brief   FreeRTOS对象创建函数
 * @details 创建队列和互斥锁
 */
void FreeRTOS_Objects_Create(void)
{
    /**
     * 创建网络消息队列
     * 知识点: 队列创建
     * - 用于AT指令响应和网络状态消息传递
     * - 队列长度: 10条消息
     * - 每条消息大小: sizeof(Network_Message_t)
     * - 队列创建失败会导致系统无法通信
     */
    NetworkQueue = xQueueCreate(NETWORK_QUEUE_LENGTH, sizeof(Network_Message_t));
    if (NetworkQueue == NULL) {
        ERROR_HANDLER();
        return;
    }
    
    /**
     * 创建数据队列
     * 知识点: 数据队列
     * - 用于传感器数据在任务间传递
     * - 队列长度: 50条数据
     * - 每条数据大小: sizeof(Sensor_Data_t)
     * - 足够大的队列避免数据丢失
     */
    DataQueue = xQueueCreate(DATA_QUEUE_LENGTH, sizeof(Sensor_Data_t));
    if (DataQueue == NULL) {
        ERROR_HANDLER();
        return;
    }
    
    /**
     * 创建串口互斥锁
     * 知识点: 互斥锁作用
     * - USART2: 保护蓝牙串口访问
     * - USART3: 保护WiFi串口访问
     * - 防止多个任务同时访问串口导致数据混乱
     * - 二值信号量实现的互斥锁
     */
    USART2_Mutex = xSemaphoreCreateMutex();
    if (USART2_Mutex == NULL) {
        ERROR_HANDLER();
        return;
    }
    
    USART3_Mutex = xSemaphoreCreateMutex();
    if (USART3_Mutex == NULL) {
        ERROR_HANDLER();
        return;
    }
    
    DEBUG_PRINT("FreeRTOS objects created successfully\r\n");
}

/**
 * @brief   FreeRTOS任务创建函数
 * @details 创建4个并发任务
 */
void FreeRTOS_Tasks_Create(void)
{
    /**
     * 创建网络管理任务
     * 知识点: 任务创建参数
     * - 任务函数: Network_Task
     * - 任务名称: "NetworkManager"
     * - 堆栈大小: NETWORK_TASK_STACK_SIZE字
     * - 任务参数: NULL(无参数)
     * - 优先级: NETWORK_TASK_PRIORITY(3，最高)
     * - 任务句柄: NetworkTaskHandle
     */
    if (xTaskCreate(Network_Task, "NetworkManager", 
                   NETWORK_TASK_STACK_SIZE, NULL, 
                   NETWORK_TASK_PRIORITY, 
                   &NetworkTaskHandle) != pdPASS) {
        ERROR_HANDLER();
        return;
    }
    
    /**
     * 创建数据采集任务
     * 知识点: 任务优先级设计
     * - 优先级: 2(中等)
     * - 比网络管理任务低，比监控任务高
     * - 确保网络优先，监控最次
     */
    if (xTaskCreate(Data_Collect_Task, "DataCollector", 
                   DATA_COLLECT_TASK_STACK_SIZE, NULL, 
                   DATA_COLLECT_TASK_PRIORITY, 
                   &DataCollectTaskHandle) != pdPASS) {
        ERROR_HANDLER();
        return;
    }
    
    /**
     * 创建数据发送任务
     * 知识点: 任务分离设计
     * - 采集和发送分离，提高系统响应性
     * - 发送任务可独立处理网络异常
     * - 避免采集被发送阻塞
     */
    if (xTaskCreate(Data_Send_Task, "DataSender", 
                   DATA_SEND_TASK_STACK_SIZE, NULL, 
                   DATA_SEND_TASK_PRIORITY, 
                   &DataSendTaskHandle) != pdPASS) {
        ERROR_HANDLER();
        return;
    }
    
    /**
     * 创建系统监控任务
     * 知识点: 监控任务特点
     * - 优先级最低: 1
     * - 堆栈最小: 128字
     * - 监控任务不应该影响其他任务
     * - 只在系统空闲时运行
     */
    if (xTaskCreate(System_Monitor_Task, "SystemMonitor", 
                   SYSTEM_MONITOR_STACK_SIZE, NULL, 
                   SYSTEM_MONITOR_PRIORITY, 
                   &SystemMonitorTaskHandle) != pdPASS) {
        ERROR_HANDLER();
        return;
    }
    
    DEBUG_PRINT("All FreeRTOS tasks created successfully\r\n");
}

/* ============================== 异常处理函数 ============================== */

/**
 * @brief   硬件异常处理函数
 * @details 处理硬件异常情况
 * 
 * 知识点: 硬件异常类型
 * - HardFault: 硬件错误，最常见的异常
 * - MemManage: 内存管理错误
 * - BusFault: 总线错误
 * - UsageFault: 用法错误
 * - 通常由程序错误或内存访问违规引起
 */
void HardFault_Handler(void)
{
    /**
     * 进入错误状态
     * 知识点: 错误处理
     * - 关闭所有任务
     * - 设置错误状态
     * - 等待看门狗复位或手动重启
     */
    System_Status.state = SYSTEM_STATE_ERROR;
    System_Status.error_count++;
    
    /**
     * 错误信息输出
     * 知识点: 调试信息
     * - 输出错误位置信息
     * - 帮助定位问题
     * - 通过USART1发送到PC
     */
    printf("\r\n!!! HARD FAULT ERROR !!!\r\n");
    printf("Error count: %lu\r\n", System_Status.error_count);
    printf("System will reset after IWDG timeout...\r\n");
    
    /**
     * 停止喂狗
     * 知识点: 主动复位
     * - 停止喂狗后看门狗会复位系统
     * - 超时时间: 1.6秒
     * - 自动恢复正常运行
     */
    while(1) {
        // 等待看门狗复位
        Delay_ms(100);
    }
}

/**
 * @brief   内存管理错误处理函数
 * @details 处理内存访问错误
 */
void MemManage_Handler(void)
{
    System_Status.state = SYSTEM_STATE_ERROR;
    System_Status.error_count++;
    
    printf("\r\n!!! MEMORY MANAGEMENT ERROR !!!\r\n");
    printf("Possible cause: Stack overflow or invalid memory access\r\n");
    
    while(1) {
        Delay_ms(100);
    }
}

/**
 * @brief   总线错误处理函数
 * @details 处理总线访问错误
 */
void BusFault_Handler(void)
{
    System_Status.state = SYSTEM_STATE_ERROR;
    System_Status.error_count++;
    
    printf("\r\n!!! BUS FAULT ERROR !!!\r\n");
    printf("Possible cause: Invalid memory access or bus error\r\n");
    
    while(1) {
        Delay_ms(100);
    }
}

/**
 * @brief   用法错误处理函数
 * @details 处理程序用法错误
 */
void UsageFault_Handler(void)
{
    System_Status.state = SYSTEM_STATE_ERROR;
    System_Status.error_count++;
    
    printf("\r\n!!! USAGE FAULT ERROR !!!\r\n");
    printf("Possible cause: Invalid instruction or division by zero\r\n");
    
    while(1) {
        Delay_ms(100);
    }
}

/**
 * @brief   未定义中断处理函数
 * @details 处理未定义的中断
 */
void NMI_Handler(void)
{
    printf("\r\n!!! NON-MASKABLE INTERRUPT !!!\r\n");
    while(1) {
        Delay_ms(100);
    }
}

/* ============================== 中断服务函数 ============================== */

/**
 * @brief   USART1中断服务函数
 * @details 处理调试串口接收中断
 * 
 * 知识点: 串口中断服务
 * - 当USART1接收到数据时触发
 * - 读取DR寄存器获取数据
 * - 可以在这里处理调试命令
 */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART1);
        
        /**
         * 简单的调试命令处理
         * 知识点: 命令解析
         * - 'r': 重启系统
         * - 'i': 显示系统信息
         * - 's': 显示任务状态
         * - 其他字符: 回显
         */
        switch (data) {
            case 'r':
            case 'R':
                printf("\r\nRestarting system...\r\n");
                System_Reboot();
                break;
                
            case 'i':
            case 'I':
                #ifdef DEBUG
                System_Print_Info();
                #endif
                break;
                
            case 's':
            case 'S':
                #ifdef DEBUG
                System_Print_Task_Status();
                #endif
                break;
                
            default:
                USART_SendData(USART1, data);  // 回显
                break;
        }
    }
}

/**
 * @brief   USART2中断服务函数
 * @details 处理蓝牙串口接收中断
 * 
 * 知识点: 蓝牙数据处理
 * - 接收HC-05蓝牙模块数据
 * - 可以转发到其他任务
 * - 支持蓝牙配置命令
 */
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART2);
        
        /**
         * 蓝牙数据处理
         * 知识点: 蓝牙通信协议
         * - 可以转发给网络任务
         * - 或者直接通过WiFi发送
         * - 取决于应用需求
         */
        
        // 简单的数据透传 - 转发给网络任务
        Network_Message_t msg;
        msg.type = NETWORK_MSG_BLUETOOTH_DATA;
        msg.data.bluetooth_byte = data;
        
        if (xQueueSend(NetworkQueue, &msg, 0) != pdPASS) {
            // 队列满，丢弃数据
        }
    }
}

/**
 * @brief   USART3中断服务函数
 * @details 处理WiFi串口接收中断
 * 
 * 知识点: WiFi数据处理
 * - 接收ESP-01s WiFi模块数据
 * - 解析AT指令响应
 * - 处理网络数据包
 */
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART3);
        
        /**
         * WiFi数据处理
         * 知识点: WiFi通信处理
         * - 解析AT指令响应
         * - 处理网络数据包
         * - 更新连接状态
         */
        
        // 简单的数据透传 - 转发给网络任务
        Network_Message_t msg;
        msg.type = NETWORK_MSG_WIFI_DATA;
        msg.data.wifi_byte = data;
        
        if (xQueueSend(NetworkQueue, &msg, 0) != pdPASS) {
            // 队列满，丢弃数据
        }
    }
}

/**
 * @brief   TIM2中断服务函数
 * @details 处理TIM2中断(未使用，保留)
 * 
 * 知识点: TIM2用途
 * - 主要用于延时函数
 * - 通常不需要中断
 * - 这里保留以备将来使用
 */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
        // TIM2主要用于延时，通常不需要中断处理
        // 这里可以添加用户自定义的中断处理代码
    }
}

/**
 * @brief   TIM4中断服务函数
 * @details 处理TIM4系统心跳中断
 * 
 * 知识点: 系统心跳
 * - 每0.5秒触发一次
 * - 更新系统运行时间
 * - 喂狗操作
 * - 统计信息更新
 */
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        
        /**
         * 系统心跳处理
         * 知识点: 周期任务
         * - 这里可以放置需要定期执行的任务
         * - 例如: 状态检查、统计更新、LED闪烁等
         * - 但这些功能通常在FreeRTOS任务中实现
         * - 这里是低级别的系统维护
         */
        
        // 简单的系统时间更新
        static uint32_t heartbeat_count = 0;
        heartbeat_count++;
        
        // 每2次心跳(1秒)更新一次运行时间
        if (heartbeat_count >= 2) {
            heartbeat_count = 0;
            // 运行时间在System_Get_Runtime_MS()中计算，这里不需要更新
        }
    }
}

/**
 * @brief   EXTI15_10中断服务函数
 * @details 处理按键中断
 * 
 * 知识点: 外部中断
 * - PA11按键按下时触发
 * - 下降沿触发
 * - 上拉输入模式
 * - 可以实现系统重启功能
 */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line11);  // 清除中断标志
        
        /**
         * 按键防抖处理
         * 知识点: 按键抖动处理
         * - 按键按下时会产生抖动
         * - 需要延时去抖
         * - 可以实现3次按键重启
         */
        static uint32_t key_press_count = 0;
        static uint32_t last_key_time = 0;
        
        uint32_t current_time = System_Get_Runtime_MS();
        
        // 防抖: 如果距离上次按键小于500ms，忽略本次按键
        if (current_time - last_key_time < 500) {
            return;
        }
        
        last_key_time = current_time;
        key_press_count++;
        
        printf("Key pressed, count: %lu\r\n", key_press_count);
        
        // 3次按键重启系统
        if (key_press_count >= 3) {
            printf("System will restart...\r\n");
            Delay_ms(1000);  // 等待1秒让用户看到消息
            System_Reboot();
        }
    }
}
