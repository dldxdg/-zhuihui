/**
 * @file    gpio.c
 * @brief   GPIO控制和外部中断实现 (标准库版本)
 * @author  MiniMax Agent
 * @version V2.0
 * @date    2025-11-08
 * 
 * 重要说明:
 * - 此版本完全使用STM32标准库，无HAL函数
 * - 实现LED、蜂鸣器、按键的完整控制
 * - 外部中断处理系统重启功能
 * - 完善的状态机管理
 * 
 * 知识点总结:
 * 
 * 【GPIO配置详解】
 * 1. 推挽输出: 可以提供足够电流驱动LED
 * 2. 上拉输入: 按键未按下时保持高电平
 * 3. 模拟输入: 传感器信号输入
 * 4. 速度配置: 50MHz满足一般应用需求
 * 
 * 【外部中断配置】
 * 1. EXTI线映射: PA11映射到EXTI_Line11
 * 2. 触发方式: 下降沿触发(按键按下)
 * 3. NVIC优先级: 较高优先级确保及时响应
 * 4. 防抖处理: 中断中进行初步防抖
 * 
 * 【状态机设计】
 * 1. LED状态机: 开关、闪烁、呼吸效果
 * 2. 按键状态机: 防抖、长按、多击检测
 * 3. 蜂鸣器状态机: 报警、时序控制
 */

#include "gpio.h"
#include "main.h"
#include "delay.h"

/* ============================== 全局变量定义 ============================== */

/**
 * GPIO模块状态
 * 知识点: GPIO模块整体状态
 * - 记录初始化状态
 * - 统计操作信息
 * - 错误计数管理
 */
GPIO_Module_Status_t GPIO_Module_Status = {
    .is_initialized = false,
    .error_count = 0,
    .led_toggle_count = 0,
    .key_press_count = 0,
    .buzzer_alarm_count = 0
};

/**
 * LED1控制结构 (系统状态LED)
 * 知识点: 系统状态LED
 * - PA0引脚控制
 * - 用于系统运行状态指示
 * - 闪烁频率表示不同状态
 */
LED_Control_t LED1_Control = {
    .state = LED_STATE_OFF,
    .is_on = false,
    .last_toggle_time = 0,
    .blink_interval = LED_BLINK_SLOW_MS,
    .on_duration = LED_ON_MS,
    .off_duration = LED_OFF_MS
};

/**
 * LED2控制结构 (网络状态LED)
 * 知识点: 网络状态LED
 * - PB8引脚控制
 * - 用于WiFi连接状态指示
 * - 网络状态直接反映LED状态
 */
LED_Control_t LED2_Control = {
    .state = LED_STATE_OFF,
    .is_on = false,
    .last_toggle_time = 0,
    .blink_interval = LED_BLINK_FAST_MS,
    .on_duration = LED_ON_MS,
    .off_duration = LED_OFF_MS
};

/**
 * 按键信息结构
 * 知识点: 按键状态管理
 * - 记录按键当前状态
 * - 管理防抖和长按检测
 * - 多击检测支持
 */
Key_Info_t Key_Info = {
    .state = KEY_STATE_IDLE,
    .is_pressed = false,
    .press_start_time = 0,
    .release_time = 0,
    .press_count = 0,
    .last_press_time = 0,
    .long_press_detected = false,
    .reset_requested = false
};

/**
 * 系统状态指示
 * 知识点: 系统状态统一管理
 * - 集中管理所有状态指示
 * - LED和蜂鸣器统一控制
 * - 错误状态集中处理
 */
System_Status_Indicator_t System_Status_Indicator = {
    .system_led_state = LED_STATE_OFF,
    .network_led_state = LED_STATE_OFF,
    .buzzer_control = {0},
    .system_error = false,
    .network_error = false,
    .data_error = false
};

/* ============================== GPIO模块初始化 ============================== */

/**
 * @brief   GPIO模块初始化函数
 * @details 初始化所有GPIO相关功能
 * 
 * 知识点: GPIO模块初始化顺序
 * 1. 初始化LED控制
 * 2. 初始化按键控制
 * 3. 初始化蜂鸣器
 * 4. 初始化外部中断
 * 5. 更新系统状态指示
 */
void GPIO_Module_Init(void)
{
    DEBUG_PRINT("Initializing GPIO module...\r\n");
    
    /**
     * 步骤1: 初始化LED控制
     * 知识点: LED初始化
     * - 配置LED引脚为推挽输出
     * - 初始化LED控制结构
     * - 设置默认状态为关闭
     */
    LED_Init();
    
    /**
     * 步骤2: 初始化按键控制
     * 知识点: 按键初始化
     * - 配置按键引脚为上拉输入
     * - 配置外部中断
     * - 初始化按键状态
     */
    Key_Init();
    
    /**
     * 步骤3: 初始化蜂鸣器
     * 知识点: 蜂鸣器初始化
     * - 配置蜂鸣器引脚
     * - 初始化控制结构
     * - 设置默认状态为关闭
     */
    Buzzer_Init();
    
    /**
     * 步骤4: 更新系统状态
     * 知识点: 状态更新
     * - 根据当前系统状态更新指示
     * - 初始状态LED为慢闪烁
     */
    System_Status_Update();
    
    /**
     * 步骤5: 更新模块状态
     * 知识点: 状态记录
     * - 标记为已初始化
     */
    GPIO_Module_Status.is_initialized = true;
    
    DEBUG_PRINT("GPIO module initialized successfully\r\n");
}

/**
 * @brief   LED初始化函数
 * @details 初始化LED控制引脚和状态
 * 
 * 知识点: LED初始化过程
 * 1. 启用GPIO时钟
 * 2. 配置GPIO引脚
 * 3. 初始化控制结构
 * 4. 设置默认状态
 */
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /**
     * 启用GPIO时钟
     * 知识点: 时钟使能
     * - LED1在GPIOA
     * - LED2在GPIOB
     */
    RCC_APB2PeriphClockCmd(LED1_CLOCK | LED2_CLOCK, ENABLE);
    
    /**
     * 配置LED1引脚 (PA0)
     * 知识点: LED1引脚配置
     * - 模式: 推挽输出
     * - 速度: 50MHz
     * - 初始状态: 关闭
     */
    GPIO_InitStructure.GPIO_Pin = LED1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  // 50MHz
    GPIO_Init(LED1_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(LED1_PORT, LED1_PIN);  // 初始关闭
    
    /**
     * 配置LED2引脚 (PB8)
     * 知识点: LED2引脚配置
     * - 模式: 推挽输出
     * - 速度: 50MHz
     * - 初始状态: 关闭
     */
    GPIO_InitStructure.GPIO_Pin = LED2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  // 50MHz
    GPIO_Init(LED2_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(LED2_PORT, LED2_PIN);  // 初始关闭
    
    /**
     * 初始化LED控制结构
     * 知识点: 控制结构初始化
     * - 设置为关闭状态
     * - 重置定时器
     */
    LED1_Control.state = LED_STATE_OFF;
    LED1_Control.is_on = false;
    LED1_Control.last_toggle_time = 0;
    
    LED2_Control.state = LED_STATE_OFF;
    LED2_Control.is_on = false;
    LED2_Control.last_toggle_time = 0;
    
    DEBUG_PRINT("LEDs initialized (PA0: System, PB8: Network)\r\n");
}

/**
 * @brief   按键初始化函数
 * @details 初始化按键控制引脚和外部中断
 * 
 * 知识点: 按键初始化过程
 * 1. 启用GPIO和AFIO时钟
 * 2. 配置按键引脚
 * 3. 配置外部中断
 * 4. 配置NVIC
 * 5. 初始化按键状态
 */
void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    /**
     * 启用GPIO和AFIO时钟
     * 知识点: 时钟使能
     * - GPIOA: 按键引脚
     * - AFIO: 外部中断复用功能
     */
    RCC_APB2PeriphClockCmd(KEY_CLOCK | RCC_APB2Periph_AFIO, ENABLE);
    
    /**
     * 配置按键引脚 (PA11)
     * 知识点: 按键引脚配置
     * - 模式: 上拉输入
     * - 按键未按下时为高电平
     * - 按键按下时为低电平
     */
    GPIO_InitStructure.GPIO_Pin = KEY_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  // 上拉输入
    GPIO_Init(KEY_PORT, &GPIO_InitStructure);
    
    /**
     * 配置外部中断映射
     * 知识点: 中断映射配置
     * - 将PA11映射到EXTI_Line11
     * - AFIO_EXTICR3寄存器配置
     */
    GPIO_EXTILineConfig(KEY_EXTI_PORT, KEY_EXTI_PIN);
    
    /**
     * 配置EXTI中断
     * 知识点: 外部中断配置
     * - 中断线: EXTI_Line11
     * - 模式: 中断模式
     * - 触发: 下降沿触发(按键按下)
     * - 使能: 开启中断
     */
    EXTI_InitStructure.EXTI_Line = KEY_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  // 下降沿
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    /**
     * 配置NVIC中断优先级
     * 知识点: 中断优先级配置
     * - 优先级: 0(最高)
     * - 子优先级: 0
     * - 使能: 开启
     */
    NVIC_InitStructure.NVIC_IRQChannel = KEY_EXTI_IRQN;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  // 最高优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /**
     * 初始化按键状态
     * 知识点: 状态初始化
     * - 初始状态为空闲
     * - 清除所有标志
     */
    Key_Info.state = KEY_STATE_IDLE;
    Key_Info.is_pressed = false;
    Key_Info.press_start_time = 0;
    Key_Info.release_time = 0;
    Key_Info.press_count = 0;
    Key_Info.last_press_time = 0;
    Key_Info.long_press_detected = false;
    Key_Info.reset_requested = false;
    
    DEBUG_PRINT("Key initialized (PA11: System Reset with EXTI)\r\n");
}

/**
 * @brief   蜂鸣器初始化函数
 * @details 初始化蜂鸣器控制引脚
 * 
 * 知识点: 蜂鸣器初始化过程
 * 1. 启用GPIO时钟
 * 2. 配置蜂鸣器引脚
 * 3. 初始化控制结构
 * 4. 设置默认状态
 */
void Buzzer_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /**
     * 启用GPIO时钟
     * 知识点: 时钟使能
     * - GPIOB: 蜂鸣器引脚
     */
    RCC_APB2PeriphClockCmd(BUZZER_CLOCK, ENABLE);
    
    /**
     * 配置蜂鸣器引脚 (PB7)
     * 知识点: 蜂鸣器引脚配置
     * - 模式: 推挽输出
     * - 速度: 50MHz
     * - 初始状态: 关闭
     */
    GPIO_InitStructure.GPIO_Pin = BUZZER_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 50MHz
    GPIO_Init(BUZZER_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);  // 初始关闭
    
    /**
     * 初始化蜂鸣器控制结构
     * 知识点: 控制结构初始化
     * - 初始状态为不活跃
     * - 清除所有标志
     */
    System_Status_Indicator.buzzer_control.is_active = false;
    System_Status_Indicator.buzzer_control.is_beeping = false;
    System_Status_Indicator.buzzer_control.beep_start_time = 0;
    System_Status_Indicator.buzzer_control.beep_duration = 0;
    System_Status_Indicator.buzzer_control.beep_interval = 0;
    System_Status_Indicator.buzzer_control.beep_count = 0;
    System_Status_Indicator.buzzer_control.current_beep = 0;
    System_Status_Indicator.buzzer_control.last_beep_time = 0;
    
    DEBUG_PRINT("Buzzer initialized (PB7: Alarm)\r\n");
}

/* ============================== LED控制函数 ============================== */

/**
 * @brief   LED控制函数
 * @details 控制LED的状态
 * @param led_id LED标识(1=LED1, 2=LED2)
 * @param state LED状态
 * 
 * 知识点: LED状态控制
 * 1. 开关控制: 直接控制引脚电平
 * 2. 闪烁控制: 定时切换状态
 * 3. 状态更新: 周期性更新控制
 */
void LED_Control(uint8_t led_id, LED_State_t state)
{
    LED_Control_t *led_control;
    GPIO_TypeDef *port;
    uint16_t pin;
    
    /**
     * 选择LED
     * 知识点: LED选择
     * - 根据ID选择对应的LED控制结构
     * - 设置对应的GPIO端口和引脚
     */
    switch (led_id) {
        case 1:  // LED1 - 系统状态
            led_control = &LED1_Control;
            port = LED1_PORT;
            pin = LED1_PIN;
            System_Status_Indicator.system_led_state = state;
            break;
            
        case 2:  // LED2 - 网络状态
            led_control = &LED2_Control;
            port = LED2_PORT;
            pin = LED2_PIN;
            System_Status_Indicator.network_led_state = state;
            break;
            
        default:
            DEBUG_PRINT("Invalid LED ID: %u\r\n", led_id);
            return;
    }
    
    /**
     * 设置LED状态
     * 知识点: 状态设置
     * - 更新控制结构状态
     * - 设置引脚输出
     * - 重置定时器
     */
    led_control->state = state;
    led_control->last_toggle_time = System_Get_Runtime_MS();
    
    switch (state) {
        case LED_STATE_OFF:
            led_control->is_on = false;
            GPIO_ResetBits(port, pin);
            break;
            
        case LED_STATE_ON:
            led_control->is_on = true;
            GPIO_SetBits(port, pin);
            break;
            
        case LED_STATE_BLINK_FAST:
            led_control->blink_interval = LED_BLINK_FAST_MS;
            led_control->on_duration = LED_ON_MS;
            led_control->off_duration = LED_OFF_MS;
            // 开始闪烁
            led_control->is_on = true;
            GPIO_SetBits(port, pin);
            break;
            
        case LED_STATE_BLINK_SLOW:
            led_control->blink_interval = LED_BLINK_SLOW_MS;
            led_control->on_duration = LED_ON_MS;
            led_control->off_duration = LED_OFF_MS;
            // 开始闪烁
            led_control->is_on = true;
            GPIO_SetBits(port, pin);
            break;
            
        case LED_STATE_BREATH:
            // 呼吸灯效果(可选实现)
            led_control->is_on = true;
            GPIO_SetBits(port, pin);
            break;
    }
    
    /**
     * 统计信息更新
     * 知识点: 统计更新
     * - LED切换计数增加
     */
    GPIO_Module_Status.led_toggle_count++;
    
    DEBUG_PRINT("LED%u set to state: %d\r\n", led_id, state);
}

/**
 * @brief   LED开关函数
 * @details 直接控制LED的开关
 * @param led_id LED标识
 * @param on 是否开启
 * 
 * 知识点: LED开关控制
 * - 简单的开关控制
 * - 不影响闪烁状态
 */
void LED_Set(uint8_t led_id, bool on)
{
    GPIO_TypeDef *port;
    uint16_t pin;
    
    /**
     * 选择LED
     * 知识点: LED选择
     */
    switch (led_id) {
        case 1:  // LED1
            port = LED1_PORT;
            pin = LED1_PIN;
            break;
        case 2:  // LED2
            port = LED2_PORT;
            pin = LED2_PIN;
            break;
        default:
            return;
    }
    
    /**
     * 设置引脚状态
     * 知识点: 引脚控制
     */
    if (on) {
        GPIO_SetBits(port, pin);
    } else {
        GPIO_ResetBits(port, pin);
    }
    
    /**
     * 更新控制结构
     * 知识点: 状态更新
     */
    LED_Control_t *led_control = (led_id == 1) ? &LED1_Control : &LED2_Control;
    led_control->is_on = on;
    
    if (on) {
        led_control->state = LED_STATE_ON;
    } else {
        led_control->state = LED_STATE_OFF;
    }
}

/**
 * @brief   LED切换函数
 * @details 切换LED的开关状态
 * @param led_id LED标识
 * 
 * 知识点: LED状态切换
 * - 当前开启则关闭
 * - 当前关闭则开启
 */
void LED_Toggle(uint8_t led_id)
{
    GPIO_TypeDef *port;
    uint16_t pin;
    
    /**
     * 选择LED
     * 知识点: LED选择
     */
    switch (led_id) {
        case 1:  // LED1
            port = LED1_PORT;
            pin = LED1_PIN;
            break;
        case 2:  // LED2
            port = LED2_PORT;
            pin = LED2_PIN;
            break;
        default:
            return;
    }
    
    /**
     * 切换引脚状态
     * 知识点: 引脚切换
     */
    GPIO_WriteBit(port, pin, (BitAction)(1 - GPIO_ReadOutputDataBit(port, pin)));
    
    /**
     * 更新控制结构
     * 知识点: 状态更新
     */
    LED_Control_t *led_control = (led_id == 1) ? &LED1_Control : &LED2_Control;
    led_control->is_on = !led_control->is_on;
    led_control->last_toggle_time = System_Get_Runtime_MS();
    
    /**
     * 统计信息更新
     * 知识点: 统计更新
     */
    GPIO_Module_Status.led_toggle_count++;
}

/**
 * @brief   LED闪烁控制函数
 * @details 控制LED的闪烁参数
 * @param led_id LED标识
 * @param interval_ms 闪烁间隔
 * @param on_duration 开启时间
 * @param off_duration 关闭时间
 * 
 * 知识点: LED闪烁控制
 * - 自定义闪烁参数
 * - 占空比控制
 */
void LED_Blink(uint8_t led_id, uint32_t interval_ms, uint32_t on_duration, uint32_t off_duration)
{
    LED_Control_t *led_control;
    
    /**
     * 选择LED控制结构
     * 知识点: LED选择
     */
    switch (led_id) {
        case 1:  // LED1
            led_control = &LED1_Control;
            break;
        case 2:  // LED2
            led_control = &LED2_Control;
            break;
        default:
            return;
    }
    
    /**
     * 设置闪烁参数
     * 知识点: 参数设置
     * - 设置闪烁间隔
     * - 设置开启持续时间
     * - 设置关闭持续时间
     * - 设置为闪烁状态
     */
    led_control->blink_interval = interval_ms;
    led_control->on_duration = on_duration;
    led_control->off_duration = off_duration;
    led_control->state = LED_STATE_BLINK_FAST;
    led_control->is_on = true;
    led_control->last_toggle_time = System_Get_Runtime_MS();
    
    /**
     * 启动闪烁
     * 知识点: 闪烁启动
     * - 初始状态为开启
     * - 开始闪烁循环
     */
    GPIO_TypeDef *port = (led_id == 1) ? LED1_PORT : LED2_PORT;
    uint16_t pin = (led_id == 1) ? LED1_PIN : LED2_PIN;
    GPIO_SetBits(port, pin);
}

/**
 * @brief   LED更新函数
 * @details 周期性更新LED状态
 * 
 * 知识点: LED状态更新
 * - 处理闪烁逻辑
 * - 控制引脚输出
 * - 周期性调用
 */
void LED_Update(void)
{
    /**
     * 更新LED1
     * 知识点: LED1状态更新
     */
    LED_StateMachine_Update(&LED1_Control, LED1_PORT, LED1_PIN);
    
    /**
     * 更新LED2
     * 知识点: LED2状态更新
     */
    LED_StateMachine_Update(&LED2_Control, LED2_PORT, LED2_PIN);
}

/* ============================== 按键处理函数 ============================== */

/**
 * @brief   按键扫描函数
 * @details 扫描按键状态并进行防抖处理
 * @return bool 是否有按键事件
 * 
 * 知识点: 按键扫描过程
 * 1. 读取按键电平
 * 2. 防抖处理
 * 3. 状态机管理
 * 4. 事件检测
 */
bool Key_Scan(void)
{
    /**
     * 读取按键当前状态
     * 知识点: 按键状态读取
     * - 读取PA11引脚电平
     * - 低电平表示按键按下
     * - 高电平表示按键释放
     */
    bool current_pressed = (GPIO_ReadInputDataBit(KEY_PORT, KEY_PIN) == Bit_RESET);
    
    /**
     * 状态机处理
     * 知识点: 按键状态机
     * - 根据当前状态处理不同逻辑
     * - 状态转换由事件驱动
     */
    switch (Key_Info.state) {
        case KEY_STATE_IDLE:
            /**
             * 空闲状态
             * 知识点: 空闲状态处理
             * - 检查是否有按键按下
             * - 按下则进入防抖状态
             */
            if (current_pressed) {
                Key_Info.state = KEY_STATE_DEBOUNCE;
                Key_Info.press_start_time = System_Get_Runtime_MS();
            }
            break;
            
        case KEY_STATE_DEBOUNCE:
            /**
             * 防抖状态
             * 知识点: 防抖处理
             * - 等待防抖时间
             * - 确认按键是否持续按下
             */
            if (current_pressed) {
                uint32_t debounce_time = System_Get_Runtime_MS() - Key_Info.press_start_time;
                if (debounce_time >= KEY_DEBOUNCE_MS) {
                    Key_Info.state = KEY_STATE_PRESSED;
                    Key_Info.is_pressed = true;
                    Key_Info.press_count++;
                    Key_Info.last_press_time = System_Get_Runtime_MS();
                    
                    DEBUG_PRINT("Key pressed detected (count: %u)\r\n", Key_Info.press_count);
                    
                    /**
                     * 多击检测
                     * 知识点: 多击检测
                     * - 检测连击间隔
                     * - 统计连续按压次数
                     */
                    static uint32_t last_multi_press_time = 0;
                    uint32_t current_time = System_Get_Runtime_MS();
                    if (current_time - last_multi_press_time < KEY_MULTI_PRESS_INTERVAL_MS) {
                        // 连续按压
                        Key_Info.press_count++;
                    } else {
                        // 重新开始计数
                        Key_Info.press_count = 1;
                    }
                    last_multi_press_time = current_time;
                }
            } else {
                /**
                 * 防抖期间释放
                 * 知识点: 错误释放处理
                 * - 按键释放过早，回到空闲状态
                 */
                Key_Info.state = KEY_STATE_IDLE;
            }
            break;
            
        case KEY_STATE_PRESSED:
            /**
             * 按下状态
             * 知识点: 按下状态处理
             * - 检测是否释放
             * - 检测长按
             */
            if (!current_pressed) {
                Key_Info.state = KEY_STATE_RELEASED;
                Key_Info.release_time = System_Get_Runtime_MS();
                Key_Info.is_pressed = false;
            } else {
                /**
                 * 长按检测
                 * 知识点: 长按检测
                 * - 按下时间超过阈值
                 * - 触发长按事件
                 */
                uint32_t press_duration = System_Get_Runtime_MS() - Key_Info.press_start_time;
                if (press_duration >= KEY_LONG_PRESS_MS && !Key_Info.long_press_detected) {
                    Key_Info.long_press_detected = true;
                    Key_Info.state = KEY_STATE_LONG_PRESS;
                    
                    DEBUG_PRINT("Key long press detected\r\n");
                    
                    /**
                     * 长按触发重启请求
                     * 知识点: 重启请求
                     * - 长按触发系统重启
                     * - 等待释放确认
                     */
                    Key_Info.reset_requested = true;
                }
            }
            break;
            
        case KEY_STATE_LONG_PRESS:
            /**
             * 长按状态
             * 知识点: 长按状态处理
             * - 等待按键释放
             * - 确认重启请求
             */
            if (!current_pressed) {
                Key_Info.state = KEY_STATE_RELEASED;
                Key_Info.release_time = System_Get_Runtime_MS();
                Key_Info.is_pressed = false;
                
                if (Key_Info.reset_requested) {
                    /**
                     * 重启确认
                     * 知识点: 重启确认流程
                     * - 释放后检查重启条件
                     * - 满足条件则重启系统
                     */
                    uint32_t reset_condition_time = System_Get_Runtime_MS() - Key_Info.press_start_time;
                    if (reset_condition_time >= KEY_RESET_CONFIRM_MS) {
                        DEBUG_PRINT("Key reset condition met, restarting system...\r\n");
                        System_Reboot();
                    } else {
                        Key_Info.reset_requested = false;
                    }
                }
            }
            break;
            
        case KEY_STATE_RELEASED:
            /**
             * 释放状态
             * 知识点: 释放状态处理
             * - 回到空闲状态
             * - 清除检测标志
             */
            Key_Info.state = KEY_STATE_IDLE;
            Key_Info.long_press_detected = false;
            break;
    }
    
    /**
     * 返回按键事件
     * 知识点: 事件检测
     * - 返回是否有按键事件
     * - 用于主循环处理
     */
    return (Key_Info.state == KEY_STATE_PRESSED || 
            Key_Info.state == KEY_STATE_LONG_PRESS ||
            Key_Info.state == KEY_STATE_RELEASED);
}

/**
 * @brief   按键状态获取函数
 * @details 获取按键当前按下状态
 * @return bool 按键是否按下
 * 
 * 知识点: 按键状态查询
 * - 直接返回当前按下状态
 * - 不包含防抖信息
 */
bool Key_Is_Pressed(void)
{
    return Key_Info.is_pressed;
}

/**
 * @brief   按键长按检测函数
 * @details 检测是否长按
 * @return bool 是否检测到长按
 * 
 * 知识点: 长按检测查询
 * - 返回长按检测结果
 * - 检测后需要手动清除
 */
bool Key_Is_Long_Pressed(void)
{
    return Key_Info.long_press_detected;
}

/**
 * @brief   按键释放检测函数
 * @details 检测按键是否释放
 * @return bool 是否检测到释放
 * 
 * 知识点: 释放检测查询
 * - 返回释放检测结果
 * - 用于确认按键事件结束
 */
bool Key_Is_Released(void)
{
    return (Key_Info.state == KEY_STATE_RELEASED);
}

/**
 * @brief   多击检测函数
 * @details 获取连续按压次数
 * @param interval_ms 击键间隔时间
 * @param max_clicks 最大击键次数
 * @return uint8_t 实际击键次数
 * 
 * 知识点: 多击检测
 * - 检测连续按压次数
 * - 用于实现多击功能
 */
uint8_t Key_Get_Click_Count(uint32_t interval_ms, uint8_t max_clicks)
{
    /**
     * 检查击键间隔
     * 知识点: 间隔检查
     * - 当前时间与最后按压时间比较
     * - 在间隔内则累加计数
     */
    uint32_t current_time = System_Get_Runtime_MS();
    if (current_time - Key_Info.last_press_time > interval_ms) {
        // 超过间隔，重置计数
        return 0;
    }
    
    /**
     * 限制最大计数
     * 知识点: 计数限制
     * - 不超过指定的最大次数
     */
    return (Key_Info.press_count > max_clicks) ? max_clicks : Key_Info.press_count;
}

/**
 * @brief   按键状态重置函数
 * @details 重置按键所有状态
 * 
 * 知识点: 按键状态重置
 * - 清除所有状态
 * - 重新开始检测
 */
void Key_Reset_State(void)
{
    Key_Info.state = KEY_STATE_IDLE;
    Key_Info.is_pressed = false;
    Key_Info.press_start_time = 0;
    Key_Info.release_time = 0;
    Key_Info.press_count = 0;
    Key_Info.last_press_time = 0;
    Key_Info.long_press_detected = false;
    Key_Info.reset_requested = false;
    
    DEBUG_PRINT("Key state reset\r\n");
}

/* ============================== 蜂鸣器控制函数 ============================== */

/**
 * @brief   蜂鸣器控制函数
 * @details 控制蜂鸣器的开关
 * @param on 是否开启
 * 
 * 知识点: 蜂鸣器开关控制
 * - 简单开关控制
 * - 不影响报警模式
 */
void Buzzer_Control(bool on)
{
    if (on) {
        GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
    } else {
        GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
    }
    
    /**
     * 更新控制结构
     * 知识点: 状态更新
     */
    System_Status_Indicator.buzzer_control.is_beeping = on;
}

/**
 * @brief   蜂鸣器报警函数
 * @details 控制蜂鸣器报警模式
 * @param beep_count 蜂鸣次数
 * @param duration_ms 每次蜂鸣持续时间
 * @param interval_ms 蜂鸣间隔时间
 * 
 * 知识点: 蜂鸣器报警控制
 * - 设置报警参数
 * - 启动报警序列
 */
void Buzzer_Alarm(uint8_t beep_count, uint32_t duration_ms, uint32_t interval_ms)
{
    /**
     * 设置报警参数
     * 知识点: 报警参数设置
     * - 设置蜂鸣次数
     * - 设置持续时间
     * - 设置间隔时间
     * - 激活报警模式
     */
    System_Status_Indicator.buzzer_control.is_active = true;
    System_Status_Indicator.buzzer_control.beep_count = beep_count;
    System_Status_Indicator.buzzer_control.beep_duration = duration_ms;
    System_Status_Indicator.buzzer_control.beep_interval = interval_ms;
    System_Status_Indicator.buzzer_control.current_beep = 0;
    System_Status_Indicator.buzzer_control.last_beep_time = System_Get_Runtime_MS();
    System_Status_Indicator.buzzer_control.beep_start_time = 0;
    
    /**
     * 统计信息更新
     * 知识点: 统计更新
     * - 报警计数增加
     */
    GPIO_Module_Status.buzzer_alarm_count++;
    
    /**
     * 启动第一次蜂鸣
     * 知识点: 首次蜂鸣
     * - 立即开始第一次蜂鸣
     * - 后续蜂鸣由状态机控制
     */
    System_Status_Indicator.buzzer_control.beep_start_time = System_Get_Runtime_MS();
    GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
    System_Status_Indicator.buzzer_control.is_beeping = true;
    
    DEBUG_PRINT("Buzzer alarm started: %u beeps, %lu ms duration, %lu ms interval\r\n", 
               beep_count, duration_ms, interval_ms);
}

/**
 * @brief   蜂鸣器更新函数
 * @details 周期性更新蜂鸣器状态
 * 
 * 知识点: 蜂鸣器状态更新
 * - 控制蜂鸣时序
 * - 处理报警序列
 * - 周期性调用
 */
void Buzzer_Update(void)
{
    Buzzer_StateMachine_Update();
}

/* ============================== 系统状态指示函数 ============================== */

/**
 * @brief   系统状态指示更新函数
 * @details 根据系统状态更新指示设备
 * 
 * 知识点: 系统状态指示更新
 * - 集中管理所有状态指示
 * - 根据错误状态调整LED
 * - 触发相应报警
 */
void System_Status_Update(void)
{
    /**
     * 系统LED状态更新
     * 知识点: 系统状态LED
     * - 系统正常: 慢闪烁
     * - 系统错误: 快速闪烁
     * - 系统停止: 关闭
     */
    if (System_Status_Indicator.system_error) {
        // 系统错误
        if (LED1_Control.state != LED_STATE_BLINK_FAST) {
            LED_Control(1, LED_STATE_BLINK_FAST);
        }
    } else {
        // 系统正常
        if (LED1_Control.state != LED_STATE_BLINK_SLOW) {
            LED_Control(1, LED_STATE_BLINK_SLOW);
        }
    }
    
    /**
     * 网络LED状态更新
     * 知识点: 网络状态LED
     * - 网络正常: 开启
     * - 网络错误: 快速闪烁
     * - 网络断开: 关闭
     */
    if (System_Status_Indicator.network_error) {
        // 网络错误
        if (LED2_Control.state != LED_STATE_BLINK_FAST) {
            LED_Control(2, LED_STATE_BLINK_FAST);
        }
    } else if (System_Status_Indicator.network_led_state == LED_STATE_ON) {
        // 网络正常
        if (LED2_Control.state != LED_STATE_ON) {
            LED_Control(2, LED_STATE_ON);
        }
    } else {
        // 网络断开或未知状态
        if (LED2_Control.state != LED_STATE_OFF) {
            LED_Control(2, LED_STATE_OFF);
        }
    }
    
    /**
     * 数据错误处理
     * 知识点: 数据错误报警
     * - 数据错误时触发蜂鸣
     * - 短促报警提示
     */
    if (System_Status_Indicator.data_error) {
        if (!System_Status_Indicator.buzzer_control.is_active) {
            Buzzer_Alarm(2, 200, 300);  // 2次短促报警
            System_Status_Indicator.data_error = false;  // 清除错误标志
        }
    }
}

/**
 * @brief   系统错误设置函数
 * @details 设置系统错误状态
 * @param error_type 错误类型
 * @param is_error 是否为错误状态
 * 
 * 知识点: 错误状态设置
 * - 分类设置不同错误
 * - 触发相应处理
 */
void System_Set_Error(uint8_t error_type, bool is_error)
{
    switch (error_type) {
        case 0:  // 系统错误
            System_Status_Indicator.system_error = is_error;
            break;
        case 1:  // 网络错误
            System_Status_Indicator.network_error = is_error;
            break;
        case 2:  // 数据错误
            System_Status_Indicator.data_error = is_error;
            break;
    }
    
    /**
     * 错误状态改变时更新指示
     * 知识点: 状态变化处理
     * - 错误状态改变时立即更新指示
     */
    System_Status_Update();
    
    DEBUG_PRINT("System error type %u set to: %s\r\n", error_type, is_error ? "ERROR" : "OK");
}

/**
 * @brief   网络状态设置函数
 * @details 设置网络连接状态
 * @param connected 是否连接
 * @param error 是否有错误
 * 
 * 知识点: 网络状态设置
 * - 设置连接状态
 * - 设置错误状态
 * - 触发状态更新
 */
void System_Set_Network_Status(bool connected, bool error)
{
    System_Status_Indicator.network_led_state = connected ? LED_STATE_ON : LED_STATE_OFF;
    System_Status_Indicator.network_error = error;
    
    System_Status_Update();
    
    DEBUG_PRINT("Network status: %s, Error: %s\r\n", 
               connected ? "Connected" : "Disconnected", 
               error ? "Yes" : "No");
}

/**
 * @brief   数据状态设置函数
 * @details 设置数据状态
 * @param has_error 是否有错误
 * 
 * 知识点: 数据状态设置
 * - 设置数据错误标志
 * - 触发报警处理
 */
void System_Set_Data_Status(bool has_error)
{
    System_Status_Indicator.data_error = has_error;
    
    if (has_error) {
        System_Status_Update();  // 立即触发报警
    }
    
    DEBUG_PRINT("Data status: %s\r\n", has_error ? "ERROR" : "OK");
}

/**
 * @brief   系统状态指示获取函数
 * @details 获取系统状态指示结构
 * @return System_Status_Indicator_t* 状态指示结构指针
 * 
 * 知识点: 状态指示查询
 * - 返回状态指示结构指针
 * - 供其他模块查询状态
 */
System_Status_Indicator_t* System_Get_Status_Indicator(void)
{
    return &System_Status_Indicator;
}

/* ============================== 状态机实现 ============================== */

/**
 * @brief   LED状态机更新函数
 * @details 更新LED状态机
 * @param led_control LED控制结构
 * @param port GPIO端口
 * @param pin 引脚号
 * 
 * 知识点: LED状态机
 * - 处理LED闪烁逻辑
 * - 控制引脚输出
 * - 时间管理
 */
void LED_StateMachine_Update(LED_Control_t *led_control, GPIO_TypeDef* port, uint16_t pin)
{
    /**
     * 闪烁状态处理
     * 知识点: 闪烁逻辑
     * - 根据时间切换LED状态
     * - 控制占空比
     */
    if (led_control->state == LED_STATE_BLINK_FAST || 
        led_control->state == LED_STATE_BLINK_SLOW) {
        
        uint32_t current_time = System_Get_Runtime_MS();
        uint32_t elapsed_time = current_time - led_control->last_toggle_time;
        
        if (elapsed_time >= led_control->blink_interval) {
            // 切换LED状态
            LED_Toggle(led_control == &LED1_Control ? 1 : 2);
            led_control->last_toggle_time = current_time;
        }
    }
}

/**
 * @brief   按键状态机更新函数
 * @details 更新按键状态机
 * 
 * 知识点: 按键状态机
 * - 周期性调用扫描按键
 * - 更新状态机状态
 * - 检测按键事件
 */
void Key_StateMachine_Update(void)
{
    Key_Scan();
}

/**
 * @brief   蜂鸣器状态机更新函数
 * @details 更新蜂鸣器状态机
 * 
 * 知识点: 蜂鸣器状态机
 * - 控制蜂鸣时序
 * - 处理报警序列
 * - 时间管理
 */
void Buzzer_StateMachine_Update(void)
{
    Buzzer_Control_t *buzzer = &System_Status_Indicator.buzzer_control;
    
    /**
     * 报警序列处理
     * 知识点: 报警时序控制
     * - 控制蜂鸣开启/关闭时间
     * - 管理蜂鸣次数
     */
    if (buzzer->is_active && buzzer->current_beep < buzzer->beep_count) {
        uint32_t current_time = System_Get_Runtime_MS();
        
        if (buzzer->is_beeping) {
            // 当前正在蜂鸣
            if (current_time - buzzer->beep_start_time >= buzzer->beep_duration) {
                // 蜂鸣时间结束，停止蜂鸣
                GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
                buzzer->is_beeping = false;
                buzzer->current_beep++;
                buzzer->last_beep_time = current_time;
                
                DEBUG_PRINT("Buzzer beep %u/%u completed\r\n", 
                           buzzer->current_beep, buzzer->beep_count);
            }
        } else {
            // 当前没有蜂鸣，检查是否开始下一次蜂鸣
            if (current_time - buzzer->last_beep_time >= buzzer->beep_interval) {
                // 开始下一次蜂鸣
                buzzer->beep_start_time = current_time;
                GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
                buzzer->is_beeping = true;
            }
        }
    } else {
        /**
         * 报警结束
         * 知识点: 报警结束处理
         * - 清除激活标志
         * - 关闭蜂鸣器
         */
        if (buzzer->is_active) {
            buzzer->is_active = false;
            GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
            buzzer->is_beeping = false;
            
            DEBUG_PRINT("Buzzer alarm completed\r\n");
        }
    }
}

/* ============================== 调试函数实现 ============================== */

#ifdef DEBUG
/**
 * @brief   GPIO状态打印函数
 * @details 打印所有GPIO状态信息
 */
void GPIO_Print_Status(void)
{
    printf("\r\nGPIO Module Status:\r\n");
    
    /**
     * 打印模块状态
     * 知识点: 模块状态显示
     */
    printf("  Module:\r\n");
    printf("    Initialized: %s\r\n", GPIO_Module_Status.is_initialized ? "Yes" : "No");
    printf("    Error Count: %lu\r\n", GPIO_Module_Status.error_count);
    printf("    LED Toggle Count: %lu\r\n", GPIO_Module_Status.led_toggle_count);
    printf("    Key Press Count: %lu\r\n", GPIO_Module_Status.key_press_count);
    printf("    Buzzer Alarm Count: %lu\r\n", GPIO_Module_Status.buzzer_alarm_count);
    
    /**
     * 打印LED状态
     * 知识点: LED状态显示
     */
    printf("  LED1 (System):\r\n");
    printf("    State: %d (%s)\r\n", LED1_Control.state,
           LED1_Control.state == LED_STATE_OFF ? "OFF" :
           LED1_Control.state == LED_STATE_ON ? "ON" :
           LED1_Control.state == LED_STATE_BLINK_FAST ? "BLINK_FAST" :
           LED1_Control.state == LED_STATE_BLINK_SLOW ? "BLINK_SLOW" : "BREATH");
    printf("    Is On: %s\r\n", LED1_Control.is_on ? "Yes" : "No");
    printf("    Pin State: %s\r\n", 
           GPIO_ReadOutputDataBit(LED1_PORT, LED1_PIN) ? "HIGH" : "LOW");
    
    printf("  LED2 (Network):\r\n");
    printf("    State: %d (%s)\r\n", LED2_Control.state,
           LED2_Control.state == LED_STATE_OFF ? "OFF" :
           LED2_Control.state == LED_STATE_ON ? "ON" :
           LED2_Control.state == LED_STATE_BLINK_FAST ? "BLINK_FAST" :
           LED2_Control.state == LED_STATE_BLINK_SLOW ? "BLINK_SLOW" : "BREATH");
    printf("    Is On: %s\r\n", LED2_Control.is_on ? "Yes" : "No");
    printf("    Pin State: %s\r\n", 
           GPIO_ReadOutputDataBit(LED2_PORT, LED2_PIN) ? "HIGH" : "LOW");
    
    /**
     * 打印按键状态
     * 知识点: 按键状态显示
     */
    printf("  Key (PA11):\r\n");
    printf("    State: %d (%s)\r\n", Key_Info.state,
           Key_Info.state == KEY_STATE_IDLE ? "IDLE" :
           Key_Info.state == KEY_STATE_DEBOUNCE ? "DEBOUNCE" :
           Key_Info.state == KEY_STATE_PRESSED ? "PRESSED" :
           Key_Info.state == KEY_STATE_LONG_PRESS ? "LONG_PRESS" : "RELEASED");
    printf("    Is Pressed: %s\r\n", Key_Info.is_pressed ? "Yes" : "No");
    printf("    Pin State: %s\r\n", 
           GPIO_ReadInputDataBit(KEY_PORT, KEY_PIN) ? "HIGH" : "LOW");
    printf("    Press Count: %u\r\n", Key_Info.press_count);
    printf("    Long Press Detected: %s\r\n", 
           Key_Info.long_press_detected ? "Yes" : "No");
    printf("    Reset Requested: %s\r\n", 
           Key_Info.reset_requested ? "Yes" : "No");
    
    /**
     * 打印蜂鸣器状态
     * 知识点: 蜂鸣器状态显示
     */
    printf("  Buzzer (PB7):\r\n");
    printf("    Is Active: %s\r\n", 
           System_Status_Indicator.buzzer_control.is_active ? "Yes" : "No");
    printf("    Is Beeping: %s\r\n", 
           System_Status_Indicator.buzzer_control.is_beeping ? "Yes" : "No");
    printf("    Pin State: %s\r\n", 
           GPIO_ReadOutputDataBit(BUZZER_PORT, BUZZER_PIN) ? "HIGH" : "LOW");
    printf("    Beep Count: %u/%u\r\n", 
           System_Status_Indicator.buzzer_control.current_beep,
           System_Status_Indicator.buzzer_control.beep_count);
    
    /**
     * 打印系统状态指示
     * 知识点: 系统状态显示
     */
    printf("  System Status:\r\n");
    printf("    System Error: %s\r\n", 
           System_Status_Indicator.system_error ? "Yes" : "No");
    printf("    Network Error: %s\r\n", 
           System_Status_Indicator.network_error ? "Yes" : "No");
    printf("    Data Error: %s\r\n", 
           System_Status_Indicator.data_error ? "Yes" : "No");
}

/**
 * @brief   按键测试函数
 * @details 按键功能测试
 * @param test_duration_ms 测试持续时间
 * 
 * 知识点: 按键功能测试
 * - 持续扫描按键
 * - 打印按键事件
 * - 测试防抖和长按
 */
void Key_Test(uint32_t test_duration_ms)
{
    printf("\r\nKey Test started (duration: %lu ms)\r\n", test_duration_ms);
    
    uint32_t start_time = System_Get_Runtime_MS();
    uint32_t test_end_time = start_time + test_duration_ms;
    
    while (System_Get_Runtime_MS() < test_end_time) {
        /**
         * 扫描按键
         * 知识点: 按键扫描
         */
        if (Key_Scan()) {
            /**
             * 检测到按键事件
             * 知识点: 事件处理
             */
            uint32_t current_time = System_Get_Runtime_MS();
            printf("Key event at %lu ms: ", current_time - start_time);
            
            if (Key_Info.state == KEY_STATE_PRESSED) {
                printf("PRESSED");
            } else if (Key_Info.state == KEY_STATE_LONG_PRESS) {
                printf("LONG_PRESS");
            } else if (Key_Info.state == KEY_STATE_RELEASED) {
                printf("RELEASED");
            }
            
            printf(" (count: %u)\r\n", Key_Info.press_count);
        }
        
        Delay_ms(50);  // 扫描间隔
    }
    
    printf("Key test completed\r\n");
}

/**
 * @brief   LED闪烁测试函数
 * @details LED功能测试
 * @param test_duration_ms 测试持续时间
 * 
 * 知识点: LED功能测试
 * - 各种LED状态测试
 * - 闪烁效果验证
 */
void LED_Blink_Test(uint32_t test_duration_ms)
{
    printf("\r\nLED Blink Test started (duration: %lu ms)\r\n", test_duration_ms);
    
    uint32_t start_time = System_Get_Runtime_MS();
    uint32_t test_end_time = start_time + test_duration_ms;
    uint8_t test_phase = 0;
    
    while (System_Get_Runtime_MS() < test_end_time) {
        uint32_t current_time = System_Get_Runtime_MS();
        uint32_t elapsed_time = current_time - start_time;
        
        /**
         * 测试阶段控制
         * 知识点: 测试阶段管理
         * - 每5秒切换一次测试模式
         */
        if ((elapsed_time / 5000) != test_phase) {
            test_phase = elapsed_time / 5000;
            
            switch (test_phase % 4) {
                case 0:  // LED1慢闪烁
                    printf("Phase %u: LED1 slow blink\r\n", test_phase);
                    LED_Control(1, LED_STATE_BLINK_SLOW);
                    LED_Control(2, LED_STATE_OFF);
                    break;
                case 1:  // LED1快闪烁
                    printf("Phase %u: LED1 fast blink\r\n", test_phase);
                    LED_Control(1, LED_STATE_BLINK_FAST);
                    LED_Control(2, LED_STATE_OFF);
                    break;
                case 2:  // LED2闪烁
                    printf("Phase %u: LED2 blink\r\n", test_phase);
                    LED_Control(1, LED_STATE_OFF);
                    LED_Control(2, LED_STATE_BLINK_FAST);
                    break;
                case 3:  // 双LED闪烁
                    printf("Phase %u: Both LEDs blink\r\n", test_phase);
                    LED_Control(1, LED_STATE_BLINK_SLOW);
                    LED_Control(2, LED_STATE_BLINK_FAST);
                    break;
            }
        }
        
        /**
         * 更新LED状态
         * 知识点: LED状态更新
         */
        LED_Update();
        
        Delay_ms(100);  // 更新间隔
    }
    
    // 测试结束，关闭所有LED
    LED_Control(1, LED_STATE_OFF);
    LED_Control(2, LED_STATE_OFF);
    
    printf("LED test completed\r\n");
}

/**
 * @brief   蜂鸣器测试函数
 * @details 蜂鸣器功能测试
 * @param test_duration_ms 测试持续时间
 * 
 * 知识点: 蜂鸣器功能测试
 * - 各种报警模式测试
 * - 声音效果验证
 */
void Buzzer_Test(uint32_t test_duration_ms)
{
    printf("\r\nBuzzer Test started (duration: %lu ms)\r\n", test_duration_ms);
    
    uint32_t start_time = System_Get_Runtime_MS();
    uint32_t test_end_time = start_time + test_duration_ms;
    uint8_t test_phase = 0;
    
    while (System_Get_Runtime_MS() < test_end_time) {
        uint32_t current_time = System_Get_Runtime_MS();
        uint32_t elapsed_time = current_time - start_time;
        
        /**
         * 测试阶段控制
         * 知识点: 测试阶段管理
         * - 每3秒切换一次测试模式
         */
        if ((elapsed_time / 3000) != test_phase) {
            test_phase = elapsed_time / 3000;
            
            switch (test_phase % 3) {
                case 0:  // 单次蜂鸣
                    printf("Phase %u: Single beep\r\n", test_phase);
                    Buzzer_Alarm(1, 500, 1000);
                    break;
                case 1:  // 多次蜂鸣
                    printf("Phase %u: Multiple beeps\r\n", test_phase);
                    Buzzer_Alarm(3, 200, 400);
                    break;
                case 2:  // 连续蜂鸣
                    printf("Phase %u: Continuous beep\r\n", test_phase);
                    Buzzer_Control(true);
                    Delay_ms(1000);
                    Buzzer_Control(false);
                    break;
            }
        }
        
        /**
         * 更新蜂鸣器状态
         * 知识点: 蜂鸣器状态更新
         */
        Buzzer_Update();
        
        Delay_ms(100);  // 更新间隔
    }
    
    // 测试结束，关闭蜂鸣器
    Buzzer_Control(false);
    
    printf("Buzzer test completed\r\n");
}
#endif

/* ============================== 错误处理函数 ============================== */

/**
 * @brief   GPIO错误处理函数
 * @details 处理GPIO模块错误
 * @param error_msg 错误信息
 * 
 * 知识点: GPIO错误处理
 * - 记录错误信息
 * - 错误计数增加
 * - 触发错误指示
 */
void GPIO_Error_Handler(const char *error_msg)
{
    GPIO_Module_Status.error_count++;
    
    DEBUG_PRINT("GPIO Error: %s (count: %lu)\r\n", error_msg, GPIO_Module_Status.error_count);
    
    /**
     * 触发错误指示
     * 知识点: 错误指示
     * - LED快速闪烁
     * - 蜂鸣报警
     */
    LED_Control(1, LED_STATE_BLINK_FAST);
    Buzzer_Alarm(BUZZER_ERROR_BEEP_COUNT, 300, 500);
}

/* ============================== 外部中断服务函数 ============================== */

/**
 * @brief   外部中断服务函数
 * @details 处理PA11按键外部中断
 * 
 * 知识点: 外部中断服务
 * - 中断服务函数名固定
 * - 处理按键按下事件
 * - 简单防抖处理
 */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(KEY_EXTI_LINE) != RESET) {
        /**
         * 清除中断标志
         * 知识点: 中断标志清除
         * - 必须清除中断标志
         * - 否则会持续触发中断
         */
        EXTI_ClearITPendingBit(KEY_EXTI_LINE);
        
        /**
         * 简单防抖
         * 知识点: 中断防抖
         * - 延时去抖
         * - 确认按键确实按下
         */
        Delay_ms(KEY_DEBOUNCE_MS);
        
        /**
         * 再次检查按键状态
         * 知识点: 状态确认
         * - 确认按键仍然按下
         * - 防止误触发
         */
        if (GPIO_ReadInputDataBit(KEY_PORT, KEY_PIN) == Bit_RESET) {
            /**
             * 确认按键按下
             * 知识点: 按键确认
             * - 更新按键状态
             * - 统计按键事件
             */
            Key_Info.is_pressed = true;
            Key_Info.press_start_time = System_Get_Runtime_MS();
            Key_Info.press_count++;
            Key_Info.last_press_time = Key_Info.press_start_time;
            
            /**
             * 启动防抖定时器
             * 知识点: 防抖定时器
             * - 启动软件定时器进行防抖
             * - 在定时器中断中处理完整防抖逻辑
             */
            
            DEBUG_PRINT("EXTI Key pressed detected (count: %u)\r\n", Key_Info.press_count);
        }
    }
}
