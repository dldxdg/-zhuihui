/**
 * @file    gpio.h
 * @brief   GPIO控制和外部中断头文件 (标准库版本)
 * @author  MiniMax Agent
 * @version V2.0
 * @date    2025-11-08
 * 
 * 知识点总结:
 * 
 * 【GPIO基础概念】
 * 1. GPIO(General Purpose Input/Output)通用输入输出
 * 2. STM32F103有多个GPIO端口(GPIOA-GPIOC)
 * 3. 每个端口有16个引脚(0-15)
 * 4. 可配置为输入、输出、复用、模拟等多种模式
 * 
 * 【GPIO工作模式】
 * 1. 输入模式: 浮空、上拉、下拉、模拟输入
 * 2. 输出模式: 推挽、开漏、复用推挽、复用开漏
 * 3. 速度配置: 2MHz、10MHz、50MHz
 * 4. 不同外设需要不同配置
 * 
 * 【外部中断EXTI】
 * 1. EXTI(External Interrupt)外部中断控制器
 * 2. 可以配置上升沿、下降沿或双边沿触发
 * 3. 支持软件中断触发
 * 4. 可以映射到不同的GPIO引脚
 * 
 * 【系统指示设计】
 * 1. LED1(PA0): 系统状态指示
 * 2. LED2(PB8): 网络状态指示
 * 3. 蜂鸣器(PB7): 报警提示
 * 4. 按键(PA11): 系统重启
 */

#ifndef __GPIO_H
#define __GPIO_H

/* ============================== 包含头文件 ============================== */

#include "stm32f1xx.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================== 宏定义 ============================== */

/**
 * LED控制定义
 * 知识点: LED引脚配置
 * - PA0: LED1，系统状态指示
 * - PB8: LED2，网络状态指示
 * - 推挽输出，最大速度50MHz
 */
#define LED1_PIN                 GPIO_Pin_0        // PA0
#define LED1_PORT                GPIOA
#define LED1_CLOCK               RCC_APB2Periph_GPIOA

#define LED2_PIN                 GPIO_Pin_8        // PB8
#define LED2_PORT                GPIOB
#define LED2_CLOCK               RCC_APB2Periph_GPIOB

/**
 * 蜂鸣器控制定义
 * 知识点: 蜂鸣器引脚配置
 * - PB7: 蜂鸣器控制
 * - 推挽输出，可产生方波驱动
 */
#define BUZZER_PIN               GPIO_Pin_7        // PB7
#define BUZZER_PORT              GPIOB
#define BUZZER_CLOCK             RCC_APB2Periph_GPIOB

/**
 * 按键控制定义
 * 知识点: 按键引脚配置
 * - PA11: 系统重启按键
 * - 上拉输入，外部中断检测
 * - 下降沿触发(按键按下时)
 */
#define KEY_PIN                  GPIO_Pin_11       // PA11
#define KEY_PORT                 GPIOA
#define KEY_CLOCK                RCC_APB2Periph_GPIOA
#define KEY_EXTI_LINE            EXTI_Line11       // 外部中断线11
#define KEY_EXTI_PORT            GPIO_PortSourceGPIOA
#define KEY_EXTI_PIN             GPIO_PinSource11  // 引脚源11
#define KEY_EXTI_IRQN            EXTI15_10_IRQn   // 中断号

/**
 * 传感器引脚定义
 * 知识点: 传感器接口配置
 * - PA1: 温度传感器(热敏电阻)
 * - PA4: 光线传感器(光敏电阻)  
 * - PB1: 气体传感器(MQ135)
 * - 配置为模拟输入模式
 */
#define TEMP_SENSOR_PIN          GPIO_Pin_1        // PA1
#define TEMP_SENSOR_PORT         GPIOA

#define LIGHT_SENSOR_PIN         GPIO_Pin_4        // PA4
#define LIGHT_SENSOR_PORT        GPIOA

#define GAS_SENSOR_PIN           GPIO_Pin_1        // PB1
#define GAS_SENSOR_PORT          GPIOB

/**
 * LED闪烁参数
 * 知识点: LED控制参数
 * - 闪烁频率控制
 * - 占空比控制
 * - 状态指示模式
 */
#define LED_BLINK_FAST_MS        200              // 快速闪烁间隔(毫秒)
#define LED_BLINK_SLOW_MS        1000             // 慢速闪烁间隔(毫秒)
#define LED_ON_MS                100              // LED亮起时间(毫秒)
#define LED_OFF_MS               100              // LED熄灭时间(毫秒)

/**
 * 按键参数
 * 知识点: 按键处理参数
 * - 防抖延时
 * - 长按检测时间
 * - 重启确认时间
 */
#define KEY_DEBOUNCE_MS          50               // 按键防抖延时(毫秒)
#define KEY_LONG_PRESS_MS        2000             // 长按检测时间(毫秒)
#define KEY_RESET_CONFIRM_MS     3000             // 重启确认时间(毫秒)
#define KEY_MULTI_PRESS_INTERVAL_MS 500           // 多击间隔时间(毫秒)

/**
 * 蜂鸣器参数
 * 知识点: 蜂鸣器控制参数
 * - 报警频率
 * - 报警持续时间
 * - 报警模式
 */
#define BUZZER_FREQ_HZ           1000             // 蜂鸣器频率(Hz)
#define BUZZER_ALARM_DURATION_MS 500              // 报警持续时间(毫秒)
#define BUZZER_ERROR_BEEP_COUNT  3                // 错误报警蜂鸣次数
#define BUZZER_SUCCESS_BEEP_COUNT 1               // 成功报警蜂鸣次数

/* ============================== 数据结构定义 ============================== */

/**
 * LED状态枚举
 * 知识点: LED状态定义
 * - 用于表示不同LED的控制状态
 */
typedef enum {
    LED_STATE_OFF = 0,            // LED关闭
    LED_STATE_ON,                 // LED开启
    LED_STATE_BLINK_FAST,         // 快速闪烁
    LED_STATE_BLINK_SLOW,         // 慢速闪烁
    LED_STATE_BREATH              // 呼吸灯效果(可选)
} LED_State_t;

/**
 * LED控制结构
 * 知识点: LED控制状态管理
 * - 记录每个LED的当前状态
 * - 管理闪烁定时
 */
typedef struct {
    LED_State_t state;            // 当前状态
    bool is_on;                   // 当前开关状态
    uint32_t last_toggle_time;    // 最后切换时间
    uint32_t blink_interval;      // 闪烁间隔
    uint32_t on_duration;         // 开启持续时间
    uint32_t off_duration;        // 关闭持续时间
} LED_Control_t;

/**
 * 按键状态枚举
 * 知识点: 按键状态定义
 * - 用于表示按键的不同状态
 */
typedef enum {
    KEY_STATE_IDLE = 0,           // 按键空闲
    KEY_STATE_DEBOUNCE,           // 防抖中
    KEY_STATE_PRESSED,            // 按键按下
    KEY_STATE_LONG_PRESS,         // 长按检测
    KEY_STATE_RELEASED            // 按键释放
} Key_State_t;

/**
 * 按键信息结构
 * 知识点: 按键状态管理
 * - 记录按键状态和时间信息
 * - 管理多击检测
 */
typedef struct {
    Key_State_t state;            // 当前状态
    bool is_pressed;              // 当前按下状态
    uint32_t press_start_time;    // 按下开始时间
    uint32_t release_time;        // 释放时间
    uint32_t press_count;         // 连续按压次数
    uint32_t last_press_time;     // 最后按压时间
    bool long_press_detected;     // 长按检测标志
    bool reset_requested;         // 重启请求标志
} Key_Info_t;

/**
 * 蜂鸣器状态结构
 * 知识点: 蜂鸣器控制状态
 * - 记录蜂鸣器工作状态
 * - 管理报警模式
 */
typedef struct {
    bool is_active;               // 是否激活
    bool is_beeping;              // 是否正在蜂鸣
    uint32_t beep_start_time;     // 蜂鸣开始时间
    uint32_t beep_duration;       // 蜂鸣持续时间
    uint32_t beep_interval;       // 蜂鸣间隔
    uint8_t beep_count;           // 蜂鸣次数
    uint8_t current_beep;         // 当前蜂鸣次数
    uint32_t last_beep_time;      // 最后蜂鸣时间
} Buzzer_Control_t;

/**
 * GPIO模块状态结构
 * 知识点: GPIO模块整体状态
 * - 记录各组件状态
 * - 统计操作信息
 */
typedef struct {
    bool is_initialized;          // 初始化状态
    uint32_t error_count;         // 错误计数
    uint32_t led_toggle_count;    // LED切换计数
    uint32_t key_press_count;     // 按键按压计数
    uint32_t buzzer_alarm_count;  // 蜂鸣报警计数
} GPIO_Module_Status_t;

/**
 * 系统状态指示结构
 * 知识点: 系统状态管理
 * - 管理各状态指示
 * - 统一控制系统反馈
 */
typedef struct {
    LED_State_t system_led_state;     // 系统LED状态
    LED_State_t network_led_state;    // 网络LED状态
    Buzzer_Control_t buzzer_control;  // 蜂鸣器控制
    bool system_error;                // 系统错误标志
    bool network_error;               // 网络错误标志
    bool data_error;                  // 数据错误标志
} System_Status_Indicator_t;

/* ============================== 全局变量声明 ============================== */

// GPIO模块状态
extern GPIO_Module_Status_t GPIO_Module_Status;

// LED控制
extern LED_Control_t LED1_Control;  // 系统状态LED
extern LED_Control_t LED2_Control;  // 网络状态LED

// 按键信息
extern Key_Info_t Key_Info;

// 系统状态指示
extern System_Status_Indicator_t System_Status_Indicator;

/* ============================== 函数声明 ============================== */

/**
 * GPIO模块初始化函数
 * 知识点: GPIO模块初始化
 * 1. 初始化LED控制
 * 2. 初始化按键控制
 * 3. 初始化蜂鸣器
 * 4. 配置外部中断
 */
void GPIO_Module_Init(void);

/**
 * LED初始化函数
 * 知识点: LED引脚初始化
 * - 配置LED引脚为推挽输出
 * - 初始化LED控制结构
 * - 设置默认状态
 */
void LED_Init(void);

/**
 * LED控制函数
 * 知识点: LED控制
 * @param led_id LED标识(1=LED1, 2=LED2)
 * @param state LED状态
 */
void LED_Control(uint8_t led_id, LED_State_t state);

/**
 * LED开关函数
 * 知识点: LED开关控制
 * @param led_id LED标识
 * @param on 是否开启
 */
void LED_Set(uint8_t led_id, bool on);

/**
 * LED切换函数
 * 知识点: LED状态切换
 * @param led_id LED标识
 */
void LED_Toggle(uint8_t led_id);

/**
 * LED闪烁控制函数
 * 知识点: LED闪烁控制
 * @param led_id LED标识
 * @param interval_ms 闪烁间隔
 * @param on_duration 开启时间
 * @param off_duration 关闭时间
 */
void LED_Blink(uint8_t led_id, uint32_t interval_ms, uint32_t on_duration, uint32_t off_duration);

/**
 * LED更新函数
 * 知识点: LED状态更新
 * - 周期性调用更新LED状态
 * - 处理闪烁和呼吸效果
 * - 根据当前状态控制引脚
 */
void LED_Update(void);

/**
 * 按键初始化函数
 * 知识点: 按键引脚初始化
 * - 配置按键引脚为上拉输入
 * - 配置外部中断
 * - 初始化按键状态
 */
void Key_Init(void);

/**
 * 按键扫描函数
 * 知识点: 按键状态扫描
 * - 读取按键电平
 * - 防抖处理
 * - 状态机管理
 * @return bool 是否有按键事件
 */
bool Key_Scan(void);

/**
 * 按键状态获取函数
 * 知识点: 按键状态查询
 * @return bool 按键是否按下
 */
bool Key_Is_Pressed(void);

/**
 * 按键长按检测函数
 * 知识点: 长按检测
 * @return bool 是否检测到长按
 */
bool Key_Is_Long_Pressed(void);

/**
 * 按键释放检测函数
 * 知识点: 释放检测
 * @return bool 是否检测到释放
 */
bool Key_Is_Released(void);

/**
 * 多击检测函数
 * 知识点: 多击检测
 * @param interval_ms 击键间隔时间
 * @param max_clicks 最大击键次数
 * @return uint8_t 实际击键次数
 */
uint8_t Key_Get_Click_Count(uint32_t interval_ms, uint8_t max_clicks);

/**
 * 按键状态重置函数
 * 知识点: 按键状态重置
 * - 重置所有按键状态
 * - 清除检测标志
 */
void Key_Reset_State(void);

/**
 * 蜂鸣器初始化函数
 * 知识点: 蜂鸣器初始化
 * - 配置蜂鸣器引脚
 * - 初始化控制结构
 * - 默认关闭状态
 */
void Buzzer_Init(void);

/**
 * 蜂鸣器控制函数
 * 知识点: 蜂鸣器控制
 * @param on 是否开启
 */
void Buzzer_Control(bool on);

/**
 * 蜂鸣器报警函数
 * 知识点: 蜂鸣器报警
 * @param beep_count 蜂鸣次数
 * @param duration_ms 每次蜂鸣持续时间
 * @param interval_ms 蜂鸣间隔时间
 */
void Buzzer_Alarm(uint8_t beep_count, uint32_t duration_ms, uint32_t interval_ms);

/**
 * 蜂鸣器更新函数
 * 知识点: 蜂鸣器状态更新
 * - 周期性调用更新蜂鸣器状态
 * - 控制蜂鸣时序
 * - 处理报警模式
 */
void Buzzer_Update(void);

/**
 * 系统状态指示更新函数
 * 知识点: 系统状态指示更新
 * - 根据系统状态更新LED
 * - 触发相应报警
 * - 统一状态反馈
 */
void System_Status_Update(void);

/**
 * 系统错误设置函数
 * 知识点: 错误状态设置
 * @param error_type 错误类型
 * @param is_error 是否为错误状态
 */
void System_Set_Error(uint8_t error_type, bool is_error);

/**
 * 网络状态设置函数
 * 知识点: 网络状态设置
 * @param connected 是否连接
 * @param error 是否有错误
 */
void System_Set_Network_Status(bool connected, bool error);

/**
 * 数据状态设置函数
 * 知识点: 数据状态设置
 * @param has_error 是否有错误
 */
void System_Set_Data_Status(bool has_error);

/**
 * 系统状态指示获取函数
 * 知识点: 状态指示查询
 * @return System_Status_Indicator_t* 状态指示结构指针
 */
System_Status_Indicator_t* System_Get_Status_Indicator(void);

/**
 * GPIO错误处理函数
 * 知识点: GPIO错误处理
 * @param error_msg 错误信息
 */
void GPIO_Error_Handler(const char *error_msg);

/**
 * 延时函数声明
 * 知识点: 延时函数使用
 * - 在GPIO模块中使用延时进行防抖
 * - 需要在delay.c中实现
 */
extern void Delay_ms(uint32_t ms);
extern void Delay_us(uint32_t us);
extern uint32_t System_Get_Runtime_MS(void);

/* ============================== 调试函数声明 ============================== */

#ifdef DEBUG
/**
 * GPIO状态打印函数
 * 知识点: GPIO状态监控
 * - 打印LED状态
 * - 打印按键状态
 * - 打印蜂鸣器状态
 */
void GPIO_Print_Status(void);

/**
 * LED状态打印函数
 * 知识点: LED状态显示
 * @param led_id LED标识
 */
void LED_Print_Status(uint8_t led_id);

/**
 * 按键状态打印函数
 * 知识点: 按键状态显示
 */
void Key_Print_Status(void);

/**
 * 蜂鸣器状态打印函数
 * 知识点: 蜂鸣器状态显示
 */
void Buzzer_Print_Status(void);

/**
 * 系统状态指示打印函数
 * 知识点: 状态指示显示
 */
void System_Status_Indicator_Print(void);

/**
 * GPIO性能测试函数
 * 知识点: GPIO性能测试
 * - 测试LED闪烁
 * - 测试按键响应
 * - 测试蜂鸣器
 */
void GPIO_Performance_Test(void);

/**
 * LED闪烁测试函数
 * 知识点: LED功能测试
 * @param test_duration_ms 测试持续时间
 */
void LED_Blink_Test(uint32_t test_duration_ms);

/**
 * 按键测试函数
 * 知识点: 按键功能测试
 * @param test_duration_ms 测试持续时间
 */
void Key_Test(uint32_t test_duration_ms);

/**
 * 蜂鸣器测试函数
 * 知识点: 蜂鸣器功能测试
 * @param test_duration_ms 测试持续时间
 */
void Buzzer_Test(uint32_t test_duration_ms);
#endif

/* ============================== 状态机相关函数 ============================== */

/**
 * 按键状态机更新函数
 * 知识点: 按键状态机
 * - 更新按键状态机
 * - 处理状态转换
 * - 检测按键事件
 */
void Key_StateMachine_Update(void);

/**
 * LED状态机更新函数
 * 知识点: LED状态机
 * - 更新LED状态机
 * - 处理闪烁逻辑
 * - 控制引脚输出
 */
void LED_StateMachine_Update(LED_Control_t *led_control, GPIO_TypeDef* port, uint16_t pin);

/**
 * 蜂鸣器状态机更新函数
 * 知识点: 蜂鸣器状态机
 * - 更新蜂鸣器状态机
 * - 处理报警时序
 * - 控制蜂鸣输出
 */
void Buzzer_StateMachine_Update(void);

/* ============================== 工具函数声明 ============================== */

/**
 * GPIO引脚状态获取函数
 * 知识点: 引脚状态查询
 * @param port GPIO端口
 * @param pin 引脚号
 * @return bool 引脚电平状态
 */
bool GPIO_Get_Pin_State(GPIO_TypeDef* port, uint16_t pin);

/**
 * GPIO引脚设置函数
 * 知识点: 引脚状态设置
 * @param port GPIO端口
 * @param pin 引脚号
 * @param state 引脚状态
 */
void GPIO_Set_Pin_State(GPIO_TypeDef* port, uint16_t pin, BitAction state);

/**
 * 时间差计算函数
 * 知识点: 时间差计算
 * @param start_time 开始时间
 * @param current_time 当前时间
 * @return uint32_t 时间差(毫秒)
 */
uint32_t Calculate_Time_Diff(uint32_t start_time, uint32_t current_time);

/**
 * 防抖检测函数
 * 知识点: 防抖检测
 * @param current_state 当前状态
 * @param stable_time 稳定时间
 * @return bool 是否稳定
 */
bool Debounce_Detect(bool current_state, uint32_t stable_time);

/**
 * 边沿检测函数
 * 知识点: 边沿检测
 * @param current_state 当前状态
 * @param last_state 上次状态
 * @param edge_type 边沿类型
 * @return bool 检测结果
 */
bool Edge_Detect(bool current_state, bool last_state, uint8_t edge_type);

/**
 * 延时控制函数
 * 知识点: 延时控制
 * @param delay_ms 延时时间
 * @param last_delay_time 最后延时时间
 * @return bool 是否延时完成
 */
bool Delay_Control(uint32_t delay_ms, uint32_t *last_delay_time);

#endif /* __GPIO_H */
