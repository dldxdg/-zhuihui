/**
 * @file    flash.h
 * @brief   Flash存储管理头文件 (标准库版本)
 * @author  MiniMax Agent
 * @version V2.0
 * @date    2025-11-08
 * 
 * 知识点总结:
 * 
 * 【Flash存储器基础】
 * 1. Flash是掉电不丢失数据的存储器
 * 2. 分为内部Flash(芯片内)和外部Flash(通过SPI连接)
 * 3. 写入前必须先擦除
 * 4. 写入粒度比读取粒度大
 * 
 * 【内部Flash特点】
 * 1. STM32F103有64KB内部Flash
 * 2. 按页擦除，每页2KB
 * 3. 适合存储配置参数等小数据
 * 4. 写入速度较快
 * 
 * 【SPI Flash特点】
 * 1. W25Q64容量8MB
 * 2. 按页写入，每页256字节
 * 3. 按扇区擦除，每扇区4KB
 * 4. 适合存储大量传感器数据
 * 
 * 【数据管理策略】
 * 1. 内部Flash: WiFi配置、系统参数
 * 2. SPI Flash: 传感器数据环形缓冲区
 * 3. 读写指针管理数据位置
 * 4. 掉电保护机制
 */

#ifndef __FLASH_H
#define __FLASH_H

/* ============================== 包含头文件 ============================== */

#include "stm32f1xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ============================== 宏定义 ============================== */

/**
 * 内部Flash配置
 * 知识点: STM32F103内部Flash参数
 * - 总容量: 64KB (0x08000000 - 0x08010000)
 * - 页大小: 2KB
 * - 配置区: 使用最后1KB (0x08007C00 - 0x08007FFF)
 */
#define INTERNAL_FLASH_START_ADDR     0x08000000  // 内部Flash起始地址
#define INTERNAL_FLASH_END_ADDR       0x08010000  // 内部Flash结束地址
#define INTERNAL_FLASH_SIZE           0x10000     // 总大小64KB
#define INTERNAL_FLASH_PAGE_SIZE      0x800       // 页大小2KB
#define INTERNAL_FLASH_CONFIG_ADDR    0x08007C00  // 配置存储地址
#define INTERNAL_FLASH_CONFIG_SIZE    0x400       // 配置区大小1KB

/**
 * SPI Flash配置 (W25Q64)
 * 知识点: W25Q64芯片参数
 * - 容量: 8MB (8,388,608字节)
 * - 页大小: 256字节
 * - 扇区大小: 4KB (16页)
 * - 块大小: 64KB (256扇区)
 */
#define SPI_FLASH_ID_W25Q64           0xEF4017    // W25Q64设备ID
#define SPI_FLASH_TOTAL_SIZE          0x800000    // 总大小8MB
#define SPI_FLASH_PAGE_SIZE           0x100       // 页大小256字节
#define SPI_FLASH_SECTOR_SIZE         0x1000      // 扇区大小4KB
#define SPI_FLASH_BLOCK_SIZE          0x10000     // 块大小64KB
#define SPI_FLASH_DATA_START_ADDR     0x100000    // 数据区起始地址(1MB后)
#define SPI_FLASH_DATA_END_ADDR       0x800000    // 数据区结束地址(7MB)

/**
 * SPI通信配置
 * 知识点: SPI Flash通信参数
 * - 时钟频率: 18MHz (STM32最高25MHz)
 * - 工作模式: Mode 0 (CPOL=0, CPHA=0)
 * - 片选引脚: PB5
 */
#define SPI_FLASH_SPI                 SPI1
#define SPI_FLASH_CS_PIN             GPIO_Pin_5
#define SPI_FLASH_CS_PORT            GPIOB
#define SPI_FLASH_SPEED              SPI_BaudRatePrescaler_4  // 18MHz

/**
 * 内部Flash操作超时
 * 知识点: Flash操作时间参数
 * - 擦除时间: 约1-3秒
 * - 写入时间: 约20-50微秒/半字
 * - 总超时: 10秒安全时间
 */
#define INTERNAL_FLASH_TIMEOUT       10000       // 操作超时时间(毫秒)
#define INTERNAL_FLASH_PAGE_ERASE_TIMEOUT 3000  // 页擦除超时时间(毫秒)
#define INTERNAL_FLASH_PROGRAM_TIMEOUT 100     // 编程超时时间(毫秒)

/**
 * SPI Flash操作超时
 * 知识点: SPI Flash操作时间
 * - 擦除时间: 扇区400ms，块3s，全片25s
 * - 写入时间: 页256字节约0.8ms
 * - 读取时间: 非常快
 */
#define SPI_FLASH_SECTOR_ERASE_TIMEOUT  400     // 扇区擦除超时(毫秒)
#define SPI_FLASH_BLOCK_ERASE_TIMEOUT   3000    // 块擦除超时(毫秒)
#define SPI_FLASH_CHIP_ERASE_TIMEOUT    25000   // 全片擦除超时(毫秒)
#define SPI_FLASH_PAGE_PROGRAM_TIMEOUT  10      // 页编程超时(毫秒)

/* ============================== 数据结构定义 ============================== */

/**
 * 内部Flash配置结构
 * 知识点: 系统配置参数
 * - WiFi网络配置
 * - 服务器连接参数
 * - 系统运行参数
 * - 版本信息
 */
typedef struct {
    char wifi_ssid[32];             // WiFi网络名称
    char wifi_password[32];         // WiFi密码
    char server_ip[16];             // 服务器IP地址
    uint16_t server_port;           // 服务器端口号
    uint8_t sample_interval;        // 采集间隔(秒)
    uint8_t send_interval;          // 发送间隔(秒)
    uint8_t version_major;          // 版本主号
    uint8_t version_minor;          // 版本次号
    uint16_t version_patch;         // 版本补丁号
    uint32_t checksum;              // 配置校验和
} Internal_Flash_Config_t;

/**
 * SPI Flash元数据结构
 * 知识点: 环形缓冲区元数据
 * - 记录读写指针位置
 * - 记录数据统计信息
 * - 管理数据区域
 */
typedef struct {
    uint32_t write_pointer;         // 写指针(指向下一个写入位置)
    uint32_t read_pointer;          // 读指针(指向下一个读取位置)
    uint32_t data_start_addr;       // 数据区起始地址
    uint32_t data_end_addr;         // 数据区结束地址
    uint32_t total_records;         // 总记录数
    uint32_t valid_records;         // 有效记录数
    uint32_t corrupted_records;     // 损坏记录数
    uint32_t last_record_addr;      // 最后一条记录地址
    uint32_t reserved[4];           // 保留字段
} SPI_Flash_Meta_t;

/**
 * 传感器数据记录结构
 * 知识点: 传感器数据存储格式
 * - 简化的数据格式(无时间戳)
 * - 固定大小便于读写
 * - 包含校准信息
 */
typedef struct {
    float temperature;              // 温度值 (摄氏度)
    float light_level;              // 光线强度 (Lux)
    float gas_level;                // 气体浓度 (ppm)
    uint16_t data_flags;            // 数据标志位
    uint16_t checksum;              // 数据校验和
} Sensor_Data_Record_t;

/**
 * Flash操作结果枚举
 * 知识点: 操作结果定义
 * - 统一的返回值定义
 * - 便于错误处理
 */
typedef enum {
    FLASH_OP_SUCCESS = 0,           // 操作成功
    FLASH_OP_ERROR,                 // 操作错误
    FLASH_OP_TIMEOUT,               // 操作超时
    FLASH_OP_INVALID_ADDR,          // 无效地址
    FLASH_OP_NOT_ERASED,            // 地址未擦除
    FLASH_OP_VERIFY_FAILED,         // 验证失败
    FLASH_OP_NO_SPACE               // 空间不足
} Flash_Operation_Result_t;

/**
 * SPI Flash状态结构
 * 知识点: SPI Flash工作状态
 * - 芯片状态监控
 * - 错误统计
 * - 性能统计
 */
typedef struct {
    bool is_initialized;            // 初始化状态
    bool is_busy;                   // 忙碌状态
    uint32_t device_id;             // 设备ID
    uint32_t total_capacity;        // 总容量
    uint32_t error_count;           // 错误计数
    uint32_t read_count;            // 读取次数
    uint32_t write_count;           // 写入次数
    uint32_t erase_count;           // 擦除次数
} SPI_Flash_Status_t;

/**
 * 内部Flash状态结构
 * 知识点: 内部Flash工作状态
 * - 配置区状态
 * - 错误统计
 * - 访问统计
 */
typedef struct {
    bool is_initialized;            // 初始化状态
    bool config_valid;              // 配置有效性
    uint32_t error_count;           // 错误计数
    uint32_t read_count;            // 读取次数
    uint32_t write_count;           // 写入次数
    uint32_t erase_count;           // 擦除次数
} Internal_Flash_Status_t;

/* ============================== 全局变量声明 ============================== */

// SPI Flash状态
extern SPI_Flash_Status_t SPI_Flash_Status;

// 内部Flash状态
extern Internal_Flash_Status_t Internal_Flash_Status;

// SPI Flash元数据
extern SPI_Flash_Meta_t SPI_Flash_Meta;

/* ============================== 函数声明 ============================== */

/**
 * Flash模块初始化函数
 * 知识点: Flash模块初始化
 * 1. 初始化内部Flash
 * 2. 初始化SPI Flash
 * 3. 加载配置参数
 * 4. 验证存储完整性
 */
void Flash_Module_Init(void);

/**
 * 内部Flash初始化函数
 * 知识点: 内部Flash初始化
 * - 启用Flash访问
 * - 设置等待周期
 * - 初始化状态结构
 */
void Internal_Flash_Init(void);

/**
 * SPI Flash初始化函数
 * 知识点: SPI Flash初始化
 * 1. 配置SPI接口
 * 2. 检测芯片型号
 * 3. 复位芯片
 * 4. 初始化元数据结构
 */
void SPI_Flash_Init(void);

/**
 * SPI接口配置函数
 * 知识点: SPI配置
 * - 配置SPI1为Flash通信
 * - 设置时钟分频
 * - 配置GPIO引脚
 */
void SPI_Flash_Config(void);

/**
 * 内部Flash读取函数
 * 知识点: 内部Flash读取
 * - 读取配置参数
 * - 验证数据完整性
 * @param addr 读取地址
 * @param data 数据缓冲区
 * @param size 读取大小
 * @return Flash_Operation_Result_t 操作结果
 */
Flash_Operation_Result_t Internal_Flash_Read(uint32_t addr, uint8_t *data, uint32_t size);

/**
 * 内部Flash写入函数
 * 知识点: 内部Flash写入
 * - 擦除目标页
 * - 写入数据
 * - 验证写入结果
 * @param addr 写入地址
 * @param data 数据缓冲区
 * @param size 写入大小
 * @return Flash_Operation_Result_t 操作结果
 */
Flash_Operation_Result_t Internal_Flash_Write(uint32_t addr, const uint8_t *data, uint32_t size);

/**
 * 内部Flash页擦除函数
 * 知识点: 内部Flash擦除
 * - 擦除指定页
 * - 等待擦除完成
 * - 验证擦除结果
 * @param page_addr 页地址
 * @return Flash_Operation_Result_t 操作结果
 */
Flash_Operation_Result_t Internal_Flash_Erase_Page(uint32_t page_addr);

/**
 * 内部Flash扇区擦除函数
 * 知识点: 内部Flash扇区擦除
 * - 擦除指定扇区
 * - STM32F103以页为单位
 * @param sector_addr 扇区地址
 * @return Flash_Operation_Result_t 操作结果
 */
Flash_Operation_Result_t Internal_Flash_Erase_Sector(uint32_t sector_addr);

/**
 * SPI Flash芯片检测函数
 * 知识点: Flash芯片检测
 * - 读取设备ID
 * - 验证芯片型号
 * - 获取容量信息
 * @return bool 检测结果
 */
bool SPI_Flash_Detect(void);

/**
 * SPI Flash复位函数
 * 知识点: Flash芯片复位
 * - 发送复位命令
 * - 等待芯片就绪
 */
void SPI_Flash_Reset(void);

/**
 * SPI Flash擦除使能函数
 * 知识点: 擦除使能
 * - 发送写使能命令
 * - 等待命令执行完成
 */
void SPI_Flash_Write_Enable(void);

/**
 * SPI Flash等待忙函数
 * 知识点: 忙状态等待
 * - 读取状态寄存器
 * - 等待芯片空闲
 * - 超时保护
 * @param timeout_ms 超时时间
 * @return bool 等待结果
 */
bool SPI_Flash_Wait_Busy(uint32_t timeout_ms);

/**
 * SPI Flash页编程函数
 * 知识点: 页编程
 * - 发送页编程命令
 * - 写入256字节数据
 * - 等待编程完成
 * @param addr 写入地址
 * @param data 数据缓冲区
 * @param size 写入大小(<=256)
 * @return Flash_Operation_Result_t 操作结果
 */
Flash_Operation_Result_t SPI_Flash_Page_Program(uint32_t addr, const uint8_t *data, uint16_t size);

/**
 * SPI Flash扇区擦除函数
 * 知识点: 扇区擦除
 * - 擦除4KB扇区
 * - 等待擦除完成
 * @param addr 扇区起始地址
 * @return Flash_Operation_Result_t 操作结果
 */
Flash_Operation_Result_t SPI_Flash_Sector_Erase(uint32_t addr);

/**
 * SPI Flash块擦除函数
 * 知识点: 块擦除
 * - 擦除64KB块
 * - 等待擦除完成
 * @param addr 块起始地址
 * @return Flash_Operation_Result_t 操作结果
 */
Flash_Operation_Result_t SPI_Flash_Block_Erase(uint32_t addr);

/**
 * SPI Flash芯片擦除函数
 * 知识点: 全片擦除
 * - 擦除整个芯片
 * - 等待擦除完成
 * @return Flash_Operation_Result_t 操作结果
 */
Flash_Operation_Result_t SPI_Flash_Chip_Erase(void);

/**
 * SPI Flash读取函数
 * 知识点: 数据读取
 * - 读取指定地址数据
 * - 支持任意长度读取
 * - 快速读取模式
 * @param addr 读取地址
 * @param data 数据缓冲区
 * @param size 读取大小
 * @return Flash_Operation_Result_t 操作结果
 */
Flash_Operation_Result_t SPI_Flash_Read(uint32_t addr, uint8_t *data, uint32_t size);

/**
 * SPI Flash快速读取函数
 * 知识点: 快速读取
 * - 使用快速读取命令
 * - 减少读取时间
 * @param addr 读取地址
 * @param data 数据缓冲区
 * @param size 读取大小
 * @return Flash_Operation_Result_t 操作结果
 */
Flash_Operation_Result_t SPI_Flash_Fast_Read(uint32_t addr, uint8_t *data, uint32_t size);

/**
 * SPI Flash写数据函数
 * 知识点: 写数据(处理跨页)
 * - 自动处理跨页写入
 * - 智能分页写入
 * - 地址对齐处理
 * @param addr 写入地址
 * @param data 数据缓冲区
 * @param size 写入大小
 * @return Flash_Operation_Result_t 操作结果
 */
Flash_Operation_Result_t SPI_Flash_Write_Data(uint32_t addr, const uint8_t *data, uint32_t size);

/**
 * 配置读取函数
 * 知识点: 系统配置读取
 * - 从内部Flash读取配置
 * - 验证校验和
 * - 加载默认配置(如果读取失败)
 * @param config 配置结构指针
 * @return bool 读取结果
 */
bool Config_Read(Internal_Flash_Config_t *config);

/**
 * 配置写入函数
 * 知识点: 系统配置写入
 * - 计算校验和
 * - 写入内部Flash
 * - 验证写入结果
 * @param config 配置结构指针
 * @return bool 写入结果
 */
bool Config_Write(const Internal_Flash_Config_t *config);

/**
 * 配置验证函数
 * 知识点: 配置验证
 * - 验证配置结构完整性
 * - 检查参数合理性
 * - 计算和验证校验和
 * @param config 配置结构指针
 * @return bool 验证结果
 */
bool Config_Verify(const Internal_Flash_Config_t *config);

/**
 * 元数据初始化函数
 * 知识点: 环形缓冲区元数据初始化
 * - 初始化读写指针
 * - 设置数据区域
 * - 统计信息清零
 */
void SPI_Flash_Meta_Init(void);

/**
 * 元数据读取函数
 * 知识点: 元数据读取
 * - 从指定位置读取元数据
 * - 验证元数据完整性
 * @return bool 读取结果
 */
bool SPI_Flash_Meta_Read(void);

/**
 * 元数据写入函数
 * 知识点: 元数据写入
 * - 写入元数据到指定位置
 * - 更新统计信息
 * @return bool 写入结果
 */
bool SPI_Flash_Meta_Write(void);

/**
 * 传感器数据写入函数
 * 知识点: 传感器数据写入环形缓冲区
 * - 写入单条传感器记录
 * - 更新写指针
 * - 处理缓冲区满的情况
 * @param record 传感器数据记录
 * @return bool 写入结果
 */
bool SPI_Flash_Write_Sensor_Record(const Sensor_Data_Record_t *record);

/**
 * 传感器数据读取函数
 * 知识点: 传感器数据读取
 * - 从环形缓冲区读取单条记录
 * - 更新读指针
 * - 数据校验
 * @param record 传感器数据记录
 * @return bool 读取结果
 */
bool SPI_Flash_Read_Sensor_Record(Sensor_Data_Record_t *record);

/**
 * 缓冲区状态查询函数
 * 知识点: 环形缓冲区状态
 * - 查询可用数据量
 * - 查询剩余空间
 * - 查询读写指针
 * @param available_data 返回可用数据量
 * @param free_space 返回剩余空间
 * @return bool 查询结果
 */
bool SPI_Flash_Buffer_Status(uint32_t *available_data, uint32_t *free_space);

/**
 * 缓冲区清空函数
 * 知识点: 缓冲区重置
 * - 重置读写指针
 * - 清空统计信息
 * - 保留元数据
 */
void SPI_Flash_Buffer_Clear(void);

/**
 * 数据格式化为文本函数
 * 知识点: 数据格式化
 * - 将传感器数据格式化为文本
 * - 格式: "TEMP:25.6 LIGHT:456 GAS:789\r\n"
 * @param record 传感器数据记录
 * @param buffer 格式化缓冲区
 * @param buffer_size 缓冲区大小
 * @return bool 格式化结果
 */
bool SPI_Flash_Format_Data_Text(const Sensor_Data_Record_t *record, char *buffer, uint16_t buffer_size);

/* ============================== 调试函数声明 ============================== */

#ifdef DEBUG
/**
 * Flash状态打印函数
 * 知识点: Flash状态监控
 * - 打印内部Flash状态
 * - 打印SPI Flash状态
 * - 打印错误统计
 */
void Flash_Print_Status(void);

/**
 * SPI Flash状态打印函数
 * 知识点: SPI Flash详细状态
 * - 打印芯片信息
 * - 打印操作统计
 * - 打印错误信息
 */
void SPI_Flash_Print_Status(void);

/**
 * 内部Flash状态打印函数
 * 知识点: 内部Flash详细状态
 * - 打印配置状态
 * - 打印操作统计
 * - 打印错误信息
 */
void Internal_Flash_Print_Status(void);

/**
 * 配置信息打印函数
 * 知识点: 配置信息显示
 * - 打印当前配置
 * - 打印校验和信息
 * - 便于调试和验证
 */
void Config_Print_Info(void);

/**
 * 环形缓冲区状态打印函数
 * 知识点: 缓冲区状态显示
 * - 打印读写指针
 * - 打印数据统计
 * - 打印利用率
 */
void SPI_Flash_Buffer_Print_Status(void);

/**
 * 传感器数据打印函数
 * 知识点: 传感器数据验证
 * - 打印原始数据
 * - 打印格式化数据
 * - 打印校验结果
 * @param record 传感器数据记录
 */
void Sensor_Record_Print_Data(const Sensor_Data_Record_t *record);

/**
 * Flash性能测试函数
 * 知识点: Flash性能测试
 * - 测试读写速度
 * - 测试擦除时间
 * - 测试稳定性
 */
void Flash_Performance_Test(void);
#endif

/* ============================== 辅助函数声明 ============================== */

/**
 * 校验和计算函数
 * 知识点: 数据校验和
 * - 计算32位校验和
 * - 用于数据完整性验证
 * @param data 数据指针
 * @param size 数据大小
 * @return uint32_t 校验和
 */
uint32_t Calculate_Checksum(const uint8_t *data, uint32_t size);

/**
 * 地址对齐检查函数
 * 知识点: 地址对齐
 * - 检查地址是否页对齐
 * - 页写入需要256字节对齐
 * @param addr 地址
 * @return bool 对齐结果
 */
bool Is_Address_Aligned(uint32_t addr, uint32_t alignment);

/**
 * 地址有效性检查函数
 * 知识点: 地址验证
 * - 检查地址是否在有效范围内
 * - 防止越界访问
 * @param addr 地址
 * @param size 大小
 * @param max_addr 最大地址
 * @return bool 有效性结果
 */
bool Is_Address_Valid(uint32_t addr, uint32_t size, uint32_t max_addr);

/**
 * 16位校验和计算函数
 * 知识点: 16位校验和
 * - 计算16位校验和
 * - 用于传感器数据校验
 * @param data 数据指针
 * @param size 数据大小
 * @return uint16_t 校验和
 */
uint16_t Calculate_Data_Checksum(const uint8_t *data, uint16_t size);

/**
 * 数据验证函数
 * 知识点: 数据完整性验证
 * - 验证传感器数据记录
 * - 检查校验和
 * - 检查数据范围
 * @param record 数据记录
 * @return bool 验证结果
 */
bool Verify_Sensor_Record(const Sensor_Data_Record_t *record);

/**
 * 文本解析函数
 * 知识点: 文本数据解析
 * - 解析"KEY:VALUE"格式文本
 * - 提取传感器数据
 * @param text 文本字符串
 * @param record 数据记录
 * @return bool 解析结果
 */
bool Parse_Data_Text(const char *text, Sensor_Data_Record_t *record);

#endif /* __FLASH_H */
