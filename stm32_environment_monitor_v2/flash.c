/**
 * @file    flash.c
 * @brief   Flash存储管理实现 (标准库版本)
 * @author  MiniMax Agent
 * @version V2.0
 * @date    2025-11-08
 * 
 * 重要说明:
 * - 此版本完全使用STM32标准库，无HAL函数
 * - 内部Flash用于系统配置存储
 * - SPI Flash用于传感器数据环形缓冲区
 * - 完善的数据完整性和错误处理
 * 
 * 知识点总结:
 * 
 * 【Flash存储器特性】
 * 1. Flash写入前必须先擦除
 * 2. 擦除后所有位变为1
 * 3. 写入操作只能将1改为0
 * 4. 擦除粒度比写入粒度大
 * 
 * 【内部Flash操作】
 * 1. 按页擦除(2KB)
 * 2. 按半字写入(16位)
 * 3. 需要等待操作完成
 * 4. 有写保护机制
 * 
 * 【SPI Flash操作】
 * 1. 支持页编程(256字节)
 * 2. 支持扇区擦除(4KB)
 * 3. 支持块擦除(64KB)
 * 4. 有状态寄存器监控操作
 */

#include "flash.h"
#include "main.h"
#include "delay.h"

/* ============================== 全局变量定义 ============================== */

/**
 * SPI Flash状态
 * 知识点: SPI Flash工作状态
 * - 记录芯片初始化状态
 * - 统计操作次数和错误
 * - 监控设备信息
 */
SPI_Flash_Status_t SPI_Flash_Status = {
    .is_initialized = false,
    .is_busy = false,
    .device_id = 0,
    .total_capacity = 0,
    .error_count = 0,
    .read_count = 0,
    .write_count = 0,
    .erase_count = 0
};

/**
 * 内部Flash状态
 * 知识点: 内部Flash工作状态
 * - 记录配置存储状态
 * - 统计操作次数和错误
 * - 监控配置有效性
 */
Internal_Flash_Status_t Internal_Flash_Status = {
    .is_initialized = false,
    .config_valid = false,
    .error_count = 0,
    .read_count = 0,
    .write_count = 0,
    .erase_count = 0
};

/**
 * SPI Flash元数据
 * 知识点: 环形缓冲区元数据
 * - 管理读写指针
 * - 记录数据统计
 * - 使用固定元数据区
 */
SPI_Flash_Meta_t SPI_Flash_Meta = {
    .write_pointer = 0,
    .read_pointer = 0,
    .data_start_addr = 0,
    .data_end_addr = 0,
    .total_records = 0,
    .valid_records = 0,
    .corrupted_records = 0,
    .last_record_addr = 0,
    .reserved = {0}
};

/* ============================== Flash模块初始化 ============================== */

/**
 * @brief   Flash模块初始化函数
 * @details 初始化所有Flash相关功能
 * 
 * 知识点: Flash模块初始化顺序
 * 1. 初始化内部Flash
 * 2. 初始化SPI Flash
 * 3. 加载系统配置
 * 4. 初始化环形缓冲区
 */
void Flash_Module_Init(void)
{
    DEBUG_PRINT("Initializing Flash module...\r\n");
    
    /**
     * 步骤1: 初始化内部Flash
     * 知识点: 内部Flash初始化
     * - 配置Flash访问参数
     * - 初始化状态结构
     * - 设置等待周期
     */
    Internal_Flash_Init();
    
    /**
     * 步骤2: 初始化SPI Flash
     * 知识点: SPI Flash初始化
     * - 配置SPI接口
     * - 检测芯片型号
     * - 初始化环形缓冲区
     */
    SPI_Flash_Init();
    
    /**
     * 步骤3: 加载系统配置
     * 知识点: 系统配置加载
     * - 从内部Flash读取配置
     * - 验证配置完整性
     * - 加载默认值(如果失败)
     */
    Internal_Flash_Config_t config;
    if (Config_Read(&config)) {
        DEBUG_PRINT("System configuration loaded successfully\r\n");
    } else {
        DEBUG_PRINT("Failed to load configuration, using defaults\r\n");
        // 使用默认配置
        strcpy(config.wifi_ssid, "MyWiFi");
        strcpy(config.wifi_password, "password123");
        strcpy(config.server_ip, "192.168.1.100");
        config.server_port = 8080;
        config.sample_interval = 5;
        config.send_interval = 10;
        config.version_major = 2;
        config.version_minor = 0;
        config.version_patch = 0;
        config.checksum = 0;
        Config_Write(&config);  // 保存默认配置
    }
    
    /**
     * 步骤4: 初始化环形缓冲区元数据
     * 知识点: 环形缓冲区初始化
     * - 设置数据区域
     * - 初始化读写指针
     * - 重置统计数据
     */
    SPI_Flash_Meta_Init();
    
    DEBUG_PRINT("Flash module initialized successfully\r\n");
}

/**
 * @brief   内部Flash初始化函数
 * @details 初始化内部Flash和配置存储
 * 
 * 知识点: 内部Flash初始化
 * 1. 启用Flash访问
 * 2. 设置访问等待周期
 * 3. 初始化状态结构
 * 4. 验证配置区域
 */
void Internal_Flash_Init(void)
{
    /**
     * 启用Flash访问
     * 知识点: Flash预取缓冲
     * - 启用预取缓冲提高访问速度
     * - 降低CPU等待时间
     * - 只读访问优化
     */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    
    /**
     * 设置Flash等待周期
     * 知识点: Flash访问时序
     * - 根据系统时钟设置等待周期
     * - 72MHz需要2个等待周期
     * - 确保Flash访问稳定性
     */
    FLASH_SetLatency(FLASH_Latency_2);
    
    /**
     * 初始化状态结构
     * 知识点: 状态初始化
     * - 设置为已初始化
     * - 统计信息清零
     */
    Internal_Flash_Status.is_initialized = true;
    Internal_Flash_Status.config_valid = false;
    
    /**
     * 验证配置区域
     * 知识点: 区域验证
     * - 检查配置地址是否有效
     * - 测试读写访问
     */
    if (INTERNAL_FLASH_CONFIG_ADDR + INTERNAL_FLASH_CONFIG_SIZE <= INTERNAL_FLASH_END_ADDR) {
        DEBUG_PRINT("Internal Flash configured (config at 0x%08X)\r\n", INTERNAL_FLASH_CONFIG_ADDR);
    } else {
        DEBUG_PRINT("ERROR: Invalid config address range\r\n");
        ERROR_HANDLER();
    }
}

/**
 * @brief   SPI Flash初始化函数
 * @details 初始化SPI Flash和环形缓冲区
 * 
 * 知识点: SPI Flash初始化流程
 * 1. 配置SPI接口
 * 2. 检测芯片型号
 * 3. 复位芯片
 * 4. 初始化环形缓冲区元数据
 */
void SPI_Flash_Init(void)
{
    /**
     * 步骤1: 配置SPI接口
     * 知识点: SPI配置
     * - 配置SPI1为Flash通信
     * - 设置合适的时钟频率
     * - 配置GPIO引脚
     */
    SPI_Flash_Config();
    
    /**
     * 步骤2: 检测芯片
     * 知识点: 芯片检测
     * - 读取设备ID
     * - 验证芯片型号
     * - 获取容量信息
     */
    if (!SPI_Flash_Detect()) {
        DEBUG_PRINT("ERROR: SPI Flash detection failed\r\n");
        SPI_Flash_Status.is_initialized = false;
        return;
    }
    
    /**
     * 步骤3: 复位芯片
     * 知识点: 芯片复位
     * - 确保芯片处于已知状态
     * - 清除挂起状态
     */
    SPI_Flash_Reset();
    
    /**
     * 步骤4: 初始化环形缓冲区
     * 知识点: 缓冲区初始化
     * - 设置元数据
     * - 初始化读写指针
     */
    SPI_Flash_Meta.data_start_addr = SPI_FLASH_DATA_START_ADDR;
    SPI_Flash_Meta.data_end_addr = SPI_FLASH_DATA_END_ADDR;
    
    // 尝试读取已存在的元数据
    if (!SPI_Flash_Meta_Read()) {
        // 如果读取失败，重新初始化
        SPI_Flash_Meta_Init();
    }
    
    /**
     * 步骤5: 更新状态
     * 知识点: 状态更新
     * - 设置为已初始化
     * - 记录设备信息
     */
    SPI_Flash_Status.is_initialized = true;
    SPI_Flash_Status.device_id = SPI_FLASH_ID_W25Q64;
    SPI_Flash_Status.total_capacity = SPI_FLASH_TOTAL_SIZE;
    
    DEBUG_PRINT("SPI Flash initialized (ID: 0x%06X, Capacity: %lu MB)\r\n", 
               SPI_Flash_Status.device_id, SPI_FLASH_TOTAL_SIZE / 1024 / 1024);
}

/**
 * @brief   SPI接口配置函数
 * @details 配置SPI1用于Flash通信
 * 
 * 知识点: SPI配置详解
 * 1. 启用SPI和GPIO时钟
 * 2. 配置SPI为Master模式
 * 3. 设置时钟分频
 * 4. 配置GPIO为复用推挽
 */
void SPI_Flash_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;
    
    /**
     * 启用时钟
     * 知识点: 时钟使能
     * - SPI1: 挂载在APB2总线
     * - GPIOA: SPI引脚在GPIOA
     * - GPIOB: CS引脚在GPIOB
     */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    
    /**
     * 配置SPI引脚
     * 知识点: SPI引脚配置
     * - PA5: SPI1_SCK (时钟线)
     * - PA6: SPI1_MISO (主输入从输出)
     * - PA7: SPI1_MOSI (主输出从输入)
     * - PB5: SPI1_CS (片选信号，推挽输出)
     */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = SPI_FLASH_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_FLASH_CS_PORT, &GPIO_InitStructure);
    
    /**
     * 配置SPI参数
     * 知识点: SPI工作模式
     * - 模式: 主设备
     * - 方向: 全双工
     * - 数据大小: 8位
     * - 时钟极性: 低电平(空闲时SCK=0)
     * - 时钟相位: 第一边沿(上升沿采样)
     * - NSS: 软NSS
     * - 预分频: 4 (18MHz)
     */
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_FLASH_SPEED;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    
    SPI_Init(SPI_FLASH_SPI, &SPI_InitStructure);
    SPI_Cmd(SPI_FLASH_SPI, ENABLE);
    
    /**
     * 初始化CS引脚
     * 知识点: 片选信号
     * - 初始状态为高电平(未选中)
     * - 避免意外选中Flash
     */
    GPIO_SetBits(SPI_FLASH_CS_PORT, SPI_FLASH_CS_PIN);
    
    DEBUG_PRINT("SPI Flash interface configured (SPI1, 18MHz, Mode 0)\r\n");
}

/* ============================== 内部Flash操作函数 ============================== */

/**
 * @brief   内部Flash读取函数
 * @details 从内部Flash读取配置数据
 * @param addr 读取地址
 * @param data 数据缓冲区
 * @param size 读取大小
 * @return Flash_Operation_Result_t 操作结果
 * 
 * 知识点: 内部Flash读取
 * - 内部Flash读取很直接
 * - 按字节读取
 * - 不需要特殊命令
 */
Flash_Operation_Result_t Internal_Flash_Read(uint32_t addr, uint8_t *data, uint32_t size)
{
    if (data == NULL || size == 0) {
        return FLASH_OP_ERROR;
    }
    
    /**
     * 地址范围检查
     * 知识点: 地址验证
     * - 确保地址在有效范围内
     * - 防止越界访问
     */
    if (addr < INTERNAL_FLASH_START_ADDR || 
        addr + size > INTERNAL_FLASH_END_ADDR) {
        DEBUG_PRINT("Internal Flash read: invalid address range (0x%08X, size: %lu)\r\n", addr, size);
        return FLASH_OP_INVALID_ADDR;
    }
    
    /**
     * 执行读取操作
     * 知识点: 内存映射读取
     * - 内部Flash可直接通过指针访问
     * - 读取速度很快
     */
    const uint8_t *flash_data = (const uint8_t*)addr;
    memcpy(data, flash_data, size);
    
    /**
     * 更新统计信息
     * 知识点: 统计更新
     * - 增加读取计数
     */
    Internal_Flash_Status.read_count++;
    
    return FLASH_OP_SUCCESS;
}

/**
 * @brief   内部Flash写入函数
 * @details 写入数据到内部Flash
 * @param addr 写入地址
 * @param data 数据缓冲区
 * @param size 写入大小
 * @return Flash_Operation_Result_t 操作结果
 * 
 * 知识点: 内部Flash写入流程
 * 1. 解锁Flash
 * 2. 擦除目标页
 * 3. 写入数据
 * 4. 锁定Flash
 * 5. 验证写入结果
 */
Flash_Operation_Result_t Internal_Flash_Write(uint32_t addr, const uint8_t *data, uint32_t size)
{
    if (data == NULL || size == 0) {
        return FLASH_OP_ERROR;
    }
    
    /**
     * 地址检查
     * 知识点: 地址验证
     * - 页对齐检查
     * - 范围验证
     */
    if (addr < INTERNAL_FLASH_CONFIG_ADDR || 
        addr + size > INTERNAL_FLASH_CONFIG_ADDR + INTERNAL_FLASH_CONFIG_SIZE) {
        DEBUG_PRINT("Internal Flash write: invalid address range (0x%08X, size: %lu)\r\n", addr, size);
        return FLASH_OP_INVALID_ADDR;
    }
    
    /**
     * 擦除目标页
     * 知识点: 页擦除
     * - 内部Flash写入前必须擦除
     * - 按页擦除(2KB)
     */
    uint32_t page_addr = addr & ~(INTERNAL_FLASH_PAGE_SIZE - 1);  // 页地址对齐
    Flash_Operation_Result_t result = Internal_Flash_Erase_Page(page_addr);
    if (result != FLASH_OP_SUCCESS) {
        return result;
    }
    
    /**
     * 解锁Flash
     * 知识点: Flash解锁
     * - 默认Flash处于写保护状态
     * - 需要解锁才能写入
     */
    FLASH_Unlock();
    
    /**
     * 写入数据
     * 知识点: 按半字写入
     * - 内部Flash按16位写入
     * - 每次写入2字节
     * - 需要等待写入完成
     */
    uint32_t remaining = size;
    uint32_t current_addr = addr;
    
    for (uint32_t i = 0; i < size; i += 2) {
        uint16_t half_word;
        if (i + 1 < size) {
            half_word = (data[i+1] << 8) | data[i];
        } else {
            half_word = data[i];  // 最后一个字节
        }
        
        if (FLASH_ProgramHalfWord(current_addr, half_word) != FLASH_COMPLETE) {
            DEBUG_PRINT("Internal Flash write failed at address 0x%08X\r\n", current_addr);
            FLASH_Lock();
            return FLASH_OP_ERROR;
        }
        
        current_addr += 2;
        remaining -= 2;
        
        /**
         * 超时检查
         * 知识点: 写入超时保护
         * - 每个半字写入都需要时间
         * - 防止无限等待
         */
        if (i % 100 == 0) {  // 每100个半字检查一次
            Delay_us(1);
        }
    }
    
    /**
     * 锁定Flash
     * 知识点: Flash保护
     * - 写操作完成后立即锁定
     * - 防止意外写入
     */
    FLASH_Lock();
    
    /**
     * 验证写入结果
     * 知识点: 写入验证
     * - 读取刚写入的数据
     * - 与原始数据比较
     */
    uint8_t verify_data[256];  // 验证缓冲区
    if (Internal_Flash_Read(addr, verify_data, size) == FLASH_OP_SUCCESS) {
        if (memcmp(data, verify_data, size) == 0) {
            Internal_Flash_Status.write_count++;
            return FLASH_OP_SUCCESS;
        } else {
            DEBUG_PRINT("Internal Flash write verification failed\r\n");
            return FLASH_OP_VERIFY_FAILED;
        }
    } else {
        return FLASH_OP_ERROR;
    }
}

/**
 * @brief   内部Flash页擦除函数
 * @details 擦除指定Flash页
 * @param page_addr 页地址
 * @return Flash_Operation_Result_t 操作结果
 * 
 * 知识点: 页擦除过程
 * 1. 检查页地址
 * 2. 解锁Flash
 * 3. 执行擦除
 * 4. 等待完成
 * 5. 验证结果
 */
Flash_Operation_Result_t Internal_Flash_Erase_Page(uint32_t page_addr)
{
    /**
     * 页地址检查
     * 知识点: 页地址验证
     * - 必须页对齐
     * - 必须在配置区域内
     */
    if (page_addr & (INTERNAL_FLASH_PAGE_SIZE - 1)) {
        DEBUG_PRINT("Internal Flash erase: address not page aligned (0x%08X)\r\n", page_addr);
        return FLASH_OP_INVALID_ADDR;
    }
    
    if (page_addr < INTERNAL_FLASH_CONFIG_ADDR || 
        page_addr >= INTERNAL_FLASH_CONFIG_ADDR + INTERNAL_FLASH_CONFIG_SIZE) {
        DEBUG_PRINT("Internal Flash erase: address out of range (0x%08X)\r\n", page_addr);
        return FLASH_OP_INVALID_ADDR;
    }
    
    /**
     * 解锁Flash
     * 知识点: 解锁操作
     * - 擦除前必须解锁
     */
    FLASH_Unlock();
    
    /**
     * 擦除页
     * 知识点: 页擦除执行
     * - 擦除整个页
     * - 擦除后所有字节变为0xFF
     */
    if (FLASH_ErasePage(page_addr) != FLASH_COMPLETE) {
        DEBUG_PRINT("Internal Flash page erase failed at 0x%08X\r\n", page_addr);
        FLASH_Lock();
        return FLASH_OP_ERROR;
    }
    
    /**
     * 等待擦除完成
     * 知识点: 擦除等待
     * - 擦除操作需要时间
     * - 轮询等待完成标志
     */
    uint32_t timeout = System_Get_Runtime_MS() + INTERNAL_FLASH_PAGE_ERASE_TIMEOUT;
    while (FLASH_GetFlagStatus(FLASH_FLAG_BSY) == SET) {
        if (System_Get_Runtime_MS() > timeout) {
            DEBUG_PRINT("Internal Flash page erase timeout\r\n");
            FLASH_Lock();
            return FLASH_OP_TIMEOUT;
        }
        Delay_ms(10);
    }
    
    /**
     * 锁定Flash
     * 知识点: 保护操作
     * - 擦除完成后锁定
     */
    FLASH_Lock();
    
    /**
     * 验证擦除结果
     * 知识点: 擦除验证
     * - 读取擦除区域
     * - 检查是否全为0xFF
     */
    for (uint32_t i = 0; i < INTERNAL_FLASH_PAGE_SIZE; i++) {
        if (*(volatile uint8_t*)(page_addr + i) != 0xFF) {
            DEBUG_PRINT("Internal Flash page erase verification failed\r\n");
            return FLASH_OP_VERIFY_FAILED;
        }
    }
    
    Internal_Flash_Status.erase_count++;
    return FLASH_OP_SUCCESS;
}

/* ============================== SPI Flash操作函数 ============================== */

/**
 * @brief   SPI Flash芯片检测函数
 * @details 检测和识别SPI Flash芯片
 * @return bool 检测结果
 * 
 * 知识点: Flash芯片检测
 * 1. 发送读ID命令
 * 2. 接收设备ID
 * 3. 验证芯片型号
 * 4. 获取容量信息
 */
bool SPI_Flash_Detect(void)
{
    /**
     * 发送读ID命令
     * 知识点: 读ID命令格式
     * - 命令码: 0x90
     * - 地址: 0x000000
     * - Dummy: 1字节
     * - 响应: 2字节(厂商ID+设备ID)
     */
    SPI_Flash_Select();
    
    SPI_Send_Byte(0x90);  // 读ID命令
    SPI_Send_Byte(0x00);  // 地址高字节
    SPI_Send_Byte(0x00);  // 地址中字节
    SPI_Send_Byte(0x00);  // 地址低字节
    
    uint8_t manufacturer_id = SPI_Receive_Byte();
    uint8_t device_id = SPI_Receive_Byte();
    
    SPI_Flash_Deselect();
    
    /**
     * 验证设备ID
     * 知识点: 设备验证
     * - 厂商ID应该为0xEF(Winbond)
     * - 设备ID应该为0x17(W25Q64)
     */
    uint32_t full_id = (manufacturer_id << 8) | device_id;
    
    DEBUG_PRINT("SPI Flash ID: 0x%06X (Manufacturer: 0x%02X, Device: 0x%02X)\r\n", 
               full_id, manufacturer_id, device_id);
    
    if (full_id == SPI_FLASH_ID_W25Q64) {
        return true;
    } else {
        DEBUG_PRINT("ERROR: Unexpected SPI Flash ID (expected: 0x%06X)\r\n", SPI_FLASH_ID_W25Q64);
        return false;
    }
}

/**
 * @brief   SPI Flash复位函数
 * @details 复位SPI Flash芯片
 * 
 * 知识点: Flash复位
 * 1. 发送复位命令
 * 2. 等待芯片就绪
 * 3. 清除挂起状态
 */
void SPI_Flash_Reset(void)
{
    /**
     * 发送复位使能命令
     * 知识点: 复位使能
     * - 命令码: 0x66
     * - 准备执行复位
     */
    SPI_Flash_Select();
    SPI_Send_Byte(0x66);  // 复位使能
    SPI_Flash_Deselect();
    
    /**
     * 发送复位命令
     * 知识点: 复位执行
     * - 命令码: 0x99
     * - 执行芯片复位
     */
    SPI_Flash_Select();
    SPI_Send_Byte(0x99);  // 复位
    SPI_Flash_Deselect();
    
    /**
     * 等待芯片就绪
     * 知识点: 就绪等待
     * - 复位后需要时间初始化
     * - 等待状态寄存器显示就绪
     */
    Delay_ms(10);
    SPI_Flash_Wait_Busy(1000);
    
    DEBUG_PRINT("SPI Flash reset completed\r\n");
}

/**
 * @brief   SPI Flash写使能函数
 * @details 启用写操作
 * 
 * 知识点: 写使能
 * 1. 发送写使能命令
 * 2. 等待命令执行
 * 3. 验证写使能状态
 */
void SPI_Flash_Write_Enable(void)
{
    /**
     * 发送写使能命令
     * 知识点: 写使能命令
     * - 命令码: 0x06
     * - 临时启用写操作
     * - 每次写操作前都需要
     */
    SPI_Flash_Select();
    SPI_Send_Byte(0x06);  // 写使能
    SPI_Flash_Deselect();
    
    /**
     * 等待命令执行
     * 知识点: 命令完成等待
     * - 确保命令执行完成
     */
    Delay_us(1);
    
    /**
     * 验证写使能状态
     * 知识点: 状态验证
     * - 读取状态寄存器
     * - 检查WEL位(Write Enable Latch)
     */
    uint8_t status = SPI_Flash_Read_Status_Register();
    if (!(status & 0x02)) {  // WEL位
        DEBUG_PRINT("WARNING: Write enable failed, status: 0x%02X\r\n", status);
    }
}

/**
 * @brief   SPI Flash等待忙函数
 * @details 等待Flash操作完成
 * @param timeout_ms 超时时间
 * @return bool 等待结果
 * 
 * 知识点: 忙状态检测
 * 1. 读取状态寄存器
 * 2. 检查BUSY位
 * 3. 轮询等待
 * 4. 超时保护
 */
bool SPI_Flash_Wait_Busy(uint32_t timeout_ms)
{
    uint32_t start_time = System_Get_Runtime_MS();
    
    while (true) {
        /**
         * 读取状态寄存器
         * 知识点: 状态寄存器
         * - 命令码: 0x05
         * - 响应: 1字节状态
         * - BIT0: BUSY (忙碌状态)
         */
        uint8_t status = SPI_Flash_Read_Status_Register();
        
        if (!(status & 0x01)) {  // BUSY位为0，表示空闲
            return true;
        }
        
        /**
         * 超时检查
         * 知识点: 超时保护
         * - 防止无限等待
         * - 返回失败状态
         */
        if (System_Get_Runtime_MS() - start_time > timeout_ms) {
            DEBUG_PRINT("SPI Flash wait busy timeout (%lu ms)\r\n", timeout_ms);
            return false;
        }
        
        Delay_us(100);  // 避免过于频繁的查询
    }
}

/**
 * @brief   SPI Flash页编程函数
 * @details 在指定页地址写入数据
 * @param addr 写入地址
 * @param data 数据缓冲区
 * @param size 写入大小(<=256)
 * @return Flash_Operation_Result_t 操作结果
 * 
 * 知识点: 页编程流程
 * 1. 检查地址对齐
 * 2. 启用写操作
 * 3. 发送页编程命令
 * 4. 发送地址和数据
 * 5. 等待编程完成
 * 6. 验证写入结果
 */
Flash_Operation_Result_t SPI_Flash_Page_Program(uint32_t addr, const uint8_t *data, uint16_t size)
{
    /**
     * 参数检查
     * 知识点: 参数验证
     * - 数据指针不能为空
     * - 大小不能超过页大小
     * - 地址必须页对齐
     */
    if (data == NULL || size == 0 || size > SPI_FLASH_PAGE_SIZE) {
        return FLASH_OP_ERROR;
    }
    
    if (addr & (SPI_FLASH_PAGE_SIZE - 1)) {
        DEBUG_PRINT("SPI Flash page program: address not page aligned (0x%08X)\r\n", addr);
        return FLASH_OP_INVALID_ADDR;
    }
    
    /**
     * 启用写操作
     * 知识点: 写使能
     * - 页编程前必须启用写操作
     */
    SPI_Flash_Write_Enable();
    
    /**
     * 发送页编程命令
     * 知识点: 命令格式
     * - 命令码: 0x02
     * - 3字节地址
     * - 数据字节
     */
    SPI_Flash_Select();
    SPI_Send_Byte(0x02);                    // 页编程命令
    
    /**
     * 发送地址
     * 知识点: 地址格式
     * - 24位地址(高8位、中8位、低8位)
     * - 按大端序发送
     */
    SPI_Send_Byte((addr >> 16) & 0xFF);     // 地址高字节
    SPI_Send_Byte((addr >> 8) & 0xFF);      // 地址中字节
    SPI_Send_Byte(addr & 0xFF);             // 地址低字节
    
    /**
     * 发送数据
     * 知识点: 数据发送
     * - 串行发送所有数据字节
     * - 数据量不能超过页大小
     */
    for (uint16_t i = 0; i < size; i++) {
        SPI_Send_Byte(data[i]);
    }
    
    SPI_Flash_Deselect();
    
    /**
     * 等待编程完成
     * 知识点: 编程等待
     * - 页编程需要时间
     * - 典型时间约0.8ms
     */
    if (!SPI_Flash_Wait_Busy(SPI_FLASH_PAGE_PROGRAM_TIMEOUT)) {
        return FLASH_OP_TIMEOUT;
    }
    
    /**
     * 验证写入结果
     * 知识点: 写入验证
     * - 读取刚写入的数据
     * - 与原数据比较
     */
    uint8_t verify_data[256];
    if (SPI_Flash_Read(addr, verify_data, size) == FLASH_OP_SUCCESS) {
        if (memcmp(data, verify_data, size) == 0) {
            SPI_Flash_Status.write_count++;
            return FLASH_OP_SUCCESS;
        } else {
            DEBUG_PRINT("SPI Flash page program verification failed\r\n");
            return FLASH_OP_VERIFY_FAILED;
        }
    } else {
        return FLASH_OP_ERROR;
    }
}

/**
 * @brief   SPI Flash扇区擦除函数
 * @details 擦除4KB扇区
 * @param addr 扇区起始地址
 * @return Flash_Operation_Result_t 操作结果
 * 
 * 知识点: 扇区擦除
 * 1. 擦除最小单位
 * 2. 4KB扇区
 * 3. 适合小量数据修改
 */
Flash_Operation_Result_t SPI_Flash_Sector_Erase(uint32_t addr)
{
    /**
     * 扇区地址检查
     * 知识点: 扇区地址验证
     * - 必须4KB对齐
     * - 地址必须在数据区域内
     */
    if (addr & (SPI_FLASH_SECTOR_SIZE - 1)) {
        DEBUG_PRINT("SPI Flash sector erase: address not sector aligned (0x%08X)\r\n", addr);
        return FLASH_OP_INVALID_ADDR;
    }
    
    if (addr < SPI_FLASH_DATA_START_ADDR || 
        addr >= SPI_FLASH_DATA_END_ADDR) {
        DEBUG_PRINT("SPI Flash sector erase: address out of data range (0x%08X)\r\n", addr);
        return FLASH_OP_INVALID_ADDR;
    }
    
    /**
     * 启用写操作
     * 知识点: 擦除使能
     * - 擦除前必须启用写操作
     */
    SPI_Flash_Write_Enable();
    
    /**
     * 发送扇区擦除命令
     * 知识点: 擦除命令格式
     * - 命令码: 0x20
     * - 3字节地址
     * - 擦除4KB扇区
     */
    SPI_Flash_Select();
    SPI_Send_Byte(0x20);                    // 扇区擦除命令
    SPI_Send_Byte((addr >> 16) & 0xFF);     // 地址高字节
    SPI_Send_Byte((addr >> 8) & 0xFF);      // 地址中字节
    SPI_Send_Byte(addr & 0xFF);             // 地址低字节
    SPI_Flash_Deselect();
    
    /**
     * 等待擦除完成
     * 知识点: 擦除等待
     * - 扇区擦除时间约400ms
     * - 有超时保护
     */
    if (!SPI_Flash_Wait_Busy(SPI_FLASH_SECTOR_ERASE_TIMEOUT)) {
        return FLASH_OP_TIMEOUT;
    }
    
    SPI_Flash_Status.erase_count++;
    return FLASH_OP_SUCCESS;
}

/**
 * @brief   SPI Flash数据写入函数
 * @details 智能写入处理跨页数据
 * @param addr 写入地址
 * @param data 数据缓冲区
 * @param size 写入大小
 * @return Flash_Operation_Result_t 操作结果
 * 
 * 知识点: 智能写入
 * 1. 自动处理跨页写入
 * 2. 页对齐优化
 * 3. 地址边界处理
 * 4. 连续写入优化
 */
Flash_Operation_Result_t SPI_Flash_Write_Data(uint32_t addr, const uint8_t *data, uint32_t size)
{
    if (data == NULL || size == 0) {
        return FLASH_OP_ERROR;
    }
    
    /**
     * 地址范围检查
     * 知识点: 数据区验证
     * - 确保在数据区域内
     */
    if (addr < SPI_FLASH_DATA_START_ADDR || 
        addr + size > SPI_FLASH_DATA_END_ADDR) {
        DEBUG_PRINT("SPI Flash write: address out of data range (0x%08X, size: %lu)\r\n", addr, size);
        return FLASH_OP_INVALID_ADDR;
    }
    
    uint32_t written = 0;
    uint32_t current_addr = addr;
    const uint8_t *current_data = data;
    
    while (written < size) {
        /**
         * 计算当前页的剩余空间
         * 知识点: 页边界计算
         * - 避免跨页写入
         * - 页内写入优化
         */
        uint32_t page_offset = current_addr & (SPI_FLASH_PAGE_SIZE - 1);
        uint32_t page_remaining = SPI_FLASH_PAGE_SIZE - page_offset;
        uint32_t write_size = size - written;
        
        if (write_size > page_remaining) {
            write_size = page_remaining;
        }
        
        /**
         * 页编程
         * 知识点: 页级写入
         * - 不超过页大小
         * - 边界对齐
         */
        Flash_Operation_Result_t result = SPI_Flash_Page_Program(current_addr, current_data, write_size);
        if (result != FLASH_OP_SUCCESS) {
            return result;
        }
        
        /**
         * 更新地址和数据指针
         * 知识点: 地址推进
         * - 移动到下一页
         * - 移动数据指针
         */
        current_addr += write_size;
        current_data += write_size;
        written += write_size;
    }
    
    return FLASH_OP_SUCCESS;
}

/* ============================== 配置管理函数 ============================== */

/**
 * @brief   配置读取函数
 * @details 从内部Flash读取系统配置
 * @param config 配置结构指针
 * @return bool 读取结果
 * 
 * 知识点: 配置读取流程
 * 1. 读取配置数据
 * 2. 验证校验和
 * 3. 验证参数合理性
 * 4. 加载默认值(如果失败)
 */
bool Config_Read(Internal_Flash_Config_t *config)
{
    if (config == NULL) {
        return false;
    }
    
    /**
     * 读取配置数据
     * 知识点: 配置数据读取
     * - 从固定地址读取
     * - 读取完整配置结构
     */
    if (Internal_Flash_Read(INTERNAL_FLASH_CONFIG_ADDR, (uint8_t*)config, sizeof(Internal_Flash_Config_t)) 
        != FLASH_OP_SUCCESS) {
        DEBUG_PRINT("Config read: failed to read flash data\r\n");
        return false;
    }
    
    /**
     * 验证校验和
     * 知识点: 数据完整性验证
     * - 临时清除校验和字段
     * - 计算实际校验和
     * - 与存储的校验和比较
     */
    uint32_t saved_checksum = config->checksum;
    config->checksum = 0;
    uint32_t calculated_checksum = Calculate_Checksum((uint8_t*)config, sizeof(Internal_Flash_Config_t));
    
    if (saved_checksum != calculated_checksum) {
        DEBUG_PRINT("Config read: checksum mismatch (saved: 0x%08X, calculated: 0x%08X)\r\n", 
                   saved_checksum, calculated_checksum);
        return false;
    }
    
    /**
     * 恢复校验和字段
     * 知识点: 字段恢复
     * - 恢复原始校验和
     */
    config->checksum = saved_checksum;
    
    /**
     * 参数合理性验证
     * 知识点: 参数验证
     * - 检查IP地址格式
     * - 检查端口号范围
     * - 检查SSID长度
     */
    if (strlen(config->server_ip) == 0 || 
        config->server_port == 0 || 
        strlen(config->wifi_ssid) == 0 ||
        config->sample_interval == 0 ||
        config->send_interval == 0) {
        DEBUG_PRINT("Config read: invalid parameter values\r\n");
        return false;
    }
    
    /**
     * 更新状态
     * 知识点: 状态更新
     * - 标记配置有效
     */
    Internal_Flash_Status.config_valid = true;
    
    return true;
}

/**
 * @brief   配置写入函数
 * @details 将系统配置写入内部Flash
 * @param config 配置结构指针
 * @return bool 写入结果
 * 
 * 知识点: 配置写入流程
 * 1. 计算校验和
 * 2. 写入内部Flash
 * 3. 验证写入结果
 * 4. 更新状态
 */
bool Config_Write(const Internal_Flash_Config_t *config)
{
    if (config == NULL) {
        return false;
    }
    
    /**
     * 临时修改配置
     * 知识点: 校验和计算
     * - 复制配置到临时变量
     * - 计算校验和
     * - 更新校验和字段
     */
    Internal_Flash_Config_t temp_config = *config;
    temp_config.checksum = 0;
    uint32_t checksum = Calculate_Checksum((uint8_t*)&temp_config, sizeof(Internal_Flash_Config_t));
    temp_config.checksum = checksum;
    
    /**
     * 写入内部Flash
     * 知识点: 配置写入
     * - 写入完整配置结构
     * - 包含校验和
     */
    if (Internal_Flash_Write(INTERNAL_FLASH_CONFIG_ADDR, (uint8_t*)&temp_config, sizeof(Internal_Flash_Config_t)) 
        != FLASH_OP_SUCCESS) {
        DEBUG_PRINT("Config write: failed to write flash data\r\n");
        return false;
    }
    
    /**
     * 验证写入结果
     * 知识点: 写入验证
     * - 重新读取配置
     * - 验证校验和
     */
    Internal_Flash_Config_t verify_config;
    if (!Config_Read(&verify_config)) {
        DEBUG_PRINT("Config write: verification failed\r\n");
        return false;
    }
    
    /**
     * 更新状态
     * 知识点: 状态更新
     * - 标记配置有效
     */
    Internal_Flash_Status.config_valid = true;
    
    return true;
}

/* ============================== 环形缓冲区管理函数 ============================== */

/**
 * @brief   元数据初始化函数
 * @details 初始化环形缓冲区元数据
 * 
 * 知识点: 环形缓冲区初始化
 * 1. 设置数据区域
 * 2. 初始化读写指针
 * 3. 重置统计信息
 * 4. 保存初始元数据
 */
void SPI_Flash_Meta_Init(void)
{
    /**
     * 设置数据区域
     * 知识点: 数据区域配置
     * - 起始地址: 1MB后
     * - 结束地址: 7MB处
     * - 预留前1MB给其他用途
     */
    SPI_Flash_Meta.data_start_addr = SPI_FLASH_DATA_START_ADDR;
    SPI_Flash_Meta.data_end_addr = SPI_FLASH_DATA_END_ADDR;
    
    /**
     * 初始化读写指针
     * 知识点: 指针初始化
     * - 读指针: 起始位置
     * - 写指针: 起始位置
     * - 空缓冲区状态
     */
    SPI_Flash_Meta.read_pointer = SPI_FLASH_DATA_START_ADDR;
    SPI_Flash_Meta.write_pointer = SPI_FLASH_DATA_START_ADDR;
    
    /**
     * 重置统计信息
     * 知识点: 统计重置
     * - 总记录数: 0
     * - 有效记录数: 0
     * - 损坏记录数: 0
     * - 最后记录地址: 0
     */
    SPI_Flash_Meta.total_records = 0;
    SPI_Flash_Meta.valid_records = 0;
    SPI_Flash_Meta.corrupted_records = 0;
    SPI_Flash_Meta.last_record_addr = 0;
    
    /**
     * 保存元数据
     * 知识点: 元数据存储
     * - 存储到固定位置
     * - 为环形缓冲区操作做准备
     */
    SPI_Flash_Meta_Write();
    
    DEBUG_PRINT("SPI Flash meta initialized (data range: 0x%08X-0x%08X)\r\n", 
               SPI_FLASH_Meta.data_start_addr, SPI_FLASH_Meta.data_end_addr);
}

/**
 * @brief   元数据读取函数
 * @details 从指定位置读取元数据
 * @return bool 读取结果
 * 
 * 知识点: 元数据读取
 * 1. 从固定位置读取元数据
 * 2. 验证元数据完整性
 * 3. 检查数据区域合理性
 */
bool SPI_Flash_Meta_Read(void)
{
    /**
     * 读取元数据
     * 知识点: 元数据读取
     * - 从固定地址读取
     * - 读取完整元数据结构
     */
    uint8_t meta_data[sizeof(SPI_Flash_Meta_t)];
    if (SPI_Flash_Read(0, meta_data, sizeof(SPI_Flash_Meta_t)) != FLASH_OP_SUCCESS) {
        return false;
    }
    
    /**
     * 复制到结构体
     * 知识点: 数据复制
     * - 字节方式复制
     * - 保持数据结构一致
     */
    memcpy(&SPI_Flash_Meta, meta_data, sizeof(SPI_Flash_Meta_t));
    
    /**
     * 验证元数据合理性
     * 知识点: 合理性验证
     * - 检查地址范围
     * - 检查指针关系
     */
    if (SPI_Flash_Meta.data_start_addr < SPI_FLASH_DATA_START_ADDR ||
        SPI_Flash_Meta.data_end_addr > SPI_FLASH_DATA_END_ADDR ||
        SPI_Flash_Meta.write_pointer < SPI_Flash_Meta.data_start_addr ||
        SPI_Flash_Meta.write_pointer > SPI_Flash_Meta.data_end_addr) {
        DEBUG_PRINT("SPI Flash meta: invalid data in meta structure\r\n");
        return false;
    }
    
    return true;
}

/**
 * @brief   元数据写入函数
 * @details 将元数据写入指定位置
 * @return bool 写入结果
 * 
 * 知识点: 元数据写入
 * 1. 擦除元数据区域
 * 2. 写入元数据
 * 3. 验证写入结果
 */
bool SPI_Flash_Meta_Write(void)
{
    /**
     * 擦除元数据区域
     * 知识点: 区域擦除
     * - 擦除第0扇区(0x000000-0x001000)
     * - 确保元数据区域干净
     */
    if (SPI_Flash_Sector_Erase(0) != FLASH_OP_SUCCESS) {
        DEBUG_PRINT("SPI Flash meta write: failed to erase meta sector\r\n");
        return false;
    }
    
    /**
     * 写入元数据
     * 知识点: 元数据写入
     * - 写入完整元数据结构
     * - 页编程方式
     */
    if (SPI_Flash_Page_Program(0, (uint8_t*)&SPI_Flash_Meta, sizeof(SPI_Flash_Meta_t)) != FLASH_OP_SUCCESS) {
        DEBUG_PRINT("SPI Flash meta write: failed to write meta data\r\n");
        return false;
    }
    
    return true;
}

/* ============================== 传感器数据操作函数 ============================== */

/**
 * @brief   传感器数据写入函数
 * @details 写入传感器数据到环形缓冲区
 * @param record 传感器数据记录
 * @return bool 写入结果
 * 
 * 知识点: 传感器数据写入
 * 1. 计算数据校验和
 * 2. 更新写指针
 * 3. 写入数据
 * 4. 处理缓冲区满的情况
 * 5. 更新统计信息
 */
bool SPI_Flash_Write_Sensor_Record(const Sensor_Data_Record_t *record)
{
    if (record == NULL) {
        return false;
    }
    
    /**
     * 计算数据校验和
     * 知识点: 数据校验
     * - 计算16位校验和
     * - 用于数据完整性验证
     */
    Sensor_Data_Record_t temp_record = *record;
    temp_record.checksum = 0;
    temp_record.checksum = Calculate_Data_Checksum((uint8_t*)&temp_record, sizeof(Sensor_Data_Record_t) - 2);
    
    /**
     * 检查写指针是否需要回绕
     * 知识点: 环形缓冲区写指针管理
     * - 写指针到达数据区末尾时回绕
     * - 保持读写指针不重叠
     */
    uint32_t next_write_addr = SPI_Flash_Meta.write_pointer;
    uint32_t record_size = sizeof(Sensor_Data_Record_t);
    
    if (next_write_addr + record_size > SPI_Flash_Meta.data_end_addr) {
        // 写指针到达末尾，回绕到起始位置
        next_write_addr = SPI_Flash_Meta.data_start_addr;
        
        /**
         * 检查读指针是否追赶上来
         * 知识点: 缓冲区满检测
         * - 如果读指针已经接近写指针，说明缓冲区满
         * - 需要先移动读指针释放空间
         */
        if (SPI_Flash_Meta.read_pointer > next_write_addr && 
            (SPI_Flash_Meta.read_pointer - next_write_addr) < record_size) {
            // 读指针在写指针前面，且距离很近，缓冲区接近满
            DEBUG_PRINT("SPI Flash buffer nearly full, advancing read pointer\r\n");
            SPI_Flash_Meta.read_pointer = next_write_addr + record_size;
        }
    }
    
    /**
     * 写入数据记录
     * 知识点: 数据写入
     * - 写入单条记录
     * - 包含校验和
     */
    if (SPI_Flash_Write_Data(next_write_addr, (uint8_t*)&temp_record, record_size) != FLASH_OP_SUCCESS) {
        DEBUG_PRINT("Failed to write sensor record\r\n");
        return false;
    }
    
    /**
     * 更新元数据
     * 知识点: 元数据更新
     * - 更新写指针
     * - 更新统计信息
     * - 更新最后记录地址
     */
    SPI_Flash_Meta.write_pointer = next_write_addr + record_size;
    SPI_Flash_Meta.total_records++;
    SPI_Flash_Meta.valid_records++;
    SPI_Flash_Meta.last_record_addr = next_write_addr;
    
    /**
     * 保存元数据
     * 知识点: 元数据持久化
     * - 写入元数据到存储区域
     * - 确保环形缓冲区状态持久化
     */
    if (!SPI_Flash_Meta_Write()) {
        DEBUG_PRINT("Failed to save meta data after record write\r\n");
        return false;
    }
    
    return true;
}

/**
 * @brief   传感器数据读取函数
 * @details 从环形缓冲区读取传感器数据
 * @param record 传感器数据记录
 * @return bool 读取结果
 * 
 * 知识点: 传感器数据读取
 * 1. 检查读指针是否到达写指针
 * 2. 读取数据记录
 * 3. 验证数据校验和
 * 4. 更新读指针
 */
bool SPI_Flash_Read_Sensor_Record(Sensor_Data_Record_t *record)
{
    if (record == NULL) {
        return false;
    }
    
    /**
     * 检查缓冲区是否有数据
     * 知识点: 缓冲区空检测
     * - 读指针等于写指针时缓冲区空
     * - 不能读取
     */
    if (SPI_Flash_Meta.read_pointer >= SPI_Flash_Meta.write_pointer) {
        return false;  // 缓冲区空
    }
    
    /**
     * 读取数据记录
     * 知识点: 数据读取
     * - 从当前位置读取
     * - 读取完整记录
     */
    uint32_t record_size = sizeof(Sensor_Data_Record_t);
    if (SPI_Flash_Read(SPI_Flash_Meta.read_pointer, (uint8_t*)record, record_size) != FLASH_OP_SUCCESS) {
        DEBUG_PRINT("Failed to read sensor record\r\n");
        return false;
    }
    
    /**
     * 验证数据完整性
     * 知识点: 数据验证
     * - 验证校验和
     * - 验证数据范围
     */
    if (!Verify_Sensor_Record(record)) {
        DEBUG_PRINT("Sensor record verification failed\r\n");
        SPI_Flash_Meta.corrupted_records++;
        SPI_Flash_Meta.valid_records--;
        return false;
    }
    
    /**
     * 更新读指针
     * 知识点: 读指针推进
     * - 移动到下一条记录
     * - 环形回绕处理
     */
    SPI_Flash_Meta.read_pointer += record_size;
    if (SPI_Flash_Meta.read_pointer >= SPI_Flash_Meta.data_end_addr) {
        SPI_Flash_Meta.read_pointer = SPI_FLASH_DATA_START_ADDR;
    }
    
    /**
     * 保存元数据
     * 知识点: 状态持久化
     * - 更新读指针
     * - 保存元数据
     */
    SPI_Flash_Meta_Write();
    
    return true;
}

/**
 * @brief   缓冲区状态查询函数
 * @details 查询环形缓冲区状态
 * @param available_data 返回可用数据量
 * @param free_space 返回剩余空间
 * @return bool 查询结果
 * 
 * 知识点: 缓冲区状态分析
 * 1. 计算可用数据量
 * 2. 计算剩余空间
 * 3. 分析缓冲区利用率
 */
bool SPI_Flash_Buffer_Status(uint32_t *available_data, uint32_t *free_space)
{
    if (available_data == NULL || free_space == NULL) {
        return false;
    }
    
    /**
     * 计算可用数据量
     * 知识点: 可用数据计算
     * - 写指针位置减去读指针位置
     * - 表示未读取的数据量
     */
    if (SPI_Flash_Meta.write_pointer >= SPI_Flash_Meta.read_pointer) {
        *available_data = SPI_Flash_Meta.write_pointer - SPI_Flash_Meta.read_pointer;
    } else {
        *available_data = (SPI_FLASH_DATA_END_ADDR - SPI_Flash_Meta.read_pointer) + 
                         (SPI_Flash_Meta.write_pointer - SPI_FLASH_DATA_START_ADDR);
    }
    
    /**
     * 计算剩余空间
     * 知识点: 剩余空间计算
     * - 数据区总大小减去已使用空间
     * - 可写入数据的空间
     */
    uint32_t data_range = SPI_FLASH_DATA_END_ADDR - SPI_FLASH_DATA_START_ADDR;
    *free_space = data_range - *available_data;
    
    return true;
}

/**
 * @brief   缓冲区清空函数
 * @details 清空环形缓冲区
 * 
 * 知识点: 缓冲区重置
 * 1. 重置读写指针
 * 2. 重置统计信息
 * 3. 保留元数据结构
 */
void SPI_Flash_Buffer_Clear(void)
{
    /**
     * 重置指针
     * 知识点: 指针重置
     * - 读指针回到起始位置
     * - 写指针回到起始位置
     */
    SPI_Flash_Meta.read_pointer = SPI_FLASH_DATA_START_ADDR;
    SPI_Flash_Meta.write_pointer = SPI_FLASH_DATA_START_ADDR;
    
    /**
     * 重置统计信息
     * 知识点: 统计重置
     * - 清空所有计数器
     */
    SPI_Flash_Meta.total_records = 0;
    SPI_Flash_Meta.valid_records = 0;
    SPI_Flash_Meta.corrupted_records = 0;
    SPI_Flash_Meta.last_record_addr = 0;
    
    /**
     * 保存更新后的元数据
     * 知识点: 状态保存
     * - 保存重置后的状态
     */
    SPI_Flash_Meta_Write();
    
    DEBUG_PRINT("SPI Flash buffer cleared\r\n");
}

/* ============================== 调试函数实现 ============================== */

#ifdef DEBUG
/**
 * @brief   Flash状态打印函数
 * @details 打印所有Flash状态信息
 */
void Flash_Print_Status(void)
{
    printf("\r\nFlash Module Status:\r\n");
    
    /**
     * 打印内部Flash状态
     * 知识点: 内部Flash状态显示
     */
    printf("  Internal Flash:\r\n");
    printf("    Initialized: %s\r\n", Internal_Flash_Status.is_initialized ? "Yes" : "No");
    printf("    Config Valid: %s\r\n", Internal_Flash_Status.config_valid ? "Yes" : "No");
    printf("    Config Address: 0x%08X\r\n", INTERNAL_FLASH_CONFIG_ADDR);
    printf("    Read Count: %lu\r\n", Internal_Flash_Status.read_count);
    printf("    Write Count: %lu\r\n", Internal_Flash_Status.write_count);
    printf("    Erase Count: %lu\r\n", Internal_Flash_Status.erase_count);
    printf("    Error Count: %lu\r\n", Internal_Flash_Status.error_count);
    
    /**
     * 打印SPI Flash状态
     * 知识点: SPI Flash状态显示
     */
    printf("  SPI Flash:\r\n");
    printf("    Initialized: %s\r\n", SPI_Flash_Status.is_initialized ? "Yes" : "No");
    printf("    Device ID: 0x%06X\r\n", SPI_Flash_Status.device_id);
    printf("    Total Capacity: %lu MB\r\n", SPI_Flash_Status.total_capacity / 1024 / 1024);
    printf("    Read Count: %lu\r\n", SPI_Flash_Status.read_count);
    printf("    Write Count: %lu\r\n", SPI_Flash_Status.write_count);
    printf("    Erase Count: %lu\r\n", SPI_Flash_Status.erase_count);
    printf("    Error Count: %lu\r\n", SPI_Flash_Status.error_count);
    
    /**
     * 打印环形缓冲区状态
     * 知识点: 缓冲区状态显示
     */
    uint32_t available_data, free_space;
    SPI_Flash_Buffer_Status(&available_data, &free_space);
    uint32_t total_space = SPI_FLASH_DATA_END_ADDR - SPI_FLASH_DATA_START_ADDR;
    float usage_percent = (float)available_data / total_space * 100.0f;
    
    printf("  Ring Buffer:\r\n");
    printf("    Data Range: 0x%08X - 0x%08X\r\n", 
           SPI_Flash_Meta.data_start_addr, SPI_Flash_Meta.data_end_addr);
    printf("    Read Pointer: 0x%08X\r\n", SPI_Flash_Meta.read_pointer);
    printf("    Write Pointer: 0x%08X\r\n", SPI_Flash_Meta.write_pointer);
    printf("    Available Data: %lu bytes (%.1f%%)\r\n", available_data, usage_percent);
    printf("    Free Space: %lu bytes\r\n", free_space);
    printf("    Total Records: %lu\r\n", SPI_Flash_Meta.total_records);
    printf("    Valid Records: %lu\r\n", SPI_Flash_Meta.valid_records);
    printf("    Corrupted Records: %lu\r\n", SPI_Flash_Meta.corrupted_records);
}

/**
 * @brief   传感器数据打印函数
 * @details 打印传感器数据记录
 * @param record 传感器数据记录
 */
void Sensor_Record_Print_Data(const Sensor_Data_Record_t *record)
{
    if (record == NULL) {
        return;
    }
    
    printf("\r\nSensor Data Record:\r\n");
    
    /**
     * 打印原始数据
     * 知识点: 原始数据显示
     */
    printf("  Raw Data:\r\n");
    printf("    Temperature: %.1f°C\r\n", record->temperature);
    printf("    Light Level: %.0fLux\r\n", record->light_level);
    printf("    Gas Level: %.0fppm\r\n", record->gas_level);
    printf("    Data Flags: 0x%04X\r\n", record->data_flags);
    printf("    Checksum: 0x%04X\r\n", record->checksum);
    
    /**
     * 打印格式化数据
     * 知识点: 格式化数据显示
     */
    char formatted_buffer[64];
    if (SPI_Flash_Format_Data_Text(record, formatted_buffer, sizeof(formatted_buffer))) {
        printf("  Formatted: %s", formatted_buffer);
    }
    
    /**
     * 打印验证结果
     * 知识点: 验证结果显示
     */
    bool is_valid = Verify_Sensor_Record(record);
    printf("  Verification: %s\r\n", is_valid ? "Valid" : "Invalid");
}

/**
 * @brief   配置信息打印函数
 * @details 打印当前系统配置
 */
void Config_Print_Info(void)
{
    printf("\r\nSystem Configuration:\r\n");
    
    /**
     * 读取当前配置
     * 知识点: 配置读取
     */
    Internal_Flash_Config_t config;
    if (Config_Read(&config)) {
        printf("  WiFi Network:\r\n");
        printf("    SSID: %s\r\n", config.wifi_ssid);
        printf("    Password: %s\r\n", "***");  // 隐藏密码
        
        printf("  Server:\r\n");
        printf("    IP Address: %s\r\n", config.server_ip);
        printf("    Port: %u\r\n", config.server_port);
        
        printf("  Operation:\r\n");
        printf("    Sample Interval: %u seconds\r\n", config.sample_interval);
        printf("    Send Interval: %u seconds\r\n", config.send_interval);
        
        printf("  Version:\r\n");
        printf("    Version: %u.%u.%u\r\n", 
               config.version_major, config.version_minor, config.version_patch);
        printf("    Checksum: 0x%08X\r\n", config.checksum);
    } else {
        printf("  Failed to read configuration\r\n");
    }
}
#endif

/* ============================== 辅助函数实现 ============================== */

/**
 * @brief   校验和计算函数
 * @details 计算32位校验和
 * @param data 数据指针
 * @param size 数据大小
 * @return uint32_t 校验和
 * 
 * 知识点: 校验和算法
 * - 简单的32位加法校验和
 * - 用于数据完整性验证
 */
uint32_t Calculate_Checksum(const uint8_t *data, uint32_t size)
{
    if (data == NULL || size == 0) {
        return 0;
    }
    
    uint32_t checksum = 0;
    for (uint32_t i = 0; i < size; i++) {
        checksum += data[i];
    }
    
    return checksum;
}

/**
 * @brief   16位校验和计算函数
 * @details 计算16位校验和
 * @param data 数据指针
 * @param size 数据大小
 * @return uint16_t 校验和
 * 
 * 知识点: 16位校验和
 * - 用于传感器数据记录
 * - 16位足够验证小数据块
 */
uint16_t Calculate_Data_Checksum(const uint8_t *data, uint16_t size)
{
    if (data == NULL || size == 0) {
        return 0;
    }
    
    uint16_t checksum = 0;
    for (uint16_t i = 0; i < size; i++) {
        checksum += data[i];
    }
    
    return checksum;
}

/**
 * @brief   数据验证函数
 * @details 验证传感器数据记录
 * @param record 数据记录
 * @return bool 验证结果
 * 
 * 知识点: 数据记录验证
 * 1. 验证校验和
 * 2. 验证数据范围
 * 3. 验证标志位
 */
bool Verify_Sensor_Record(const Sensor_Data_Record_t *record)
{
    if (record == NULL) {
        return false;
    }
    
    /**
     * 校验和验证
     * 知识点: 校验和检查
     * - 重新计算校验和
     * - 与存储的校验和比较
     */
    uint16_t calculated_checksum = 0;
    calculated_checksum = Calculate_Data_Checksum((uint8_t*)record, sizeof(Sensor_Data_Record_t) - 2);
    
    if (calculated_checksum != record->checksum) {
        return false;
    }
    
    /**
     * 数据范围验证
     * 知识点: 范围检查
     * - 温度范围: -40~125°C
     * - 光线范围: 0~1000Lux
     * - 气体范围: 10~1000ppm
     */
    if (record->temperature < -40.0f || record->temperature > 125.0f) {
        return false;
    }
    
    if (record->light_level < 0.0f || record->light_level > 1000.0f) {
        return false;
    }
    
    if (record->gas_level < 10.0f || record->gas_level > 1000.0f) {
        return false;
    }
    
    return true;
}

/* ============================== SPI底层操作函数 ============================== */

/**
 * @brief   SPI发送字节函数
 * @details 通过SPI发送一个字节
 * @param data 要发送的数据
 * 
 * 知识点: SPI发送
 * 1. 等待发送缓冲区空
 * 2. 发送数据
 * 3. 等待发送完成
 */
void SPI_Send_Byte(uint8_t data)
{
    while (SPI_I2S_GetFlagStatus(SPI_FLASH_SPI, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI_FLASH_SPI, data);
    while (SPI_I2S_GetFlagStatus(SPI_FLASH_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    SPI_I2S_ReceiveData(SPI_FLASH_SPI);  // 清除RXNE标志
}

/**
 * @brief   SPI接收字节函数
 * @details 通过SPI接收一个字节
 * @return uint8_t 接收到的数据
 * 
 * 知识点: SPI接收
 * 1. 发送空数据启动接收
 * 2. 等待接收完成
 * 3. 返回接收数据
 */
uint8_t SPI_Receive_Byte(void)
{
    while (SPI_I2S_GetFlagStatus(SPI_FLASH_SPI, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI_FLASH_SPI, 0xFF);  // 发送空数据启动接收
    while (SPI_I2S_GetFlagStatus(SPI_FLASH_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    return SPI_I2S_ReceiveData(SPI_FLASH_SPI);
}

/**
 * @brief   SPI Flash片选函数
 * @details 选择SPI Flash芯片
 */
void SPI_Flash_Select(void)
{
    GPIO_ResetBits(SPI_FLASH_CS_PORT, SPI_FLASH_CS_PIN);
    SPI_Flash_Status.is_busy = true;
}

/**
 * @brief   SPI Flash取消片选函数
 * @details 取消选择SPI Flash芯片
 */
void SPI_Flash_Deselect(void)
{
    GPIO_SetBits(SPI_FLASH_CS_PORT, SPI_FLASH_CS_PIN);
    SPI_Flash_Status.is_busy = false;
}

/**
 * @brief   SPI Flash状态寄存器读取函数
 * @details 读取Flash状态寄存器
 * @return uint8_t 状态寄存器值
 * 
 * 知识点: 状态寄存器读取
 * 1. 发送读状态命令
 * 2. 接收状态值
 * 3. 返回状态
 */
uint8_t SPI_Flash_Read_Status_Register(void)
{
    SPI_Flash_Select();
    SPI_Send_Byte(0x05);  // 读状态寄存器命令
    uint8_t status = SPI_Receive_Byte();
    SPI_Flash_Deselect();
    
    return status;
}

/* ============================== 更多SPI Flash操作函数实现 ============================== */

/**
 * @brief   SPI Flash读取函数
 * @details 从指定地址读取数据
 * @param addr 读取地址
 * @param data 数据缓冲区
 * @param size 读取大小
 * @return Flash_Operation_Result_t 操作结果
 */
Flash_Operation_Result_t SPI_Flash_Read(uint32_t addr, uint8_t *data, uint32_t size)
{
    if (data == NULL || size == 0) {
        return FLASH_OP_ERROR;
    }
    
    /**
     * 地址检查
     * 知识点: 地址验证
     */
    if (addr + size > SPI_FLASH_TOTAL_SIZE) {
        return FLASH_OP_INVALID_ADDR;
    }
    
    /**
     * 发送读取命令
     * 知识点: 读取命令格式
     * - 命令码: 0x03
     * - 3字节地址
     * - 数据字节
     */
    SPI_Flash_Select();
    SPI_Send_Byte(0x03);                    // 读取命令
    SPI_Send_Byte((addr >> 16) & 0xFF);     // 地址高字节
    SPI_Send_Byte((addr >> 8) & 0xFF);      // 地址中字节
    SPI_Send_Byte(addr & 0xFF);             // 地址低字节
    
    /**
     * 接收数据
     * 知识点: 数据接收
     */
    for (uint32_t i = 0; i < size; i++) {
        data[i] = SPI_Receive_Byte();
    }
    
    SPI_Flash_Deselect();
    
    SPI_Flash_Status.read_count++;
    return FLASH_OP_SUCCESS;
}

/**
 * @brief   数据格式化为文本函数
 * @details 将传感器数据格式化为文本
 * @param record 传感器数据记录
 * @param buffer 格式化缓冲区
 * @param buffer_size 缓冲区大小
 * @return bool 格式化结果
 */
bool SPI_Flash_Format_Data_Text(const Sensor_Data_Record_t *record, char *buffer, uint16_t buffer_size)
{
    if (record == NULL || buffer == NULL || buffer_size < 32) {
        return false;
    }
    
    /**
     * 格式化数据
     * 知识点: 文本格式
     * - 格式: "TEMP:25.6 LIGHT:456 GAS:789\r\n"
     * - 简单明了，无时间戳
     */
    int formatted = snprintf(buffer, buffer_size, "TEMP:%.1f LIGHT:%.0f GAS:%.0f\r\n",
                            record->temperature, 
                            record->light_level, 
                            record->gas_level);
    
    return (formatted > 0 && formatted < buffer_size);
}
