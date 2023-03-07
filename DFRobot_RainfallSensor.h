/*!
 * @file  DFRobot_RainfallSensor.h
 * @brief  Define infrastructure of DFRobot_RainfallSensor class
 * @details  该库实现了与SEN0575设备进行通信的所有功能，包括配置设备参数和读取设备数据
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [fary](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2023-01-28
 * @url  https://github.com/DFRobot/DFRobot_RainfallSensor
 */
#ifndef __DFROBOT_RAINFALLSENSOR_H__
#define __DFROBOT_RAINFALLSENSOR_H__
#include <Arduino.h>
#include "DFRobot_RTU.h"
#include "Wire.h"
class DFRobot_RainfallSensor{
  public: 
    #define I2C_REG_PID                            0x00
    #define I2C_REG_VID                            0x02
    #define I2C_REG_VERSION                        0x0A
    #define I2C_REG_TIME_RAINFALL                  0x0C
    #define I2C_REG_CUMULATIVE_RAINFALL            0x10
    #define I2C_REG_RAW_DATA                       0x14
    #define I2C_REG_SYS_TIME                       0x18
    #define I2C_REG_RAIN_HOUR                      0x26
    #define I2C_REG_BASE_RAINFALL                  0x28
    #define IIC_MODE  0
    #define UART_MODE 1
    /**
     * @enum  eSEN0575InputReg_t
     * @brief  设备的输入寄存器地址
     */
    typedef enum{
      eInputRegPidSEN0575=0x0000,                 /**< SEN0575 存储PID的输入寄存器的地址 */
      eInputRegVidSEN0575,                        /**< SEN0575 存储VID的输入寄存器的地址 */
      eInputRegRegAddrSEN0575,                    /**< SEN0575 储存设备地址的寄存器 */
      eInputRegBaudSEN0575,                       /**< SEN0575 储存串口波特率的寄存器 */
      eInputRegVerifyAndStopSEN0575,              /**< SEN0575 存储串口奇偶检验位与停止位的输入寄存器的地址 */
      eInputRegVersionSEN0575,                    /**< SEN0575 存储固件版本的输入寄存器地址 */
      eInputRegTimeRainFallLSEN0575,              /**< SEN0575 存储设置时间内的累计雨量低16位 */
      eInputRegTimeRainFallHSEN0575,              /**< SEN0575 存储设置时间内的累计雨量高16位 */
      eInputRegCumulativeRainFallLSEN0575,        /**< SEN0575 存储开始工作后累计雨量低16位 */
      eInputRegCumulativeRainFallHSEN0575,        /**< SEN0575 存储开始工作后累计雨量高16位 */
      eInputRegRawDataLSEN0575,                   /**< SEN0575 存储原始数据低16位 */
      eInputRegRawDataHSEN0575,                   /**< SEN0575 存储原始数据高16位 */
      eInputRegSysWorkingTimeSEN0575,             /**< SEN0575 系统工作时间 */
    }eSEN0575InputReg_t;
  
    /**
     * @enum  eSEN0575HoldingReg_t
     * @brief  设备的保持寄存器地址
     */
    typedef enum{
      eHoldingRegReserved0SEN0575=0x0000,         /**< SEN0575 该寄存器保留 */
      eHoldingRegReserved1SEN0575,                /**< SEN0575 该寄存器保留 */
      eHoldingRegReserved2SEN0575,                /**< SEN0575 该寄存器保留 */
      eHoldingRegReserved3SEN0575,                /**< SEN0575 该寄存器保留 */
      eHoldingRegReserved4SEN0575,                /**< SEN0575 该寄存器保留 */
      eHoldingRegReserved5SEN0575,                /**< SEN0575 该寄存器保留 */
      eHoldingRegRainHourSEN0575,                 /**< SEN0575 设置计算累计雨量的时间 */
      eHoldingRegBaseRainFallSEN0575,             /**< SEN0575 设置雨量累加值 */
    }eSEN0575HoldingReg_t;
  
    /**
     * @fn DFRobot_RainfallSensor
     * @brief Construct a new dfrobot windspeedwinddirectionrainsensor object
     * @param mode 工作模式，IIC_MODE:0 ,UART_MODE:1
     */
    DFRobot_RainfallSensor(uint8_t mode);
    ~DFRobot_RainfallSensor(){};
  
    /**
     * @fn begin
     * @brief 本函数将会尝试与从机设备进行通信,根据返回值判断通信是否成功
     * @return 返回通信结果
     * @retval true  Succeed
     * @retval false Failed
     */
    bool begin(void);
  
    /**
     * @fn getFirmwareVersion
     * @brief  get firmware version
     * @return  Return  firmware version
     */
    String getFirmwareVersion(void);
  
    /**
     * @fn getRainfall
     * @brief 获取累计雨量
     * @return float 累计雨量
     */
    float getRainfall(void);
  
    /**
     * @fn getRainfallD
     * @brief 获取指定时间内的累计雨量
     * @param hour 指定时间(有效设置为1-24小时)
     * @return float 累计雨量
     */
    float getRainfall(uint8_t hour);
  
    /**
     * @fn getRawData
     * @brief Get the Rawdata object
     * @return 雨量的翻斗次数，单位 次
     */
    uint32_t getRawData();
  
    /**
     * @fn getSensorWorkingTime
     * @brief Obtain the sensor working time
     * @return 工作时间,单位小时
     */
    float getSensorWorkingTime();
  
    /**
     * @fn setRainAccumulatedValue
     * @brief Set the Rain Accumulated Value object
     * @param accumulatedValue 雨量累加值，单位为毫米
     * @return 返回设置结果
     * @retval 0 设置成功
     * @retval 其他值设置失败 
     */
    uint8_t setRainAccumulatedValue(float accumulatedValue = 0.2794);
    uint32_t vid;
    uint32_t pid;
  private:
    /**
     * @fn getVid
     * @brief  Get VID and PID
     * @return  获取的结果
     * @retval true:正确获取
     * @retval false:获取数据失败或者获取数据错误
     */
    bool getPidVid(void);
    virtual uint8_t readRegister(uint8_t reg,void* pBuf, size_t size){return 0;};
    virtual uint16_t readRegister(uint16_t reg){return 0;};
    virtual uint8_t writeRegister(uint8_t reg,void* pBuf,size_t size){return 0;};
    virtual uint16_t writeRegister(uint16_t reg,uint16_t data){return 0;};
  private:
    int    _dePin;
    uint8_t   _mode;
};
class DFRobot_RainfallSensor_UART:public DFRobot_RainfallSensor,public DFRobot_RTU{
  public: 
    /**
     * @fn DFRobot_RainfallSensor_UART
     * @brief Construct a new dfrobot windspeedwinddirectionrainsensor uart object
     * @param s 需要使用的串口设备
     */
    DFRobot_RainfallSensor_UART(Stream *s);
    ~DFRobot_RainfallSensor_UART(){};
    /**
     * @fn begin
     * @brief 本函数将会尝试与从机设备进行通信,根据返回值判断通信是否成功
     * @return 返回通信结果
     * @retval true  Succeed
     * @retval false Failed
     */
    bool begin(void);
  private:
    /**
     * @fn readRegister
     * @brief 读输入寄存器
     * @param reg 输入寄存器地址
     * @return uint16_t 读到的数据
     */
    uint16_t readRegister(uint16_t reg);
  
    /**
     * 
     * @fn writeRegister
     * @brief 写保持寄存器,写完后内部延时了12MS,传感器需要下写入后花费12Ms进行保存，此时不能进行通信
     * @param reg 保持寄存器地址
     * @param data 要写入的数据
     * @return uint16_t 写入结果
     * @retval 0 表示成功
     * @retval 其他值设置失败 
     */
    uint16_t writeRegister(uint16_t reg,uint16_t data);
  private:
    uint8_t _deviceAddr;
};

class DFRobot_RainfallSensor_I2C:public DFRobot_RainfallSensor{
public: 
  /**
   * @fn DFRobot_RainfallSensor_I2C
   * @brief Construct a new dfrobot windspeedwinddirectionrainsensor iic object
   * @param pWire 需要使用的I2C设备
   */
  DFRobot_RainfallSensor_I2C(TwoWire *pWire);
  ~DFRobot_RainfallSensor_I2C(){};
  /**
   * @fn begin
   * @brief 本函数将会尝试与从机设备进行通信,根据返回值判断通信是否成功
   * @return 返回通信结果
   * @retval true  Succeed
   * @retval false Failed
   */
  bool begin(void);
private:
  /**
   * @fn readRegister
   * @brief 读I2C寄存器
   * @param reg I2C寄存器地址
   * @param pBuf 缓存空间
   * @param size 读取长度
   * @return 读取长度
   */
  uint8_t readRegister(uint8_t reg,void* pBuf, size_t size);

  /**
   * @fn writeRegister
   * @brief 写I2C寄存器,写完后内部延时了12MS,传感器需要下写入后花费12Ms进行保存，此时不能进行通信
   * @param reg I2C寄存器地址
   * @param pBuf 数据存储空间
   * @param size 读取长度
   * @return 写入的结果
   * @retval 0 表示成功
   * @retval 其他值设置失败 
   */
  uint8_t writeRegister(uint8_t reg,void* pBuf,size_t size);
private:
  TwoWire* _pWire;
  uint8_t _deviceAddr;
};
#endif