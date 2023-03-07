# -*- coding: utf-8 -*
'''!
  @file        DFRobot_RainfallSensor.py
  @brief       这是SEN0575设备基库
  @copyright   Copyright (c) 2021 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author      fary(feng.yang@dfrobot.com)
  @version     V1.0
  @date        2023-02-28
  @url         https://github.com/DFRobor/DFRobot_RainfallSensor
'''

import serial
import smbus
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
import time


class DFRobot_RainfallSensor(object):
  ## SEN0575 存储PID的输入寄存器的地址。
  SEN0575_INPUT_REG_PID                          = 0x0000
  ## SEN0575 存储VID的输入寄存器的地址。
  SEN0575_INPUT_REG_VID                          = 0x0001
  ## SEN0575 存储设备地址的输入寄存器的地址。
  SEN0575_INPUT_REG_ADDR                         = 0x0002
  ## SEN0575 存储设备波特率的输入寄存器的地址。
  SEN0575_INPUT_REG_BAUD                         = 0x0003
  ## SEN0575 存储RS485奇偶检验位与停止位的输入寄存器的地址。
  SEN0575_INPUT_REG_VERIFYANDSTOP                = 0x0004
  ## SEN0575 存储固件版本的输入寄存器地址。
  SEN0575_INPUT_REG_VERSION                      = 0x0005
  
  ## SEN0575 存储设置时间内的低16位累计雨量的输入寄存器地址。
  SEN0575_INPUT_REG_TIME_RAIN_FALL_L             = 0x0006
  ## SEN0575 存储设置时间内的高16位累计雨量的输入寄存器的地址。
  SEN0575_INPUT_REG_TIME_RAIN_FALL_H             = 0x0007
  ## SEN0575 存储开始工作后低16位累计雨量的输入寄存器的地址。
  SEN0575_INPUT_REG_CUMULATIVE_RAINFALL_L        = 0x0008  
  ## SEN0575 存储开始工作后高16位累计雨量的输入寄存器的地址。
  SEN0575_INPUT_REG_CUMULATIVE_RAINFALL_H        = 0x0009  
  ## SEN0575 存储原始数据（低16位）的输入寄存器的地址。
  SEN0575_INPUT_REG_RAW_DATA_L                   = 0x000A  
  ## SEN0575 存储原始数据（高16位）的输入寄存器的地址。
  SEN0575_INPUT_REG_RAW_DATA_H                   = 0x000B  
  ## SEN0575 存储系统工作时间的输入寄存器的地址。
  SEN0575_INPUT_REG_SYS_TIME                     = 0x000C
  ## 设置计算累计雨量的时间
  SEN0575_HOLDING_REG_RAW_RAIN_HOUR              = 0x0006
  ## 设置雨量累加值
  SEN0575_HOLDING_REG_RAW_BASE_RAINFALL          = 0x0007
  
  ## SEN0575 存储PID的输入寄存器的地址。
  SEN0575_I2C_REG_PID                          = 0x00
  ## SEN0575 存储VID的输入寄存器的地址。
  SEN0575_I2C_REG_VID                          = 0x02
  ## SEN0575 存储固件版本的输入寄存器地址。
  SEN0575_I2C_REG_VERSION                      = 0x0A
  ## SEN0575 存储设置时间内的累计雨量
  SEN0575_I2C_REG_TIME_RAINFALL                = 0x0C
  ## SEN0575 存储开始工作后累计雨量
  SEN0575_I2C_REG_CUMULATIVE_RAINFALL          = 0x10
  ## SEN0575 存储原始数据低16位
  SEN0575_I2C_REG_RAW_DATA                     = 0x14
  ## SEN0575 存储原始数据低16位
  SEN0575_I2C_REG_SYS_TIME                     = 0x18
  
  ## 设置计算累计雨量的时间  
  SEN0575_I2C_REG_RAW_RAIN_HOUR                = 0x26
  ## 设置雨量累加值  
  SEN0575_I2C_REG_RAW_BASE_RAINFALL            = 0x28

  I2C_MODE  =0
  UART_MODE =1

  '''!
    @brief Define DFRobot_RainfallSensor basic class
    @details Drive the sensor
  '''
  def __init__(self, mode):
    self._mode = mode
    self.vid = 0
    self.pid = 0
  
  def begin(self):
    '''!
      @brief 本函数将会尝试与从机设备进行通信,根据返回值判断通信是否成功
      @return 返回通信结果
      @retval true  Succeed
      @retval false Failed
    '''
    return self.get_pid_vid()

  def get_firmware_version(self):
    '''!
      @brief  get firmware version
      @return  Return  firmware version
    '''
    if self._mode == self.I2C_MODE :
      list = self._read_register(self.SEN0575_I2C_REG_VERSION, 2)
      version = list[0] | ( list[1] << 8 )
    else:
      list = self._read_register(self.SEN0575_INPUT_REG_VERSION, 1)
      version = list[0]
    return str(version >> 12) + '.'+ str( ( (version >> 8) & 0x0F ) ) + '.' + str( ( (version >> 4) & 0x0F ) ) + '.' + str( (version & 0x0F) )

  def get_sensor_working_time(self):
    '''!
      @brief Obtain the sensor working time
      @return 工作时间,单位小时
    '''
    if self._mode == self.I2C_MODE:
      list = self._read_register(self.SEN0575_I2C_REG_SYS_TIME, 2)
      working_time = list[0] | (list[1] << 8)
    else:
      list = self._read_register(self.SEN0575_INPUT_REG_SYS_TIME, 1)
      working_time = list[0]
    return working_time / 60.0

  def get_pid_vid(self):
    '''!
      @brief  获取pid和vid
      @return  Return  true:正确获取，false:获取数据失败或者获取数据错误
    '''
    ret = False
    if self._mode == self.I2C_MODE:
      list = self._read_register(self.SEN0575_I2C_REG_PID, 4)
      self.pid = list[0] | ( list[1] << 8 ) | ( ( list[3] & 0xC0 ) << 10 )
      self.vid = list[2] | ( ( list[3] & 0x3F ) << 8 )
    else:
      list = self._read_register(self.SEN0575_INPUT_REG_PID, 2)
      self.pid = ( ( list[1] & 0xC000 ) << 2 ) | list[0]
      self.vid = list[1] & 0x3FFF
    if (self.vid == 0x3343) and (self.pid == 0x100C0):
      ret = True
    return ret

  def get_rainfall(self):
    '''!
      @brief  获取累计雨量
      @return 累计雨量
    '''
    if self._mode == self.I2C_MODE:
      list = self._read_register(self.SEN0575_I2C_REG_CUMULATIVE_RAINFALL, 4)
      rainfall = list[0] | ( list[1] << 8 ) | ( list[2] << 16 ) | ( list[3] << 24 )
    else:
      list = self._read_register(self.SEN0575_INPUT_REG_CUMULATIVE_RAINFALL_L, 2)
      rainfall = list[0] | ( list[1] << 16 )
    return rainfall / 10000.0

  def get_rainfall_time(self,hour):
    '''!
      @brief  获取指定时间内的累计雨量
      @param hour 指定时间(有效设置为1-24小时)
      @return 累计雨量
    '''
    if hour>24:
      return 0
    list = [hour]
    if self._mode == self.I2C_MODE:
      self._write_register(self.SEN0575_I2C_REG_RAW_RAIN_HOUR, list)
      list = self._read_register(self.SEN0575_I2C_REG_TIME_RAINFALL, 4)
      rainfall = list[0] | ( list[1] << 8 ) | ( list[2] << 16 ) | ( list[3] << 24 )
    else:
      self._write_register(self.SEN0575_HOLDING_REG_RAW_RAIN_HOUR, list)
      list = self._read_register(self.SEN0575_INPUT_REG_TIME_RAIN_FALL_L, 2)
      rainfall = list[0] | ( list[1] << 16 )
    return rainfall / 10000.0

  def get_raw_data(self):
    '''!
      @brief 获取原始数据
      @return 雨量的翻斗次数，单位 次
    '''
    if self._mode == self.I2C_MODE:
      list = self._read_register(self.SEN0575_I2C_REG_RAW_DATA, 4)
      raw_data = list[0] | ( list[1] << 8 ) | ( list[2] << 16 ) | ( list[3] << 24 )
    else:
      list = self._read_register( self.SEN0575_INPUT_REG_RAW_DATA_L, 2 )
      raw_data = list[0] | ( list[1] << 16 )
    return raw_data

  def set_rain_accumulated_value(self, value):
    '''!
      @brief 设置雨量累加值
      @param value 雨量累加值，单位为毫米
      @return 返回 0 设置成功，其他值设置失败 
    '''
    data = int(value * 10000)
    if self._mode == self.I2C_MODE:
      list=[data & 0xFF , data >> 8]
      ret=self._write_register(self.SEN0575_I2C_REG_RAW_BASE_RAINFALL, list)
    else:
      ret=self._write_register(self.SEN0575_HOLDING_REG_RAW_BASE_RAINFALL, [data])
    return ret

  def _write_reg(self, reg, data):
    '''!
      @brief writes data to a register
      @param reg register address
      @param data written data
    '''
    # Low level register writing, not implemented in base class
    raise NotImplementedError()

  def _read_reg(self, reg, length):
    '''!
      @brief read the data from the register
      @param reg register address
      @param length read data length
      @return read data list
    '''
    # Low level register writing, not implemented in base class
    raise NotImplementedError()

class DFRobot_RainfallSensor_UART(DFRobot_RainfallSensor):
  '''!
    @brief Define DFRobot_RainfallSensor_UART class
    @details Drive the sensor
  '''
  def __init__(self,):
    '''!
      @brief 初始化串口
    '''
    self._baud = 9600
    self._addr = 0xC0
    self.master = modbus_rtu.RtuMaster( serial.Serial(port="/dev/ttyAMA0", baudrate=self._baud, bytesize=8, parity='N', stopbits=1) )
    self.master.set_timeout(1.0)
    super(DFRobot_RainfallSensor_UART, self).__init__(self.UART_MODE)

  def _write_register(self, reg, data):
    '''!
      @brief writes data to a register
      @param reg register address
      @param data written data
    '''
    ret=True
    list_write=list( self.master.execute(self._addr, cst.WRITE_MULTIPLE_REGISTERS, reg, output_value=data) )
    time.sleep(0.030)
    if list_write[1] != len(data):
      ret=False
    return ret

  def _read_register(self, reg, length):
    '''!
      @brief read the data from the register
      @param reg register address
      @param length read data length
      @return read data list
    '''
    return list( self.master.execute(self._addr, cst.READ_INPUT_REGISTERS, reg, length) )

class DFRobot_RainfallSensor_I2C(DFRobot_RainfallSensor):
  '''!
    @brief Define DFRobot_RainfallSensor_I2C class
    @details Drive the sensor
  '''
  def __init__(self,bus=1):
    '''!
      @brief 初始化串口
    '''
    self._addr = 0x1D
    self.i2c = smbus.SMBus(bus)
    super(DFRobot_RainfallSensor_I2C, self).__init__(self.I2C_MODE)

  def _write_register(self, reg, data):
    '''!
      @brief writes data to a register
      @param reg register address
      @param data written data
    '''
    try:
      self.i2c.write_i2c_block_data(self._addr, reg, data)
      time.sleep(0.050)
    except:
      pass
    return True

  def _read_register(self, reg, length):
    '''!
      @brief read the data from the register
      @param reg register address
      @param length read data length
      @return read data list
    '''
    ret=[]
    ret = [0]*length
    try:
      ret=self.i2c.read_i2c_block_data(self._addr, reg, length)
    except:
      pass
    return ret