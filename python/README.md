DFRobot_RainfallSensor
===========================

- [中文版](./README_CN.md)

本库提供了获取SEN0575采集数据的全部方法，用户只需要简单的使用本库就可以获取到SEN0575采集的数据。

![产品效果图](../resources/images/SEN0575.png)

## Product Link (https://www.dfrobot.com)

    SKU：SEN0575

## Table of Contents

  * [summary](#summary)
  * [installation](#installation)
  * [methods](#methods)
  * [compatibility](#compatibility)
  * [history](#history)
  * [credits](#credits)

## Summary
* 读取雨量计数据

## Installation

要使用库, 首先下载库文件, 将其粘贴到指定的目录中, 然后打开examples文件夹并在该文件夹中运行演示。
本库关联了modbus_tk库，使用前请确保树莓派已经下载了modbus_tk。

## Methods

```python
  def begin(self)
    '''!
      @brief 本函数将会尝试与从机设备进行通信,根据返回值判断通信是否成功
      @return 返回通信结果
      @retval true  Succeed
      @retval false Failed
    '''

  def get_firmware_version(self)
    '''!
      @brief  get firmware version
      @return  Return  firmware version
    '''

  def get_pid_vid(self)
    '''!
      @brief  获取pid和vid
      @return  Return  true:正确获取，false:获取数据失败或者获取数据错误
    '''


  def get_rainfall(self)
    '''!
      @brief  获取累计雨量
      @return 累计雨量
    '''

  def get_rainfall_time(self,hour)
    '''!
      @brief  获取指定时间内的累计雨量
      @param hour 指定时间(有效设置为1-24小时)
      @return 累计雨量
    '''

  def get_raw_data(self)
    '''!
      @brief 获取原始数据
      @return 雨量的翻斗次数，单位 次
    '''

  def set_rain_accumulated_value(self,value)
    '''!
      @brief 设置雨量累加值
      @param value 雨量累加值，单位为毫米
      @return 返回 0 设置成功，其他值设置失败 
    '''

  def get_sensor_working_time(self):
    '''!
      @brief Obtain the sensor working time
      @return 工作时间,单位小时
    '''
```

## Compatibility

* RaspberryPi Version

| Board        | Work Well | Work Wrong | Untested | Remarks |
| ------------ | :-------: | :--------: | :------: | ------- |
| Raspberry Pi2 |           |            |    √     |         |
| Raspberry Pi3 |           |            |    √     |         |
| Raspberry Pi4 |       √   |            |          |         |

* Python Version

| Python  | Work Well | Work Wrong | Untested | Remarks |
| ------- | :-------: | :--------: | :------: | ------- |
| Python2 |     √     |            |          |         |
| Python3 |     √     |            |          |         |

## History

- 2023/02/28 - Version 1.0.0 released.

## Credits

Written by fary(feng.yang@dfrobot.com), 2023. (Welcome to our [website](https://www.dfrobot.com/))
