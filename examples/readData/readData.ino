/*!
 * @file  readData.ino
 * @brief  本示例介绍了使用本模块测试一小时内降雨量的采集的方法
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license    The MIT License (MIT)
 * @author     [fary](feng.yang@dfrobot.com)
 * @version    V1.0
 * @date       2023-02-28
 * @url        https://github.com/DFRobot/DFRobot_RainfallSensor
 */
#include "DFRobot_RainfallSensor.h"

//#define MODE_UART
#ifdef MODE_UART //串口通信
 #if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
   #include "SoftwareSerial.h"
   SoftwareSerial mySerial(/*rx =*/4, /*tx =*/5);
   DFRobot_RainfallSensor_UART Sensor(/*Stream *=*/&mySerial);
 #else
   DFRobot_RainfallSensor_UART Sensor(/*Stream *=*/&Serial1);
 #endif
#else //I2C通信
  DFRobot_RainfallSensor_I2C Sensor(&Wire);
#endif

void setup(void)
{
#ifdef MODE_UART
  #if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
    mySerial.begin(9600);
  #elif defined(ESP32)
    Serial1.begin(9600, SERIAL_8N1, /*rx =*/D3, /*tx =*/D2);
  #else
    Serial1.begin(9600);
  #endif
#endif 
  Serial.begin(9600);
  
  delay(1000);
  while(!Sensor.begin()){
    Serial.println("Sensor init err!!!");
    delay(1000);
  }
  Serial.print("vid:\t");
  Serial.println(Sensor.vid,HEX);
  Serial.print("pid:\t");
  Serial.println(Sensor.pid,HEX);
  Serial.print("Version:\t");
  Serial.println(Sensor.getFirmwareVersion());
  //设置雨量累加值，单位为mm
  //Sensor.setRainAccumulatedValue(0.2794);
}

void loop()
{
  //获取传感器运行时间，单位 小时
  Serial.print("Sensor WorkingTime:\t");
  Serial.print(Sensor.getSensorWorkingTime());
  Serial.println(" H");
  //获取传感器运行时间内的累计雨量
  Serial.print("Rainfall:\t");
  Serial.println(Sensor.getRainfall());
  //获取系统1小时内的累计雨量（函数参数可选1-24）
  Serial.print("1 Hour Rainfall:\t");
  Serial.print(Sensor.getRainfall(1));
  Serial.println(" mm");
  //获取原始数据，雨量的翻斗次数，单位 次
  Serial.print("rainfall raw:\t");
  Serial.println(Sensor.getRawData());
  delay(1000);
}
