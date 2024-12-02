#include <SPI.h>

// SPI引脚定义
#define CS_PIN 5      // CS引脚
#define SCK_PIN 18    // SPI时钟引脚
#define MISO_PIN 19   // SPI主机输入引脚
#define MOSI_PIN 23   // SPI主机输出引脚

// 初始化GY-953寄存器地址
#define ROLL_H 0x15   // 横滚角高8位寄存器
#define ROLL_L 0x16   // 横滚角低8位寄存器
#define PITCH_H 0x17  // 俯仰角高8位寄存器
#define PITCH_L 0x18  // 俯仰角低8位寄存器
#define YAW_H 0x19    // 航向角高8位寄存器
#define YAW_L 0x1A    // 航向角低8位寄存器

void setup() {
  Serial.begin(256000);  // 初始化
  pinMode(CS_PIN, OUTPUT); // 设置片选引脚为输出模式
  digitalWrite(CS_PIN, HIGH); // 默认将 CS 拉高，表示 SPI 通信关闭

  // 初始化SPI
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  SPI.setFrequency(400000);  // 设置SPI频率为400kHz，GY-953 的最大支持频率
  SPI.setDataMode(SPI_MODE3); // 设置 SPI 数据模式为 CPOL=1（时钟空闲时为高电平），CPHA=1（数据在第二个时钟沿采样）
  Serial.println("SPI Initialized");




  setFrequency200Hz(); // 调用设置频率的函数
  delay(100);
    // 发送初始化命令
  initializeGY953();
}



void loop() {
  // 读取欧拉角数据
  float roll = readEulerAngle(ROLL_H, ROLL_L);    // 横滚角
  float pitch = readEulerAngle(PITCH_H, PITCH_L); // 俯仰角
  float yaw = readEulerAngle(YAW_H, YAW_L);       // 航向角

  // 打印结果
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" Pitch: ");
  Serial.print(pitch);
  Serial.print(" Yaw: ");
  Serial.println(yaw);

  delay(10);
}

// 初始化GY-953
void initializeGY953() {
  digitalWrite(CS_PIN, LOW); // 拉低片选，ESP32和GY953开始通信

  SPI.transfer(0xA5);        // 发送帧头
  SPI.transfer(0x45);        // 发送初始化命令
  SPI.transfer(0xEA);        // 欧拉角

  digitalWrite(CS_PIN, HIGH); // 拉高片选，表示通信完成，关闭

  delay(5);                // 等待模块完成初始化
}

void setFrequency200Hz() {
  digitalWrite(CS_PIN, LOW); // 拉低片选，开始通信

  // 发送设置频率的指令
  SPI.transfer(0xA5); // 帧头
  SPI.transfer(0xA6); // 指令
  SPI.transfer(0x4B); // 校验和

  digitalWrite(CS_PIN, HIGH); // 拉高片选，结束通信
  delay(100); // 确保模块处理完成

  // 检查是否有反馈
  digitalWrite(CS_PIN, LOW);
  uint8_t response = SPI.transfer(0x00); // 尝试读取反馈
  digitalWrite(CS_PIN, HIGH);
  Serial.print("Response after setting frequency: ");
  Serial.println(response, HEX);
}

// 从GY-953读取欧拉角数据
float readEulerAngle(uint8_t regHigh, uint8_t regLow) {
  uint8_t highByte, lowByte;

  // 读取高8位
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x80 | regHigh); // 读寄存器指令
  highByte = SPI.transfer(0x00); // 读取数据
  digitalWrite(CS_PIN, HIGH);

  // 读取低8位
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x80 | regLow); // 读寄存器指令
  lowByte = SPI.transfer(0x00); // 读取数据
  digitalWrite(CS_PIN, HIGH);

  // 将高8和低8合成成一个16位的整数
  int16_t rawData = (highByte << 8) | lowByte; 
  return rawData / 100.0; // 数据单位为0.01度，除以100
}
