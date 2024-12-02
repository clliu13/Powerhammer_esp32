#include <SPI.h>

// SPI引脚定义
#define CS_PIN 5      // CS引脚
#define SCK_PIN 18    // SPI时钟引脚
#define MISO_PIN 19   // SPI主机输入引脚
#define MOSI_PIN 23   // SPI主机输出引脚

// 加速度寄存器地址
#define ACC_X_H 0x03  // X轴加速度高8位寄存器
#define ACC_X_L 0x04  // X轴加速度低8位寄存器
#define ACC_Y_H 0x05  // Y轴加速度高8位寄存器
#define ACC_Y_L 0x06  // Y轴加速度低8位寄存器
#define ACC_Z_H 0x07  // Z轴加速度高8位寄存器
#define ACC_Z_L 0x08  // Z轴加速度低8位寄存器

// 四元数寄存器地址
#define Q0_H 0x1B     // q0高8位寄存器
#define Q0_L 0x1C     // q0低8位寄存器
#define Q1_H 0x1D     // q1高8位寄存器
#define Q1_L 0x1E     // q1低8位寄存器
#define Q2_H 0x1F     // q2高8位寄存器
#define Q2_L 0x20     // q2低8位寄存器
#define Q3_H 0x21     // q3高8位寄存器
#define Q3_L 0x22     // q3低8位寄存器

// 状态寄存器地址
#define STATUS_REG 0x24 // 状态寄存器地址

void setup() {
  Serial.begin(115200);  // 初始化串口，用于调试
  pinMode(CS_PIN, OUTPUT); // 设置片选引脚为输出模式
  digitalWrite(CS_PIN, HIGH); // 默认拉高CS信号，关闭SPI通信

  // 初始化SPI
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  SPI.setFrequency(400000);  // 设置SPI频率为400kHz
  SPI.setDataMode(SPI_MODE3); // 设置SPI模式为CPOL=1, CPHA=1
  Serial.println("SPI Initialized");

  initializeGY953(); // 初始化GY-953
}

// 读取状态寄存器,检查是否真的更新了频率为200
uint8_t checkDataUpdate() {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x80 | STATUS_REG); // 读取寄存器指令
  uint8_t status = SPI.transfer(0x00); // 获取寄存器值
  digitalWrite(CS_PIN, HIGH);
  return status;
}

void loop() {
  // 记录起始时间
  unsigned long startTime = micros();

  // 读取加速度值
  float accX = readSensorData(ACC_X_H, ACC_X_L);
  float accY = readSensorData(ACC_Y_H, ACC_Y_L);
  float accZ = readSensorData(ACC_Z_H, ACC_Z_L);

  // 读取四元数
  float q0 = readSensorData(Q0_H, Q0_L);
  float q1 = readSensorData(Q1_H, Q1_L);
  float q2 = readSensorData(Q2_H, Q2_L);
  float q3 = readSensorData(Q3_H, Q3_L);

  // 打印结果
  Serial.print("Acceleration (m/s^2) -> X: ");
  Serial.print(accX);
  Serial.print(" Y: ");
  Serial.print(accY);
  Serial.print(" Z: ");
  Serial.println(accZ);

  Serial.print("Quaternion -> Q0: ");
  Serial.print(q0);
  Serial.print(" Q1: ");
  Serial.print(q1);
  Serial.print(" Q2: ");
  Serial.print(q2);
  Serial.print(" Q3: ");
  Serial.println(q3);

  // 记录结束时间并计算频率
  unsigned long endTime = micros();
  unsigned long elapsedTime = endTime - startTime;
  float frequency = 1000000.0 / elapsedTime;
  Serial.print("Data Collection Frequency: ");
  Serial.print(frequency);
  Serial.println(" Hz");

  static unsigned long lastTime = 0;
  static int updateCount = 0;
  
  //检查更新位置
  uint8_t status = checkDataUpdate();
  if (status & 0x80) { // 检查数据更新标志位（bit7）
    Serial.println("Data has been updated");
    updateCount++;
  } else {
    Serial.println("No new data");
  }

  // 每秒输出一次更新频率
  if (millis() - lastTime >= 1000) {
    Serial.print("Update Frequency: ");
    Serial.print(updateCount);
    Serial.println(" Hz");
    updateCount = 0;
    lastTime = millis();
  }

  delay(1); // 增加间隔以便于观察输出
}

void initializeGY953() {
  // Step 1: 启用加速度计、陀螺仪、磁力计
  sendCommand(0xA5, 0x51, 0xF6); // 开启加速度计
  sendCommand(0xA5, 0x52, 0xF7); // 开启陀螺仪
  sendCommand(0xA5, 0x53, 0xF8); // 开启磁力计

  // Step 2: 校准加速度计和陀螺仪
  sendCommand(0xA5, 0x57, 0xFC); // 校准加速度计和陀螺仪
  delay(2000);                   // 等待校准完成

  // Step 3: 校准磁力计
  sendCommand(0xA5, 0x58, 0xFD); // 校准磁力计
  delay(2000);                   // 等待校准完成

  // Step 4: 设置输出速率
  sendCommand(0xA5, 0xA6, 0x4B); // 设置输出速率为200Hz

  // Step 5: 设置数据类型为自动输出模式
  sendCommand(0xA5, 0x15, 0xBA); // 设置自动输出加速度原始数据
  sendCommand(0xA5, 0x65, 0x0A); // 设置自动输出四元数

  Serial.println("GY-953 Initialized");
}

// 发送命令的辅助函数
void sendCommand(uint8_t header, uint8_t command, uint8_t checksum) {
  digitalWrite(CS_PIN, LOW); // 拉低片选信号，开始通信
  SPI.transfer(header);      // 发送帧头
  SPI.transfer(command);     // 发送指令
  SPI.transfer(checksum);    // 发送校验和
  digitalWrite(CS_PIN, HIGH); // 拉高片选信号，结束通信
  delay(50);                 // 确保指令被模块处理
}

// 读取传感器数据
float readSensorData(uint8_t regHigh, uint8_t regLow) {
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

  // 合成16位数据并转换为实际值
  int16_t rawData = (highByte << 8) | lowByte;
  return rawData / 100.0; // 数据单位为 0.01，需要除以100
}
