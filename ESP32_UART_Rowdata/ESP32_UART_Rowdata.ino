#include <Wire.h> 
int YPR[3];
unsigned char Re_buf[11], counter = 0;
unsigned char sign = 0;

// 定义用于 GY-953 的 UART 引脚
#define RXD2 16 // 接收引脚，连接 GY-953 的 TX
#define TXD2 17 // 发送引脚，连接 GY-953 的 RX

//-----------------------------------------------------------
void setup() {
  Serial.begin(115200);  // 初始化调试串口，用于查看数据
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // 初始化 UART2，用于 GY-953 通信

  delay(2000);

  // 发送初始化命令到 GY-953，设置为连续输出模式
  Serial2.write(0xA5); 
  Serial2.write(0x45);    // 初始化GY953，连续输出模式
  Serial2.write(0xEA);    // 初始化GY953，连续输出模式
}
//-------------------------------------------------------------
void loop() {
  if (sign) {
    sign = 0;
    if (Re_buf[0] == 0x5A && Re_buf[1] == 0x5A) { // 检查帧头
      YPR[0] = (Re_buf[8] << 8 | Re_buf[9]) / 100; // 航向角
      YPR[1] = (Re_buf[6] << 8 | Re_buf[7]) / 100; // 俯仰角
      YPR[2] = (Re_buf[4] << 8 | Re_buf[5]) / 100; // 横滚角
      Serial.print("YPR:\t");
      Serial.print(YPR[0], DEC); Serial.print("\t"); // 显示航向角
      Serial.print(YPR[1], DEC); Serial.print("\t"); // 显示俯仰角
      Serial.println(YPR[2], DEC);                  // 显示横滚角
      delay(10);
    }
  }
}
//---------------------------------------------------------------
// 使用 Serial2 来读取 GY-953 数据
void serialEvent2() {
  while (Serial2.available()) {
    Re_buf[counter] = (unsigned char)Serial2.read();
    if (counter == 0 && Re_buf[0] != 0x5A) return; // 检查帧头
    counter++;
    if (counter == 11) { // 接收到完整帧
      counter = 0;       // 重新计数
      sign = 1;
    }
  }
}
