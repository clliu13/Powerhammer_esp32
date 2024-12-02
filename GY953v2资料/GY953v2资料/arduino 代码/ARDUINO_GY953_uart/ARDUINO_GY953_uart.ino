//GY-953  ARDUINO  测试代码 串口显示角度
//注意1：下载程序时候，请先断开GY953的连线!否则将导致下载不成功
//注意2：GY953模块使用时，上电自校正,建议不要用手拿着模块,
//       保持3秒以上静止状态，才能发送命令开始工作
//   GY953                  arduino pro mini
//   VCC----------------------VCC
//   RX-----------------------TX
//   TX-----------------------RX
//   GND----------------------GND
/*
使用步骤：
1.先下载GY953_uart程序至arduino
2.再接上GY953模块
3.按复位按键
4.打开串口，波特率115200
*/
#include <Wire.h> 
int YPR[3];
unsigned char Re_buf[11],counter=0;
unsigned char sign=0;
int led = 13;


//-----------------------------------------------------------
void setup()
{
  Serial.begin(115200);  
  delay(2000);
  
      
  Serial.write(0XA5); 
  Serial.write(0X45);    //初始化GY953,连续输出模式
  Serial.write(0XEA);    //初始化GY953,连续输出模式
  
}
//-------------------------------------------------------------
void loop() {
  if(sign)
  {  
     sign=0;
     if(Re_buf[0]==0x5A&&Re_buf[1]==0x5A )        //检查帧头，帧尾
     {  	       
            YPR[0]=(Re_buf[8]<<8|Re_buf[9])/100;   //合成数据，去掉小数点后2位
            YPR[1]=(Re_buf[6]<<8|Re_buf[7])/100;
            YPR[2]=(Re_buf[4]<<8|Re_buf[5])/100;
           Serial.print("YPR:\t");
           Serial.print(YPR[0], DEC); Serial.print("\t"); //显示航向
           Serial.print(YPR[1], DEC); Serial.print("\t"); //显示俯仰角
           Serial.println(YPR[2], DEC);                    //显示横滚角                
           delay(10);           
   }
  } 
} 
//---------------------------------------------------------------
void serialEvent() {
  while (Serial.available()) {   
    Re_buf[counter]=(unsigned char)Serial.read();
    if(counter==0&&Re_buf[0]!=0x5A) return;      // 检查帧头         
    counter++;       
    if(counter==11)                //接收到数据
    {    
       counter=0;                 //重新赋值，准备下一帧数据的接收 
       sign=1;
    }      
  }
}

