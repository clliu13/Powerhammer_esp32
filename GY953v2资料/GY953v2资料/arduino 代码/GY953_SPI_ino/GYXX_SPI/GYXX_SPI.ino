#include <SPI.h>
/*
Arduino:1.0.5-r2
MCU:ATmega328 5v 16M
本程序采用SPI读取GY-953模块
硬件接法：
中断0-INT;10-CS;13-SCK;12-MISO;11-MOSI;ATmega328 RX--ft232TX;ATmega328 TX--ft232RX
程序说明：
由于GY-953数据更新时,为低电平,所以待出现上升沿时为数据更新,所以设置的外部
中断0（数字引脚2）为上升沿中断，在中断中更新标志位，然后在主循环查询标志为
1则开始读取0x01后的所有寄存器(共41个),本程序通过串口接收输出指令,
可在serial monitor输入对应的指令：
'1'-- ON/OFF Roll Pitch Yaw output
'2'-- ON/OFF raw ACC data output
'3'-- ON/OFF raw GYR data output
'4'-- ON/OFF raw MAG data output
'5'-- ON/OFF raw  Q    data output
'A'-- Set the output frequency 50hz
'B'-- Set the output frequency 100hz
'C'-- Set the output frequency 200hz
'I' -- Calibration Accelerometer、Gyro
'J' -- Calibration compass
'K'-- Clear saved data
'P'-- Read ACC、GYR、MAG calibration accuracy
//注意字母为大写
----------------------------------------
ACC:0~3
GYR:0~3
compass:0~3
0 being not accurate,  3 being accurate.
output frequency:
3---50HZ
4---100HZ
5---200HZ
----------------------------------------
'Q'-- Read ACC、GYR、MAG range 
-----------------------------------------
ACC:0---------------+-2g
GYR:3---------------+-2000dps/s
compass:1:---------16bit
-----------------------------------------
'U'--Open All sensors
'V'--Close accelerometer
'W'--Close gyro
'X'--Close compass
------------------
GY953--ATmega328
CS--10;MOSI--MOSI/11;MISO--MISO/12;SCK--SCK/13;INT--2
------------------
------------------
GY953--Leonardo
CS--9;MOSI--MOSI/16;MISO--MISO/14;SCK--SCK/15;INT--3
------------------
GY953--MEAG2560
CS--10(PB4/D10);MOSI--MOSI/50;MISO--MISO/51;SCK--SCK/52;INT--PE5(D2)
------------------
*/
#define ACC 0X01
#define GYR 0X02
#define MAG 0X04
#define RPY 0X08
#define Q4  0X10
#define RegisterA 0X08//寄存器1固定值
#define RegisterB 0X11//寄存器2固定值

//#define Leonardo
//#define MEAG2560

#ifdef Leonardo
#define spi_cs
int  CS=9;
#endif
#ifdef MEAG2560
#define spi_cs
int  CS=10;
#endif

int INT = 0;
byte read_key=0;
byte KEY=0;
byte stata_reg=0;
byte ACM_BUF[41]={0};
volatile boolean BIT=0;
void Exti();
void CHeck(uint8_t *re_data);
void writeRegister(byte add, byte *data,int Length);
unsigned int readRegister(byte add, byte *data,int Length );
void setup()
{
   Serial.begin(115200); //设置波特率为115200
   attachInterrupt(INT, Exti, RISING);//设置外部中断0上升沿中断
   SPI.setBitOrder(MSBFIRST);//数据高位先传输
   //设置spi时钟频率16M/128=125K，一个一个寄存器操作，spi时钟选择较低频率
   SPI.setClockDivider(SPI_CLOCK_DIV64);
   SPI.setDataMode(SPI_MODE3);//SPI时钟极性CPOL=1;CPHA=1
   SPI.begin();//SPI引脚初始化设置SS--10;MOSI--11;MISO--12;SCK--13;
   //开所有传感器，输出频率设置成100hz（默认开所有传感器，输出50hz，不能关所有传感器）
  //即写成0x04，并不关闭所有传感器，这样也是开所有传感器，输出频率为100hz
  #ifdef spi_cs
  pinMode(CS, OUTPUT); 
  #endif
  while(!BIT);//等待GY953初始化完毕 
  delay(5);
  byte data=0x74|RegisterA;//配置GY953数据更新频率为100hz
  writeRegister(0x41,&data,1);
}
void loop() 
{
   int ROLL=0,PITCH=0,YAW=0;
  int Acc[3]={0},Gyr[3]={0},Mag[3]={0},Q[4]={0};
  float rpy[4]={0};
  byte i=0,sum=0;
  if(BIT)
  {
    readRegister(0xc1,ACM_BUF,41);
  // for(i=0;i<41;i++)
   //readRegister(0x81+i,&ACM_BUF[i],1);
    CHeck(ACM_BUF);//串口指令检查
    if(ACM_BUF[34]==0x0d)
   {
      if(stata_reg&RPY)//欧拉角输出
    {
      for(i=0;i<6;i++)
      sum+=ACM_BUF[20+i];
      if(sum==ACM_BUF[39])//校验和，减少spi通讯错误
      {
        ROLL=(ACM_BUF[20]<<8)|ACM_BUF[21];
        PITCH=(ACM_BUF[22]<<8)|ACM_BUF[23];
        YAW=(ACM_BUF[24]<<8)|ACM_BUF[25];
        rpy[0]=(float)ROLL/100;
        rpy[1]=(float)PITCH/100;
        rpy[2]=(float)YAW/100;
        Serial.print("ROLL = "); Serial.print(rpy[0]);Serial.print(',');
        Serial.print("PITCH = ");Serial.print(rpy[1]);Serial.print(',');
        Serial.print("YAW = "); Serial.println(rpy[2]); 
      }
     }
     if(stata_reg&ACC)//加数度数据输出
    {
      Acc[0]=(ACM_BUF[2]<<8)|ACM_BUF[3];//ACC_X
      Acc[1]=(ACM_BUF[4]<<8)|ACM_BUF[5];//ACC_Y
      Acc[2]=(ACM_BUF[6]<<8)|ACM_BUF[7];//ACC_Z
      Serial.print("Acc_X= "); Serial.println(Acc[0]);
      Serial.print("Acc_Y = ");Serial.println(Acc[1]);
      Serial.print("Acc_Z = "); Serial.println(Acc[2]); 
    }
    if(stata_reg&GYR)//陀螺数据输出
    {
      Gyr[0]=(ACM_BUF[8]<<8)|ACM_BUF[9];//GYR_X
      Gyr[1]=(ACM_BUF[10]<<8)|ACM_BUF[11];//GYR_Y
      Gyr[2]=(ACM_BUF[12]<<8)|ACM_BUF[13];//GYR_Z
      Serial.print("Gyr_X= "); Serial.println(Gyr[0]);
      Serial.print("Gyr_Y = ");Serial.println(Gyr[1]);
      Serial.print("Gyr_Z = "); Serial.println(Gyr[2]); 
    }
    if(stata_reg&MAG)//磁场数据输出
    {
       Mag[0]=(ACM_BUF[14]<<8)|ACM_BUF[15];//MAG_X
       Mag[1]=(ACM_BUF[16]<<8)|ACM_BUF[17];//MAG_Y
       Mag[2]=(ACM_BUF[18]<<8)|ACM_BUF[19];//MAG_Z
      Serial.print("Mag_X= "); Serial.println(Mag[0]);
      Serial.print("Mag_Y = ");Serial.println(Mag[1]);
      Serial.print("Mag_Z = "); Serial.println(Mag[2]);
     }
     if(stata_reg&Q4)//四元数数据输出
     {
        Q[0]=(ACM_BUF[26]<<8)|ACM_BUF[27];//q0
        Q[1]=(ACM_BUF[28]<<8)|ACM_BUF[29];//q1
        Q[2]=(ACM_BUF[30]<<8)|ACM_BUF[31];//q2
        Q[3]=(ACM_BUF[32]<<8)|ACM_BUF[33];//q3
      Serial.print("Q0= "); Serial.println(Q[0]);
      Serial.print("Q1 = ");Serial.println(Q[1]);
      Serial.print("Q2 = "); Serial.println(Q[2]); 
      Serial.print("Q3 = "); Serial.println(Q[3]); 
      }
   }
    BIT=0;//清数据更新标志
  }
}
void Exti()
{
 if(!BIT) 
  BIT=1;//数据更新标志
}
void CHeck(uint8_t *re_data)
{
	byte num[4]={0};
	byte data;
        byte rebuf;
        if (Serial.available() > 0)
        {
          rebuf=byte(Serial.read());//读串口缓存
          Serial.flush();
	    switch(rebuf)
	  {    
		case '1':stata_reg^=RPY;break;//输出欧拉角指令
		case '2':stata_reg^=ACC;break;//输出加速度数据指令
		case '3':stata_reg^=GYR;break;//输出陀螺仪数据指令
		case '4':stata_reg^=MAG;break;//输出磁场数据指令
		case '5':stata_reg^=Q4;break;//输出四元数数据指令
		case 'A':KEY=1;break;//模块输出频率50hz设置
		case 'B':KEY=2;break;//模块输出频率100hz设置
		case 'C':KEY=3;break;//模块输出频率200hz设置
		case 'I':KEY=4;break;//加陀校准
		case 'J':KEY=5;break;//磁场校准
		case 'K':KEY=6;break;//恢复出厂设置，即清除保存的校准数据
		case 'P':read_key=1;break;//读取校准精度\数据更新频率
		case 'Q':read_key=2;break;//读取传感器量程
		case 'U':KEY=7;break;//开启所有传感器
		case 'V':KEY=8;break;//关闭加计
		case 'W':KEY=9;break;//关闭陀螺
		case 'X':KEY=10;break;//关闭磁场
		default:KEY=0,read_key=0;break;	
	  }
        }
	switch(read_key)
	{
		case 1://发送精度
		{
                       
			num[0]=((re_data[35]>>4)&0x03);
			num[1]=((re_data[35]>>2)&0x03);
			num[2]=(re_data[35]&0x03);
                        num[3]=(re_data[0]&0x07);
			Serial.print("ACC Accuracy: "); Serial.println(num[0]);
                        Serial.print("GYR Accuracy:");Serial.println(num[1]);
                        Serial.print("MAG Accuracy: "); Serial.println(num[2]); 
                        Serial.print(" output frequency: "); Serial.println(num[3]);
			read_key=0;
                        stata_reg=0x00;
		}
			break;
		case 2://发送量程
		{
			num[0]=((re_data[34]>>4)&0x03);
			num[1]=((re_data[34]>>2)&0x03);
			num[2]=(re_data[34]&0x03);
			Serial.print("ACC Range: "); Serial.println(num[0]);
                        Serial.print("GYR Range:");Serial.println(num[1]);
                        Serial.print("MAG Range: "); Serial.println(num[2]); 
			read_key=0;
                        stata_reg=0x00;
		}	
		break;
		default:read_key=0;break;
	}
	switch(KEY)
	{
		case 1:data=0x73|RegisterA;writeRegister(0x41,&data,1);KEY=0;break;//50hz
		case 2:data=0x74|RegisterA;writeRegister(0x41,&data,1);KEY=0;break;//100hz
		case 3:data=0x75|RegisterA;writeRegister(0x41,&data,1);KEY=0;break;//200hz
		case 4:data=0x04|RegisterB;writeRegister(0x42,&data,1);KEY=0;break;//加陀校准
		case 5:data=0x08|RegisterB;writeRegister(0x42,&data,1);KEY=0;break;//磁场校准
		case 6:data=0x80|RegisterB;writeRegister(0x42,&data,1);KEY=0;break;//恢复出厂设置，即清除保存的校准数据
		case 7:data=0x73|RegisterA;writeRegister(0x41,&data,1);KEY=0;break;//开启所有传感器
		case 8:data=0x63|RegisterA;writeRegister(0x41,&data,1);KEY=0;break;//关闭加计
		case 9:data=0x53|RegisterA;writeRegister(0x41,&data,1);KEY=0;break;//关闭陀螺
		case 10:data=0x33|RegisterA;writeRegister(0x41,&data,1);KEY=0;break;//关闭磁场
	  default:KEY=0;break;
	}
}
//Read Register
unsigned int readRegister(byte add, byte *data,int Length ) {
  byte i=0;
  #ifdef spi_cs
  digitalWrite(CS, LOW);
  #else
  digitalWrite(SS, LOW);
  #endif
  // send the device the register you want to read:
  SPI.transfer(add);
  while(i<Length)
  data[i++] = SPI.transfer(0);
  // take the chip select high to de-select:
   #ifdef spi_cs
  digitalWrite(CS, HIGH);
  #else
  digitalWrite(SS, HIGH);
  #endif
}
//Write Register
void writeRegister(byte add, byte *data,int Length) {
  byte i=0;
   #ifdef spi_cs
  digitalWrite(CS, LOW);
  #else
  digitalWrite(SS, LOW);
  #endif
  SPI.transfer(add); //Send register location
  while(i<Length)
  SPI.transfer(data[i++]);  //Send value to record into register
  // take the chip select high to de-select:
   #ifdef spi_cs
  digitalWrite(CS, HIGH);
  #else
  digitalWrite(SS, HIGH);
  #endif
}

