#include <I2Cdev.h>
#include "MPU6050.h"
#include <Wire.h>
#include <SoftwareSerial.h>
MPU6050 accelgyro;
//---------------
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define Gry_offset -20     // 陀螺仪偏移量
#define Gyr_Gain 0.00763358    //对应的1G
#define pi 3.14159


/*********** PID控制器参数 *********/
float kp, ki, kd,kpp; 
float angleA,omega;
float LOutput,ROutput;   
int direction_L=4;
int motor_L=5;
int motor_R=6;
int direction_R=7;

//--------------------------------------
float LSpeed_Need=0.0,RSpeed_Need=0.0;
//char data;
int data,adata;

unsigned long now;
unsigned long preTime = 0;
float SampleTime = 0.08;  //-------------------互补滤波+PID 采样时间0.08 s
unsigned long lastTime;
float Input, Output, Setpoint;
float errSum,dErr,error,lastErr,f_angle;
int timeChange; 
void setup()
{
    //===============motor Init===========
    pinMode(motor_L,OUTPUT);
    pinMode(direction_L,OUTPUT);
    pinMode(motor_R,OUTPUT); 
    pinMode(direction_R,OUTPUT); 
  //===============end initial===========
  Wire.begin();
  Serial.begin(9600);
  Serial.println("start");
  accelgyro.initialize();
 

}
 char cmd;
void loop()
{
 cmd=Serial.read();
  if(cmd!=0){
   Serial.print(cmd);
  }
 accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
 angleA= atan2(ay , az) * 180 / pi-0.4;   // 根据加速度分量得到的角度(degree)加0.5偏移量
 //180度至0至-180（360度）取0度为坚直时中立点 因为坚直时有偏差，所以加0.5....
 omega=  Gyr_Gain * (gx +  Gry_offset); // 当前角速度(degree/s)
if (abs(angleA)<45) {    // 角度小于45度 运行程序
   PID_Filter();
   motor_out();
}else{
  analogWrite(motor_L, 0); 
  analogWrite(motor_R, 0);   
}    
}
void PID_Filter()
{
     kp = 250*0.03; //取0~1024*  (这些值是小车调试后得出，请按自己小车调试后修改)
     ki = 150*0.0002;//取0~1024* .........
     kd = 200*1;  //取0~1024* .........
 //kpp = analogRead(11)*0.005; 
    //------------------互补滤波 ------------------------
   unsigned long now = millis();                           // 当前时间(ms)
    float dt = (now - preTime) / 1000.0;                    // 微分时间(ms)
    preTime = now;  
    float K = 0.8;                    
    float A = K / (K + dt);                    
   f_angle = A * (f_angle + omega * dt) + (1-A) * angleA;  // 互补滤波算法 
//----------------------------PID控制器 ------------------------------ 
  //now = millis();
   timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
     Setpoint=LSpeed_Need;// =0，(+ -值使电机前进或后退)
        Input =f_angle;
      error = Setpoint- Input;
      errSum += error* timeChange;
      dErr = (error - lastErr)/ timeChange;
 //PID Output
      Output = kp * error + ki * errSum + kd * dErr;
      LOutput=Output+RSpeed_Need;//左电机
      ROutput=Output-RSpeed_Need;//右电机
      lastErr = error;
      lastTime = now; 
   }
}
void motor_out(){
   if(LOutput>0.1)//左电机-------或者取0
{
  digitalWrite(direction_L, LOW);//后  
   
}
  else if(LOutput<-0.1)//-------或者取0
{
  
  digitalWrite(direction_L, HIGH);//后
}
  
if(ROutput>0.1)//右电机--------或者取0
{
 digitalWrite(direction_R, HIGH);//后
}
  else if(ROutput<-0.1)//-------或者取0
{
  digitalWrite(direction_R, LOW);//后
  
}
    analogWrite(motor_L,min(255,abs(LOutput)+25)); //PWM调速a==0-255
    analogWrite(motor_R,min(255,abs(Output)+25)); //PWM调速a==0-255
}
void bluetoothCMD(){
  while(Serial.available())
   {
     char bluetooth=Serial.read();        
    }
    
}

