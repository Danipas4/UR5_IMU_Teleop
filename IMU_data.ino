#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h> 
#include <Wire.h>

/*  JOYSTICK INPUTS  */
const int pinLED = 13;
const int pinJoyX = A3;
const int pinJoyY = A2;
const int pinJoyButton = 9;
int joyX=0, joyY=0; 
bool joyButton = false;

/*  MPU INPUTS  */
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t aX,aY,aZ,Tmp,gX,gY,gZ;

/*  ROS TOPIC STETUP  */
//Set up the ros node and publisher
std_msgs::String imu_msg;
ros::Publisher imu("imu", &imu_msg);
ros::NodeHandle nh;

const int RATE_TIME = 50; 
long publisher_timer, tiempo_prev;
float dt, ang_x, ang_y; 
float ang_x_prev, ang_y_prev; 
                           
void setup()
{

  // ROS node init
  nh.initNode();
  nh.advertise(imu);

  // Init MPU comunication
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MgMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // Joystick button
  pinMode(pinJoyButton , INPUT_PULLUP);
  
  Serial.begin(57600);
  publisher_timer = 0;
}



void updateFiltered()
{
   dt = (millis() - tiempo_prev) / 1000.0;
   tiempo_prev = millis();
   
   //Calcular los ángulos con acelerómetro
   float accel_ang_x = atan(aY / sqrt(pow(aX, 2) + pow(aZ, 2)))*(180.0 / 3.14);
   float accel_ang_y = atan(-aX / sqrt(pow(aY, 2) + pow(aZ, 2)))*(180.0 / 3.14);
   
   //Calcular angulo de rotación con giroscopio y filtro complementario
   ang_x = 0.98*(ang_x_prev + (gX / 131)*dt) + 0.02*accel_ang_x;
   ang_y = 0.98*(ang_y_prev + (gY / 131)*dt) + 0.02*accel_ang_y;
   ang_x_prev = ang_x;
   ang_y_prev = ang_y;
}



void loop()
{

  // Read MPU info 
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (aCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers  String AX = String(mpu6050.getacX());

  // Extract acceleration and orientation from the IMU
  aX=Wire.read()<<8|Wire.read();  // 0x3B (aCEL_XOUT_H) & 0x3C (aCEL_XOUT_L)    
  aY=Wire.read()<<8|Wire.read();  // 0x3D (aCEL_YOUT_H) & 0x3E (aCEL_YOUT_L)
  aZ=Wire.read()<<8|Wire.read();  // 0x3F (aCEL_ZOUT_H) & 0x40 (aCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gX=Wire.read()<<8|Wire.read();  // 0x43 (gRO_XOUT_H) & 0x44 (gRO_XOUT_L)
  gY=Wire.read()<<8|Wire.read();  // 0x45 (gRO_YOUT_H) & 0x46 (gRO_YOUT_L)
  gZ=Wire.read()<<8|Wire.read();  // 0x47 (gRO_ZOUT_H) & 0x48 (gRO_ZOUT_L)

  // Data filter
  updateFiltered(); 

  // Read Joystick values
  joyX = analogRead(pinJoyX);
  joyButton = digitalRead(pinJoyButton); 
  

  // Create the topic message
  String AX = String(ang_x);
  String AY = String(ang_y);
  String Joy = String(joyX);
  String But = String(joyButton);
  
  String data = "A" + AX + "B"+ AY + "C" + Joy + "D" + But + "E";
  
  int length = data.indexOf("E") +2;
  char data_final[length+1];
  data.toCharArray(data_final, length+1);

  Serial.println(data);

  // Publish to the topic every RATE_TIME ms, also check the joystick state
  if (millis() > publisher_timer) {
    // step 1: request reading from sensor
    imu_msg.data = data_final;
    imu.publish(&imu_msg);
    publisher_timer = millis() + RATE_TIME; //publish at 'RATE_TIME' frecuency
    nh.spinOnce();
  }
 
}
