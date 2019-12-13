// This example reads the raw values from the LSM6DS33
// accelerometer and gyro and prints those raw values to the
// serial monitor.
//
// The accelerometer readings can be converted to units of g
// using the conversion factors specified in the "Mechanical
// characteristics" table in the LSM6DS33 datasheet.  We use a
// full scale (FS) setting of +/- 16 g, so the conversion factor
// is 0.488 mg/LSB (least-significant bit).  A raw reading of
// 2048 would correspond to 1 g.
//
// The gyro readings can be converted to degrees per second (dps)
// using the "Mechanical characteristics" table in the LSM6DS33
// datasheet.  We use a full scale (FS) of +/- 1000 dps so the
// conversion factor is 35 mdps/LSB.  A raw reading of 2571
// would correspond to 90 dps.
//
// To run this sketch, you will need to install the LSM6 library:
//
// https://github.com/pololu/lsm6-arduino


#include <ros.h> 
#include <std_msgs/Float32.h> 
#include <Wire.h> 
#include <Romi32U4.h>
#include <LSM6.h>

#define USE_USB0 

LSM6 imu1;
float AX_out, AY_out, AZ_out, GX_out, GY_out, GZ_out; 
float aXOffset, aYOffset, aZOffset, gXOffset, gYOffset, gZOffset;
int calibrationIterations = 10; 
int oneGAccel = 2048;
char report[120];

//Set up the ros node and publisher 
std_msgs::Float32 imu_msg; 
ros::Publisher imu("imu", &imu_msg);
ros::NodeHandle nh; 

void setup()
{
  nh.initNode();
  nh.advertise(imu);
  
  Wire.begin();

  if (!imu1.init())
  {
    // Failed to detect the LSM6.
    ledRed(1);
    while(1)
    {
      Serial.println(F("Failed to detect the LSM6."));
      delay(100);
    }
  }

  imu1.enableDefault();

  // Set the gyro full scale to 1000 dps because the default
  // value is too low, and leave the other settings the same.
  imu1.writeReg(LSM6::CTRL2_G, 0b10001000);

  // Set the accelerometer full scale to 16 g because the default
  // value is too low, and leave the other settings the same.
  imu1.writeReg(LSM6::CTRL1_XL, 0b10000100);

  //Calibration for accelerometer and gyro
  int xTotal = 0;
  int yTotal = 0;
  int zTotal = 0;
  int xGTotal = 0;
  int yGTotal = 0;
  int zGTotal = 0;  
  delay(100);
  for (int i = 0; i < calibrationIterations; i++)
  {
    imu1.read();
    xTotal = xTotal + imu1.a.x;
    yTotal = yTotal + imu1.a.y;
    zTotal = zTotal + imu1.a.z;
    xGTotal = xGTotal + imu1.g.x;
    yGTotal = yGTotal + imu1.g.y;
    zGTotal = zGTotal + imu1.g.z;
    delay(100);
  }
  aXOffset = xTotal/(calibrationIterations); 
  aYOffset = yTotal /(calibrationIterations);
  aZOffset = zTotal /(calibrationIterations);
  gXOffset = xGTotal/(calibrationIterations); 
  gYOffset = yGTotal /(calibrationIterations);
  gZOffset = zGTotal /(calibrationIterations);
}
long publisher_timer; 
void loop()
{
  imu1.read();
  int16_t ax = imu1.a.x - aXOffset;
  AX_out = ax*0.000488;
  int16_t ay = imu1.a.y - aYOffset;
  AY_out = ay*0.000488;
  int16_t az = imu1.a.z - aZOffset;
  AZ_out = az*0.000488;

  
  int16_t gx = imu1.g.x - gXOffset;
  GX_out = gx*0.035;
  int16_t gy = imu1.g.y - gYOffset;
  GY_out = gy*0.035;
  int16_t gz = imu1.g.z - gZOffset;  
  GZ_out = gz*0.035;
  
  Serial.println(AX_out);
  if (millis() > publisher_timer) {
    //step 1: request reading from sensor) 
    imu_msg.data = AX_out;
    imu.publish(&imu_msg);
    //imu_msg.data = AY_out;
    //imu.publish(&imu_msg);
    //imu_msg.data = AZ_out;
    //imu.publish(&imu_msg);
    publisher_timer = millis() + 10;//publish ten times a second
    nh.spinOnce();
  }


}
