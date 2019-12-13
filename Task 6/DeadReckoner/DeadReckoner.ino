#include <Romi32U4.h>
#include <Romi32U4Odometer.h>
#include <TimedPID.h>
#include <Wire.h> 
#include <LSM6.h>
#include <math.h>

// Define robot geometrical properties
// Distance travelled per encoder tick
const float tickDist = .15271631; //2pi(R)/1440
// Width between the two wheels
const float track = 141;

// Define motor PID gains
const float Kp = 1.0; // 2.0
const float Ki = 6.0; // 5.0
const float Kd = 0.01;

// Define motors max command
const float motorsMaxCommand = 400;

// Define motor trims in 1/40
const int leftTrim = 40;
const int rightTrim = 40;

// Define speed variables for acceleration control
int lastSpeedCmdLeft = 0;
int lastSpeedCmdRight = 0;

// Define maximum speed command change per time step
const int accelMax = 20;

// Define different objects from RasPiBot202V2 library
Romi32U4Encoders encoders;
Odometer odometer(tickDist, track);
TimedPID PIDLeft(Kp, Ki, Kd);
TimedPID PIDRight(Kp, Ki, Kd);

// Define objects from AStar32U4 library
Romi32U4Motors motors;
Romi32U4ButtonA btnA;
Romi32U4ButtonB btnB;
Romi32U4ButtonC btnC;

// Define variables used for target speed generation and time step calculation
unsigned long initialTime;
unsigned long lastTime;
bool moveRomi = false;


//tolerance for movements
float tolForDist = 20; 
float tolForAng = 0.0348*(180/3.14); 

LSM6 imu; 
void setup() {
  Serial.begin(9600);

  // Set encoders directions
  //encoders.flipDirection(false, true);

  // Set PID controllers command range
  PIDLeft.setCmdRange(-motorsMaxCommand, motorsMaxCommand);
  PIDRight.setCmdRange(-motorsMaxCommand, motorsMaxCommand);

  // Initialize time variables
  initialTime = micros();
  lastTime = initialTime;
  delay(10);
}

void loop() {

  // Calculate the time step between passes of the main loop
  // Get current time
  unsigned long currentTime = micros();
  // Calculate time step in seconds (micro seconds / 1 000 000)
  //float timeStep = float(currentTime - lastTime) / 1E6;
  // Store current time as last time for next pass of the loop
  lastTime = currentTime;

  // Define the target speed as function of time
  //float targetSpeed = 0;

  //flag to start robot movement
  //bool moveRomi = false;


  // Print actual motor speeds from odometer
  //Serial.print(odometer.getSpeedRight());
  //Serial.print("\t");
  //Serial.print(odometer.getSpeedLeft());
  
  
  // Press button A to initiate Romi movement 
  if (btnA.isPressed())
  {
    moveRomi = true;
  }
  if (btnC.isPressed())
  {
    moveRomi = false;
  }
  
  if (moveRomi == true)
  {    
    // Get encoder counts
    int left = encoders.getCountsLeft();
    int right = encoders.getCountsRight();
    // Update odometer
    odometer.update(left, right);
    int motorSpeed = 0; 
    while(!(odometer.getX() < 609.6 + tolForDist && odometer.getX() > 609.6 - tolForDist))
    {
        motorSpeed = motorSpeed + 1;
        if (motorSpeed >= 130)
        {
          motorSpeed = 130; 
        }
        setMotorSpeeds(motorSpeed, motorSpeed);
        left = encoders.getCountsLeft();
        right = encoders.getCountsRight();
        odometer.update(left, right);
        Serial.print(odometer.getX());
        Serial.print("\t");
        Serial.println(odometer.getY());
    }
    motorSpeed = 0;
    setMotorSpeeds(0,0); 
    delay(5);
    lastTime = micros();
    while(!(odometer.getPhi()*(180/3.14) < 90 + tolForAng && odometer.getPhi()*(180/3.14) > 90 - tolForAng))
    {
      currentTime = micros();
      motorSpeed = motorSpeed + 1;
      if (motorSpeed >= 130)
      {
        motorSpeed = 130; 
      }
      
      setMotorSpeeds(-motorSpeed,motorSpeed);
      left = encoders.getCountsLeft();
      right = encoders.getCountsRight();
      odometer.update(left, right);     
      Serial.println(odometer.getPhi()*(180/3.14));
    }
    motorSpeed = 0;
    setMotorSpeeds(0,0); 
    delay(5);
    while(!(odometer.getY() < 609.6 + tolForDist && odometer.getY() > 609.6 - tolForDist))
    {
        motorSpeed = motorSpeed + 1;
        if (motorSpeed >= 130)
        {
          motorSpeed = 130; 
        }
        setMotorSpeeds(motorSpeed, motorSpeed);
        left = encoders.getCountsLeft();
        right = encoders.getCountsRight();
        odometer.update(left, right);
        Serial.print(odometer.getX());
        Serial.print("\t");
        Serial.println(odometer.getY());
    }
    motorSpeed = 0;
    setMotorSpeeds(0,0);    
    delay(5);
    while(!(odometer.getPhi()*(180/3.14) < 180 + tolForAng && odometer.getPhi()*(180/3.14) > 180 - tolForAng))
    {
      motorSpeed = motorSpeed + 1;
      if (motorSpeed >= 130)
      {
        motorSpeed = 130; 
      }
      setMotorSpeeds(-motorSpeed,motorSpeed);
      left = encoders.getCountsLeft();
      right = encoders.getCountsRight();
      odometer.update(left, right);
      Serial.println(odometer.getPhi()*(180/3.14));
    }
    motorSpeed = 0;
    setMotorSpeeds(0,0);  
    delay(5);
    while(!(odometer.getX() < 0 + tolForDist && odometer.getX() > 0 - tolForDist))
    {
      motorSpeed = motorSpeed + 1;
      if (motorSpeed >= 130)
      {
        motorSpeed = 130; 
      }
      setMotorSpeeds(motorSpeed, motorSpeed);
      left = encoders.getCountsLeft();
      right = encoders.getCountsRight();
      odometer.update(left, right);
      Serial.print(odometer.getX());
      Serial.print("\t");
      Serial.println(odometer.getY());
    }
    motorSpeed = 0;
    setMotorSpeeds(0,0);
    delay(5);
    while(!(odometer.getPhi()*(180/3.14) < 270 + tolForAng && odometer.getPhi()*(180/3.14) > 270 - tolForAng))
    {
      motorSpeed = motorSpeed + 1;
      if (motorSpeed >= 130)
      {
        motorSpeed = 130; 
      }
      setMotorSpeeds(-motorSpeed,motorSpeed);
      left = encoders.getCountsLeft();
      right = encoders.getCountsRight();
      odometer.update(left, right);
      Serial.println(odometer.getPhi()*(180/3.14));
    }
    motorSpeed = 0;
    setMotorSpeeds(0,0);    
    delay(5);
    while(!(odometer.getY() < 0 + tolForDist && odometer.getX() > 0 - tolForDist))
    {
      motorSpeed = motorSpeed + 1;
      if (motorSpeed >= 130)
      {
        motorSpeed = 130; 
      }
      setMotorSpeeds(motorSpeed, motorSpeed);
      left = encoders.getCountsLeft();
      right = encoders.getCountsRight();
      odometer.update(left, right);
      Serial.print(odometer.getX());
      Serial.print("\t");
      Serial.println(odometer.getY());
    }
    motorSpeed = 0;
    setMotorSpeeds(0,0);    
    delay(5);
    while(!(odometer.getPhi()*(180/3.14) < 360 + tolForAng && odometer.getPhi()*(180/3.14) > 360 - tolForAng))
    {
      motorSpeed = motorSpeed + 1;
      if (motorSpeed >= 130)
      {
        motorSpeed = 130; 
      }
      setMotorSpeeds(-motorSpeed,motorSpeed);
      left = encoders.getCountsLeft();
      right = encoders.getCountsRight();
      odometer.update(left, right);
      Serial.println(odometer.getPhi()*(180/3.14));
    }
    motorSpeed = 0;
    setMotorSpeeds(0,0);    
    delay(10);
    moveRomi = false;
    odometer.reset();
  }
  if (moveRomi == false)
  {
    setMotorSpeeds(0, 0);
    //Serial.print("\t");
    //Serial.println(0);
  }

  // Ensure a constant time step of the main loop (10 milliseconds)
  while (micros() - currentTime < 1000)
  {
    // Wait until time step is reached
  }
}

// Sets the motor speeds using PID controllers
void setMotorSpeeds(int speedLeft, int speedRight)
{
  // Read odometer counts
  int countsLeft = encoders.getCountsLeft();
  int countsRight = encoders.getCountsRight();

  // Update odometer
  odometer.update(countsLeft, countsRight);

  // get speed command from PID controllers
  int speedCmdLeft = PIDLeft.getCmdAutoStep(speedLeft, odometer.getSpeedLeft());
  int speedCmdRight = PIDRight.getCmdAutoStep(speedRight, odometer.getSpeedRight());

  // Handle speed commands

  // Control maximum acceleration
  if (speedCmdLeft - lastSpeedCmdLeft > accelMax)
  {
    speedCmdLeft = lastSpeedCmdLeft + accelMax;
  }
  if (speedCmdLeft - lastSpeedCmdLeft < -accelMax)
  {
    speedCmdLeft = lastSpeedCmdLeft - accelMax;
  }
  if (speedCmdRight - lastSpeedCmdRight > accelMax)
  {
    speedCmdRight = lastSpeedCmdRight + accelMax;
  }
  if (speedCmdRight - lastSpeedCmdRight < -accelMax)
  {
    speedCmdRight = lastSpeedCmdRight - accelMax;
  }

  // Stop immediately if target speed is zero
  if (speedLeft == 0)
  {
    speedCmdLeft = 0;
    PIDLeft.reset();
  }
  if (speedRight == 0)
  {
    speedCmdRight = 0;
    PIDRight.reset();
  }

  // Set motor speeds
  motors.setSpeeds(speedCmdLeft * leftTrim / 40, speedCmdRight * rightTrim / 40);

  lastSpeedCmdLeft = speedCmdLeft;
  lastSpeedCmdRight = speedCmdRight;
}
