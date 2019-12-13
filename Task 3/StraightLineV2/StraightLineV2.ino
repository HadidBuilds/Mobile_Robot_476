#include <Romi32U4.h>

Romi32U4Encoders encoders;
Romi32U4Motors motors;
Romi32U4ButtonA buttonA;

void setup() {

}

void loop() {
   // Parameters
   const int drive_distance = 200;   // cm
   const int motor_power = 50;      // 0-255
   const int motor_offset = 5;       // Diff. when driving straight
   const int wheel_d = 70;           // Wheel diameter (mm)  
   const float wheel_c = PI * wheel_d; // Wheel circumference (mm)
   const int counts_per_rev = 1440;   // (4 pairs N-S) * (120:1 gearbox) * (2 falling/rising edges) = 384

   int16_t enc_r = 0;
   int16_t enc_l = 0;

   int16_t enc_r_prev = enc_r;
   int16_t enc_l_prev = enc_l;

   // Set initial motor power
   int power_l = motor_power;
   int power_r = motor_power;

   // Used to determine which way to turn to adjust
   unsigned long diff_l;
   unsigned long diff_r;

   bool flag = true;

   float num_rev = (drive_distance * 10) / wheel_c;  // Convert to mm
   unsigned long target_count = num_rev * counts_per_rev;

    if(buttonA.isPressed())
    {
    while ( (enc_l < target_count) && (enc_r < target_count) ) {
      enc_r = encoders.getCountsRight();
      enc_l = encoders.getCountsLeft();
    
      // Print out current number of ticks
      Serial.print(enc_l);
      Serial.print("\t");
      Serial.println(enc_r);

      // Drive
      motors.setSpeeds(power_l, power_r);

      // Number of ticks counted since last time
      diff_l = enc_l - enc_l_prev;
      diff_r = enc_r - enc_r_prev;

      // Store current tick counter for next time
      enc_l_prev = enc_l;
      enc_r_prev = enc_r;

      // If left is faster, slow it down and speed up right
      if ( diff_l > diff_r ) {
        power_l -= motor_offset;
        power_r += motor_offset;
      }

      // If right is faster, slow it down and speed up left
      if ( diff_l < diff_r ) {
        power_l += motor_offset;
        power_r -= motor_offset;
      }
    }
    motors.setSpeeds(0, 0);
    enc_l = encoders.getCountsAndResetLeft();
    enc_r = encoders.getCountsAndResetRight();
    }

}
