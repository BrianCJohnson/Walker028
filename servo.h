//========================================================
// servo.h
// definitions for servo functions
//========================================================

#ifndef servo_h
  #define servo_h
  #include "Arduino.h"
  #include "com.h"

  //const float servo_pi =  3.141593;
  const float SERVO_HALF_PI =  1.570796;
  const float SERVO_QUAR_PI =  0.785398;
  const float SERVO_MAX_PIVOT =  SERVO_QUAR_PI;
  const float SERVO_MAX_ANGLE[NUM_LEGS][NUM_JOINTS_LEG] = {
    { SERVO_MAX_PIVOT,  SERVO_HALF_PI,  SERVO_HALF_PI},
    { SERVO_MAX_PIVOT,  SERVO_HALF_PI,  SERVO_HALF_PI},
    { SERVO_MAX_PIVOT,  SERVO_HALF_PI,  SERVO_HALF_PI},
    { SERVO_MAX_PIVOT,  SERVO_HALF_PI,  SERVO_HALF_PI}}; // radians
  const float SERVO_MIN_ANGLE[NUM_LEGS][NUM_JOINTS_LEG] = {
    {-SERVO_MAX_PIVOT, -SERVO_QUAR_PI, -SERVO_QUAR_PI},
    {-SERVO_MAX_PIVOT, -SERVO_QUAR_PI, -SERVO_QUAR_PI},
    {-SERVO_MAX_PIVOT, -SERVO_QUAR_PI, -SERVO_QUAR_PI},
    {-SERVO_MAX_PIVOT, -SERVO_QUAR_PI, -SERVO_QUAR_PI}}; // radians

  void servo_setup();
  uint16_t servo_get_target(uint8_t servo);
  void servo_set_target(uint8_t servo, uint16_t target);
  void servo_set_targets(uint16_t target[]);
  void servo_print_target(void);
  void servo_print_angle(void);
  void servo_set_angle(uint8_t servo, float radian, uint8_t indent);
  void servo_set_angles(float radian[NUM_LEGS][NUM_JOINTS_LEG], uint8_t indent);
//  void servo_test();
//  void move_to(boolean positive_angle, boolean going_down, uint16_t i, uint16_t total_steps);
//  void servo_math_test();
  
#endif
