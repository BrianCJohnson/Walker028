//========================================================
// com.h
// com definitions for all functions
//========================================================

#ifndef com_h
  #define com_h
  #include "Arduino.h"

//  #include "servo.h"
//  #include "body.h"
//  #include "legs.h"
//  #include "mode.h"

  const uint8_t NUM_LEGS = 4;
  const uint8_t NUM_JOINTS_LEG = 3;
  const uint8_t XYZ = 3;

  const boolean signed_distance = false;
  
  struct com_move_param_s {
    float dist;
    float v_max;
    float a_max;
    float v_beg;
    float v_end;
  };
  typedef struct com_move_param_s com_move_param_t;

  struct com_point_s {
    float dist;
    float time;
  };
  typedef struct com_point_s com_point_t;
  
  struct com_move_data_s {
    com_move_param_t param;
    float com_point[7];
  };
  typedef struct com_move_data_s com_move_data_t;


  const static float COM_ZERO = 0.00001;

//  uint8_t my_mode;

  void com_free_mem(void);
  void com_indent(int8_t indent);
  void com_print_beg(const char *routine, int8_t indent);
  void com_print_beg2(const char *routine, const char *text, int8_t indent);
  void com_print_end(const char *routine, int8_t indent);
  void com_sign_mag(float number, int8_t *sign, float *mag);
  void com_err_msg_int(const char routine[], const char err_msg[]);
  void com_err_msg(const char routine[], const char err_msg[]);
//  void com_err_msg(const char routine[], const char err_msg[], uint8_t uint8_t_value);
//  void com_err_msg(const char routine[], const char err_msg[], float float_value);
  void com_err_msg(const char routine[], const char err_msg[], uint8_t uint8_t_value, float float_value);

#endif
