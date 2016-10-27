//========================================================
// legs.h
// definitions for leg functions
//========================================================

#ifndef legs_h
  #define legs_h
  #include "Arduino.h"
  #include "com.h"

  struct legs_move_point_s {
    int8_t dir;
    float v_max;
    float a_max;
    uint32_t start_time; // unsigned long ;
  };
  typedef struct mode_phase_s mode_phase_t;

  const static float LEGS_MIN_MOVE_TIME = 0.1; // set minimum move time to 1/10 sec

  // values of robot leg geometry
  const static float LEGS_CENTER_2_PIVOT_XY = 75.0;
  const static float LEGS_PIVOT_2_HIP = 45.5;
  const static float LEGS_HIP_2_KNEE = 100.0;
  const static float LEGS_KNEE_2_KNEECAP = 33.0;
  const static float LEGS_KNEECAP_2_FOOT = 241.0;
  //const static float LEGS_KNEE_2_FOOT = 243.24884;
  const static float LEGS_KNEE_2_FOOT = sqrt(LEGS_KNEE_2_KNEECAP*LEGS_KNEE_2_KNEECAP + LEGS_KNEECAP_2_FOOT*LEGS_KNEECAP_2_FOOT);
  const static float LEGS_HIP_2_KNEE_SQR = LEGS_HIP_2_KNEE * LEGS_HIP_2_KNEE;
  const static float LEGS_KNEE_2_FOOT_SQR = LEGS_KNEE_2_FOOT * LEGS_KNEE_2_FOOT;
  const static float LEGS_HIP_2_KNEE_SQR_PLUS_KNEE2FOOT_SQR = LEGS_HIP_2_KNEE_SQR + LEGS_KNEE_2_FOOT_SQR;
  const static float LEGS_TWO_HIP2KNEE_TIMES_KNEE2FOOT = 2.0 * LEGS_HIP_2_KNEE * LEGS_KNEE_2_FOOT;
  //const static float LEGS_PI_MINUS_KNEE_ANGLE_OFFSET =  1.70688;
  const static float LEGS_PI_MINUS_KNEE_ANGLE_OFFSET = PI - atan2(LEGS_KNEECAP_2_FOOT, LEGS_KNEE_2_KNEECAP);
  const static float LEGS_HIP_2_KNEE_SQR_MINUS_KNEE2FOOT_SQR = LEGS_HIP_2_KNEE_SQR - LEGS_KNEE_2_FOOT_SQR;
  const static float LEGS_TWO_HIP_2_KNEE = 2.0 * LEGS_HIP_2_KNEE;
  
  const static uint8_t LEGS_PARAM_DIST = 0; // displacement, may be positive or negative initially, will be converted to distance = abs(displacement)
  const static uint8_t LEGS_PARAM_DIR = 1; // will be set to sign of displacement
  const static uint8_t LEGS_PARAM_UPDN = 2; // -1.0 indicates it's not an up_down move, 1.0 indicates an up_down move (only expected for Z)
  const static uint8_t LEGS_PARAM_V_MAX = 3; // the maximum velocity (positive number)
  const static uint8_t LEGS_PARAM_A_MAX = 4; // the maximum acceleration (positive number)
  const static uint8_t LEGS_PARAM_V_BEG = 5; // velocity at beginning, may be negative
  const static uint8_t LEGS_PARAM_V_END = 6; // velocity at end, may be negative
  const static uint8_t LEGS_PARAM_NUM = 7; // number of move part parameters
  const static String LEGS_PARAM_NAME[LEGS_PARAM_NUM] = {"_DIST", "__DIR", "_UPDN", "V_MAX", "A_MAX", "V_BEG", "V_END"};

  const static uint8_t LEGS_MOVE_END = 0; // start of the end (after the end)
  const static uint8_t LEGS_MOVE_DEC2 = 1; // start of the (2nd) deceleration
  const static uint8_t LEGS_MOVE_CV2 = 2; // start of the (2nd) constant velocity
  const static uint8_t LEGS_MOVE_ACC2 = 3; // start of the (2nd) acceleration
  const static uint8_t LEGS_MOVE_WAIT = 4; // start of the wait
  const static uint8_t LEGS_MOVE_DEC1 = 5; // start of the deceleration
  const static uint8_t LEGS_MOVE_CV1 = 6; // start of the constant velocity
  const static uint8_t LEGS_MOVE_POINT_NUM = 7;
  const static String LEGS_MOVE_NAME[LEGS_MOVE_POINT_NUM] = {"_END", "DEC2", "_CV2", "ACC2", "WAIT", "DEC1", "_CV1"};
  
  const static uint8_t LEGS_MOVE_TIME = 0;
  const static uint8_t LEGS_MOVE_DIST = 1;
  const static uint8_t LEGS_MOVE_TD_NUM = 2;
  const static String LEGS_MOVE_TD_NAME[LEGS_MOVE_TD_NUM] = {"TIME", "DIST"};

  const float C90 = cos(PI*90.0/180.0); // absolute angle 105 degrees
  const float S90 = sin(PI*90.0/180.0); // absolute angle 105 degrees
  const float C45 = cos(PI*45.0/180.0); // same as sqrt(0.5)

  const float READY_Z_ADJ = -00.0; // negative values lower the body height, must be between +14 to -96
  //                                         =  (center_2_pivot         + pivot_2_hip           + hip_2_knee          + knee_2_kneecap           - kneecap_2_foot     )
  const static float LEGS_XYZ_RETRACTED[XYZ] = {(LEGS_CENTER_2_PIVOT_XY + C90*LEGS_PIVOT_2_HIP  + 0.0                 + C90*LEGS_KNEE_2_KNEECAP  - 0.0                ),
                                                (LEGS_CENTER_2_PIVOT_XY + S90*LEGS_PIVOT_2_HIP  + 0.0                 + S90*LEGS_KNEE_2_KNEECAP  - 0.0                ),
                                                (0.0                    + 0.0                   + LEGS_HIP_2_KNEE     + 0.0                      - LEGS_KNEECAP_2_FOOT)};
                                                
  //                                         =  (center_2_pivot         + pivot_2_hip           + hip_2_knee          + knee_2_kneecap           - kneecap_2_foot     )
  const static float LEGS_XYZ_READY[XYZ]     = {(LEGS_CENTER_2_PIVOT_XY + C45*LEGS_PIVOT_2_HIP  + C45*LEGS_HIP_2_KNEE + C45*LEGS_KNEE_2_KNEECAP  - 0.0                ),
                                                (LEGS_CENTER_2_PIVOT_XY + C45*LEGS_PIVOT_2_HIP  + C45*LEGS_HIP_2_KNEE + C45*LEGS_KNEE_2_KNEECAP  - 0.0                ),
                                                (0.0                    + 0.0                   + 0.0                 + 0.0                      - (LEGS_KNEECAP_2_FOOT + READY_Z_ADJ))};

  const float LEGS_FOOT_XYZ_SIGNS[NUM_LEGS][XYZ] = {{ 1.0, 1.0, 1.0},
                                                    { 1.0,-1.0, 1.0},
                                                    {-1.0,-1.0, 1.0},
                                                    {-1.0, 1.0, 1.0}}; // feet are indexed clockwise from upper right

//  float legs_foot_xyz_retracted[NUM_LEGS][XYZ]; // calculated at setup from above
//  float legs_foot_xyz_ready[NUM_LEGS][XYZ]; // calculated at setup from above

  const float LEGS_ANG_VA_MAX[XYZ][2] = {{0.5, 1.5},
                                         {0.5, 1.5},
                                         {0.5, 1.5}}; // angular velocity and acceleration max

  const float LEGS_XYZ_VA_MAX[XYZ][2] = {{1000.0, 4000.0},
                                         {1000.0, 4000.0},
                                         {1000.0, 2000.0}}; // linear velocity and acceleration max
//  const float LEGS_XYZ_VA_MAX[XYZ][2] = {{640.0, 320.0},
//                                         {640.0, 320.0},
//                                         {640.0, 320.0}}; // linear velocity and acceleration max
//  const float LEGS_XYZ_VA_MAX[XYZ][2] = {{320.0, 160.0},
//                                         {320.0, 160.0},
//                                         {320.0, 160.0}}; // linear velocity and acceleration max
//  const float LEGS_XYZ_VA_MAX[XYZ][2] = {{160.0, 80.0},
//                                         {160.0, 80.0},
//                                         {160.0, 80.0}}; // linear velocity and acceleration max

  void legs_setup(int8_t indent);
  void legs_print_values(String value_name, float value[NUM_LEGS][XYZ], int8_t indent);
  void legs_compute_retracted_and_ready(int8_t indent);
//  void legs_update(void);
  void legs_position_tests(void);
  void legs_position(float the_time, float parameters[XYZ][LEGS_PARAM_NUM], float move_points[XYZ][LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM], float legs_position[3], int8_t indent);
  float legs_distance(float the_time, float parameters[LEGS_PARAM_NUM], float move_point[][LEGS_MOVE_TD_NUM], int8_t indent);
  void legs_print_parameters(float parameters[XYZ][LEGS_PARAM_NUM], int8_t indent);
  void legs_print_move_points(float move_points[XYZ][LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM], int8_t indent);
  void legs_coor_move_points(float &target_time, float parameters[XYZ][LEGS_PARAM_NUM], float move_points[XYZ][LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM], int8_t indent);
  void legs_move_point(float &target_time, float parameters[LEGS_PARAM_NUM], float move_point[3][LEGS_MOVE_TD_NUM], int8_t indent);
  void legs_move_point_for_updn(float move_point[LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM], int8_t indent);
  void legs_move_point_scale_a_max(float parameters[LEGS_PARAM_NUM], float move_point[3][LEGS_MOVE_TD_NUM], float t, int8_t indent);
  void legs_angles(float foot_xyz[NUM_LEGS][XYZ], float angle_phk[NUM_LEGS][NUM_JOINTS_LEG], int8_t indent);
  void leg_angles(uint8_t leg, float foot_xyz[XYZ], float angle[NUM_JOINTS_LEG], int8_t indent);
//  boolean legs_change_sequence(uint8_t new_sequence); // change sequence
//  uint8_t legs_sequence(void); // returns the current legs_sequence
//  boolean legs_folded(void); // returns whether the legs are folded
//  boolean legs_unfolded(void); // returns whether the legs are unfolded
//  boolean legs_walking(void); // returns whether the legs are unfolded
#endif
