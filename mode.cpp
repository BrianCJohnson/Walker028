//========================================================
// mode.cpp
// mode variable and functions
// =======================================================

#include "Arduino.h"
#include "debug.h"
#include "com.h"
#include "mode.h"
#include "sbus.h"
#include "servo.h"
#include "legs.h"

mode_phase_t mode_phase; // mode and phase
mode_values_t mode_values;  // commands from radio [fold, vx, vy, vt, height, angle]
mode_seq_t mode_seq[MODE_NUM][MODE_PHASE_MAX_NUM]; // mode phase sequences (end of move xyz position and velocities)
float move_part_points[MODE_SEQ_PART_NUM][XYZ][LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM]; // move points for moving parts
float move_part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM]; // parameters for each of the part move
float move_part_xyz[MODE_PART_NUM][XYZ]; // xyz coordinates of each part (each leg and the body) or xyz angle of BDY_A
float move_part_beg_xyz[MODE_PART_NUM][XYZ]; // xyz coordinates of each part at the beginning of the current move seq (each leg and the body) or xyz angle of BDY_A
uint8_t move_part_activity[MODE_PART_NUM]; // updated between move sequences, indicates the activity and type of each part (the part type: MODE_PART_TYPE_ & {"SUPPORT_LEG", "ACTIVE_LEG", "STATIC_BODY", "ACTIVE_BODY"})
uint8_t move_part_seq_part[MODE_PART_NUM]; // updated between move sequences, indicates which seq_part (if any) is associated with the part
uint8_t gmode_next_mode; // may be none or a commanded mode, set in mode_check_new_mode
uint8_t gmode_next_dir; // used in conjunction with mode_next_mode above, also set in mode_check_new_mode
//float move_part_v_end


//========================================================
// mode_setup()
// initialize mode_phase()
//========================================================
void mode_setup(int8_t indent) {
  const static char routine[] = "mode_setup";
  LOCAL_DEBUG_ENABLED
  if (local_debug) DEBUG_PRINT_BEG(routine, indent);
  mode_phase.alternate = 0; // need to initialize this once only?
  mode_create_sequences(indent+1); // create the sequences that will be used over and over
  mode_initialize_parts(indent+1); // sets all of the move_part_xyz[MODE_PART_NUM][XYZ] and move_part_beg_xyz[MODE_PART_NUM][XYZ]
  uint8_t new_phase = 0;
  mode_set(MODE_FOLDED, MODE_DIR_NONE, new_phase, indent+1); // do this after everything else is set up
  mode_phase.start_time = millis();
  //mode_phase.end_time = mode_phase.start_time + 100; // give it 1/10 sec 
  mode_phase.end_time = mode_phase.start_time; 
  if (local_debug) DEBUG_PRINT_END(routine, indent);
} // end mode_setup


//========================================================
// mode_display_position
//========================================================
void mode_display_position(void){
  Serial.printf("%7.2f", float(millis())/1000.0);
  Serial.printf("\t%s", MODE_NAME[mode_phase.mode].c_str());
  uint8_t len = strlen(MODE_NAME[mode_phase.mode].c_str());
  for(uint8_t i=0; i<(16-len); i++) Serial.print(" ");
  Serial.printf("%d", mode_phase.phase);
  boolean show_delta = false;
  if(show_delta){
    static float old_part_xyz[MODE_PART_NUM][XYZ] = {};
    for(uint8_t part=0; part<MODE_PART_NUM; part++){
      for(uint8_t coor=0; coor<XYZ; coor++){
        Serial.printf(" %7.2f", move_part_xyz[part][coor] - old_part_xyz[part][coor]);
        old_part_xyz[part][coor] = move_part_xyz[part][coor];
      }
      Serial.printf("  ");
    }
  } else {
    for(uint8_t part=0; part<MODE_PART_NUM; part++){
      for(uint8_t coor=0; coor<XYZ; coor++){
        Serial.printf(" %7.2f", move_part_xyz[part][coor]);
      }
      Serial.printf("  ");
    }
  }
  Serial.println();
} // end mode_display_position


//========================================================
// mode_initialize_parts
//========================================================
void mode_initialize_parts(int8_t indent){
  LOCAL_DEBUG_ENABLED
  if (local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("Beg mode_initialize_parts");
  }
  static const uint8_t xi = 0;
  static const uint8_t yi = 1;
  static const uint8_t zi = 2;
  // set all part coordinates to 0.0 initially
  for(uint8_t part=0; part<MODE_PART_NUM; part++){
    for(uint8_t coor=0; coor<XYZ; coor++){
      move_part_xyz[part][coor] = 0.0; 
    }     
  }
  // set the leg coordinates to the retracted position
  for(uint8_t leg=0; leg<NUM_LEGS; leg++){
    for(uint8_t coor=0; coor<XYZ; coor++){ 
      if((leg == 1) || (leg == 3)){
        if(coor == xi){
          move_part_xyz[leg][xi] = LEGS_FOOT_XYZ_SIGNS[leg][xi] * LEGS_XYZ_RETRACTED[yi];
        } else if (coor == yi){
          move_part_xyz[leg][yi] = LEGS_FOOT_XYZ_SIGNS[leg][yi] * LEGS_XYZ_RETRACTED[xi];
        } else {
          move_part_xyz[leg][coor] = LEGS_FOOT_XYZ_SIGNS[leg][coor] * LEGS_XYZ_RETRACTED[coor];
        }
      } else {
        move_part_xyz[leg][coor] = LEGS_FOOT_XYZ_SIGNS[leg][coor] * LEGS_XYZ_RETRACTED[coor];
      }
    }
  }  
  // for the body the coordinates are all zero except for z
  move_part_xyz[MODE_PART_BDY_D][zi] = -LEGS_XYZ_RETRACTED[zi];

  // now set all part beginning coordinates to match the initial values
  for(uint8_t part=0; part<MODE_PART_NUM; part++){
    for(uint8_t coor=0; coor<XYZ; coor++){
      move_part_beg_xyz[part][coor] = move_part_xyz[part][coor]; 
    }     
  }
  
  if (local_debug){
    mode_print_parts_xyz("move_part_xyz", move_part_xyz, indent+1);
    mode_print_parts_xyz("move_part_beg_xyz", move_part_beg_xyz, indent+1);
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("End mode_initialize_parts");
  }
} // end mode_initialize_parts


//========================================================
// mode_create_sequences()
//========================================================
void mode_create_sequences(int8_t indent) {
  LOCAL_DEBUG_ENABLED
  if (local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("Beg mode_create_sequences");
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTLN("READY\tRETRACTED");
    for(uint8_t coor=0; coor<XYZ; coor++){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINTF("%7.2f\t%7.2f\n", LEGS_XYZ_READY[coor], LEGS_XYZ_RETRACTED[coor]);
    }
  }
  float step_time;
  static const uint8_t X = 0;
  static const uint8_t Y = 1;
  static const uint8_t Z = 2;
  
  static const float F_LFT = 50.0; // ?? foot lift amount
  legs_compute_retracted_and_ready(indent+1); // make sure legs_xyz_retracted and legs_xyz_retracted values are available
  static const float B_ADJ = 20.0; // body adjustment during extention/retraction
  static const float F_RDY = LEGS_XYZ_READY[X]; // nominal ready position, x & y
  static const float F_EXT = F_RDY + B_ADJ; // ready but account for the body shift during the leg extension
  //static const float F_FLX = LEGS_XYZ_RETRACTED[X]; // nominal folded (retracted) position, x
  static const float F_RTB = LEGS_XYZ_RETRACTED[Y] + B_ADJ; // nominal folded (retracted) position, y, but account for the body shift during leg retraction
  static const float F_RTS = LEGS_XYZ_RETRACTED[X]; // nominal folded (retracted) position, x
  static const float BZNOM = -LEGS_XYZ_READY[Z]; // nominal body height
  static const float BZLOW = -LEGS_XYZ_RETRACTED[Z]; // lowered body height
  static const float BZHAF = 0.5 * (BZNOM + BZLOW); // halfway between nominal and lowered body height
  // call mode_calc_step_time() with the active leg step to see how long it takes to move a foot F_EXT-F_FLD and then b_vel = B_STP / TIME
  step_time = mode_calc_step_time(F_EXT - LEGS_XYZ_RETRACTED[0], F_EXT - LEGS_XYZ_RETRACTED[1], F_LFT, indent+1);
  static const float B_ARC = 0.5 * PI * B_ADJ; // the arc is 1/4 of the circumference = 1/4 * PI * 2 * radius = 0.5 * PI * radius
  static float badjv = B_ARC / step_time; // body adjust velocity during extention/retraction

  if (local_debug){
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("BZNOM: %7.2f, BZLOW: %7.2f, BZHAF: %7.2f, \n", BZNOM, BZLOW, BZHAF);
  }

  // the maximum stride, M_STD = 2.0 * (F_RDY - F_RAD), the maximum half stride, MHSTD = F_RDY - F_RAD
  // the maximum body step, B_STP = 0.5 * MHSTD, and the maximum half body step, BHSTP = 0.25 * MHSTD
  // the leg movement for these strides is 3/4 of the strides since the body movement adds another 1/4 of the stride during the leg movement
  // we'll use the stride minus the body step to get the foot step
  // while walking the rear foot start position should be -F_STP = -(M_STD - B_STP) and the end position will be - F_RAD
  // while walking the front foot start position should be +F_RAD and the end position will be F_STP = M_STD - B_STP
  // when beginning to walk the rear foot start position should be -F_RDY and the end position will be - F_RAD
  // when beginning to walk the front foot start position should be F_RDY and the end position will be F_STP = M_STD - F_RAD
  static const float F_RAD = 40.0; // foot radius, can't put two feet in the same place so use this to make sure they're at separated locations
  static const float H_STD = F_RDY - F_RAD; // max half stride = the foot ready position minus the foot radius
  static const float F_STD = 2.00 * H_STD; // full stride = 2.0 * (the foot ready position minus the foot radius)
  static const float Q_STD = 0.50 * H_STD; // body stride of 1 step !!! this should be 1/3 of full stride !!!
  static const float B0STD = 4.0 * Q_STD / 16.0; // body walk, begin, phase 0 stride
  static const float B1STD = 5.0 * Q_STD / 16.0; // body walk, begin, phase 1 stride
  static const float B2STD = 7.0 * Q_STD / 16.0; // body walk, begin, phase 2 stride
  static const float F1WLK = -F_RAD; // position relative to body, foot 1 end = minus foot radius
  static const float F0WLK = 2.0*F_RDY -F_RAD - Q_STD; // position relative to body, foot 0 walk = 2* foot ready - foot radius - body stride
  static const float F1WBG = -F_RAD - (B0STD + B1STD); // position relative to body, foot 1 walk beg = - foot radius - body stride
  static const float F0WBG = 2.0*F_RDY -F_RAD -(B0STD + B1STD + B2STD); // position relative to body, foot 0 walk beg = 2* foot ready - foot radius - half body stride
  static const float F2WEN = -F_RDY +(B0STD + B1STD); // position relative to body, foot 2 walk end = - foot radius - body stride
  static const float F3WEN =  F_RDY +(B0STD); // position relative to body, foot 3 walk end = 2* foot ready - foot radius - half body stride
  // call legs_coor_move_points() with the active leg step to see how long it takes to move a foot F_STP and then b_vel = B_STP / TIME
  step_time = mode_calc_step_time(0.0, F_STD, F_LFT, indent+1);
  static float b_vel = Q_STD / step_time; // body velocity, ??? Placeholder, will be scaled up later
  static float b0vel = 0.5 * b_vel; // end vel phase 0, after start walking 1/3
  static float b1vel = 0.75 * b_vel; // end vel phase 1, after start walking 2/3
  static float b2vel = b_vel; // end vel after phase 2, start walking 3/3

  static const float ROT_UNITS_NEAR =  5.0; // 
  static const float ROT_UNITS_FAR =  9.0; // 
  static const float ROT_UNITS_TOTAL =  3.0 * ROT_UNITS_FAR + ROT_UNITS_NEAR; // total units around circle
  static const float BAROT = 2.0 * PI / ROT_UNITS_TOTAL; // Body angle, rotating (radians per unit)
  static const float ROT_RADIANS_TO_FOOT3 = (PI / 2.0) + (ROT_UNITS_FAR * BAROT / 2.0);
  static const float ROT_RADIUS = - F_RDY / cos(ROT_RADIANS_TO_FOOT3);
  static const float BORBG = F_RDY - ROT_RADIUS * sin(ROT_RADIANS_TO_FOOT3); // body offset, rotating beginning
  Serial.printf("near: %d, far: %d, total: %d, rad/unit: %7.2f, F_RDY: %7.2f, rad2f3: %7.2f, radius: %7.2f, offset: %7.2f\n", ROT_UNITS_NEAR, ROT_UNITS_FAR, ROT_UNITS_TOTAL, BAROT, F_RDY, ROT_RADIANS_TO_FOOT3, ROT_RADIUS, BORBG);
  static const float ROT_RADIANS_TO_FOOT2 = ROT_RADIANS_TO_FOOT3 - (2 * ROT_UNITS_FAR + ROT_UNITS_NEAR) * BAROT; // this is before the body rotates BAROT radians
  static const float L2XRB = ROT_RADIUS * cos(ROT_RADIANS_TO_FOOT2); // leg2 x, relative to new body center before rotation, rotating beginning
  static const float L2YRB = ROT_RADIUS * sin(ROT_RADIANS_TO_FOOT2); // leg2 y, relative to new body center before rotation, rotating beginning
  static const float ROT_RADIANS_TO_FOOT0 = (ROT_UNITS_NEAR / 2.0) * BAROT; // this is after the body rotates BAROT radians
  static const float L0XRB = ROT_RADIUS * cos(ROT_RADIANS_TO_FOOT0); // leg0 x, relative to new body center after rotation, rotating beginning
  static const float L0YRB = ROT_RADIUS * sin(ROT_RADIANS_TO_FOOT0); // leg0 y, relative to new body center after rotation, rotating beginning
  static const float L_YRE = F_RDY + BORBG; // y position of foward legs before body moves forward
  step_time = mode_calc_step_time(0.0, (2.0 * L0YRB), F_LFT, indent+1);
  static float bvlrb = BAROT / step_time; // maximum angular velocity around Z axis
  static float l0xvr = bvlrb * L0YRB; // leg 0 x velocity during rotation
  static float l0yvr = bvlrb * L0XRB; // leg 0 y velocity during rotation
  Serial.printf("rad2f2: %7.2f, rad2f0: %7.2f, f2x: %7.2f, f2y: %7.2f, f0x: %7.2f, f0y: %7.2f, av: %7.2f, l0xvr: %7.2f, l0yvr: %7.2f, L_YRE: %7.2f\n", ROT_RADIANS_TO_FOOT2, ROT_RADIANS_TO_FOOT0, L2XRB, L2YRB, L0XRB, L0YRB, bvlrb, l0xvr, l0yvr, L_YRE);
  

  // explanation of move_seq:
  //                            {descriptive string,                    {one part,       {     x,     y,     z}, {end_vx, end_vy, end_vz}},
  //                                                                    {another part,   {     x,     y,     z}, {end_vx, end_vy, end_vz}}};
  // body positions are relative to the body position at the start of the move for x and y, but z is absolute
  // active leg positions x, y are relative to the body position at the end of the move, but z is just used for the leg lift amount
  // the body end velocities are absolute, the leg end velocities are relative to the body
  static const boolean last_phase = true;
  static const boolean not_last_phase = false;

  // sequence for shut down {last_phase, descriptive string,                    {part_id,         {     x,     y,     z}, {end_vx, end_vy, end_vz}},
  mode_seq[MODE_SHUT_DOWN][0] = {last_phase, "shut down",                      {{MODE_PART_NONE_, {   0.0,   0.0, BZNOM}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};

  // sequence for folded     {last_phase, descriptive string,                   {part_id,         {     x,     y,     z}, {end_vx, end_vy, end_vz}},
  mode_seq[MODE_FOLDED][0] = {last_phase, "folded",                            {{MODE_PART_NONE_, {   0.0,   0.0, BZNOM}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};

  // sequence for ready     {last_phase, descriptive string,                    {part_id,         {     x,     y,     z}, {end_vx, end_vy, end_vz}},
  mode_seq[MODE_READY][0] = {last_phase, "ready",                              {{MODE_PART_NONE_, {   0.0,   0.0, BZNOM}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};

  // sequence to start walking    {last_phase, descriptive string,              {part_id,         {     x,     y,     z}, {end_vx, end_vy, end_vz}},
  mode_seq[MODE_WALKING_BEG][0] = {not_last_phase, "beg w 1/3, move body",     {{MODE_PART_BDY_D, {   0.0, B0STD, BZNOM}, {   0.0, b0vel,   0.0}},  // move body only
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};// move rear leg to -F_RAD
  mode_seq[MODE_WALKING_BEG][1] = {not_last_phase, "beg w 2/3, mov bdy, leg1", {{MODE_PART_BDY_D, {   0.0, B1STD, BZNOM}, {   0.0, b1vel,   0.0}},  // move body forward
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_1, { F_RDY, F1WBG, F_LFT}, {   0.0,-b1vel,   0.0}}}};// move front leg to -F_RAD
  mode_seq[MODE_WALKING_BEG][2] = {last_phase, "beg w 3/3, mov bdy, leg0",     {{MODE_PART_BDY_D, {   0.0, B2STD, BZNOM}, {   0.0, b2vel,   0.0}},  // move body forward
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_0, { F_RDY, F0WBG, F_LFT}, {   0.0,-b2vel,   0.0}}}};// move front leg to -F_RAD

  // sequence to stop walking     {last_phase, descriptive string,              {part_id,         {     x,     y,     z}, {end_vx, end_vy, end_vz}},
  mode_seq[MODE_WALKING_END][0] = {not_last_phase, "end walking, move leg2",   {{MODE_PART_BDY_D, {   0.0, B2STD, BZNOM}, {   0.0, b1vel,   0.0}},  // move forward
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_2, {-F_RDY, F2WEN, F_LFT}, {   0.0,-b1vel,   0.0}}}};// move rear leg to -F_RDY
  mode_seq[MODE_WALKING_END][1] = {not_last_phase, "end walking, move leg3",   {{MODE_PART_BDY_D, {   0.0, B1STD, BZNOM}, {   0.0, b0vel,   0.0}},  // move forward
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_3, {-F_RDY, F3WEN, F_LFT}, {   0.0,-b0vel,   0.0}}}};// move front leg to F_RDY
  mode_seq[MODE_WALKING_END][2] = {last_phase, "end walking, stop body",       {{MODE_PART_BDY_D, {   0.0, B0STD, BZNOM}, {   0.0,   0.0,   0.0}},  // move forward
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};// move front leg to F_RDY

  // sequence for walking,    {last_phase, descriptive string,                  {part_id,         {     x,     y,     z}, {end_vx, end_vy, end_vz}},
  mode_seq[MODE_WALKING][0] = {not_last_phase, "walking, move leg2",           {{MODE_PART_BDY_D, {   0.0, Q_STD, BZNOM}, {   0.0, b_vel,   0.0}},  // move forward
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_2, {-F_RDY, F1WLK, F_LFT}, {   0.0,-b_vel,   0.0}}}};// move rear leg to -F_RAD
  mode_seq[MODE_WALKING][1] = {not_last_phase, "walking, move leg3",           {{MODE_PART_BDY_D, {   0.0, Q_STD, BZNOM}, {   0.0, b_vel,   0.0}},  // move forward
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_3, {-F_RDY, F0WLK, F_LFT}, {   0.0,-b_vel,   0.0}}}};// move front leg to F_STP
  mode_seq[MODE_WALKING][2] = {not_last_phase, "walking, move leg1",           {{MODE_PART_BDY_D, {   0.0, Q_STD, BZNOM}, {   0.0, b_vel,   0.0}},  // move forward
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_1, { F_RDY, F1WLK, F_LFT}, {   0.0,-b_vel,   0.0}}}};// move rear leg to -F_RAD
  mode_seq[MODE_WALKING][3] = {last_phase, "walking, move leg0",               {{MODE_PART_BDY_D, {   0.0, Q_STD, BZNOM}, {   0.0, b_vel,   0.0}},  // move forward
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_0, { F_RDY, F0WLK, F_LFT}, {   0.0,-b_vel,   0.0}}}};// move front leg to F_STP

  // sequence to start sidestepping     {last_phase, descriptive string,        {part_id,         {     x,     y,     z}, {end_vx, end_vy, end_vz}}, 
  mode_seq[MODE_SIDESTEPPING_BEG][0] = {not_last_phase, "ss beg 0, move body", {{MODE_PART_BDY_D, { B0STD,   0.0, BZNOM}, { b0vel,   0.0,   0.0}},  // move body only
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};// move rear leg to -F_RAD
  mode_seq[MODE_SIDESTEPPING_BEG][1] = {not_last_phase, "ss beg 1, body +leg2",{{MODE_PART_BDY_D, { B1STD,   0.0, BZNOM}, { b1vel,   0.0,   0.0}},  // move body forward
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_2, { F1WBG,-F_RDY, F_LFT}, {-b1vel,   0.0,   0.0}}}};// move front leg to -F_RAD
  mode_seq[MODE_SIDESTEPPING_BEG][2] = {last_phase, "ss beg 2, body +leg1",    {{MODE_PART_BDY_D, { B2STD,   0.0, BZNOM}, { b2vel,   0.0,   0.0}},  // move body forward
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_1, { F0WBG,-F_RDY, F_LFT}, {-b2vel,   0.0,   0.0}}}};// move front leg to -F_RAD

  // sequence for sidestepping,    {last_phase, descriptive string,             {part_id,         {     x,     y,     z}, {end_vx, end_vy, end_vz}},
  mode_seq[MODE_SIDESTEPPING][0] = {not_last_phase, "ss, body +leg3",          {{MODE_PART_BDY_D, { Q_STD,   0.0, BZNOM}, { b_vel,   0.0,   0.0}},  // move forward
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_3, { F1WLK, F_RDY, F_LFT}, {-b_vel,   0.0,   0.0}}}};// move rear leg to -F_RAD
  mode_seq[MODE_SIDESTEPPING][1] = {not_last_phase, "sss, body +leg0",         {{MODE_PART_BDY_D, { Q_STD,   0.0, BZNOM}, { b_vel,   0.0,   0.0}},  // move forward
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_0, { F0WLK, F_RDY, F_LFT}, {-b_vel,   0.0,   0.0}}}};// move front leg to F_STP
  mode_seq[MODE_SIDESTEPPING][2] = {not_last_phase, "ss, body leg2",           {{MODE_PART_BDY_D, { Q_STD,   0.0, BZNOM}, { b_vel,   0.0,   0.0}},  // move forward
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_2, { F1WLK,-F_RDY, F_LFT}, {-b_vel,   0.0,   0.0}}}};// move rear leg to -F_RAD
  mode_seq[MODE_SIDESTEPPING][3] = {last_phase, "ss, body leg1",               {{MODE_PART_BDY_D, { Q_STD,   0.0, BZNOM}, { b_vel,   0.0,   0.0}},  // move forward
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_1, { F0WLK,-F_RDY, F_LFT}, {-b_vel,   0.0,   0.0}}}};// move front leg to F_STP

  // sequence to stop sidestepping     {last_phase, descriptive string,         {part_id,         {     x,     y,     z}, {end_vx, end_vy, end_vz}},
  mode_seq[MODE_SIDESTEPPING_END][0] = {not_last_phase, "ss end 0, body +leg3",{{MODE_PART_BDY_D, { B2STD,   0.0, BZNOM}, { b1vel,   0.0,   0.0}},  // move forward
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_3, { F2WEN, F_RDY, F_LFT}, {-b1vel,   0.0,   0.0}}}};// move rear leg to -F_RDY
  mode_seq[MODE_SIDESTEPPING_END][1] = {not_last_phase, "ss end 1, body +leg0",{{MODE_PART_BDY_D, { B1STD,   0.0, BZNOM}, { b0vel,   0.0,   0.0}},  // move forward
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_0, { F3WEN, F_RDY, F_LFT}, {-b0vel,   0.0,   0.0}}}};// move front leg to F_RDY
  mode_seq[MODE_SIDESTEPPING_END][2] = {last_phase, "ss end 2, stop body",     {{MODE_PART_BDY_D, { B0STD,   0.0, BZNOM}, {   0.0,   0.0,   0.0}},  // move forward
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};// move front leg to F_RDY

  // sequence to start rotating,   {last_phase, descriptive string,             {part_id,         {     x,     y,     z}, {end_vx, end_vy, end_vz}},  
  mode_seq[MODE_ROTATING_BEG][0] = {not_last_phase, "rotating beg 0, bdy",     {{MODE_PART_BDY_D, {   0.0, BORBG, BZNOM}, {   0.0,   0.0,   0.0}},  // move body fwd
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};
  mode_seq[MODE_ROTATING_BEG][1] = {not_last_phase, "rotating beg 1, leg2",    {{MODE_PART_LEG_2, { L2XRB, L2YRB, F_LFT}, {   0.0,   0.0,   0.0}},  // move leg2 right onto circle
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};
  mode_seq[MODE_ROTATING_BEG][2] = {last_phase, "rotating beg 2, leg1 bdy",    {{MODE_PART_LEG_1, { L0XRB,-L0YRB, F_LFT}, {-l0xvr,-l0yvr,   0.0}},  // move leg1 fwd onto circle
                                                                                {MODE_PART_BDY_A, {   0.0,   0.0, BAROT}, {   0.0,   0.0, bvlrb}},  // rotate body ccw
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};

  // sequence to rotate,       {last_phase, descriptive string,                 {part_id,         {     x,     y,     z}, {end_vx, end_vy, end_vz}},  
  mode_seq[MODE_ROTATING][0] = {not_last_phase, "rotating 0, leg0 bdy",        {{MODE_PART_LEG_0, { L0YRB, L0XRB, F_LFT}, { l0yvr,-l0xvr,   0.0}},  // move leg0 ccw around circle
                                                                                {MODE_PART_BDY_A, {   0.0,   0.0, BAROT}, {   0.0,   0.0, bvlrb}},  // rotate body ccw
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};
  mode_seq[MODE_ROTATING][1] = {not_last_phase, "rotating 1, leg3 bdy",        {{MODE_PART_LEG_3, {-L0XRB, L0YRB, F_LFT}, { l0xvr, l0yvr,   0.0}},  // move leg3 ccw around circle
                                                                                {MODE_PART_BDY_A, {   0.0,   0.0, BAROT}, {   0.0,   0.0, bvlrb}},  // rotate body ccw
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};
  mode_seq[MODE_ROTATING][2] = {not_last_phase, "rotating 2, leg2 bdy",        {{MODE_PART_LEG_2, {-L0YRB,-L0XRB, F_LFT}, {-l0yvr, l0xvr,   0.0}},  // move leg2 ccw around circle
                                                                                {MODE_PART_BDY_A, {   0.0,   0.0, BAROT}, {   0.0,   0.0, bvlrb}},  // rotate body ccw
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};
  mode_seq[MODE_ROTATING][3] = {last_phase, "rotating 3, leg 1 bdy",           {{MODE_PART_LEG_1, { L0XRB,-L0YRB, F_LFT}, {-l0xvr,-l0yvr,   0.0}},  // move leg1 ccw around circle
                                                                                {MODE_PART_BDY_A, {   0.0,   0.0, BAROT}, {   0.0,   0.0, bvlrb}},  // rotate body ccw
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};

  // sequence to stop rotating,   {last_phase, descriptive string,              {part_id,         {     x,     y,     z}, {end_vx, end_vy, end_vz}},  
  mode_seq[MODE_ROTATING_END][0] = {not_last_phase, "rotating end 0, leg0 bdy",{{MODE_PART_LEG_0, { F_RDY, L_YRE, F_LFT}, {   0.0,   0.0,   0.0}},  // move leg0 fwd into stance
                                                                                {MODE_PART_BDY_A, {   0.0,   0.0, BAROT}, {   0.0,   0.0,   0.0}},  // rotate body ccw
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};
  mode_seq[MODE_ROTATING_END][1] = {not_last_phase, "rotating end 1, leg3",    {{MODE_PART_LEG_3, {-F_RDY, L_YRE, F_LFT}, {   0.0,   0.0,   0.0}},  // move leg3 left into stance
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};
  mode_seq[MODE_ROTATING_END][2] = {last_phase, "rotating end 2, bdy",         {{MODE_PART_BDY_D, {   0.0, BORBG, BZNOM}, {   0.0,   0.0,   0.0}},  // move body fwd
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing 
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};

  // sequence for unfolding,    {last_phase,     descriptive string,            {part_id,         {     x,     y,     z}, {end_vx, end_vy, end_vz}},  
  mode_seq[MODE_UNFOLDING][0] = {not_last_phase, "raise body 1/2",             {{MODE_PART_BDY_D, {   0.0,   0.0, BZHAF}, {   0.0,   0.0,  20.0}},  // body: raise 1/2 way up
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};
  mode_seq[MODE_UNFOLDING][1] = {not_last_phase, "raise 2/2, ready ext leg0",  {{MODE_PART_BDY_D, {   0.0,-B_ADJ, BZNOM}, {-badjv,   0.0,   0.0}},  // body: finish raising, move back
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};
  mode_seq[MODE_UNFOLDING][2] = {not_last_phase, "ext leg0, ready to ext leg1",{{MODE_PART_BDY_D, {-B_ADJ, B_ADJ, BZNOM}, {   0.0, badjv,   0.0}},  // body: move left & fwd
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_0, { F_EXT, F_RDY, F_LFT}, {   0.0,-badjv,   0.0}}}}; // leg0: extend
  mode_seq[MODE_UNFOLDING][3] = {not_last_phase, "ext leg1, ready to ext leg2",{{MODE_PART_BDY_D, { B_ADJ, B_ADJ, BZNOM}, { badjv,   0.0,   0.0}},  // body: move right & fwd
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_1, { F_RDY,-F_EXT, F_LFT}, {-badjv,   0.0,   0.0}}}}; // leg1: extend
  mode_seq[MODE_UNFOLDING][4] = {not_last_phase, "ext leg2, ready to ext leg3",{{MODE_PART_BDY_D, { B_ADJ,-B_ADJ, BZNOM}, {   0.0,-badjv,   0.0}},  // body: move right & back
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_2, {-F_EXT,-F_RDY, F_LFT}, {   0.0, badjv,   0.0}}}}; // leg2: extend
  mode_seq[MODE_UNFOLDING][5] = {not_last_phase, "ext leg3, ready to ext leg0",{{MODE_PART_BDY_D, {-B_ADJ,-B_ADJ, BZNOM}, {-badjv,   0.0,   0.0}},  // body: move left & back
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_3, {-F_RDY, F_EXT, F_LFT}, { badjv,   0.0,   0.0}}}}; // leg3: extend
  mode_seq[MODE_UNFOLDING][6] = {last_phase , "center body",                   {{MODE_PART_BDY_D, {   0.0, B_ADJ, BZNOM}, {   0.0,   0.0,   0.0}},  // body: center (fwd)
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};

  // sequence for folding,    {last_phase,     descriptive string,              {part_id,         {     x,     y,     z}, {end_vx, end_vy, end_vz}}, 
  mode_seq[MODE_FOLDING][0] = {not_last_phase, "shift body, ready to ret leg3",{{MODE_PART_BDY_D, {   0.0,-B_ADJ, BZNOM}, { badjv,   0.0,   0.0}},  // body: move back
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};
  mode_seq[MODE_FOLDING][1] = {not_last_phase, "ret leg3, ready to ret leg2",  {{MODE_PART_BDY_D, { B_ADJ, B_ADJ, BZNOM}, {   0.0, badjv,   0.0}},  // body: move left 
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_3, {-F_RTB, F_RTS, F_LFT}, {   0.0,   0.0,   0.0}}}}; // leg0: retract
  mode_seq[MODE_FOLDING][2] = {not_last_phase, "ret leg2, ready to ret leg1",  {{MODE_PART_BDY_D, {-B_ADJ, B_ADJ, BZNOM}, {-badjv,   0.0,   0.0}},  // body: move fwd
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_2, {-F_RTS,-F_RTB, F_LFT}, {   0.0,   0.0,   0.0}}}}; // leg1: retract
  mode_seq[MODE_FOLDING][3] = {not_last_phase, "ret leg1, ready to ret leg0",  {{MODE_PART_BDY_D, {-B_ADJ,-B_ADJ, BZNOM}, {   0.0,-badjv,   0.0}},  // body: move right
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_1, { F_RTB,-F_RTS, F_LFT}, {   0.0,   0.0,   0.0}}}}; // leg2: retract
  mode_seq[MODE_FOLDING][4] = {not_last_phase, "ret leg0, ready to ret leg3",  {{MODE_PART_BDY_D, { B_ADJ,-B_ADJ, BZNOM}, { badjv,   0.0,   0.0}},  // body: move back
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_LEG_0, { F_RTS, F_RTB, F_LFT}, {   0.0,   0.0,   0.0}}}}; // leg3: retract
  mode_seq[MODE_FOLDING][5] = {not_last_phase, "center body, start lowering",  {{MODE_PART_BDY_D, {   0.0, B_ADJ, BZHAF}, {   0.0,   0.0, -20.0}},  // body: center, start lowering
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};
  mode_seq[MODE_FOLDING][6] = {last_phase , "finish lowering",                 {{MODE_PART_BDY_D, {   0.0,   0.0, BZLOW}, {   0.0,   0.0,   0.0}},  // body: finish lowering
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE_, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};

  if (local_debug || true){
    mode_print_sequences(indent+1);
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("End mode_create_sequences");
  }
} // end mode_create_sequences


//========================================================
// mode_print_sequences()
//========================================================
void mode_print_sequences(int8_t indent){
  mode_seq_t this_mode_seq;
  DEBUG_INDENT(indent);
  Serial.println("Beg mode_print_sequences");
  for(uint8_t mode=0; mode<MODE_NUM; mode++){
    for(uint8_t phase=0; phase<MODE_PHASE_MAX_NUM; phase++){
      DEBUG_INDENT(indent+1);
      Serial.printf("mode_seq[%s][%u]: \n", MODE_NAME[mode].c_str(), phase);
      this_mode_seq = mode_seq[mode][phase];
      mode_print_sequence(this_mode_seq, indent+1);
      if(this_mode_seq.last_phase) break;
    }
  }
  DEBUG_INDENT(indent);
  Serial.println("End mode_print_sequences");
} // end mode_print_sequences


//========================================================
// mode_print_sequence()
//========================================================
void mode_print_sequence(mode_seq_t this_mode_seq, int8_t indent){
  DEBUG_INDENT(indent);
  Serial.println("Beg mode_print_sequence");
  DEBUG_INDENT(indent+1);
  if(this_mode_seq.last_phase) {
    Serial.print("last_phase");
  } else {
    Serial.print("not_last_phase");
  }
  Serial.printf(", %s\n", this_mode_seq.phase_name.c_str());
  mode_seq_part_t seq_part;
  for(uint8_t seq_part_i=0; seq_part_i<MODE_SEQ_PART_NUM; seq_part_i++){
    seq_part = this_mode_seq.part[seq_part_i];
    DEBUG_INDENT(indent+1);
//    Serial.print("-\t\t\t\t");
    Serial.printf("%s D:", MODE_PART_NAME[seq_part.part_id].c_str());
    for(uint8_t coor=0; coor<XYZ; coor++){
      Serial.printf(" %7.2f", seq_part.d[coor]);
    }
    Serial.printf(", V:", MODE_PART_NAME[seq_part.part_id].c_str());
    for(uint8_t coor=0; coor<XYZ; coor++){
      Serial.printf(" %7.2f", seq_part.v[coor]);
    }
    Serial.println();
  }
  DEBUG_INDENT(indent);
  Serial.println("End mode_print_sequence");
} // end mode_print_sequence


//========================================================
// mode_calc_step_time()
// use the step_size and lift height to calculate the step_time
//========================================================
float mode_calc_step_time(float step_x, float step_y, float lift_height, int8_t indent){
  const char *routine = "mode_calc_step_time";
  LOCAL_DEBUG_ENABLED
  if (local_debug){
    DEBUG_PRINT_BEG(routine, indent);
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("step_x: %7.2f, step_y: %7.2f, lift_height: %7.2f\n", step_x, step_y, lift_height);
  }
  float step_time;
  //LEGS_XYZ_VA_MAX[XYZ][2]
  //     LEGS_PARAM_NAME[LEGS_PARAM_NUM] = {     "DIST", "DIR", "UPDN",               "V_MAX",               "A_MAX", "V_BEG", "V_END"};
  float parameters[XYZ][LEGS_PARAM_NUM] = {{     step_x,   1.0,   -1.0, LEGS_XYZ_VA_MAX[0][0], LEGS_XYZ_VA_MAX[0][1],     0.0,     0.0},
                                           {     step_y,   1.0,   -1.0, LEGS_XYZ_VA_MAX[1][0], LEGS_XYZ_VA_MAX[1][1],     0.0,     0.0},
                                           {lift_height,   1.0,    1.0, LEGS_XYZ_VA_MAX[2][0], LEGS_XYZ_VA_MAX[2][1],     0.0,     0.0}};
  float move_points[XYZ][LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM];
  for(uint8_t coor=0; coor<XYZ; coor++){
    for(uint8_t move_point=0; move_point<LEGS_MOVE_POINT_NUM; move_point++){
      for(uint8_t td=0; td<LEGS_MOVE_TD_NUM; td++){
        move_points[coor][move_point][td] = 0.0;
      }
    }
  }

  float target_time = -1.0;
  legs_coor_move_points(target_time, parameters, move_points, indent+1);
  step_time = move_points[0][0][0];  
  if (local_debug){
    legs_print_move_points(move_points, indent+1);
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("step_time: %7.2f\n", step_time);
    DEBUG_PRINT_END(routine, indent);
  }
  return step_time;
} // end mode_calc_step_time


//========================================================
// mode_update()
// check to see if mode chould be changed based on current mode and inputs
// execute current mode sequence (new or previously started)
//========================================================
void mode_update(int8_t indent) {
  const static char *routine = "mode_update";
  LOCAL_DEBUG_ENABLED
  if (local_debug) DEBUG_PRINT_BEG(routine, indent);
  mode_check_new_mode(indent+1);
  mode_execute_seq(indent+1);
  if (local_debug) DEBUG_PRINT_END(routine, indent);
} // end mode_update


//========================================================
// mode_check_new_mode()
// check to see if mode chould be changed based on current mode and inputs
//========================================================
void mode_check_new_mode(int8_t indent) {
  uint8_t current_mode = mode_phase.mode; // get the current mode
  uint8_t current_dir = mode_phase.direction; // get the current direction
  uint8_t temp_next_mode;
  uint8_t temp_next_dir;
  uint8_t temp_next_phase;
  const static char *routine = "mode_check_new_mode";
  LOCAL_DEBUG_ENABLED  
//  local_debug = true; ////////
  if (local_debug){
    DEBUG_PRINT_BEG(routine, indent);
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("current_mode: %s, phase: %d\n", MODE_NAME[current_mode].c_str(), mode_phase.phase);
  }
  mode_values_update(indent+1);
  temp_next_mode = MODE_NONE; // default is no new commanded mode
  temp_next_dir = MODE_DIR_NONE; // default is no new commanded direction

  if(sbus_panic(indent+1)){
    // panic button has been hit, stop everything immediately with a mode_set
    if (local_debug){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINTF("current_mode: %s, phase: %d\n", MODE_NAME[current_mode].c_str(), mode_phase.phase);
    }
    temp_next_phase = 0;
    mode_set(MODE_SHUT_DOWN, MODE_DIR_NONE, temp_next_phase, indent+1);
  } else {
    // not in panic, proceed as normal
    float vy, vx, vt;
    vy= mode_value_vy();
    vx = mode_value_vx();
    vt = mode_value_vt();
//    Serial.printf("in mode_check_new_mode, vy: %7.2f vx: %7.2f, vt: %7.2f\n", vy, vx, vt);
    switch (current_mode) {
      case MODE_FOLDED:
      case MODE_FOLDING:
        // currently folded or folding, for the next mode we can either stay folded or unfold
        // should check other conditions: powered down, check if radio, power, orientation are OK
        if (!mode_value_fold()) {
          temp_next_mode = MODE_UNFOLDING;
        } else {
          temp_next_mode = MODE_FOLDED;
        }
        break;
      case MODE_UNFOLDING:
      case MODE_WALKING_END:
      case MODE_SIDESTEPPING_END:
      case MODE_ROTATING_END:
      case MODE_READY:
        // if the current_mode is MODE_READY or any of the others that end at MODE_READY, we can start doing anything based on the inputs
        if (mode_value_fold()) {
          // folding gets priority over other inputs, so it comes first
          temp_next_mode = MODE_FOLDING;
        } else if(abs(vy) > COM_ZERO) {
          // We're going to walk! Which direction though?
          temp_next_mode = MODE_WALKING_BEG;
          if(vy > 0.0) {
            temp_next_dir = MODE_DIR_PLUS;
          } else {
            temp_next_dir = MODE_DIR_MINUS;
          }
        } else if(abs(vx) > COM_ZERO) {
          // We're going to sidestep! Which direction though?
          temp_next_mode = MODE_SIDESTEPPING_BEG;
          if(vx > 0.0) {
            temp_next_dir = MODE_DIR_PLUS;
          } else {
            temp_next_dir = MODE_DIR_MINUS;
          }
        } else if(abs(vt) > COM_ZERO) {
          // We're going to rotate! Which direction though?
          temp_next_mode = MODE_ROTATING_BEG;
          if(vt > 0.0) {
            temp_next_dir = MODE_DIR_PLUS;
          } else {
            temp_next_dir = MODE_DIR_MINUS;
          }
        } else {
          temp_next_mode = MODE_READY;
        }
        break;
      case MODE_WALKING_BEG:
      case MODE_WALKING:
        temp_next_dir = current_dir; // keep going the same direction
        if(current_dir == MODE_DIR_PLUS){
          if(vy > COM_ZERO){
            // keep walking forward
            temp_next_mode = MODE_WALKING;
          } else {
            // stop walking forward
            temp_next_mode = MODE_WALKING_END;
          }
        } else {
          // was going backwards
          Serial.printf("in mode_check_new_mode, vy: %14.7f, COM_ZERO: %14.7f\n", vy, COM_ZERO);
          if(vy < -COM_ZERO){
            // keep walking backward
            temp_next_mode = MODE_WALKING;
          } else {
            // stop walking backward
            temp_next_mode = MODE_WALKING_END;
          }
        }
        break;
      case MODE_SIDESTEPPING_BEG:
      case MODE_SIDESTEPPING:
        temp_next_dir = current_dir; // keep going the same direction
        if(current_dir == MODE_DIR_PLUS){
          if(vx > COM_ZERO){
            // keep sidestepping to the right
            temp_next_mode = MODE_SIDESTEPPING;
          } else {
            // stop sidestepping to the right
            temp_next_mode = MODE_SIDESTEPPING_END;
          }
        } else {
          if(vx < -COM_ZERO){
            // keep sidestepping to the left
            temp_next_mode = MODE_SIDESTEPPING;
          } else {
            // stop sidestepping to the left
            temp_next_mode = MODE_SIDESTEPPING_END;
          }
        }
        break;
      case MODE_ROTATING_BEG:
      case MODE_ROTATING:
        temp_next_dir = current_dir; // keep going the same direction
        if(current_dir == MODE_DIR_PLUS){
          if(vt > COM_ZERO){
            // keep rotating ccw
            temp_next_mode = MODE_ROTATING;
          } else {
            // stop rotating ccw
            temp_next_mode = MODE_ROTATING_END;
          }
        } else {
          if(vt < -COM_ZERO){
            // keep rotating cw
            temp_next_mode = MODE_ROTATING;
          } else {
            // stop rotating cw
            temp_next_mode = MODE_ROTATING_END;
          }
        }
        break;
      default:
        // the current_mode wasn't accounted for in the above switch/case statment
        com_err_msg(routine,"Should never have an unknown current_mode");
        break;
    }
  }
  if(temp_next_mode == MODE_NONE){
    // this shouldn't happen as we should always set a valid mode in the above code
    com_err_msg(routine,"temp_next_mode == MODE_NONE");
  }

  mode_set_next(temp_next_mode, temp_next_dir, indent+1);
//  if (local_debug && true){       
  if (true){       
    DEBUG_INDENT(indent+1);
//    DEBUG_PRINTF("temp_next_mode: %s, temp_next_dir: %s\n", MODE_NAME[temp_next_mode].c_str(), MODE_DIR_NAME[temp_next_dir].c_str());
    Serial.printf("temp_next_mode: %s, temp_next_dir: %s\n", MODE_NAME[temp_next_mode].c_str(), MODE_DIR_NAME[temp_next_dir].c_str());
  }
  if (local_debug) DEBUG_PRINT_END(routine, indent);
} // end mode_check_new_mode


//========================================================
// mode_print_current_time
//========================================================
void mode_print_current_time(int8_t indent){
  uint32_t start_time = mode_phase.start_time;
  uint32_t end_time = mode_phase.end_time;
  float current_seq_time = float(millis()-start_time)/1000.0; // get float current_time = time since the start of this phase in seconds
  DEBUG_INDENT(indent);
  DEBUG_PRINTF("current_seq_time: %7.2f, millis(): %d, start_time: %d, end_time: %d\n", current_seq_time, millis(), start_time, end_time);
} // end mode_print_current_time


//========================================================
// mode_execute_seq()
// execute current mode sequence (new or previously started)
// move body and legs as required
//========================================================
void mode_execute_seq(int8_t indent){
  const static char *routine = "mode_execute_seq";
  uint8_t current_mode = mode_phase.mode;
  uint8_t current_dir = mode_phase.direction;
  uint8_t current_phase = mode_phase.phase;
  uint32_t start_time = mode_phase.start_time;
  // get the current mode sequence phase and parts
  mode_seq_t mode_seq_phase = mode_seq[current_mode][current_phase];
  mode_seq_part_t part[2];
  part[0] = mode_seq_phase.part[0];
  part[1] = mode_seq_phase.part[1];
  uint8_t part0_id = part[0].part_id;
  uint8_t part1_id = part[1].part_id;

  LOCAL_DEBUG_ENABLED
  if (local_debug){
    DEBUG_PRINT_BEG(routine, indent);
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("alternate: %d, mode: %s, phase: %s, part0: %s, part1: %s\n", mode_phase.alternate, MODE_NAME[current_mode].c_str(), mode_seq_phase.phase_name.c_str(), MODE_PART_NAME[part0_id].c_str(), MODE_PART_NAME[part1_id].c_str());
    mode_print_current_time(indent+1);
    mode_print_parts_xyz("move_part_xyz", move_part_xyz, indent+1);
  }
  if(current_mode == MODE_SHUT_DOWN){
    // we've been shut down, do nothing!
  } else {
    // no panic, not shut down, proceed as normal
    // update parts
    float current_seq_time = float(millis()-start_time)/1000.0; // get float current_seq_time = time since the start of this phase in seconds
    boolean move_done = false;
    uint8_t next_phase = 0;

    mode_execute_move(current_seq_time, mode_seq_phase, &move_done, indent+1);

    if(move_done){
      // the move for all parts has completed, move to the next phase if there is one
      if (local_debug){
        DEBUG_INDENT(indent+1);
        DEBUG_PRINTLN("move_done == true");
      }
      if(mode_seq_phase.last_phase){
        if (local_debug){
          DEBUG_INDENT(indent+1);
          DEBUG_PRINTF("mode_seq_phase.last_phase == true, mode_next_mode(): %s, mode_next_dir(): %s\n", MODE_NAME[mode_next_mode()].c_str(), MODE_DIR_NAME[mode_next_dir()].c_str());
        }
        next_phase = 0;
        mode_set(mode_next_mode(), mode_next_dir(), next_phase, indent+1);
      } else {
        // not the last phase, go to the next phase, adjust start_time
        if (local_debug){
          DEBUG_INDENT(indent+1);
          DEBUG_PRINTLN("mode_seq_phase.last_phase != true");
        }
        if((mode_next_mode() == MODE_FOLDED) || (mode_next_mode() == MODE_READY)){
          // don't set debug_new_mode
        } else {
          debug_set_new_mode();
        }
        next_phase = current_phase + 1;
        mode_set(current_mode, current_dir, next_phase, indent+1); // update the data for the next move seq
      }
    } else {
      if (local_debug){
        DEBUG_INDENT(indent+1);
        DEBUG_PRINTLN("move_done != true");
      }    
    }
  }
  if (local_debug) DEBUG_PRINT_END(routine, indent);
} // end mode_execute_seq


//========================================================
// mode_copy_part_xyz
//========================================================
void mode_copy_part_xyz(float src_part_xyz[MODE_PART_NUM][XYZ], float dst_part_xyz[MODE_PART_NUM][XYZ], int8_t indent){
  const static char *routine = "mode_copy_part_xyz";
//  const boolean local_debug = false;
  LOCAL_DEBUG_DISABLED
  if (local_debug) DEBUG_PRINT_BEG(routine, indent);
  for(uint8_t part=0; part<MODE_PART_NUM; part++){
    for(uint8_t coor=0; coor<XYZ; coor++){
      dst_part_xyz[part][coor] = src_part_xyz[part][coor];
    }
  }
  if (local_debug) DEBUG_PRINT_END(routine, indent);  
} // end mode_copy_part_xyz


//========================================================
// mode_copy_part_parameters
//========================================================
void mode_copy_part_parameters(float src_part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM], float dst_part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM], int8_t indent){
  const static char *routine = "mode_copy_part_parameters";
//  const boolean local_debug = false;
  LOCAL_DEBUG_DISABLED
  if (local_debug) DEBUG_PRINT_BEG(routine, indent);
  for(uint8_t seq_part=0; seq_part<MODE_SEQ_PART_NUM; seq_part++){
    for(uint8_t coor=0; coor<XYZ; coor++){
      for(uint8_t param=0; param<LEGS_PARAM_NUM; param++){
        dst_part_parameters[seq_part][coor][param] = src_part_parameters[seq_part][coor][param];
      }
    }
  }
  if (local_debug) DEBUG_PRINT_END(routine, indent);  
} // end mode_copy_part_parameters


//========================================================
// mode_copy_part_data
//========================================================
void mode_copy_part_data(uint8_t src_part_data[MODE_PART_NUM], uint8_t dst_part_data[MODE_PART_NUM], int8_t indent){
  const static char *routine = "mode_copy_part_data";
//  const boolean local_debug = false;
  LOCAL_DEBUG_DISABLED
  if (local_debug) DEBUG_PRINT_BEG(routine, indent);
  for(uint8_t part=0; part<MODE_PART_NUM; part++){
    dst_part_data[part] = src_part_data[part];
  }
  if (local_debug) DEBUG_PRINT_END(routine, indent);  
} // end mode_copy_part_data


//========================================================
// mode_print_move_part_points
//========================================================
void mode_print_move_part_points(int8_t indent){
  String coor_name[XYZ] = {"X", "Y", "Z"};
  const static char *routine = "mode_print_move_part_points";
  DEBUG_PRINT_BEG(routine, indent);
  DEBUG_INDENT(indent+1);
  DEBUG_PRINT("    \t\t");
  for(uint8_t point=0; point<LEGS_MOVE_POINT_NUM; point++){
    DEBUG_PRINTF("\t%s", LEGS_MOVE_NAME[point].c_str());
  }
  DEBUG_PRINTLN();
  for(uint8_t part=0; part<MODE_SEQ_PART_NUM; part++){
    for(uint8_t coor=0; coor<XYZ; coor++){
      for(uint8_t td=0; td<LEGS_MOVE_TD_NUM; td++){
        DEBUG_INDENT(indent+1);
        DEBUG_PRINTF("seq part %d, %s, %s:", part, coor_name[coor].c_str(), LEGS_MOVE_TD_NAME[td].c_str());
        for(uint8_t point=0; point<LEGS_MOVE_POINT_NUM; point++){
          DEBUG_PRINTF("\t%7.2f", move_part_points[part][coor][point][td]);
        }
        DEBUG_PRINTLN();
      }
    }
  }
  DEBUG_PRINT_END(routine, indent);
}
// end mode_print_move_part_points


//========================================================
// mode_print_part_parameters
//========================================================
void mode_print_part_parameters(const char *text, float part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM], int8_t indent){
  const static char *routine = "mode_print_part_parameters";
  DEBUG_PRINT_BEG2(routine, text, indent);
  String coor_name[XYZ] = {"X", "Y", "Z"};
  DEBUG_INDENT(indent+1);
  DEBUG_PRINT("\t\t");
  for(uint8_t param=0; param<LEGS_PARAM_NUM; param++){
    DEBUG_PRINTF("\t%s", LEGS_PARAM_NAME[param].c_str());
  }
  DEBUG_PRINTLN();
  for(uint8_t part=0; part<MODE_SEQ_PART_NUM; part++){
    for(uint8_t coor=0; coor<XYZ; coor++){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINTF("seq part %d, %s:", part, coor_name[coor].c_str());
      for(uint8_t param=0; param<LEGS_PARAM_NUM; param++){
        DEBUG_PRINTF("\t%7.2f", part_parameters[part][coor][param]);
      }
      DEBUG_PRINTLN();
    }
  }
  DEBUG_PRINT_END(routine, indent);
}
// end mode_print_part_parameters


//========================================================
// mode_print_parts_xyz
//========================================================
void mode_print_parts_xyz(const char *text, float part_xyz[MODE_PART_NUM][XYZ], int8_t indent){
  const static char *routine = "mode_print_parts_xyz";
  DEBUG_PRINT_BEG2(routine, text, indent);
  for(uint8_t part=0; part<MODE_PART_NUM; part++){
    DEBUG_INDENT(indent+1);
    for(uint8_t coor=0; coor<XYZ; coor++){
      DEBUG_PRINTF("%7.2f\t", part_xyz[part][coor]);
    }
    DEBUG_PRINTLN();
  }
  DEBUG_PRINT_END(routine, indent);
} // end mode_print_parts_xyz


//========================================================
// mode_print_parts_activity_seq_part
//========================================================
void mode_print_parts_activity_seq_part(const char *text, uint8_t part_activity[MODE_PART_NUM], uint8_t part_seq_part[MODE_PART_NUM], int8_t indent){
  const static char *routine = "mode_print_parts_activity_seq_part";
  DEBUG_PRINT_BEG(routine, indent);
  DEBUG_INDENT(indent+1);
  DEBUG_PRINTLN(text);
  for(uint8_t part=0; part<MODE_PART_NUM; part++){
    DEBUG_INDENT(indent+1);
//    DEBUG_PRINTF("%s", MODE_PART_NAME[part].c_str());
//    DEBUG_PRINTF("\t%s", MODE_PART_ACTIVITY_NAME[part_activity[part]].c_str());
//    DEBUG_PRINTF("\t%s\n", MODE_SEQ_PART_NAME[part_seq_part[part]].c_str());
    DEBUG_PRINTF("%s\t%s\t%s\n", MODE_PART_NAME[part].c_str(), MODE_PART_ACTIVITY_NAME[part_activity[part]].c_str(), MODE_SEQ_PART_NAME[part_seq_part[part]].c_str());
  }
  DEBUG_PRINT_END(routine, indent);
} // end mode_print_parts_activity_seq_part


//========================================================
// mode_update_move_part_data()
// gets old move_points data and updates move_points for part to prepare for next phase
// called when there is a mode or phase change
// will also update mode_phase.start_time and mode_phase.end_time
//========================================================
void mode_update_move_part_data(uint8_t new_mode, uint8_t new_dir, uint8_t new_phase, int8_t indent){
  // use the move_seq to calculate to move parameters and move_points for the active parts in this phase of the move_sequence
  // need to create the move parameters (distance, direction, up_down, v_max, a_max, v_beg, v_end) for each part
  // mode_seq[mode][phase].part[i] contains part_id, and d[XYZ] and v[XYZ] for each of the two moving parts
  // float move_part_points[MODE_SEQ_PART_NUM][XYZ][LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM]; // move points for parts
  //     LEGS_PARAM_NAME[LEGS_PARAM_NUM] = {"DIST", "DIR", "UPDN", "V_MAX", "A_MAX", "V_BEG", "V_END"};
  // need to update the move_part_beg_xyz values
  const static char *routine = "mode_update_move_part_data";
  LOCAL_DEBUG_ENABLED
  if (local_debug){
    DEBUG_PRINT_BEG(routine, indent);
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("new_mode: %s, new_mode_dir: %s, new_phase: %u\n", MODE_NAME[new_mode].c_str(), MODE_DIR_NAME[new_dir].c_str(), new_phase);
    mode_print_current_time(indent+1);
  }

//  static const uint8_t zi = 2; // z index to make code more readable

  if(new_mode == MODE_SHUT_DOWN){
    // new_mode is MODE_SHUT_DOWN, do nothing except change the mode to MODE_SHUT_DOWN
    mode_phase.mode = new_mode;
  } else {
    // we're not shut down, continue as normal
    // create a variable to hold the next move_seq_phase
    mode_seq_t next_seq_phase;
    if(false){
      next_seq_phase = mode_seq[new_mode][new_phase]; // next_move_seq_phase for this move seq
    } else {
      mode_create_new_seq_phase(&next_seq_phase, new_mode, new_dir, new_phase, indent+1); // next_move_seq_phase for this move seq
    }
    mode_print_sequence(next_seq_phase, indent+1);

    // create temporary versions of next move_part_parameters, move_part_beg_xyz, move_part_activity and move_part_seq_part
    float next_part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM]; // parameters for each of the part move
    float next_part_beg_xyz[MODE_PART_NUM][XYZ]; // xyz coordinates of each part at the beginning of the current move seq (each leg and the body)
    uint8_t next_part_activity[MODE_PART_NUM]; // updated between move sequences, indicates the activity and type of each part (the part type: MODE_PART_TYPE_ & {"SUPPORT_LEG", "ACTIVE_LEG", "STATIC_BODY", "ACTIVE_BODY"})
    uint8_t next_part_seq_part[MODE_PART_NUM]; // updated between move sequences, indicates which seq_part (if any) is associated with the part
    mode_create_next_part_beg_xyz(next_seq_phase, move_part_parameters, move_part_activity, move_part_seq_part, move_part_beg_xyz, next_part_beg_xyz, indent+1);
    mode_create_next_part_parameters(next_seq_phase, next_part_beg_xyz, move_part_parameters, move_part_activity, move_part_seq_part, next_part_parameters, indent+1);
    mode_create_next_part_activity_seq_part(next_seq_phase, next_part_activity, next_part_seq_part, indent+1);
  
    if(local_debug){
      mode_print_parts_xyz("next_part_beg_xyz", next_part_beg_xyz, indent+1);
      mode_print_part_parameters("next_part_parameters", next_part_parameters, indent+1);
      mode_print_parts_activity_seq_part("next_part_activity, next_part_seq_part", next_part_activity, next_part_seq_part, indent+1);
    }
  
    // OK, now update everything!
    mode_copy_part_xyz(next_part_beg_xyz, move_part_beg_xyz, indent+1);
    mode_copy_part_parameters(next_part_parameters, move_part_parameters, indent+1);
    mode_copy_part_data(next_part_activity, move_part_activity, indent+1);
    mode_copy_part_data(next_part_seq_part, move_part_seq_part, indent+1);
  
    if(local_debug && (indent>=0)){
      mode_print_parts_xyz("move_part_beg_xyz", move_part_beg_xyz, indent+1);
      mode_print_part_parameters("move_part_parameters", move_part_parameters, indent+1);
      mode_print_parts_activity_seq_part("move_part_activity, move_part_seq_part", move_part_activity, move_part_seq_part, indent+1);
    }
  
    // now update the move_part_points
    float target_time = -1.0; // initially try to get the part(s) to move as fast as possible
    float slowest_time = 0.0; // assume the slowest part is very fast initially
    float part_time[MODE_SEQ_PART_NUM]; // store the time required for each part
    uint8_t moving_parts = 0;
    for(uint8_t seq_part=0; seq_part<MODE_SEQ_PART_NUM; seq_part++){
      part_time[seq_part] = 0.0; // initialize the move time required by each part to zero
      if(next_seq_phase.part[seq_part].part_id != MODE_PART_NONE_){
        moving_parts = moving_parts + 1;
        // legs_coor_move_points(float target_time, float parameters[XYZ][LEGS_PARAM_NUM], float move_points[XYZ][LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM], int8_t indent)
        target_time = -1.0; // have to make sure it is set to -1.0 for each part initially since we want to see how long each part move takes
        legs_coor_move_points(target_time, move_part_parameters[seq_part], move_part_points[seq_part], indent+1);
        part_time[seq_part] = target_time;
        if(target_time > slowest_time) slowest_time = target_time;
      } else {
        // this part isn't used, set everything to 0.0
        for(uint8_t coor=0; coor<XYZ; coor++){
          for(uint8_t point=0; point<LEGS_MOVE_POINT_NUM; point++){
            for(uint8_t td=0; td<LEGS_MOVE_TD_NUM; td++){
              move_part_points[seq_part][coor][point][td] = 0.0;
            }
          }
        }
      }
    }
  
    if(local_debug){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINTF("slowest_time: %7.2f\n", slowest_time);
      for(uint8_t seq_part=0; seq_part<MODE_SEQ_PART_NUM; seq_part++){
        DEBUG_INDENT(indent+1);
        DEBUG_PRINTF("part_time[%d]: %7.2f\n", seq_part, part_time[seq_part]);
      }
    }
    
    if(moving_parts > 1){
      // make sure all parts have the same time by recomputing the faster part(s)
      for(uint8_t seq_part=0; seq_part<MODE_SEQ_PART_NUM; seq_part++){
        if(part_time[seq_part] < slowest_time){
          legs_coor_move_points(slowest_time, move_part_parameters[seq_part], move_part_points[seq_part], indent+1);
        }
      }
    }
    
    float move_time;
    if(moving_parts == 0){
      // no moving parts, set move time to LEGS_MIN_MOVE_TIME
      move_time = LEGS_MIN_MOVE_TIME; 
    } else {
      // one or more moving parts, set move time to target_time which is the slowest of all the moves
      move_time = slowest_time;
    }
  
    // now update mode_phase times
    if(local_debug && true){
      DEBUG_PRINTF("mode_phase.start_time: %d, mode_phase.end_time: %d\n", mode_phase.start_time, mode_phase.end_time);
    }
    mode_phase.start_time = mode_phase.end_time;
    mode_phase.end_time = mode_phase.start_time + uint32_t(1000.0 * (move_time + 0.0005)); // calculate end_time based on start_time and move_time
//    if(local_debug && true){
    if(true){
      DEBUG_PRINTF("mode_phase.start_time: %d, mode_phase.end_time: %d\n", mode_phase.start_time, mode_phase.end_time);
    }
    
    // update mode_phase.alternate
    if(mode_phase.alternate == 0){
      mode_phase.alternate = 1;
    } else {
      mode_phase.alternate = 0;
    }
    
    // update mode_phase.mode and seq
    mode_phase.mode = new_mode;
    mode_phase.direction = new_dir;
    mode_phase.phase = new_phase;
    mode_seq_t new_mode_seq = mode_seq[new_mode][new_phase];
  }
  if (local_debug){
    mode_print_parts_xyz("move_part_xyz", move_part_xyz, indent+1);
    mode_print_parts_xyz("move_part_beg_xyz", move_part_beg_xyz, indent+1);
    mode_print_parts_activity_seq_part("move_part_activity, move_part_seq_part", move_part_activity, move_part_seq_part, indent+1);
    mode_print_part_parameters("move_part_parameters", move_part_parameters, indent+1);
    mode_print_move_part_points(indent+1);
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("alternate: %d, mode: %s, direction: %d, phase: %d, start_time: %d, end_time: %d\n", mode_phase.alternate, MODE_NAME[mode_phase.mode].c_str(), mode_phase.direction, mode_phase.phase, mode_phase.start_time, mode_phase.end_time);
    DEBUG_PRINT_END(routine, indent);
  }
} // end mode_update_move_part_data


//========================================================
// mode_create_new_seq_phase()
//  creates a new seq_phase that is a reversal of an existing one
//  used for reversing direction
//========================================================
void mode_create_new_seq_phase(mode_seq_t *new_seq_phase, uint8_t new_mode, uint8_t new_dir, uint8_t new_phase, int8_t indent){
  const static char *routine = "mode_create_new_seq_phase";
  LOCAL_DEBUG_ENABLED
  if (local_debug){
    DEBUG_PRINT_BEG(routine, indent);
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("new_mode: %s, new_dir: %s, new_phase: %d\n", MODE_NAME[new_mode].c_str(), MODE_DIR_NAME[new_dir].c_str(), new_phase);
  }

  //if new_dir == MODE_DIR_PLUS, just use normal sequence phase data
  //if new_dir == MODE_DIR_MINUS, use reversed sequence phase data
  //  to reverse MODE_WALKING_BEG, MODE_WALKING, MODE_WALKING_END
  //    switch legs: 0=>2, 1=>3, 2=>0, 3=>1 
  //    reverse sign of all (leg and body) d[y] and v[y]values, 
  //  to reverse MODE_SIDESTEPPING_BEG, MODE_SIDESTEPPING, MODE_SIDESTEPPING_END
  //    switch legs: 0=>2, 1=>3, 2=>0, 3=>1
  //    reverse sign of all (leg and body) d[x] and v[x]values, 
  //  to reverse MODE_ROTATING_BEG, MODE_ROTATING, MODE_ROTATING_END
  //    ??switch legs: 0=>2, 1=>3, 2=>0, 3=>1
  //    ??reverse sign of all (leg and body) d[x] and v[x]values, 

  const uint8_t X = 0;
  const uint8_t Y = 1;
  const uint8_t Z = 2;
  // walking DIR == PLUS: 1, 0, 2, 3  DIR == MINUS: 3, 2, 0, 1  remap: 0=>2, 1=>3, 2=>0, 3=>1
  // sidestepping DIR == PLUS: 2, 1, 3, 0  DIR == MINUS: 0, 3, 1, 2  remap: 0=>2, 1=>3, 2=>0, 3=>1
  uint8_t part_map[MODE_PART_NUM+1] = {0, 1, 2, 3, 4, 5, 6};  // default is to keep legs, bdy_d, bdy_a and none as is
  // const uint8_t part_map_swapped[MODE_PART_NUM+1] = {2, 3, 0, 1, 4, 5, 6}; // reorder legs for walking or sidestepping , keep bdy_d, bdy_a  and none as is
  float xyz_lin_signs[XYZ] = { 1.0, 1.0, 1.0}; // this is for changing the signs on the legs and body linear moves
  // rotating DIR == PLUS: 0, 3, 2, 1  DIR == MINUS: 3, 0, 1, 2  remap: 0=>3, 1=>2, 2=>1, 3=>0
  // const uint8_t part_map_ang_swapped[MODE_PART_NUM+1] = {3, 2, 1, 0, 4, 5, 6}; // reorder legs for rotating , keep bdy_d, bdy_a  and none as is
  float xyz_ang_signs[XYZ] = { 1.0, 1.0, 1.0}; // this is for changing the signs on the body angular moves
  
  if(new_dir == MODE_DIR_MINUS){
    // reverse
    switch(new_mode){
      case MODE_WALKING:
      case MODE_WALKING_BEG:
      case MODE_WALKING_END:
        //for(uint8_t i=0; i<MODE_PART_NUM+1; i++) part_map[i] = part_map_swapped[i]; // use part_map_swapped ordering
        part_map[MODE_PART_LEG_0] = MODE_PART_LEG_2;
        part_map[MODE_PART_LEG_1] = MODE_PART_LEG_3;
        part_map[MODE_PART_LEG_2] = MODE_PART_LEG_0;
        part_map[MODE_PART_LEG_3] = MODE_PART_LEG_1;
        xyz_lin_signs[X] = -1.0; // reverse linear signs for x's
        xyz_lin_signs[Y] = -1.0; // reverse linear signs for y's
        break;
      case MODE_SIDESTEPPING:
      case MODE_SIDESTEPPING_BEG:
      case MODE_SIDESTEPPING_END:
        //for(uint8_t i=0; i<MODE_PART_NUM+1; i++) part_map[i] = part_map_swapped[i]; // use part_map_swapped ordering
        part_map[MODE_PART_LEG_0] = MODE_PART_LEG_2;
        part_map[MODE_PART_LEG_1] = MODE_PART_LEG_3;
        part_map[MODE_PART_LEG_2] = MODE_PART_LEG_0;
        part_map[MODE_PART_LEG_3] = MODE_PART_LEG_1;
        xyz_lin_signs[X] = -1.0; // reverse linear signs for x's
        xyz_lin_signs[Y] = -1.0; // reverse linear signs for y's
        break;
      case MODE_ROTATING:
      case MODE_ROTATING_BEG:
      case MODE_ROTATING_END:
        //for(uint8_t i=0; i<MODE_PART_NUM+1; i++) part_map[i] = part_map_ang_swapped[i]; // use part_map_rot_swapped ordering
        part_map[MODE_PART_LEG_0] = MODE_PART_LEG_3;
        part_map[MODE_PART_LEG_1] = MODE_PART_LEG_2;
        part_map[MODE_PART_LEG_2] = MODE_PART_LEG_1;
        part_map[MODE_PART_LEG_3] = MODE_PART_LEG_0;
        xyz_lin_signs[X] = -1.0; // reverse linear signs for x's
        xyz_ang_signs[Z] = -1.0; // reverse angular signs for az's
        Serial.printf("updated part_map:");
        for(uint8_t i=0; i<MODE_PART_NUM+1; i++){
          Serial.printf("%d ", part_map[i]);
        }
        Serial.println();
        break;
    }
  }

  new_seq_phase->last_phase = mode_seq[new_mode][new_phase].last_phase;
  new_seq_phase->phase_name = mode_seq[new_mode][new_phase].phase_name;

  for(uint8_t part=0; part<MODE_SEQ_PART_NUM; part++){
    mode_seq_part_t seq_part = mode_seq[new_mode][new_phase].part[part];
    if(seq_part.part_id == MODE_PART_BDY_A){
      // it's a body angle move
      new_seq_phase->part[part].part_id = part_map[seq_part.part_id]; // change leg(s) to be moved
      for(uint8_t coor=0; coor<XYZ; coor++){
        new_seq_phase->part[part].d[coor] = xyz_ang_signs[coor] * seq_part.d[coor]; // reverse angular signs where required
        new_seq_phase->part[part].v[coor] = xyz_ang_signs[coor] * seq_part.v[coor]; // reverse angular signs where required
      }      
    } else {
      // for all other part types (body linear and legs), reorder and reverse linear signs
      new_seq_phase->part[part].part_id = part_map[seq_part.part_id]; // change leg(s) to be moved
      for(uint8_t coor=0; coor<XYZ; coor++){
        new_seq_phase->part[part].d[coor] = xyz_lin_signs[coor] * seq_part.d[coor]; // reverse linear signs where required
        new_seq_phase->part[part].v[coor] = xyz_lin_signs[coor] * seq_part.v[coor]; // reverse linear signs where required
      }
    }
  }
  
  if (local_debug){
    mode_print_sequence(*new_seq_phase, indent+1);
    DEBUG_PRINT_END(routine, indent);
  }
} // end mode_create_new_seq_phase


//========================================================
// mode_execute_move()
// use current time and move_points for all parts to calculate its position and move it there
// move_done will be set true when time >= last time in move, the zero_vel variable will be set to true if velocity is zero
//========================================================
void mode_execute_move(float current_seq_time, mode_seq_t mode_seq_phase, boolean *move_done, int8_t indent){
  const static char *routine = "mode_execute_move";
  LOCAL_DEBUG_ENABLED
  if (local_debug){
    DEBUG_PRINT_BEG(routine, indent);
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("current_seq_time: %7.2f, mode_seq_phase description: %s\n", current_seq_time, mode_seq_phase.phase_name.c_str());
    mode_print_current_time(indent+1);
  }

  if(mode_phase.mode == MODE_SHUT_DOWN){
    // do nothing, we're shut down
  } else {
    // proceed as normal
    static const uint8_t xi = 0;
    static const uint8_t yi = 1;
    static const uint8_t zi = 2;
    boolean the_body_moves = (move_part_activity[MODE_PART_BDY_D] == MODE_PART_ACTIVITY_ACTIVE_BDY_D);
    boolean the_body_rotates = (move_part_activity[MODE_PART_BDY_A] == MODE_PART_ACTIVITY_ACTIVE_BDY_A);
    uint8_t body_move_seq_part_i = move_part_seq_part[MODE_PART_BDY_D];
    uint8_t body_rot_seq_part_i = move_part_seq_part[MODE_PART_BDY_A];
  
    if(local_debug && true){
      mode_print_part_parameters("move_part_parameters", move_part_parameters, indent+1);
      mode_print_move_part_points(indent+1);
    }
    if(local_debug && true){
      mode_print_parts_xyz("move_part_xyz", move_part_xyz, indent+1);
      mode_print_parts_xyz("move_part_beg_xyz", move_part_beg_xyz, indent+1);
    }
  
    float body_move_position[XYZ] = {}; // initialize array to zero
    float body_rot_position[XYZ] = {}; // initialize array to zero
    float leg_move_position[XYZ] = {}; // initialize array to zero
    // first let's update the body part if there is one
    if(the_body_moves){
      // yes, there is a moving body in this move seq
      // this needs access to the move_points for the part, the start coordinates of the part
      legs_position(current_seq_time, move_part_parameters[body_move_seq_part_i], move_part_points[body_move_seq_part_i], body_move_position, indent+1);
      // update the coordinates of the body
      for(uint8_t coor=0; coor<XYZ; coor++){
        move_part_xyz[MODE_PART_BDY_D][coor] = move_part_beg_xyz[MODE_PART_BDY_D][coor] + body_move_position[coor];
      }
    }
    if(the_body_rotates){
      // yes, there is a rotating body in this move seq
      // this needs access to the move_points for the part, the start coordinates of the part
      legs_position(current_seq_time, move_part_parameters[body_rot_seq_part_i], move_part_points[body_rot_seq_part_i], body_rot_position, indent+1);
      // update the coordinates of the body
      for(uint8_t coor=0; coor<XYZ; coor++){
        move_part_xyz[MODE_PART_BDY_A][coor] = move_part_beg_xyz[MODE_PART_BDY_A][coor] + body_rot_position[coor];
      }
    }
    
//    if (local_debug && true){
    if (true){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINT("move_part_beg_zyx[MODE_PART_BDY_D]:\t");
      for(uint8_t coor=0; coor<XYZ; coor++){
        DEBUG_PRINTF("%7.2f\t", move_part_beg_xyz[MODE_PART_BDY_D][coor]);
      }
      DEBUG_PRINTLN();
      DEBUG_INDENT(indent+1);
      DEBUG_PRINT("body_move_position:\t\t\t");
      for(uint8_t coor=0; coor<XYZ; coor++){
        DEBUG_PRINTF("%7.2f\t", body_move_position[coor]);
      }
      DEBUG_PRINTLN();
      DEBUG_INDENT(indent+1);
      DEBUG_PRINT("move_part_xyz[MODE_PART_BDY_D]:\t");
      for(uint8_t coor=0; coor<XYZ; coor++){
        DEBUG_PRINTF("%7.2f\t", move_part_xyz[MODE_PART_BDY_D][coor]);
      }
      DEBUG_PRINTLN();

      DEBUG_INDENT(indent+1);
      DEBUG_PRINT("move_part_beg_zyx[MODE_PART_BDY_A]:\t");
      for(uint8_t coor=0; coor<XYZ; coor++){
        DEBUG_PRINTF("%7.2f\t", move_part_beg_xyz[MODE_PART_BDY_A][coor]);
      }
      DEBUG_PRINTLN();
      DEBUG_INDENT(indent+1);
      DEBUG_PRINT("body_rot_position:\t\t\t");
      for(uint8_t coor=0; coor<XYZ; coor++){
        DEBUG_PRINTF("%7.2f\t", body_rot_position[coor]);
      }
      DEBUG_PRINTLN();
      DEBUG_INDENT(indent+1);
      DEBUG_PRINT("move_part_xyz[MODE_PART_BDY_A]:\t");
      for(uint8_t coor=0; coor<XYZ; coor++){
        DEBUG_PRINTF("%7.2f\t", move_part_xyz[MODE_PART_BDY_A][coor]);
      }
      DEBUG_PRINTLN();
}
  
    // now let's update the legs in the list of all possible parts
    for(uint8_t part=0; part<MODE_PART_NUM; part++){
      if(local_debug){
        DEBUG_INDENT(indent+1);
        DEBUG_PRINTF("part: %d, activity type: %d, activity name: %s\n", part, move_part_activity[part], MODE_PART_ACTIVITY_NAME[move_part_activity[part]].c_str());
      }

      if(move_part_activity[part] == MODE_PART_ACTIVITY_ACTIVE_LEG){
        // if the part is an ACTIVE_LEG, update the coordinates based on the active leg move only
        legs_position(current_seq_time, move_part_parameters[move_part_seq_part[part]], move_part_points[move_part_seq_part[part]], leg_move_position, indent+1);
        for(uint8_t coor=0; coor<XYZ; coor++){
           move_part_xyz[part][coor] = move_part_beg_xyz[part][coor] + leg_move_position[coor];
        }
      }

      if(move_part_activity[part] ==  MODE_PART_ACTIVITY_SUPPORT_LEG){
        // if the part is a SUPPORT_LEG, update the coordinates based on the body's linear and angular motion
        float temp_xyz[XYZ];
        for(uint8_t coor=0; coor<XYZ; coor++){
          // find the foot position based on the linear motion first
          temp_xyz[coor] = move_part_beg_xyz[part][coor] - body_move_position[coor];
        }
        // now update the relative position based on the body rotation, we are only supporting rotation around z axis for now
        move_part_xyz[part][xi] = temp_xyz[xi] * cos(body_rot_position[zi]) + temp_xyz[yi] * sin(body_rot_position[zi]);
        move_part_xyz[part][yi] = temp_xyz[yi] * cos(body_rot_position[zi]) - temp_xyz[xi] * sin(body_rot_position[zi]);
        move_part_xyz[part][zi] = temp_xyz[zi];
      }
    }
  
    float angle_phk[NUM_LEGS][NUM_JOINTS_LEG];
    legs_angles(move_part_xyz, angle_phk, indent+1);
    servo_set_angles(angle_phk, indent+1);
    *move_done = (millis() >= mode_phase.end_time);
//    *zero_vel = true; // place holder!!!
  }
  if (local_debug){
    mode_print_parts_xyz("move_part_xyz", move_part_xyz, indent+1);
    //mode_print_move_part_points(indent+1);
    //mode_print_parts_beg_xyz(indent+1);
    DEBUG_INDENT(indent+1);
    if(millis() >= mode_phase.end_time){
      DEBUG_PRINTF("millis(): %d >= mode_phase.end_time: %d", millis(), mode_phase.end_time);
    } else {
      DEBUG_PRINTF("millis(): %d < mode_phase.end_time: %d", millis(), mode_phase.end_time);
    }
//    if(current_seq_time >= move_part_points[0][0][0][0]){
//      DEBUG_PRINTF(", current_seq_time: %7.2f >= move_part_points[0][0][0][0]: %7.2f", current_seq_time, move_part_points[0][0][0][0]);
//    } else {
//      DEBUG_PRINTF(", current_seq_time: %7.2f < move_part_points[0][0][0][0]: %7.2f", current_seq_time, move_part_points[0][0][0][0]);      
//    }
//    if(current_seq_time >= move_part_points[1][0][0][0]){
//      DEBUG_PRINTF(", current_seq_time: %7.2f >= move_part_points[1][0][0][0]: %7.2f", current_seq_time, move_part_points[1][0][0][0]);
//    } else {
//      DEBUG_PRINTF(", current_seq_time: %7.2f < move_part_points[1][0][0][0]: %7.2f", current_seq_time, move_part_points[1][0][0][0]);
//    }
    DEBUG_PRINTF(", move_done: %d\n", *move_done);
    DEBUG_PRINT_END(routine, indent);
  }
}
// end mode_execute_move


//========================================================
//  mode_create_next_part_parameters
//    create the next move_part_parameters for each new active seq_part
//    note: the leg move_parameters are relative to the body center exept for z which will be an amout of up/down motion
//          the body move parameters are relative to the beginning body center at ground level
//========================================================
void mode_create_next_part_parameters(mode_seq_t next_seq_phase, float next_part_beg_xyz[MODE_PART_NUM][XYZ], float part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM], uint8_t part_activity_type[MODE_PART_NUM], uint8_t part_seq_part[MODE_PART_NUM], float next_part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM], int8_t indent){
  const static char *routine = "mode_create_next_part_parameters";
  LOCAL_DEBUG_ENABLED
  if (local_debug) DEBUG_PRINT_BEG(routine, indent);

  static const uint8_t xi = 0; // x index to make code more readable
  static const uint8_t yi = 1; // y index to make code more readable
  static const uint8_t zi = 2; // z index to make code more readable

  // initialize to zeros
  for(uint8_t next_seq_part=0; next_seq_part<MODE_SEQ_PART_NUM; next_seq_part++){
    for(uint8_t coor=0; coor<XYZ; coor++){
      for(uint8_t param=0; param<LEGS_PARAM_NUM; param++){
        next_part_parameters[next_seq_part][coor][param] = 0.0;
      }
    }
  }
  //if(local_debug && (indent>=0)) mode_print_part_parameters("next_part_parameters", next_part_parameters, indent+1);

  for(uint8_t next_seq_part=0; next_seq_part<MODE_SEQ_PART_NUM; next_seq_part++){
    // check each of the seq_parts, don't bother with this part if it's not moving
    uint8_t next_part = next_seq_phase.part[next_seq_part].part_id; // this is the actual part index for this seq_part
    if(next_part != MODE_PART_NONE_){
      // an actual part is specified in the move sequence
      if(local_debug && (indent>=0)){
        DEBUG_INDENT(indent+1);
        DEBUG_PRINTF("next_seq_part: %u, part: %s\n", next_seq_part, MODE_PART_NAME[next_part].c_str());
      }
      if(local_debug && (indent>=0)){
        for(uint8_t coor=0; coor<XYZ; coor++){
          DEBUG_INDENT(indent+1);
          DEBUG_PRINTF("next_seq_phase.part[%d].d[%d]: %7.2f\n", next_seq_part, coor, next_seq_phase.part[next_seq_part].d[coor]); // amount to move this part, either absolute or relative
//         DEBUG_PRINTF(", move_part_xyz[next_part][coor]: %7.2f\n", move_part_xyz[next_part][coor]); // current coordinate of part
        }
      }

      // First, set DIST and UPDN
      for(uint8_t coor=0; coor<XYZ; coor++){
        // for each coordinate store the move parameters for this part
        if((next_part != MODE_PART_BDY_D) && (next_part != MODE_PART_BDY_A)){
          // this moving part isn't the body so it must be a leg
          // active leg positions x, y are relative to the body position at the start of the mode, but z is just used for the leg lift amount
          if(coor == zi){
            // moving part is a leg and Z coor
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_DIST] = next_seq_phase.part[next_seq_part].d[coor]; // this is just the lift amount for z
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_UPDN] = 1.0; // set to > 0.0 to signify that this is an up/down move for z
          } else {
            // moving part is a leg and X or Y coor
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_DIST] = next_seq_phase.part[next_seq_part].d[coor] - next_part_beg_xyz[next_part][coor]; // convert the absolute position of d[XY] to a distance = absolute - current
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_UPDN] = -1.0; // set to < 0.0 to signify that this is NOT an up/down move for z
          }
        } else {
          // else (the part is a body)
          // body positions are relative to the body position at the start of the move for x and y, but z is absolute
          if(coor == zi){
            // coor is z so this is absolute position, dist = next position - current position
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_DIST] = next_seq_phase.part[next_seq_part].d[coor] - next_part_beg_xyz[next_part][coor]; // convert the absolute position of d[XY] to a distance = absolute - current
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_UPDN] = -1.0; // set to < 0.0 to signify that this is NOT an up/down move for z
          } else {
            // coor is x or y so this is the distance to move
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_DIST] = next_seq_phase.part[next_seq_part].d[coor]; // this is just the amount to move
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_UPDN] = -1.0; // set to < 0.0 to signify that this is NOT an up/down move
          }
        }
      }
        
      // Second, set DIR
      for(uint8_t coor=0; coor<XYZ; coor++){
        // for each coordinate store the move parameters for this part
        next_part_parameters[next_seq_part][coor][LEGS_PARAM_DIR] = 1.0; // will be changed by legs_coor_move_points() if distance is negative
      }
        
      // Third, set the V_MAX and A_MAX
      for(uint8_t coor=0; coor<XYZ; coor++){
        // for each coordinate store the move parameters for this part
        if(next_part == MODE_PART_BDY_A){
          next_part_parameters[next_seq_part][coor][LEGS_PARAM_V_MAX] = LEGS_ANG_VA_MAX[coor][0]; // initialize with the v_max value from LEGS_ANG_VA_MAX
          next_part_parameters[next_seq_part][coor][LEGS_PARAM_A_MAX] = LEGS_ANG_VA_MAX[coor][1]; // initialize with the a_max value from LEGS_ANG_VA_MAX
        } else {
          next_part_parameters[next_seq_part][coor][LEGS_PARAM_V_MAX] = LEGS_XYZ_VA_MAX[coor][0]; // initialize with the v_max value from LEGS_XYZ_VA_MAX
          next_part_parameters[next_seq_part][coor][LEGS_PARAM_A_MAX] = LEGS_XYZ_VA_MAX[coor][1]; // initialize with the a_max value from LEGS_XYZ_VA_MAX
        }
      }

      // Fourth, set the beginning velocities for the next move for each ACTIVE part
      // if the BDY_D  will be MODE_PART_ACTIVITY_ACTIVE_BDY_D and it was MODE_PART_ACTIVITY_ACTIVE_BDY_D: v_beg velocity is the body's previous v_end velocity
      // if the BDY_D will be MODE_PART_ACTIVITY_ACTIVE_BDY_D and it was MODE_PART_ACTIVITY_STATIC_BDY_D: v_beg velocity is 0.0
      // if the BDY_A will be MODE_PART_ACTIVITY_ACTIVE_BDY_A and it was MODE_PART_ACTIVITY_ACTIVE_BDY_A: v_beg velocity is the body's previous v_end velocity
      // if the BDY_A will be MODE_PART_ACTIVITY_ACTIVE_BDY_A and it was MODE_PART_ACTIVITY_STATIC_BDY_A: v_beg velocity is 0.0
      // if a leg will be MODE_PART_ACTIVITY_ACTIVE_LEG and it was MODE_PART_ACTIVITY_SUPPORT_LEG: v_beg = - body's previous v_end velocity
      // if a leg will be MODE_PART_ACTIVITY_ACTIVE_LEG and it was MODE_PART_ACTIVITY_ACTIVE_LEG: v_beg = the feet's previous v_end velocity
      if(next_part == MODE_PART_BDY_D){
        // the body will be MODE_PART_ACTIVITY_ACTIVE_BDY_D
        for(uint8_t coor=0; coor<XYZ; coor++){
          // for each coordinate store the move parameters for this part
          if(part_activity_type[MODE_PART_BDY_D] == MODE_PART_ACTIVITY_ACTIVE_BDY_D){
            // and it was MODE_PART_ACTIVITY_ACTIVE_BDY_D: v_beg velocity is the body's previous v_end velocity
            uint8_t seq_part_body = part_seq_part[MODE_PART_BDY_D]; // this is the seq_part the body had in the previous move
            // use the previous LEGS_PARAM_V_END but multiply it by the LEGS_PARAM_DIR to get the original LEGS_PARAM_V_END sign back
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_V_BEG] = part_parameters[seq_part_body][coor][LEGS_PARAM_V_END] * part_parameters[seq_part_body][coor][LEGS_PARAM_DIR]; 
          } else {
            // and it was MODE_PART_ACTIVITY_STATIC_BDY_D: v_beg velocity is 0.0
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_V_BEG] = 0.0; 
          }
        }
      } else if(next_part == MODE_PART_BDY_A){
        for(uint8_t coor=0; coor<XYZ; coor++){
          // for each coordinate store the move parameters for this part
          // the body will be MODE_PART_ACTIVITY_ACTIVE_BDY_A
          if(part_activity_type[MODE_PART_BDY_A] == MODE_PART_ACTIVITY_ACTIVE_BDY_A){
            // and it was MODE_PART_ACTIVITY_ACTIVE_BDY_A: v_beg velocity is the body's previous v_end velocity
            uint8_t seq_part_body = part_seq_part[MODE_PART_BDY_A]; // this is the seq_part the body had in the previous move
            // use the previous LEGS_PARAM_V_END but multiply it by the LEGS_PARAM_DIR to get the original LEGS_PARAM_V_END sign back
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_V_BEG] = part_parameters[seq_part_body][coor][LEGS_PARAM_V_END] * part_parameters[seq_part_body][coor][LEGS_PARAM_DIR]; 
          } else {
            // and it was MODE_PART_ACTIVITY_STATIC_BDY_A: v_beg velocity is 0.0
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_V_BEG] = 0.0; 
          }
        }
      } else {
        // it's not the body, it must be a leg, and it will be active in the next move
        // the leg will be MODE_PART_ACTIVITY_ACTIVE_LEG
        if(part_activity_type[next_part] == MODE_PART_ACTIVITY_SUPPORT_LEG){
          // and it was previously MODE_PART_ACTIVITY_SUPPORT_LEG
          float v_beg[XYZ] = {0.0, 0.0, 0.0}; // initialize it to zero before potentially adding velocity components due to body translation and rotation
          if(part_activity_type[MODE_PART_BDY_D] == MODE_PART_ACTIVITY_ACTIVE_BDY_D){
            // the body moved linearly previously, v_beg = v_beg - body's previous v_end linear velocity (if it had any v_end velocity)
            uint8_t seq_part_body_d = part_seq_part[MODE_PART_BDY_D]; // this is the seq_part the body had in the previous move
            // use the previous LEGS_PARAM_V_END but multiply it by the LEGS_PARAM_DIR to get the original LEGS_PARAM_V_END sign back
            for(uint8_t coor=0; coor<XYZ; coor++){
              // for each coordinate store the move parameters for this part
              v_beg[coor] = v_beg[coor] - (part_parameters[seq_part_body_d][coor][LEGS_PARAM_V_END] * part_parameters[seq_part_body_d][coor][LEGS_PARAM_DIR]); 
            }
          }
          if(part_activity_type[MODE_PART_BDY_A] == MODE_PART_ACTIVITY_ACTIVE_BDY_A){
            // the body moved angularly previously, v_beg = v_beg - body's previous v_end angular velocity (if it had any v_end velocity)
            uint8_t seq_part_body_a = part_seq_part[MODE_PART_BDY_A]; // this is the seq_part the body had in the previous move
            // we're only implementing angular rotation around z
            float ang_v_end = part_parameters[seq_part_body_a][zi][LEGS_PARAM_V_END] * part_parameters[seq_part_body_a][zi][LEGS_PARAM_DIR];
            v_beg[xi] = v_beg[xi] - ang_v_end * next_part_beg_xyz[next_part][yi]; // combine the relative linear motion of the body with the relative angular motion of the body
            v_beg[yi] = v_beg[xi] + ang_v_end * next_part_beg_xyz[next_part][xi]; // combine the relative linear motion of the body with the relative angular motion of the body
          }
          for(uint8_t coor=0; coor<XYZ; coor++){
            // for each coordinate store the move parameters for this part
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_V_BEG] = v_beg[coor];
          }
        } else {
          // it was previously MODE_PART_ACTIVITY_ACTIVE_LEG: v_beg = the feet's previous v_end velocity
          uint8_t seq_part_leg = part_seq_part[next_part]; // this is the seq_part this leg had in the previous move
          // use the previous LEGS_PARAM_V_END but multiply it by the LEGS_PARAM_DIR to get the original LEGS_PARAM_V_END sign back
          for(uint8_t coor=0; coor<XYZ; coor++){
            // for each coordinate store the move parameters for this part
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_V_BEG] = part_parameters[seq_part_leg][coor][LEGS_PARAM_V_END] * part_parameters[seq_part_leg][coor][LEGS_PARAM_DIR]; 
          }
        }
      }

      // now write the ending velocities for the next active parts
      //DEBUG_PRINTF("about to write V_END of :%7.2f\n", next_move_seq_phase.part[next_seq_part].v[coor]);
      for(uint8_t coor=0; coor<XYZ; coor++){
        // for each coordinate store the move parameters for this part
        next_part_parameters[next_seq_part][coor][LEGS_PARAM_V_END] = next_seq_phase.part[next_seq_part].v[coor]; // initialize with the move_seq values
      }
      
    } else {
      // part_i == MODE_PART_NONE_
      // do nothing
    }
  }
  
  if (local_debug){
    //mode_print_part_parameters("next_part_parameters", next_part_parameters, indent+1);
    DEBUG_PRINT_END(routine, indent);
  }
} // end mode_create_next_part_parameters


//========================================================
//  mode_create_next_part_activity_seq_part
//========================================================
void mode_create_next_part_activity_seq_part(mode_seq_t next_seq_phase, uint8_t next_part_activity[MODE_PART_NUM], uint8_t next_part_seq_part[MODE_PART_NUM], int8_t indent){
  const static char routine[] = "mode_create_next_part_activity_seq_part";
//  const boolean local_debug = true;
  LOCAL_DEBUG_ENABLED
  if (local_debug) DEBUG_PRINT_BEG(routine, indent);

  // set all part activity to inactive initially
  // set all part seq part to none
  for(uint8_t part=0; part<MODE_PART_NUM; part++){
    if(part == MODE_PART_BDY_D){
      next_part_activity[part] = MODE_PART_ACTIVITY_STATIC_BDY_D;
    } else if(part == MODE_PART_BDY_A){
      next_part_activity[part] = MODE_PART_ACTIVITY_STATIC_BDY_A;
    } else {
      next_part_activity[part] = MODE_PART_ACTIVITY_SUPPORT_LEG;      
    }
    next_part_seq_part[part] = MODE_SEQ_NONE;
  }

  // now see the active ones to active
  for(uint8_t seq_part=0; seq_part<MODE_SEQ_PART_NUM; seq_part++){
    uint8_t part = next_seq_phase.part[seq_part].part_id;
    if(part == MODE_PART_NONE_){
      // do nothing
    } else if (part == MODE_PART_BDY_D){
      next_part_activity[part] = MODE_PART_ACTIVITY_ACTIVE_BDY_D;
      next_part_seq_part[part] = seq_part;
    } else if (part == MODE_PART_BDY_A){
      next_part_activity[part] = MODE_PART_ACTIVITY_ACTIVE_BDY_A;
      next_part_seq_part[part] = seq_part;
    } else {
      next_part_activity[part] = MODE_PART_ACTIVITY_ACTIVE_LEG;
      next_part_seq_part[part] = seq_part;
    }
  }
  
  if (local_debug) DEBUG_PRINT_END(routine, indent);
} // end mode_create_next_part_activity_seq_part


//========================================================
//  mode_create_next_part_beg_xyz
//========================================================
void mode_create_next_part_beg_xyz(mode_seq_t next_seq_phase, float part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM], uint8_t part_activity_type[MODE_PART_NUM], uint8_t part_seq_part[MODE_PART_NUM], float part_beg_xyz[MODE_PART_NUM][XYZ], float next_part_beg_xyz[MODE_PART_NUM][XYZ], int8_t indent){
  // create next_move_part_beg_xyz[MODE_PART_NUM][XYZ] with where they should be at at the end of the move that just finished
  // if the body moved, that will impact the body and all SUPPORT legs (they moved relative to the body because they are at fixed positions on the ground)
  // if any legs were ACTIVE, just use the info from the previous move_points for the ACTIVE legs as that motion is relative to the body
  // what about first time, when there is no previous move? We will initialize the move_points to the correct locations in mode_setup
  // Potential drift:
  // - Body: the body may accumulate some potential drift due to accumulating error from adding incremental changes, for XY this is not critical at this time
  //         for Z this could become an issue but due to the fact that Z is relative to ground level and that ACTIVE legs will be set to absolute positions relative to the body,
  //         this is not expected to be an issue
  // - Legs: the Support legs may accumulate error due to the body error described above but once a leg moves, is ACTIVE, then is is set to an absolute position relative to the body

  //  uses the following passed in globals:
  //    part_activity_type[MODE_PART_NUM]    to see if body or leg was moving
  //    part_seq_part[MODE_PART_NUM]         to find the index of the previously moving body part
  //    move_part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM]               to get DIST & DIR for the distance moved
  
  const static char routine[] = "mode_create_next_part_beg_xyz";
//  const boolean local_debug = true;
  LOCAL_DEBUG_ENABLED
  if (local_debug) DEBUG_PRINT_BEG(routine, indent);

  // first, set next_part_beg_xyz to part_beg_xyz assuming nothing moved during the previous move
  for(uint8_t part=0; part<MODE_PART_NUM; part++){ // for every part
    for(uint8_t coor=0; coor<XYZ; coor++){ // for each coomode_update_move_part_datar
      next_part_beg_xyz[part][coor] = part_beg_xyz[part][coor];
    }
  }
  
  // next, let's see if the body had a linear move, that will impact whether we do anything for the SUPPORT legs
  if(part_activity_type[MODE_PART_BDY_D] == MODE_PART_ACTIVITY_ACTIVE_BDY_D){
    // the body had a linear move in the last move_seq, update the body and all SUPPORT legs
    uint8_t seq_part_body = part_seq_part[MODE_PART_BDY_D]; // since we know the body moved, this may be MODE_SEQ_PART0 or MODE_SEQ_PART1
    float body_displacement[XYZ];
    for(uint8_t coor=0; coor<XYZ; coor++){ // for each coor
      body_displacement[coor] = part_parameters[seq_part_body][coor][LEGS_PARAM_DIST] * part_parameters[seq_part_body][coor][LEGS_PARAM_DIR];
    }
    for(uint8_t part=0; part<MODE_PART_NUM; part++){ // for every part
      for(uint8_t coor=0; coor<XYZ; coor++){ // for each coor
        if(part_activity_type[part] == MODE_PART_ACTIVITY_ACTIVE_BDY_D){
            // update the body begin_XYZ by ADDING the amount of the last body move to each coor
            next_part_beg_xyz[part][coor] = part_beg_xyz[part][coor] + body_displacement[coor];
        } else if(part_activity_type[part] == MODE_PART_ACTIVITY_SUPPORT_LEG){
            // update the SUPPORT leg(s) begin_XYZ by SUBTRACTING the amount of the last body move
            next_part_beg_xyz[part][coor] = part_beg_xyz[part][coor] - body_displacement[coor];
        }
      }
    }
  }

  // now update any ACTIVE leg(s)
  for(uint8_t part=0; part<MODE_PART_NUM; part++){
    if(part_activity_type[part] == MODE_PART_ACTIVITY_ACTIVE_LEG){
      for(uint8_t coor=0; coor<XYZ; coor++){
        // update the ACTIVE leg(s) begin_XYZ by ADDING the amount of this leg's last move
        uint8_t seq_part_leg = part_seq_part[part];
        float leg_displacement = 0.0;
        if(part_parameters[seq_part_leg][coor][LEGS_PARAM_UPDN] < 0.0){
          // it's not updown so add the amount of the move
          leg_displacement =  part_parameters[seq_part_leg][coor][LEGS_PARAM_DIST] * part_parameters[seq_part_leg][coor][LEGS_PARAM_DIR];
        } else {
          // don't update it because the updown in this coor results in a zero move in this coor
        }
        next_part_beg_xyz[part][coor] = part_beg_xyz[part][coor] + leg_displacement;
      }
    }
  }

  // now update for angle changes
  if(part_activity_type[MODE_PART_BDY_A] == MODE_PART_ACTIVITY_ACTIVE_BDY_A){
    // the body had an angular move in the last move_seq, update the body angle and all legs positions, SUPPORT and ACTIVE
    uint8_t xi = 0;
    uint8_t yi = 1;
    uint8_t zi = 2;
    uint8_t seq_part = part_seq_part[MODE_PART_BDY_A];
    float z_angle =  part_parameters[seq_part][zi][LEGS_PARAM_DIST] * part_parameters[seq_part][zi][LEGS_PARAM_DIR];
    float cosz = cos(z_angle);
    float sinz = sin(z_angle);
    for(uint8_t part=0; part<MODE_PART_NUM; part++){
      if(part_activity_type[part] == MODE_PART_ACTIVITY_SUPPORT_LEG){
        float temp_xyz[XYZ];
        for(uint8_t coor=0; coor<XYZ; coor++){
          temp_xyz[coor] = next_part_beg_xyz[part][coor];
        }
        // we're only supporting rotation around z axis for now
        next_part_beg_xyz[part][xi] = temp_xyz[xi] * cosz + temp_xyz[yi] * sinz;
        next_part_beg_xyz[part][yi] = temp_xyz[yi] * cosz - temp_xyz[xi] * sinz;
        next_part_beg_xyz[part][zi] = temp_xyz[zi];
      }
    }
  } 
  
  if (local_debug) DEBUG_PRINT_END(routine, indent);
} // end mode_create_next_part_beg_xyz


////========================================================
//// mode_set_folded
////========================================================
//void mode_set_folded(int8_t indent){
//  mode_set(MODE_FOLDED, MODE_DIR_NONE, indent+1);
//} // end mode_set_folded
//
//
////========================================================
//// mode_set_ready
////========================================================
//void mode_set_ready(int8_t indent){
//  mode_set(MODE_READY, MODE_DIR_NONE, indent+1);
//} // end mode_set_ready
//
//
////========================================================
//// mode_set_dir
////========================================================
//void mode_set_dir(uint8_t newdir, int8_t indent){
//  mode_phase.direction = newdir;
//} // end mode_set_dir


//========================================================
// mode_set_next
// save next mode and direction
//========================================================
void mode_set_next(uint8_t new_mode, uint8_t new_dir, int8_t indent){
  const static char *routine = "mode_set_next";
  LOCAL_DEBUG_ENABLED
  if(local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTF("Beg %s, new_mode: %s, new_mode_dir: %s\n", routine, MODE_NAME[new_mode].c_str(), MODE_DIR_NAME[new_dir].c_str());
    mode_print_current_time(indent+1);
  }

  gmode_next_mode = new_mode;
  gmode_next_dir = new_dir;
  
  if(local_debug){
    DEBUG_PRINT_END(routine, indent);
  }
} // end mode_set_next


//========================================================
// mode_next_mode
//========================================================
uint8_t mode_next_mode(void){
  return gmode_next_mode;
} // end mode_next_mode


//========================================================
// mode_next_dir
//========================================================
uint8_t mode_next_dir(void){
  return gmode_next_dir;
} // end mode_next_dir


//========================================================
// mode_shut_down
//========================================================
void mode_shut_down(int8_t indent){
  mode_set(MODE_SHUT_DOWN, MODE_DIR_PLUS, 0, indent);
} // end mode_shut_down


//========================================================
// mode_set
// change to new mode, new direction, new phase, update start_time and end_time
// update mode_move_part_points for new sequence
//========================================================
void mode_set(uint8_t new_mode, uint8_t new_mode_dir, uint8_t new_mode_phase, int8_t indent){
  const static char routine[] = "mode_set";
//  const boolean local_debug = true;
  uint8_t current_mode = mode_phase.mode;
  uint8_t current_dir = mode_phase.direction;
  uint8_t current_phase = mode_phase.phase;
  if((new_mode == current_mode) && (new_mode_dir == current_dir) && (new_mode_phase == current_phase)){
    // don't set debug_new_mode since we're probably looping at MODE_READY, MODE_FOLDED or MODE_SHUT_DOWN
  } else {
    debug_set_new_mode(); // set new_mode to enable debug
  }
  LOCAL_DEBUG_ENABLED
  if (local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTF("Beg %s, new_mode: %s, new_mode_dir: %s, new_mode_phase: %d\n", routine, MODE_NAME[new_mode].c_str(), MODE_DIR_NAME[new_mode_dir].c_str(), new_mode_phase);
    mode_print_current_time(indent+1);
  }
//  uint8_t new_phase = 0;
  mode_update_move_part_data(new_mode, new_mode_dir, new_mode_phase, indent+1);

  if (local_debug) DEBUG_PRINT_END(routine, indent);
} // end mode_set


//========================================================
// mode_mode_phase_get
//========================================================
mode_phase_t mode_mode_phase_get(void){
  return mode_phase;
} // end mode_mode_phase_get


//========================================================
// mode_mode_get
//========================================================
uint8_t mode__mode_get(void){
//  return mode_phase[0];
  return mode_phase.mode;
} // end mode_mode_get


//========================================================
// mode_phase_get
//========================================================
uint8_t mode_phase_get(void){
  return mode_phase.phase;
} // end mode_phase_get


//========================================================
// mode_value_fold()
//========================================================
boolean mode_value_fold(void){
  return mode_values.fold;
} // end mode_value_fold


//========================================================
// mode_value_vy()
//========================================================
float mode_value_vy(void){
  return mode_values.vy;
} // end mode_value_vy


//========================================================
// mode_value_vx()
//========================================================
float mode_value_vx(void){
  return mode_values.vx;
} // end mode_value_vx


//========================================================
// mode_value_vt()
//========================================================
float mode_value_vt(void){
  return mode_values.vt;
} // end mode_value_throttle


//========================================================
// mode_value_height()
//========================================================
float mode_value_height(void){
  return mode_values.height;
} // end mode_value_height


//========================================================
// mode_value_angle()
//========================================================
float mode_value_angle(void){
  return mode_values.angle;
} // end mode_value_angle


//========================================================
// mode_values_update
// gets the sbus values and converts them to mode values
// based on center, dead-zone and gain
// typical sbus values range from 
//========================================================
void mode_values_update(int8_t indent){
//  const boolean local_debug = false;
//  if(!local_debug) indent = -1;
  LOCAL_DEBUG_DISABLED
  if (local_debug){
    DEBUG_PRINT("Beg mode_values_update; DXe: F, VY, VX, VT, H, A: ");
  }
  int16_t temp; // will have +/- values

  const int16_t FLIGHT_CENTER = 1024; // should be the center reading of the gear switch
  if(sbus_channel(SBUS_FLIGHT) < FLIGHT_CENTER) {
    mode_values.fold = true;
  } else {
    mode_values.fold = false;
  }
//  Serial.printf("in %s, mode_values.fold: %d\n", __func__, mode_values.fold);

  const int16_t THROTTLE_CENTER = 1024; // should be the center reading of the throttle
  const int16_t THROTTLE_DEAD_ZONE = 16; // amount we'll allow the reading to go above or below center while keeping 0.0 value
  const float VY_POS_GAIN = 0.01; // move forward at 0.01 mm/sec for 1 usec pulse, minus sign reverses direction
  const float VY_NEG_GAIN = 0.01; // move backward at 0.01 mm/sec for 1 usec pulse, minus sign reverses direction
  temp = sbus_channel(SBUS_THROTTLE) - THROTTLE_CENTER;
  if(temp > THROTTLE_DEAD_ZONE){
    mode_values.vy = VY_POS_GAIN * float(temp - THROTTLE_DEAD_ZONE);
  } else if(temp < -THROTTLE_DEAD_ZONE) {
    mode_values.vy = VY_NEG_GAIN * float(temp + THROTTLE_DEAD_ZONE);
  } else {
    mode_values.vy = 0.0;
  }
  const int16_t RUDDER_CENTER = 1024; // should be the center reading of the throttle
  const int16_t RUDDER_DEAD_ZONE = 16; // amount we'll allow the reading to go above or below center while keeping 0.0 value
  const float VX_POS_GAIN = -0.01; // move right at 0.01 mm/sec for 1 usec pulse, minus sign reverses direction
  const float VX_NEG_GAIN = -0.01; // move left at 0.01 mm/sec for 1 usec pulse, minus sign reverses direction
  temp = sbus_channel(SBUS_RUDDER) - RUDDER_CENTER;
  if(temp > RUDDER_DEAD_ZONE){
    mode_values.vx = VX_POS_GAIN * float(temp - RUDDER_DEAD_ZONE);
  } else if(temp < -RUDDER_DEAD_ZONE) {
    mode_values.vx = VX_NEG_GAIN * float(temp + RUDDER_DEAD_ZONE);
  } else {
    mode_values.vx = 0.0;
  }
  const int16_t AILERON_CENTER = 1024; // should be the center reading of the throttle
  const int16_t AILERON_DEAD_ZONE = 16; // amount we'll allow the reading to go above or below center while keeping 0.0 value
  const float VT_POS_GAIN = 0.01; // counter clockwise 0.01 radian/sec for 1 usec pulse, minus sign reverses direction 
  const float VT_NEG_GAIN = 0.01; // clockwise 0.01 radian/sec for 1 usec pulse, minus sign reverses direction
  temp = sbus_channel(SBUS_AILERON) - AILERON_CENTER;
  if(temp > AILERON_DEAD_ZONE){
    mode_values.vt = VT_POS_GAIN * float(temp - AILERON_DEAD_ZONE);
  } else if(temp < -AILERON_DEAD_ZONE) {
    mode_values.vt = VT_NEG_GAIN * float(temp + AILERON_DEAD_ZONE);
  } else {
    mode_values.vt = 0.0;
  }
//  const int16_t AUX3_CENTER = 1024; // should be the center reading of the throttle
//  const int16_t AUX3_DEAD_ZONE = 32; // amount we'll allow the reading to go above or below center while keeping 0.0 value
//  const float HEIGHT_POS_GAIN = -0.01; // plus 0.01 mm for 1 usec pulse, minus sign reverses direction
//  const float HEIGHT_NEG_GAIN = -0.01; // minus 0.01 mm for 1 usec pulse, minus sign reverses direction
//  temp = sbus_channel(SBUS_AUX3) - AUX3_CENTER;
//  if(temp > AUX3_DEAD_ZONE){
//    mode_values.height = HEIGHT_POS_GAIN * float(temp - AUX3_DEAD_ZONE);
//  } else if(temp < -AUX3_DEAD_ZONE) {
//    mode_values.height = HEIGHT_NEG_GAIN * float(temp + AUX3_DEAD_ZONE);
//  } else {
//    mode_values.height = 0.0;
//  }
  const int16_t ELEVATOR_CENTER = 1024; // should be the center reading of the throttle
  const int16_t ELEVATOR_DEAD_ZONE = 16; // amount we'll allow the reading to go above or below center while keeping 0.0 value
  const float ANGLE_POS_GAIN = -0.01; // plus 0.01 radian for 1 usec pulse, minus sign reverses direction
  const float ANGLE_NEG_GAIN = -0.01; // minus 0.01 radian for 1 usec pulse, minus sign reverses direction
  temp = sbus_channel(SBUS_ELEVATOR) - ELEVATOR_CENTER;
  if(temp > ELEVATOR_DEAD_ZONE){
    mode_values.angle = ANGLE_POS_GAIN * float(temp - ELEVATOR_DEAD_ZONE);
  } else if(temp < -ELEVATOR_DEAD_ZONE) {
    mode_values.angle = ANGLE_NEG_GAIN * float(temp + ELEVATOR_DEAD_ZONE);
  } else {
    mode_values.angle = 0.0;
  }

  if(local_debug){
    DEBUG_PRINT("\t");
    DEBUG_PRINT(mode_values.fold);
    DEBUG_PRINT("\t");
    DEBUG_PRINTF("%7.2f", mode_values.vy);
    DEBUG_PRINT("\t");
    DEBUG_PRINTF("%7.2f", mode_values.vx);
    DEBUG_PRINT("\t");
    DEBUG_PRINTF("%7.2f", mode_values.vt);
    DEBUG_PRINT("\t");
    DEBUG_PRINTF("%7.2f", mode_values.height);
    DEBUG_PRINT("\t");
    DEBUG_PRINTF("%7.2f\n", mode_values.angle);
  }
  
  return;
} // end mode_values_update
