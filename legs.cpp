//========================================================
// legs.cpp
// leg functions
// functions for updating walker position
//========================================================

#include "Arduino.h"
#include "com.h"
#include "legs.h"
#include "servo.h" // to call servo_set_angles 
#include "debug.h"
//#include "mode.h" // to call mode_print_move_part_points(int8_t indent)

static float legs_foot_xyz_retracted[NUM_LEGS][XYZ]; // calculated at setup from above
static float legs_foot_xyz_ready[NUM_LEGS][XYZ]; // calculated at setup from above

//boolean legs_are_folded;
//boolean legs_are_unfolded;
float legs_xyz[NUM_LEGS][XYZ]; // xyz location of each foot, in mm relative to body
float legs_v_xyz[NUM_LEGS][XYZ]; // xyz velocity of each foot, in mm/sec relative to body
float legs_angle[NUM_LEGS][NUM_JOINTS_LEG]; // joint angles of each foot, in radians
//boolean legs_supporting[NUM_LEGS]; // if true, that leg is on the ground, only one may not be supporting 

//========================================================
// legs setup
//========================================================
void legs_setup(int8_t indent){
  const static char *routine = "legs_setup";
  LOCAL_DEBUG_ENABLED
  if (local_debug) DEBUG_PRINT_BEG(routine, indent);
  legs_compute_retracted_and_ready(indent+1);
  for(uint8_t leg=0; leg<NUM_LEGS; leg++){
    for(uint8_t coor=0; coor<XYZ; coor++){
      legs_xyz[leg][coor] = legs_foot_xyz_retracted[leg][coor]; // set initial leg coordinates to "retracted"
      legs_v_xyz[leg][coor] = 0.0; // set initial leg velocities to 0.0
    }
  }
  if(local_debug) legs_print_values("legs_xyz", legs_xyz, indent+1);
  legs_angles(legs_xyz, legs_angle, indent+1); // update leg angles
  if(local_debug) legs_print_values("legs_angle", legs_angle, indent+1);
  servo_set_angles(legs_angle, indent+1); // update servos
  if (local_debug) DEBUG_PRINT_END(routine, indent);
} // end legs_setup


//========================================================
// legs_print_values
//========================================================
void legs_print_values(String value_name, float value[NUM_LEGS][XYZ], int8_t indent){
  com_indent(indent);
  Serial.print("Beg legs_print_values, ");
  Serial.println(value_name);
  for(uint8_t leg=0; leg<NUM_LEGS; leg++){
    com_indent(indent+1);
    for(uint8_t coor=0; coor<XYZ; coor++){
      Serial.printf("%7.2f ", value[leg][coor]);
    }
    Serial.println();
  }
  com_indent(indent);
  Serial.println("End legs_print_values, ");
} // end legs_print_values


//========================================================
// legs_compute_retracted_and_ready
//========================================================
void legs_compute_retracted_and_ready(int8_t indent){
//  const boolean local_debug = false;
//  if(!local_debug) indent = -1;
  LOCAL_DEBUG_DISABLED
  if(local_debug) DEBUG_PRINTLN("Beg legs_compute_retracted_and_ready()");
  static const uint8_t xi = 0;
  static const uint8_t yi = 1;
//  static const uint8_t z = 2;
  
  for(uint8_t leg=0; leg<NUM_LEGS; leg++){
    for(uint8_t coor=0; coor<XYZ; coor++){
      if((leg == 1) || (leg == 3)){
        if(coor == xi){
          legs_foot_xyz_retracted[leg][xi] = LEGS_FOOT_XYZ_SIGNS[leg][xi] * LEGS_XYZ_RETRACTED[yi];
        } else if (coor == yi){
          legs_foot_xyz_retracted[leg][yi] = LEGS_FOOT_XYZ_SIGNS[leg][yi] * LEGS_XYZ_RETRACTED[xi];
        } else {
          legs_foot_xyz_retracted[leg][coor] = LEGS_FOOT_XYZ_SIGNS[leg][coor] * LEGS_XYZ_RETRACTED[coor];
        }
      } else {
        legs_foot_xyz_retracted[leg][coor] = LEGS_FOOT_XYZ_SIGNS[leg][coor] * LEGS_XYZ_RETRACTED[coor];
      }
      legs_foot_xyz_ready[leg][coor] = LEGS_FOOT_XYZ_SIGNS[leg][coor] * LEGS_XYZ_READY[coor];
    }
  }  
  if(local_debug) DEBUG_PRINTLN("End legs_compute_retracted_and_ready()");
} // end legs_compute_retracted_and_ready


////========================================================
//// legs update
////========================================================
//void legs_update(){
//  switch (legs_active_sequence){
//    case LEGS_SEQ_UNFOLDING:
//      // continue unfolding
//      
//      break;
//    case LEGS_SEQ_FOLDING:
//      // continue folding
//      break;
//    case LEGS_SEQ_MOVING:
//      // continue moving
//      break;
//  }
//} // end legs_update


//========================================================
// legs position_tests
// tests:
//   void legs_position(float the_time, float v_max[XYZ], float a_max[XYZ], int8_t dir[XYZ], float move_points[XYZ][LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM], float legs_position[3]){
//   void legs_coor_move_points(float target_time, float move_xyz[XYZ], float v_max[XYZ], float a_max[XYZ], int8_t dir[XYZ], float move_points[XYZ][LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM]){
//   float legs_distance(float the_time, float v_max, float a_max, float move_point[3][LEGS_MOVE_TD_NUM]){
//========================================================
void legs_position_tests(int8_t indent){
  com_indent(indent);
  Serial.println("In legs_position_tests");
  //     LEGS_PARAM_NAME[LEGS_PARAM_NUM] = {"DIST", "DIR", "UPDN", "V_MAX", "A_MAX", "V_BEG", "V_END"};
  float parameters[XYZ][LEGS_PARAM_NUM] = {{   0.0,   1.0,   -1.0,   400.0,   200.0,     0.0,     0.0},
                                           { 300.0,   1.0,   -1.0,   400.0,   200.0,     0.0,     0.0},
                                           {  50.0,   1.0,    1.0,   400.0,   200.0,     0.0,     0.0}};
//  int8_t dir[3];
  for(uint8_t param=0; param<LEGS_PARAM_NUM; param++){
    com_indent(indent);
    Serial.print("\t");
    Serial.print(LEGS_PARAM_NAME[param]);
    Serial.print("\t");
    for(uint8_t coor=0; coor<XYZ; coor++){
      Serial.print("\t");
      Serial.print(parameters[coor][param]);
    }
    Serial.println();
  }
  float move_points[XYZ][LEGS_MOVE_POINT_NUM][2];
  float target_time = -1.0;
  legs_coor_move_points(target_time, parameters, move_points, indent+1);
  Serial.println("  after legs_coor_move_points()");
  for(uint8_t param=0; param<LEGS_PARAM_NUM; param++){
    DEBUG_INDENT(indent);
    Serial.print("\t");
    Serial.print(LEGS_PARAM_NAME[param]);
    Serial.print("\t");
    for(uint8_t coor=0; coor<XYZ; coor++){
      Serial.print("\t");
      Serial.print(parameters[coor][param]);
    }
    Serial.println();
  }
//  Serial.println();
//  Serial.print("dir:\t\t");
//  for(uint8_t coor=0; coor<3; coor++){
//    Serial.print("\t");
//    Serial.print(dir[coor]);
//  }
//  Serial.println();
  com_indent(indent);
  Serial.println("\tPoint_\t\ttd__\tX___\tY___\tZ___");
  for(uint8_t point=0; point<3; point++){
    com_indent(indent);
    for(uint8_t td=0; td<2; td++){
      Serial.print("\t");
      Serial.print(LEGS_MOVE_NAME[point]);
      Serial.print("\t");
      Serial.print(LEGS_MOVE_TD_NAME[td]);
      for(uint8_t coor=0; coor<3; coor++){
        Serial.print("\t");
        Serial.print(move_points[coor][point][td]);
      }
      Serial.println();
    }
  }
  for(uint8_t point=3; point<7; point++){
    com_indent(indent);
    for(uint8_t td=0; td<2; td++){
      Serial.print("\t");
      Serial.print(LEGS_MOVE_NAME[point]);
      Serial.print("\t");
      Serial.print(LEGS_MOVE_TD_NAME[td]);
      Serial.print("\t");
      Serial.print("\t");
      for(uint8_t coor=2; coor<3; coor++){
        Serial.print("\t");
        Serial.print(move_points[coor][point][td]);
      }
      Serial.println();
    }
  }
  // now calculate and display the x, y, z positions for the sequence over time
  float leg_position[3];
  uint8_t time_points = 50; // number of points to show for test
  float the_time;
  float end_time = move_points[0][0][0]; // x, point0, time
  com_indent(indent);
  Serial.println("\t\tTime_\tX___\tY___\tZ___");
  for(uint8_t point = 0; point<time_points; point++){
    the_time = float(point)/float(time_points-1)*end_time;
    legs_position(the_time, parameters, move_points, leg_position, indent+1);
    com_indent(indent);
    Serial.print("\t\t");
    Serial.print(the_time);
    for(uint8_t i=0; i<3; i++){
      Serial.print("\t");
      Serial.print(leg_position[i]);     
    }
    Serial.println();
  }
  com_indent(indent);
  Serial.println("End legs_position_tests");
} // end legs_position_tests


//========================================================
// legs position
// computes the x, y, z position for a given time based on 
// maximum velocities: {v_max_x, v_max_y, v_max_z}
// maximum accelerations: {a_max_x, a_max_y, a_max_z}
// and three arrays of move points
//   for x: {{end_time, end_dist}, {dec_time, dec_dist}, {cv_time, cv_dist}, , , , }
//   for y: {{end_time, end_dist}, {dec_time, dec_dist}, {cv_time, cv_dist}, , , , }
//   for z: {{end_time, end_dist}, {dec2_time, dec2_dist}, {cv2_time, cv2_dist}, {acc2_time, acc2_dist}, {wait_time, wait_dist}, {dec1_time, dec1_dist}, {cv1_time, cv1_dist}}
// remember, distances are alway positive!
//========================================================
void legs_position(float the_time, float parameters[XYZ][LEGS_PARAM_NUM], float move_points[XYZ][LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM], float leg_position[XYZ], int8_t indent){
  // get the x and y positions which only acc, optional constant vel, dec
  const static char *routine = "legs_position";
  LOCAL_DEBUG_ENABLED
  if (local_debug) DEBUG_PRINT_BEG(routine, indent);
  for (uint8_t coor=0; coor<XYZ; coor++){
    leg_position[coor] = legs_distance(the_time, parameters[coor], move_points[coor], indent+1);
    if(parameters[coor][LEGS_PARAM_DIR] < 0.0){
      if (local_debug){
        DEBUG_INDENT(indent+1);
        DEBUG_PRINTLN("reversing direction");
      }
      leg_position[coor] = -leg_position[coor];
    }
  }
  if (local_debug) DEBUG_PRINT_END(routine, indent);
} // end legs_position


//========================================================
// legs distance
// computes the distance for a given time based on a maximum velocity, maximum acceleration
// and an array of move points {{end_time, end_dist}, {dec_time, dec_dist}, {cv_time, cv_dist}}
// remember, distances are alway positive!
//========================================================
float legs_distance(float the_time, float parameters[LEGS_PARAM_NUM], float move_point[LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM], int8_t indent){
  const static char routine[] = "legs_distance";
  float updn = parameters[LEGS_PARAM_UPDN];
  float v_max = parameters[LEGS_PARAM_V_MAX];
  float a_max = parameters[LEGS_PARAM_A_MAX];
  float v_beg = parameters[LEGS_PARAM_V_BEG];
  float v_end = parameters[LEGS_PARAM_V_END];
  float delta_time;
  float distance;

//  static const boolean local_debug = false;
//  if(!local_debug) indent = -1;
  LOCAL_DEBUG_ENABLED
  if (local_debug){
    DEBUG_PRINT_BEG(routine, indent);
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("the_time: %7.2f", the_time);
    DEBUG_PRINTF(", updn: %7.2f", updn);
    DEBUG_PRINTF(", v_max: %7.2f", v_max);
    DEBUG_PRINTF(", a_max: %7.2f", a_max);
    DEBUG_PRINTF(", v_beg: %7.2f", v_beg);
    DEBUG_PRINTF(", v_end: %7.2f\n", v_end);
  }

  boolean up_down = updn > 0.0;
  if(the_time > move_point[LEGS_MOVE_END][LEGS_MOVE_TIME]){
    // after end of move
    delta_time = the_time - move_point[LEGS_MOVE_END][LEGS_MOVE_TIME]; // in this case it is time after the end
    if(local_debug) DEBUG_PRINTLN("at 0");
    distance = move_point[LEGS_MOVE_END][LEGS_MOVE_DIST] + delta_time * v_end; // no more acc/dec, just the v_end
  } else if(the_time > move_point[LEGS_MOVE_DEC2][LEGS_MOVE_TIME]){

    // in deceleration 2, near end of 2nd movement
    delta_time = move_point[LEGS_MOVE_END][LEGS_MOVE_TIME] - the_time; // in this case it is time before the end
    if(up_down){
      if(local_debug) DEBUG_PRINTLN("at 1");
      distance = move_point[LEGS_MOVE_END][LEGS_MOVE_DIST] + (0.5 * a_max * delta_time * delta_time); // ignores non-zero v_end
    } else {
      if(local_debug){
        DEBUG_PRINTLN("at 2");
        DEBUG_PRINTF("move_point[LEGS_MOVE_END][LEGS_MOVE_DIST]: %7.2f", move_point[LEGS_MOVE_END][LEGS_MOVE_DIST]);
        DEBUG_PRINTF(", delta_time: %7.2f", delta_time);
        DEBUG_PRINTF(", v_end: %7.2f", v_end);
        DEBUG_PRINTF(", a_max: %7.2f", a_max);
        DEBUG_PRINTF(", minus term: %7.2f\n", ((delta_time * v_end) + (0.5 * a_max * delta_time * delta_time)));
      }
      distance = move_point[LEGS_MOVE_END][LEGS_MOVE_DIST] - ((delta_time * v_end) + (0.5 * a_max * delta_time * delta_time)); // handles non-zero v_end
    }
  } else if(the_time > move_point[LEGS_MOVE_CV2][LEGS_MOVE_TIME]){

    // during constant velocity 2, in middle of 2nd movement
    delta_time = the_time - move_point[LEGS_MOVE_CV2][LEGS_MOVE_TIME];
    if(up_down){
      if(local_debug) DEBUG_PRINTLN("at 3");
      distance = move_point[LEGS_MOVE_CV2][LEGS_MOVE_DIST] - (delta_time * v_max); // if up_down the cv2 is negative
    } else {
      if(local_debug) DEBUG_PRINTLN("at 4");
      distance = move_point[LEGS_MOVE_CV2][LEGS_MOVE_DIST] + (delta_time * v_max);
    }
  } else if(!up_down){

    // in acc if only one movement
    if(local_debug) DEBUG_PRINTLN("at 5");
    distance = (v_beg * the_time) + (0.5 * a_max * the_time * the_time); // handles non-zero v_beg
  } else if(the_time > move_point[LEGS_MOVE_ACC2][LEGS_MOVE_TIME]){

    // in acceleration 2, near start of 2nd movement
    delta_time = the_time - move_point[LEGS_MOVE_ACC2][LEGS_MOVE_TIME];
    if(local_debug) DEBUG_PRINTLN("at 6");
    distance = move_point[LEGS_MOVE_ACC2][LEGS_MOVE_DIST] - 0.5 * a_max * delta_time * delta_time; // ignores non-zero v_beg
  } else if(the_time > move_point[LEGS_MOVE_WAIT][LEGS_MOVE_TIME]){

    // in wait period, between two movements
    if(local_debug) DEBUG_PRINTLN("at 7");
    distance = move_point[LEGS_MOVE_WAIT][LEGS_MOVE_DIST];
  } else if(the_time > move_point[LEGS_MOVE_DEC1][LEGS_MOVE_TIME]){

    // in dec 1, near end of 1st movement
    delta_time = move_point[LEGS_MOVE_WAIT][LEGS_MOVE_TIME] - the_time; // the time before the end of the dec
    if(local_debug) DEBUG_PRINTLN("at 8");
    distance = move_point[LEGS_MOVE_WAIT][LEGS_MOVE_DIST] - 0.5 * a_max * delta_time * delta_time;
  } else if(the_time > move_point[LEGS_MOVE_CV1][LEGS_MOVE_TIME]){

    // in cv 1, in middle of 1st movement
    delta_time = the_time - move_point[LEGS_MOVE_CV1][LEGS_MOVE_TIME];
    if(local_debug) DEBUG_PRINTLN("at 9");
    distance = move_point[LEGS_MOVE_CV1][LEGS_MOVE_DIST] + v_max * delta_time;
  } else {

    // in acceleration 1, near start of 1st movement
    distance = 0.5 * a_max * the_time * the_time;
  }
  if (local_debug){
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("distance: %7.2f\n", distance);
    DEBUG_PRINT_END(routine, indent);
  }
  return distance;
} // end legs_distance


//========================================================
// legs_print_parameters
//========================================================
void legs_print_parameters(float parameters[XYZ][LEGS_PARAM_NUM], int8_t indent){
  String coor_name[XYZ] = {"x:", "y:", "z:"};
  DEBUG_INDENT(indent);
  DEBUG_PRINTLN("Beg legs_print_parameters");
  DEBUG_INDENT(indent+1);
  DEBUG_PRINT("  \t");
  for(uint8_t param=0; param<LEGS_PARAM_NUM; param++){
    DEBUG_PRINTF("  %s\t", LEGS_PARAM_NAME[param].c_str());
  }
  DEBUG_PRINTLN();
  for(uint8_t coor=0; coor<XYZ; coor++){
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("%s",coor_name[coor].c_str());
    for(uint8_t param=0; param<LEGS_PARAM_NUM; param++){
      DEBUG_PRINTF("\t%7.2f",parameters[coor][param]);
    }
    DEBUG_PRINTLN();
  }
  DEBUG_INDENT(indent);
  DEBUG_PRINTLN("End legs_print_parameters");
} // end legs_print_parameters


//========================================================
// legs_print_move_points
//========================================================
void legs_print_move_points(float move_points[XYZ][LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM], int8_t indent){
  String coor_name[XYZ] = {"x", "y", "z"};
  DEBUG_INDENT(indent);
  DEBUG_PRINTLN("Beg legs_print_move_points");
  DEBUG_INDENT(indent+1);
  DEBUG_PRINT("\t\t");
  for(uint8_t point=0; point<LEGS_MOVE_POINT_NUM; point++){
    DEBUG_PRINTF("  %s\t",LEGS_MOVE_NAME[point].c_str());
  }
  DEBUG_PRINTLN();
  for(uint8_t coor=0; coor<XYZ; coor++){
    for(uint8_t td=0; td<LEGS_MOVE_TD_NUM; td++){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINTF("%s, %s:", coor_name[coor].c_str(), LEGS_MOVE_TD_NAME[td].c_str());
      for(uint8_t point=0; point<LEGS_MOVE_POINT_NUM; point++){
        DEBUG_PRINTF("\t%7.2f",move_points[coor][point][td]);
      }
      DEBUG_PRINTLN();
    }
  }
  DEBUG_INDENT(indent);
  DEBUG_PRINTLN("End legs_print_move_points");
} // end legs_print_move_points


//========================================================
// legs_coor_move_points
// computes an array of move points {{end_time, end_dist}, {dec_time, dec_dist}, {cv_time, cv_dist}}
// based on a distance, maximum velocity, maximum acceleration, beginning velocity, ending velocity
// const String LEGS_PARAM_NAME[] = {"dist", "v_max", "a_max", "v_beg", "v_end"};
// the v_max[] and a_max[] x or y values will probably be changed so that the x and y moves happen during the same time period
// the move_xyz z value may be reduced in order to allow up_down motion during the horizontal move
// note the path starts at x=0, y=0, z=0 and will change z to the specified z value and then return it to zero if parameters[zi][LEGS_PARAM_UPDN] is > 0.0
// remember, distances are alway positive! If parameters[coor][LEGS_PARAM_DIST] is not, it will become positive and the sign of parameters[coor][LEGS_PARAM_DIR] will be reversed
//========================================================
void legs_coor_move_points(float &target_time, float parameters[XYZ][LEGS_PARAM_NUM], float move_points[XYZ][LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM], int8_t indent){
  // if target_time < 0.0 then do as fast as possible
  // if target_time > 0.0 do in target time if it's >= as fast as possible
  const char *routine = "legs_coor_move_points";
  const char *time_lt_zero = "time is less than zero";
  const char *times_are_zero = "all times are zero";

  LOCAL_DEBUG_ENABLED
  if(local_debug) DEBUG_PRINT_BEG(routine, indent);

  // first convert postitions to distances and sign
  if(signed_distance){
    // do nothing
  } else {
    for(uint8_t coor=0; coor<XYZ; coor++){
      if(abs(parameters[coor][LEGS_PARAM_DIST]) < COM_ZERO) parameters[coor][LEGS_PARAM_DIST] = 0.0; // make -0.0 => +0.0 positive
      if(parameters[coor][LEGS_PARAM_DIST] < -COM_ZERO){
        parameters[coor][LEGS_PARAM_DIST] = -parameters[coor][LEGS_PARAM_DIST];
        parameters[coor][LEGS_PARAM_V_BEG] = -parameters[coor][LEGS_PARAM_V_BEG];
        parameters[coor][LEGS_PARAM_V_END] = -parameters[coor][LEGS_PARAM_V_END];
        parameters[coor][LEGS_PARAM_DIR] = -1.0;  
      }
    }
  }
  if(local_debug){
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("target_time: %7.2f\n", target_time);
    legs_print_parameters(parameters, indent+1);
  }

  if(target_time < 0.0){
    // now compute the x, y and z move times as fast as possible
    for(uint8_t coor=0; coor<XYZ; coor++){
      legs_move_point(target_time, parameters[coor], move_points[coor], indent+1); // calculate the move_points coor
    }
    if(local_debug){
      legs_print_move_points(move_points, indent+1);
    }
    // if the move time is close to zero, make it zero
    // and find the slowest time and use that for the move
    float t[XYZ]; // time for each coordinate's move
    float slowest_time;
    if(target_time < 0.0){
      slowest_time = 0.0; // initialize the slowest time to zero
    } else {
      slowest_time = target_time; // initilize the slowest time to target_time
    }
    
    int8_t slowest_coor = -1; // initialize to -1 so we know if 
    for(uint8_t coor=0; coor<XYZ; coor++){
      t[coor] = move_points[coor][LEGS_MOVE_END][LEGS_MOVE_TIME];
      if(abs(t[coor]) <= COM_ZERO){
        t[coor] = 0.0; // if the time for a coor is close to zero, make it zero
        move_points[coor][LEGS_MOVE_END][LEGS_MOVE_TIME] = t[coor];
      }
      if(t[coor] < 0.0) com_err_msg(routine, time_lt_zero); // make sure the time are not less than zero, otherwise error!
      // if this is an up/down move, double the time
      if(parameters[coor][LEGS_PARAM_UPDN] > 0.0){
        t[coor] = 2.0 * t[coor];
      }
      if(t[coor] > slowest_time){
        // this coordinate is slower
        slowest_coor = coor; // assume that x is the slowest coordinate
        slowest_time = t[coor]; // set the new slowest time
      }
    }
    
    // check to see if the slowest is zero, that means they're all zero
    // do we expect to ever have a move with no time??
    if(slowest_time < COM_ZERO){
      // they are all effectively zero
      com_err_msg(routine, times_are_zero); 
    }
  
    // now slow down the faster movements by recomputing their a_max parameters
    for(uint8_t coor=0; coor<XYZ; coor++){
      if(coor == slowest_coor){
        // it's already slow, do nothing
      } else {
        // need to slow down this coor move unless it's very close to the slowest_time
        if(abs(slowest_time - t[coor]) < COM_ZERO){
          // it's nearly the same time as the slowest_time, do nothing
        } else {
          // scale the faster coor moves
          if(parameters[coor][LEGS_PARAM_UPDN] > 0.0){
            // this is an up/down move so halve the target time
            legs_move_point_scale_a_max(parameters[coor], move_points[coor], 0.5*slowest_time, indent+1); // now recompute the move_points for the fast coordinate that we want slowed down
 //           legs_move_point_for_updn(move_points[coor], indent+1); // now compute the move_points for the up/down
          } else {
            // this is not an up/down move (so don't halve the target time)
            legs_move_point_scale_a_max(parameters[coor], move_points[coor], slowest_time, indent+1); // now recompute the move_points for the fast coordinate that we want slowed down
          }
        }
      }
      if(parameters[coor][LEGS_PARAM_UPDN] > 0.0){
        legs_move_point_for_updn(move_points[coor], indent+1); // now compute the move_points for the up/down
      }
    }
    target_time = slowest_time; // update target_time with the slowest time
    if(local_debug){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINTF("slowest_time: %7.2f, target_time: %7.2f\n", slowest_time, target_time);
    }
    
  } else {
    // target_time > 0.0, force the moves to happen in target_time
    // now slow down the faster movements by recomputing their a_max parameters
    for(uint8_t coor=0; coor<XYZ; coor++){
      // need to slow down this coor move unless it's very close to the slowest_time
      // scale the faster coor moves
      if(parameters[coor][LEGS_PARAM_UPDN] > 0.0){
        // this is an up/down move so halve the target time
        legs_move_point_scale_a_max(parameters[coor], move_points[coor], 0.5*target_time, indent+1); // now recompute the move_points for the fast coordinate that we want slowed down
        legs_move_point_for_updn(move_points[coor], indent+1); // now compute the move_points for the up/down
      } else {
        // this is not an up/down move (so don't halve the target time)
        legs_move_point_scale_a_max(parameters[coor], move_points[coor], target_time, indent+1); // now recompute the move_points for the fast coordinate that we want slowed down
      }
    }
  }
  
  if(local_debug){
    legs_print_parameters(parameters, indent+1);
    legs_print_move_points(move_points, indent+1);
    DEBUG_PRINT_END(routine, indent);
  }
} // end legs_coor_move_points


//========================================================
// legs move point
// computes the key points required to move a given distance based on a maximum velocity and acceleration
// the returned array includes time and distance to cv (constant velocity / stop accelerating), to dec (decelerating) and to end
// calculates an array with times and distances of {{end_time, end_dist}, {dec_time, dec_dist}, {cv_time, cv_dist}} 
// the array could be used along with v_max, a_max to calculate the correct position at any given time
// if target_time < 0.0 then do as fast as possible
// if target_time > 0.0 do in target time if it's >= as fast as possible
//========================================================
void legs_move_point(float &target_time, float parameters[LEGS_PARAM_NUM], float move_point[3][LEGS_MOVE_TD_NUM], int8_t indent){
  float distance = parameters[LEGS_PARAM_DIST]; // desired move distance
  float v_max = parameters[LEGS_PARAM_V_MAX]; // maximum allowed velocity
  float a_max = parameters[LEGS_PARAM_A_MAX]; // maximum allowed acceleration
  float v_beg = parameters[LEGS_PARAM_V_BEG]; // velocity at beginning of move
  float v_end = parameters[LEGS_PARAM_V_END]; // velocity at end of move
  const char *routine = "legs_move_point";
  const char v_beg_gt_v_max[] = "abs(v_beg) > v_max";
  const char v_end_gt_v_max[] = "abs(v_end) > v_max";
  const char v_max_le_zero[] = "v_max <= zero";
  const char a_max_le_zero[] = "a_max <= zero";
  const char v_mid_sqrd_le_zero[] = "v_mid_sqrd <= zero";

  LOCAL_DEBUG_ENABLED
  if(local_debug){
    DEBUG_PRINT_BEG(routine, indent);
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("target_time: %7.2f\n", target_time);
  }
  
  if(abs(v_beg) > v_max){
    com_err_msg(routine, v_beg_gt_v_max);
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("abs(v_beg) (%7.2f) > v_max (%7.2f)\n", abs(v_beg), v_max);
  }
  if(abs(v_end) > v_max){
    com_err_msg(routine, v_end_gt_v_max);
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("abs(v_end) (%7.2f) > v_max (%7.2f)\n", abs(v_end), v_max);
  }
  if(v_max < COM_ZERO){
    com_err_msg(routine, v_max_le_zero);
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("v_max (%7.2f) <= zero\n", v_max);
  }
  if(a_max < COM_ZERO){
    com_err_msg(routine, a_max_le_zero);
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("a_max (%7.2f) <= zero\n", a_max);
  }

  if(local_debug){
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("distance: %7.2f", distance);
    DEBUG_PRINTF(", v_max: %7.2f", v_max);
    DEBUG_PRINTF(", a_max: %7.2f", a_max);
    DEBUG_PRINTF(", v_beg: %7.2f", v_beg);
    DEBUG_PRINTF(", v_end: %7.2f\n", v_end);
  }

  if((abs(distance) < COM_ZERO) && (abs(v_beg) < COM_ZERO) && (abs(v_end) < COM_ZERO)){
    // don't have to do any calculations, just set everything to zero unlesss target_time > 0.0
    // if target_time > 0.0 set the end time to target_time for each coordinate
    for(uint8_t point=0; point<LEGS_MOVE_POINT_NUM; point++){
      for(uint8_t td=0; td<LEGS_MOVE_TD_NUM; td++){
        if((target_time > 0.0) && (point == LEGS_MOVE_END) && (td == LEGS_MOVE_TIME)){
          move_point[point][td] = target_time;
        }else{
          move_point[point][td] = 0.0;
        }
      }
    }
  } else {
    // non-zero move request
    float t_to_vm = (v_max - v_beg) / a_max; // time to v_max from beginning
    float d_to_vm = (v_beg * t_to_vm) + (0.5 * a_max * t_to_vm * t_to_vm); // distance to v_max from beginning
    float t_fm_vm = (v_max - v_end) / a_max; // time from v_max to end
    float d_fm_vm = (v_end * t_fm_vm) + (0.5 * a_max * t_fm_vm * t_fm_vm); // distance from v_max to end
    if(local_debug){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINTF("t_to_vm: %7.2f, d_to_vm: %7.2f, t_fm_vm: %7.2f, d_fm_vm: %7.2f\n", t_to_vm, d_to_vm, t_fm_vm, d_fm_vm);
    }
    float end_time;
    if(target_time < 0.0){
      // not being forced to target_time, just do as fast as possible limited by v_max and a_max
      if((d_to_vm + d_fm_vm) < distance){
        // limited by v_max
        end_time = ((distance - (d_to_vm + d_fm_vm)) / v_max) + t_to_vm + t_fm_vm; // end_time when limited by v_max
        if(local_debug){
          DEBUG_INDENT(indent+1);
          DEBUG_PRINTF("limited by v_max, end_time: %7.2f\n", end_time);
        }
        move_point[LEGS_MOVE_END][LEGS_MOVE_TIME] = end_time; // end_time
        move_point[LEGS_MOVE_END][LEGS_MOVE_DIST] = distance;
        move_point[LEGS_MOVE_DEC2][LEGS_MOVE_TIME] = end_time - t_fm_vm; // dec time
        move_point[LEGS_MOVE_DEC2][LEGS_MOVE_DIST] = distance - d_fm_vm; // dec distance
        move_point[LEGS_MOVE_CV2][LEGS_MOVE_TIME] = t_to_vm; // t_to_mv = start of cv_time
        move_point[LEGS_MOVE_CV2][LEGS_MOVE_DIST] = d_to_vm;
      }else{
        // limited by a_max, never get to v_max
        float v_mid_sqrd = distance * a_max + 0.5*(v_beg * v_beg + v_end * v_end); // v_mid = sqrt(distance*a_max + 0.5*(v_beg^2 + v_end^2))
        if(v_mid_sqrd < COM_ZERO){
          com_err_msg(routine, v_mid_sqrd_le_zero);
          Serial.printf("v_mid_sqrd (%7.2f) <= zero\n", v_mid_sqrd);
        }
        float v_mid = sqrt(v_mid_sqrd); // v_mid = sqrt(distance*a_max + 0.5*(v_beg^2 + v_end^2))
        float t_beg = (v_mid - v_beg) / a_max; // time from beginning to mid point
        float t_end = (v_mid - v_end) / a_max; // time from end to mid point
        //float d_beg = (v_beg * t_beg) + (0.5 * a_max * t_beg * t_beg); // distance from beginning to mid point, not required
        float d_end = (v_end * t_end) + (0.5 * a_max * t_end * t_end); // distance from end to mid point
        end_time = t_beg + t_end; // total time
        if(local_debug){
          DEBUG_INDENT(indent+1);
          DEBUG_PRINTF("limited by a_max, never get to v_max, v_mid: %7.2f", v_mid);
          DEBUG_PRINTF(", t_beg: %7.2f", t_beg);
          DEBUG_PRINTF(", t_end: %7.2f", t_end);
          //DEBUG_PRINTF(", d_beg: %7.2f", d_beg);
          DEBUG_PRINTF(", d_end: %7.2f", d_end);
          DEBUG_PRINTF(", end_time: %7.2f\n", end_time);
        }
        move_point[LEGS_MOVE_END][LEGS_MOVE_TIME] = end_time; // end time
        move_point[LEGS_MOVE_END][LEGS_MOVE_DIST] = distance; // end distance
        move_point[LEGS_MOVE_DEC2][LEGS_MOVE_TIME] = end_time - t_end; // dec time
        move_point[LEGS_MOVE_DEC2][LEGS_MOVE_DIST] = distance - d_end; // dec distance
        move_point[LEGS_MOVE_CV2][LEGS_MOVE_TIME] = end_time - t_end; // cv time, shouldn't be required but...
        move_point[LEGS_MOVE_CV2][LEGS_MOVE_DIST] = distance - d_end; // cv LEGS_MOVE_DISTst, shouldn't be required but...
      }
    } else {
      // target_time > 0.0
      // forced to target_time, a_max should have been precomputed properly already
      // should be limited by calculated a_max, so we never get to v_max
      float v_mid = 0.5 * (target_time * a_max + v_beg + v_end); // v_mid = (t*a+v_beg+v_end)/2
      float t_beg = (v_mid - v_beg) / a_max; // time from beginning to mid point
      float t_end = (v_mid - v_end) / a_max; // time from end to mid point
      float d_beg = (v_beg * t_beg) + (0.5 * a_max * t_beg * t_beg); // distance from beginning to mid point, not required
      float d_end = (v_end * t_end) + (0.5 * a_max * t_end * t_end); // distance from end to mid point
      end_time = t_beg + t_end; // total time
      if( abs(end_time - target_time) > COM_ZERO){
        // these should be the same, report error
        com_err_msg(routine, "end_time != target_time");
        DEBUG_INDENT(indent+1);
        DEBUG_PRINTF("end_time (%7.2f) != target_time (%7.2f)\n", end_time, target_time);
      }
      if(local_debug){
        DEBUG_INDENT(indent+1);
        DEBUG_PRINTF("forced to target_time, limited by a_max, never get to v_max, v_mid: %7.2f", v_mid);
        DEBUG_PRINTF(", t_beg: %7.2f", t_beg);
        DEBUG_PRINTF(", t_end: %7.2f", t_end);
        DEBUG_PRINTF(", d_beg: %7.2f", d_beg);
        DEBUG_PRINTF(", d_end: %7.2f", d_end);
        DEBUG_PRINTF(", end_time: %7.2f\n", end_time);
      }
      move_point[LEGS_MOVE_END][LEGS_MOVE_TIME] = end_time; // end time
      move_point[LEGS_MOVE_END][LEGS_MOVE_DIST] = distance; // end distance
      move_point[LEGS_MOVE_DEC2][LEGS_MOVE_TIME] = end_time - t_end; // dec time
      move_point[LEGS_MOVE_DEC2][LEGS_MOVE_DIST] = distance - d_end; // dec distance
      move_point[LEGS_MOVE_CV2][LEGS_MOVE_TIME] = end_time - t_end; // cv time, shouldn't be required but...
      move_point[LEGS_MOVE_CV2][LEGS_MOVE_DIST] = distance - d_end; // cv LEGS_MOVE_DISTst, shouldn't be required but...
    }
  }
  if(local_debug) DEBUG_PRINT_END(routine, indent);
} // end legs_move_point

  
//========================================================
// legs_move_point_for_updn
// uses the move_point data from a unidirectional move to create the move_point data for an up/down move
// note: the time for the up/down move will be twice as much as for the unidirectional move
//========================================================
void legs_move_point_for_updn(float move_point[LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM], int8_t indent){
//  static const boolean local_debug = true;
//  if(!local_debug) indent = -1;
  LOCAL_DEBUG_ENABLED
  if(local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("Beg legs_move_point_for_updn");
  }
  // shift the values and compute the other move_points so we include the downward part of an up/down move
  // {{end_time, end_dist}, {dec2_time, dec2_dist}, {cv2_time, cv2_dist}, {acc2_time, acc2_dist}, {wait_time, wait_dist}, {dec1_time, dec1_dist}, {cv1_time, cv1_dist}}
  // get some info on acc/dec and cv periods
  float end_time =  2.0 * move_point[LEGS_MOVE_END][LEGS_MOVE_TIME]; // the time for the up/down move will be twice as much as for the unidirectional move
  float acc_t =  move_point[LEGS_MOVE_CV2][LEGS_MOVE_TIME]; // all 4 acc/dec periods have the same time
  float acc_d =  move_point[LEGS_MOVE_CV2][LEGS_MOVE_DIST]; // all 4 acc/dec periods have the same distance
  float cv_t =  move_point[LEGS_MOVE_DEC2][LEGS_MOVE_TIME] - acc_t; // both cv periods have the same time
  float cv_d =  move_point[LEGS_MOVE_DEC2][LEGS_MOVE_DIST] - acc_d; // both cv periods have the same distance
  // calculate time and distance for the first 3 points from the start
  move_point[LEGS_MOVE_CV1][LEGS_MOVE_TIME] = acc_t; // at start of cv 1
  move_point[LEGS_MOVE_CV1][LEGS_MOVE_DIST] = acc_d; // at start of cv 1
  move_point[LEGS_MOVE_DEC1][LEGS_MOVE_TIME] = move_point[LEGS_MOVE_CV1][LEGS_MOVE_TIME] + cv_t; // at start of dec 1
  move_point[LEGS_MOVE_DEC1][LEGS_MOVE_DIST] = move_point[LEGS_MOVE_CV1][LEGS_MOVE_DIST] + cv_d; // at start of dec 1
  move_point[LEGS_MOVE_WAIT][LEGS_MOVE_TIME] = move_point[LEGS_MOVE_DEC1][LEGS_MOVE_TIME] + acc_t; // at start of wait
  move_point[LEGS_MOVE_WAIT][LEGS_MOVE_DIST] = move_point[LEGS_MOVE_DEC1][LEGS_MOVE_DIST] + acc_d; // at start of wait
  // calculate time and distance for the last 4 points from the end
  move_point[LEGS_MOVE_END][LEGS_MOVE_TIME] = end_time; // the end of the move
  move_point[LEGS_MOVE_END][LEGS_MOVE_DIST] = 0.0; // the end of the move, back to z = 0.0
  move_point[LEGS_MOVE_DEC2][LEGS_MOVE_TIME] = move_point[LEGS_MOVE_END][LEGS_MOVE_TIME] - acc_t; // at start of dec 2
  move_point[LEGS_MOVE_DEC2][LEGS_MOVE_DIST] = move_point[LEGS_MOVE_END][LEGS_MOVE_DIST] + acc_d; // at start of dec 2
  move_point[LEGS_MOVE_CV2][LEGS_MOVE_TIME] = move_point[LEGS_MOVE_DEC2][LEGS_MOVE_TIME] - cv_t; // at start of cv 2
  move_point[LEGS_MOVE_CV2][LEGS_MOVE_DIST] = move_point[LEGS_MOVE_DEC2][LEGS_MOVE_DIST] + cv_d; // at start of cv 2
  move_point[LEGS_MOVE_ACC2][LEGS_MOVE_TIME] = move_point[LEGS_MOVE_CV2][LEGS_MOVE_TIME] - acc_t; // at start of acc 2
  move_point[LEGS_MOVE_ACC2][LEGS_MOVE_DIST] = move_point[LEGS_MOVE_CV2][LEGS_MOVE_DIST] + acc_d; // at start of acc 2
  if(local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("End legs_move_point_for_updn");
  }
} // end legs_move_point_for_updn


//========================================================
// legs_move_point_scale_a_max
// calculates a scaled a_max to cause the move to happen during specified time = t
// note that in some cases a_max may be negative!
// the time, t, should be 1/2 in the call if this is an updn move
//========================================================
void legs_move_point_scale_a_max(float parameters[LEGS_PARAM_NUM], float move_point[3][LEGS_MOVE_TD_NUM], float t, int8_t indent){
  const char *routine = "legs_move_point_scale_a_max";
  const char term_lt_zero[] = "term < zero";
  const char time_le_zero[] = "time <= zero";

  static const boolean local_debug_1 = false;
  LOCAL_DEBUG_ENABLED
  if (local_debug) DEBUG_PRINT_BEG(routine, indent);
  
  if(t <= COM_ZERO) com_err_msg(routine, time_le_zero); // error!
  
  // need to solve a quadratic of form a*a_max^2 + b*a_max + c = 0.0
  float distance = parameters[LEGS_PARAM_DIST]; // desired move distance
  float updn = parameters[LEGS_PARAM_UPDN]; // desired move distance
  float v_max = parameters[LEGS_PARAM_V_MAX]; // maximum allowed velocity
  float a_max = parameters[LEGS_PARAM_A_MAX]; // maximum allowed acceleration
  float v_beg = parameters[LEGS_PARAM_V_BEG]; // velocity at beginning of move
  float v_end = parameters[LEGS_PARAM_V_END]; // velocity at end of move

  if(local_debug){
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("distance: %7.2f, updn: %7.2f, v_max: %7.2f, a_max: %7.2f, v_beg: %7.2f, v_end: %7.2f, time: %7.2f\n", distance, updn, v_max, a_max, v_beg, v_end, t);
  }
  if((abs(distance) < COM_ZERO) && (abs(v_beg) < COM_ZERO) && (abs(v_end) < COM_ZERO)){
    // don't have to do any calculations, just set everything to zero except end time, v_max and a_max
    for(uint8_t point=0; point<LEGS_MOVE_POINT_NUM; point++){
      for(uint8_t td=0; td<LEGS_MOVE_TD_NUM; td++){
        move_point[point][td] = 0.0;
      }
    }
    move_point[LEGS_MOVE_END][LEGS_MOVE_TIME] = t; // set the end time to the requested time
    parameters[LEGS_PARAM_V_MAX] = 0.0; // set v_max to zero
    parameters[LEGS_PARAM_A_MAX] = 0.0; // set a_max to zero
  
  } else {
    
    // we need to do calculations
    float a = t*t; // t^2
    float b = 2.0 * t * (v_beg + v_end) - 4.0 * distance; // 2*t*(v_beg + v_end) - 4*d
    float c = - (v_beg - v_end) * (v_beg - v_end); // -(v_beg-v_end)^2
    float term = b * b - (4.0 * a * c); // b^2-4*a*c
    if(abs(term) < COM_ZERO) term = 0.0; // in case term is close to zero, make it zero
    if(term < 0.0) com_err_msg(routine, term_lt_zero); // error!
    
    if(local_debug_1){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINTF(">>>> term is: %7.2f\n", term);
    }
    float sqrt_term = sqrt(term); // sqrt(b^2-4*a*c)
    if(local_debug_1){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINTF(">>>> a: %7.2f, b: %7.2f, c: %7.2f, sqrt_term is: %7.2f\n", a, b, c, sqrt_term);
    }
    float a_max_plus = -b + sqrt_term; // we want either the sum, or the difference
    float a_max_minus = -b - sqrt_term; // we want either the sum, or the difference
    if(abs(a_max_plus) >= abs(a_max_minus)){
      a_max = a_max_plus / (2.0 * a);
    } else {
      a_max = a_max_minus / (2.0 * a);
    }
    if(local_debug_1){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINTF(">>>> a_max is: %7.2f, a_max_plus is: %7.2f, a_max_minus is: %7.2f\n", a_max, a_max_plus, a_max_minus);
    }
    parameters[LEGS_PARAM_A_MAX] = a_max; // maximum allowed acceleration
  
    if(local_debug){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINTF("distance: %7.2f, updn: %7.2f, v_max: %7.2f, a_max: %7.2f, v_beg: %7.2f, v_end: %7.2f, time: %7.2f\n", distance, updn, v_max, a_max, v_beg, v_end, t);
    }

    float v_mid = 0.5*(t*a_max + v_beg + v_end);
    float t_beg = (v_mid - v_beg)/a_max;
    float t_end = (v_mid - v_end)/a_max;
    float acc_term = 0.5 * a_max * t_beg * t_beg;
    float d_beg = v_beg*t_beg + acc_term;
    float d_end = v_end*t_end + acc_term;

    if(local_debug){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINTF("v_mid: %7.2f, t_beg: %7.2f, t_end: %7.2f, d_beg: %7.2f, d_end: %7.2f\n", v_mid, t_beg, t_end, d_beg, d_end);
    }

    move_point[LEGS_MOVE_END][LEGS_MOVE_TIME] = t; // end time
    move_point[LEGS_MOVE_END][LEGS_MOVE_DIST] = distance; // end distance
    move_point[LEGS_MOVE_DEC2][LEGS_MOVE_TIME] = t_beg; // dec time
    move_point[LEGS_MOVE_DEC2][LEGS_MOVE_DIST] = d_beg; // dec distance
    move_point[LEGS_MOVE_CV2][LEGS_MOVE_TIME] = t_beg; // cv time, shouldn't be required but...
    move_point[LEGS_MOVE_CV2][LEGS_MOVE_DIST] = d_beg; // cv LEGS_MOVE_DISTst, shouldn't be required but...
  }
  
  if (local_debug) DEBUG_PRINT_END(routine, indent);
} // end legs_move_point_scale_a_max


//========================================================
// legs angles
// computes the angles for the three leg joints for all legs (angle_phk[foot][phk]
// input the xyz coordiates of each leg foot (foot_xyz[foot][xyz])
//========================================================
void legs_angles(float foot_xyz[NUM_LEGS][3], float angle_phk[NUM_LEGS][NUM_JOINTS_LEG], int8_t indent){
  const boolean local_debug = false;
  if(!local_debug) indent = -1;
  if(local_debug && (indent>=0)) DEBUG_PRINTLN("In legs_angles(): ");
  for(uint8_t leg=0; leg<NUM_LEGS; leg++){
    leg_angles(leg, foot_xyz[leg], angle_phk[leg], indent+1);
  }
  if(local_debug && (indent>=0)){
    for(uint8_t leg=0; leg<NUM_LEGS; leg++){
      for(uint8_t joint=0; joint<NUM_JOINTS_LEG; joint++){
        DEBUG_PRINT("\t");
        DEBUG_PRINTF("%7.2f", angle_phk[leg][joint]);
      }
      DEBUG_PRINTLN();
    }
  }
} // legs_angles


//========================================================
// leg angles
// computes the angles for the three leg joints
// input leg index and the xyz coordiates of the leg foot
//========================================================
void leg_angles(uint8_t leg, float foot_xyz[3], float angle[NUM_JOINTS_LEG], int8_t indent){
//  const boolean local_debug = true;
  LOCAL_DEBUG_ENABLED
  if(local_debug) DEBUG_PRINT("In leg_angles: ");
  //const static uint8_t dim = 3;
  const static uint8_t x = 0;
  const static uint8_t y = 1;
  const static uint8_t z = 2;
  const static uint8_t a = 3;
  const static uint8_t pivot = 0;
  const static uint8_t hip = 1;
  const static uint8_t knee = 2;
  const static float pivot_xyza[NUM_LEGS][4] = {
    { LEGS_CENTER_2_PIVOT_XY,  LEGS_CENTER_2_PIVOT_XY, 0.0,  0.78540},
    { LEGS_CENTER_2_PIVOT_XY, -LEGS_CENTER_2_PIVOT_XY, 0.0, -0.78540},
    {-LEGS_CENTER_2_PIVOT_XY, -LEGS_CENTER_2_PIVOT_XY, 0.0, -2.35619},
    {-LEGS_CENTER_2_PIVOT_XY,  LEGS_CENTER_2_PIVOT_XY, 0.0,  2.35619}
  };
  float pivot2foot_xyz[XYZ];
  for(uint8_t i=0; i<XYZ; i++){
    pivot2foot_xyz[i] = foot_xyz[i] - pivot_xyza[leg][i];
  }
  float pivot_angle = atan2(pivot2foot_xyz[y],pivot2foot_xyz[x]);
  float pivot2foot_gnd = sqrt((pivot2foot_xyz[x] * pivot2foot_xyz[x]) + (pivot2foot_xyz[y] * pivot2foot_xyz[y]));
  float hip2foot_gnd = pivot2foot_gnd - LEGS_PIVOT_2_HIP;
  float hip2foot = sqrt((pivot2foot_xyz[z] * pivot2foot_xyz[z]) + (hip2foot_gnd * hip2foot_gnd));
  float hip2foot_angle = asin(pivot2foot_xyz[z] / hip2foot);
  float knee_angle = acos((LEGS_HIP_2_KNEE_SQR_PLUS_KNEE2FOOT_SQR - (hip2foot * hip2foot)) / LEGS_TWO_HIP2KNEE_TIMES_KNEE2FOOT);
  float thigh2hipfoot_angle = acos(((hip2foot * hip2foot) + LEGS_HIP_2_KNEE_SQR_MINUS_KNEE2FOOT_SQR) / (hip2foot * LEGS_TWO_HIP_2_KNEE));
  angle[hip] = hip2foot_angle + thigh2hipfoot_angle;
  angle[pivot] = pivot_angle - pivot_xyza[leg][a];
  if(angle[pivot] > PI) angle[pivot] = angle[pivot] - 2.0*PI;
  if(angle[pivot] < -PI){ 
    //Serial.printf("angle[pivot] (%7.2f) < -PI (%7.2f)", angle[pivot], -PI);
    angle[pivot] = angle[pivot] + 2.0*PI;
    //Serial.printf(", after, angle[pivot] (%7.2f)\n", angle[pivot]);
  }
  angle[knee] = LEGS_PI_MINUS_KNEE_ANGLE_OFFSET - knee_angle;
  // we are not checking to make sure we haven't exceed the max angles
  // we will check for that in the call to servo_set_angle_to_target()
  if(local_debug){
    DEBUG_PRINT(", pivot2foot_x: ");
    DEBUG_PRINTF("%7.2f", pivot2foot_xyz[x]);
    DEBUG_PRINT(", pivot2foot_y: ");
    DEBUG_PRINTF("%7.2f", pivot2foot_xyz[y]);
    DEBUG_PRINT(", pivot_angle: ");
    DEBUG_PRINTF("%7.2f", pivot_angle);
    DEBUG_PRINT(", pivot2foot_gnd: ");
    DEBUG_PRINTF("%7.2f", pivot2foot_gnd);
    DEBUG_PRINT(", hip2foot_gnd: ");
    DEBUG_PRINTF("%7.2f", hip2foot_gnd);
    DEBUG_PRINT(", hip2foot: ");
    DEBUG_PRINTF("%7.2f", hip2foot);
    DEBUG_PRINT(", hip2foot_angle: ");
    DEBUG_PRINTF("%7.2f\n", hip2foot_angle);
  }
//    Serial.print(", pivot2foot_x: ");
//    Serial.printf("%7.2f", pivot2foot_xyz[x]);
//    Serial.print(", pivot2foot_y: ");
//    Serial.printf("%7.2f", pivot2foot_xyz[y]);
//    Serial.print(", pivot_angle: ");
//    Serial.printf("%7.2f", pivot_angle);
//    Serial.print(", pivot2foot_gnd: ");
//    Serial.printf("%7.2f", pivot2foot_gnd);
//    Serial.print(", hip2foot_gnd: ");
//    Serial.printf("%7.2f", hip2foot_gnd);
//    Serial.print(", hip2foot: ");
//    Serial.printf("%7.2f", hip2foot);
//    Serial.print(", hip2foot_angle: ");
//    Serial.printf("%7.2f\n", hip2foot_angle);
} // end leg_angles


////========================================================
//// legs_change_sequence
//// called from mode to start a new leg sequence
////========================================================
//boolean legs_change_sequence(uint8_t new_request){
//  boolean request_is_valid;
//  switch (new_request){
//    case LEGS_SEQ_FOLDED:
//      switch (legs_active_sequence){
//        case LEGS_SEQ_FOLDED:
//        case LEGS_SEQ_FOLDING:
//          // currently LEGS_SEQ_FOLDED or LEGS_SEQ_FOLDING
//          // request to be folded
//          // OK, no change
//          request_is_valid = true;
//          break;
//        case LEGS_SEQ_UNFOLDED:
//        case LEGS_SEQ_UNFOLDING:
//          // currently LEGS_SEQ_UNFOLDING or LEGS_SEQ_UNFOLDED
//          // request to be folded
//          // OK, change to folding
//          legs_active_sequence = LEGS_SEQ_FOLDING;
//          request_is_valid = true;
//          break;
//        case LEGS_SEQ_MOVING:
//          // currently LEGS_SEQ_MOVING
//          // request to be folded
//          // Error, can't change to folded from moving, must change to unfolded (not moving) first
//          request_is_valid = true;
//          break;
//      }
//      break;
//    case LEGS_SEQ_UNFOLDING:
//      // Error, can't request a change to a transitory state
//      request_is_valid = false;
//      break;
//    case LEGS_SEQ_UNFOLDED:
//      switch (legs_active_sequence){
//        case LEGS_SEQ_FOLDED:
//        case LEGS_SEQ_FOLDING:
//          // currently LEGS_SEQ_FOLDED or LEGS_SEQ_FOLDING
//          // request to be unfolded
//          // OK, change to unfolding
//          legs_active_sequence = LEGS_SEQ_UNFOLDING;
//          request_is_valid = true;
//          break;
//        case LEGS_SEQ_UNFOLDING:
//        case LEGS_SEQ_UNFOLDED:
//          // currently LEGS_SEQ_UNFOLDING or LEGS_SEQ_UNFOLDED
//          // request to be unfolded
//          // OK, no change
//          request_is_valid = true;
//          break;
//        case LEGS_SEQ_MOVING:
//          // currently LEGS_SEQ_MOVING
//          // request to be unfolded
//          // OK, 
//          request_is_valid = false;
//          break;
//      }
//      break;
//    case LEGS_SEQ_FOLDING:
//      request_is_valid = true;
//      break;
//    case LEGS_SEQ_MOVING:
//      request_is_valid = true;
//      break;
//  }
//  return request_is_valid;
//} // end legs_change_sequence
//
//
////========================================================
//// legs sequence
//// returns legs_active_sequence
////========================================================
//uint8_t legs_sequence(void){
//  return legs_active_sequence;
//}
//
//
////========================================================
//// legs folded
//// returns true if legs_active_sequence == LEGS_SEQ_FOLDED
////========================================================
//boolean legs_folded(void){
//  return (legs_active_sequence == LEGS_SEQ_FOLDED);
//}
//
//
////========================================================
//// legs unfolded
//// returns true if legs_active_sequence == LEGS_SEQ_UNFOLDED
////========================================================
//boolean legs_unfolded(void){
//  return (legs_active_sequence == LEGS_SEQ_UNFOLDED);
//}
//
//
////========================================================
//// legs fold
//// sequence to fold legs, could be from any starting position
//// returns true when done
////========================================================
//boolean legs_fold(void){
//  if (legs_are_folded) {
//    return true; 
//  } else {
//    // fold legs
//    return false; // only if still in process of folding
//  }
//} // end legs_fold
//
//
////========================================================
//// legs unfold
//// sequence to unfold legs, could be from any starting position
//// returns true when done
////========================================================
//boolean legs_unfold(void){
//  if (legs_are_unfolded) {
//    return true; 
//  } else {
//    // unfold legs
//    return false; // only if still in process of unfolding
//  }
//} // end legs_unfold
