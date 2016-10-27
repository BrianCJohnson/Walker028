bool debug_new_mode_v;


//========================================================
// debug_new_mode
//========================================================
bool debug_new_mode(void){
  return debug_new_mode_v;
}


//========================================================
// debug_set_new_mode
//========================================================
void debug_set_new_mode(void){
  debug_new_mode_v = true;
}


//========================================================
// debug_clr_new_mode
//========================================================
void debug_clr_new_mode(void){
  debug_new_mode_v = false;
}


