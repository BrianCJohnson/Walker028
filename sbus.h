//========================================================
// sbus.h
// definitions for servo functions
//========================================================

#ifndef sbus_h
  #define sbus_h
  #include "Arduino.h"

  #define DXe
  #ifdef DXe
    
    #define SBUS_THROTTLE 0
    #define SBUS_AILERON 1
    #define SBUS_ELEVATOR 2
    #define SBUS_RUDDER 3
    #define SBUS_FLIGHT 4
    #define SBUS_PANIC 5
    #define SBUS_FLAPS 6
    #define SBUS_AUX 7
  #else
    #define SBUS_THROTTLE 0
    #define SBUS_AILERON 1
    #define SBUS_ELEVATOR 2
    #define SBUS_RUDDER 3
    #define SBUS_GEAR 4
    #define SBUS_FLAPS 5
    #define SBUS_AUX2 6
    #define SBUS_AUX3 7
  #endif
  
  void sbus_setup(int8_t indent);
  void sbus_update(int8_t indent);
  int16_t sbus_channel(uint8_t channel);
  void sbus_print_channels(void);
  boolean sbus_gear_up(int8_t indent);
  boolean sbus_panic(int8_t indent);
#endif
