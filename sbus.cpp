//====================================================
// sbus.cpp
// sbus functions
// all functions for getting sBus data from
// RC receiver
//====================================================

#include "Arduino.h"
#include "sbus.h"
#include "debug.h"
#include "com.h"

#include <FUTABA_SBUS.h> //note, FUTABA_SBUS.h was modified to "#define port Serial2"

// create sBus, a FUTABA_SBUS object
FUTABA_SBUS sBus;

static const uint8_t SBUS_NUM_CHANNELS = 6;
//uint16_t sbus_channel[SBUS_NUM_CHANNELS];
boolean sbus_panic_pressed = false;

//====================================================
// setup sbus
//====================================================
void sbus_setup(int8_t indent){
  sBus.begin();
//  for(uint8_t i=0; i<; i++){
//    sbus_channel[i] = 0;
//  }
  delay(20); // needed to get receiver ready?
  sbus_panic_pressed = false;
  sbus_update(indent);
}
// end sbus_setup


//====================================================
// update sbus
//====================================================
void sbus_update(int8_t indent){
  const static char *routine = "sbus_update";
  LOCAL_DEBUG_DISABLED
  if (local_debug) DEBUG_PRINT_BEG(routine, indent);
  
  static unsigned long oldmilliseconds, newmilliseconds;
  sBus.FeedLine();
  if (sBus.toChannels == 1){
    sBus.UpdateChannels();
    sBus.toChannels = 0;
    oldmilliseconds = newmilliseconds;
    newmilliseconds = millis();
    if (local_debug) {
      DEBUG_INDENT(indent+1);
      DEBUG_PRINT(newmilliseconds-oldmilliseconds);
      if(true){
        DEBUG_PRINTF("\t%d",sBus.channels[SBUS_THROTTLE]); // throttle
        DEBUG_PRINTF("\t%d",sBus.channels[SBUS_AILERON]); // aileron
        DEBUG_PRINTF("\t%d",sBus.channels[SBUS_ELEVATOR]); // elevator
        DEBUG_PRINTF("\t%d",sBus.channels[SBUS_RUDDER]); // rudder
        DEBUG_PRINTF("\t%d",sBus.channels[SBUS_FLIGHT]); // gear
        DEBUG_PRINTF("\t%d",sBus.channels[SBUS_PANIC]); // flaps
        DEBUG_PRINTF("\t%d",sBus.channels[SBUS_FLAPS]); // aux2
        DEBUG_PRINTF("\t%d\n",sBus.channels[SBUS_AUX]); // aux3
      }
    }
  }  
  if(sBus.channels[SBUS_PANIC] < 400){
    sbus_panic_pressed = true;
    Serial.println("SBUS PANIC PRESSED!");
  }
  if (local_debug) DEBUG_PRINT_END(routine, indent);
}
// end update_sbus


//====================================================
// sbus_print_channels
//====================================================
void sbus_print_channels(void){
  Serial.printf("in sbus_print_channels\t%d,\t%d,\t%d,\t%d", sBus.channels[SBUS_THROTTLE], sBus.channels[SBUS_AILERON], sBus.channels[SBUS_ELEVATOR], sBus.channels[SBUS_RUDDER]);
//  Serial.printf("\t%d,\t%d,\t%d,\t%d\n", sBus.channels[SBUS_GEAR], sBus.channels[SBUS_FLAPS], sBus.channels[SBUS_AUX2], sBus.channels[SBUS_AUX3]);
  Serial.printf("\t%d,\t%d,\t%d,\t%d\n", sBus.channels[SBUS_FLIGHT], sBus.channels[SBUS_PANIC], sBus.channels[SBUS_FLAPS], sBus.channels[SBUS_AUX]);
//  if(false){
//    DEBUG_PRINT("in sbus_print_channels\t");
//    DEBUG_PRINT(sBus.channels[SBUS_THROTTLE]); // throttle
//    DEBUG_PRINT("\t");
//    DEBUG_PRINT(sBus.channels[SBUS_AILERON]); // aileron
//    DEBUG_PRINT("\t");
//    DEBUG_PRINT(sBus.channels[SBUS_ELEVATOR]); // elevator
//    DEBUG_PRINT("\t");
//    DEBUG_PRINT(sBus.channels[SBUS_RUDDER]); // rudder
//    DEBUG_PRINT("\t");
//    DEBUG_PRINT(sBus.channels[SBUS_GEAR]); // gear
//    DEBUG_PRINT("\t");
//    DEBUG_PRINT(sBus.channels[SBUS_FLAPS]); // flaps
//    DEBUG_PRINT("\t");
//    DEBUG_PRINT(sBus.channels[SBUS_AUX2]); // aux2
//    DEBUG_PRINT("\t");
//    DEBUG_PRINT(sBus.channels[SBUS_AUX3]); // aux3
//    DEBUG_PRINTLN();
//  }
} // end sbus_print_channels


//====================================================
// sbus_channel returns data from specified channel
//====================================================
int16_t sbus_channel(uint8_t channel){
  return sBus.channels[channel];
}
// end sbus_channel


//====================================================
// SBUS_GEAR_up returns boolean if "gear" is up
//====================================================
boolean sbus_gear_up(int8_t indent){
  LOCAL_DEBUG_DISABLED
  if(local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINT("in SBUS_GEAR_up, sBus.channels[SBUS_GEAR]:");
    DEBUG_PRINTLN(sBus.channels[SBUS_FLIGHT]);
  }
  return (sBus.channels[SBUS_FLIGHT] < 1024);
}
// end SBUS_GEAR_up


//====================================================
// sbus_panic returns boolean true if PANIC button was ever pressed
//====================================================
boolean sbus_panic(int8_t indent){
  return sbus_panic_pressed;
}
// end sbus_panic

