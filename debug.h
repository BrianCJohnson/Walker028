//========================================================
// debug.h
// definitions for debug functions
//========================================================

#ifndef debug_h
  #define debug_h
  #include "Arduino.h"

//  #define DEBUG // comment this out to disable debug printout

  #ifdef DEBUG
//    #define LOCAL_DEBUG_ENABLED   boolean local_debug; \
//                            if(indent<0){ \
//                              local_debug = false; \
//                            } else { \
//                              local_debug = true;  \
//                            } // if we want local_debug enabled but controlled by indent
    #define LOCAL_DEBUG_ENABLED   boolean local_debug; \
                            if(indent<0){ \
                              local_debug = false; \
                            } else { \
                              local_debug = debug_new_mode();  \
                            } // if we want local_debug enabled but controlled by indent
    #define LOCAL_DEBUG_DISABLED    static const boolean local_debug = false; \
                              indent = -9; // if we don't want local_debug or subroutines enabled
    #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
    #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
    #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
    #define DEBUG_INDENT(...) com_indent(__VA_ARGS__)
    #define DEBUG_PRINT_BEG(...) com_print_beg(__VA_ARGS__)
    #define DEBUG_PRINT_BEG2(...) com_print_beg2(__VA_ARGS__)
    #define DEBUG_PRINT_END(...) com_print_end(__VA_ARGS__)
  #else
    #define LOCAL_DEBUG_ENABLED  static const boolean local_debug = false;
    #define LOCAL_DEBUG_DISABLED  static const boolean local_debug = false;
    #define DEBUG_PRINT(...)
    #define DEBUG_PRINTLN(...)
    #define DEBUG_PRINTF(...)
    #define DEBUG_INDENT(...)
    #define DEBUG_PRINT_BEG(...)
    #define DEBUG_PRINT_BEG2(...)
    #define DEBUG_PRINT_END(...)
  #endif

  boolean debug_new_mode(void);
  void debug_set_new_mode(void);
  void debug_clr_new_mode(void);
  
#endif
