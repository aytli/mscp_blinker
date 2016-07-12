#ifndef CAN_TELEM_H
#define CAN_TELEM_H

// NOTE: The following tables are x-macros
// X macro tutorial: http://www.embedded.com/design/programming-languages-and-tools/4403953/C-language-coding-errors-with-X-macros-Part-1

//////////////////////////////
// CAN COMMAND DEFINES ///////
//////////////////////////////

#define EXPAND_AS_MISC_ID_ENUM(a,b)  a##_ID  = b,

// X macro table of miscellaneous CANbus packets
//        Packet name                  ,    ID
#define CAN_MISC_TABLE(ENTRY)                   \
    ENTRY(COMMAND_LEFT_SIGNAL          , 0x300) \
    ENTRY(COMMAND_RIGHT_SIGNAL         , 0x301) \
    ENTRY(COMMAND_HAZARD_SIGNAL        , 0x302) \
    ENTRY(COMMAND_BPS_TRIP_SIGNAL      , 0x303)
#define N_CAN_COMMAND 4

enum {CAN_MISC_TABLE(EXPAND_AS_MISC_ID_ENUM)};


#endif
