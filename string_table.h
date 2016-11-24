#include <stdint.h>

#ifdef __AVR__
#define _PGM_ PROGMEM
#else
#define _PGM_
#endif

// Standard string table nastiness to save space.
// I'm pretty sure storing small strings here actually wastes space.
// like STR_DEGSYM for example.


enum { 
    STR_HLT=0,
    STR_MASH,
    STR_PANEL_WARN,
    STR_HLT_SET,
    STR_MASH_SET,
    STR_DATE,
    STR_HLT_DELTA,
    STR_MASH_DELTA,
    STR_TIME,
    STR_HLT_STARTAT,
    STR_MASH_STEPMASH,
    STR_M,
    STR_DEGSYM,
    STR_ON,
    STR_OFF,
    STR_DELAYED_START,
    STR_SENSOR_FAIL,
    STR_MASH_RUNNING,
    
    STR_NULL
};

#define MAX_MESSAGE_LEN 20

// was "prog_char" for Arduino 1.5.x, not "const char" for 1.6.3+
const char msg0[]  PROGMEM = "HLT";
const char msg1[]  PROGMEM = "Mash";
const char msg2[]  PROGMEM = "Panel > 50C";
const char msg3[]  PROGMEM = "HLT Set";
const char msg4[]  PROGMEM = "Mash Set";
const char msg5[]  PROGMEM = "YY-MM-DD";
const char msg6[]  PROGMEM = "HLT Delta";
const char msg7[]  PROGMEM = "Mash Delta";
const char msg8[]  PROGMEM = "HH:MM";
const char msg9[]  PROGMEM = "Start";
const char msg10[] PROGMEM = "Step Mash";
const char msg11[] PROGMEM = "m";
const char msg12[] PROGMEM = "o";
const char msg13[] PROGMEM = "*ON*";
const char msg14[] PROGMEM = " off";
const char msg15[] PROGMEM = "Delayed Start";
const char msg16[] PROGMEM = "Sensor Failure";
const char msg17[] PROGMEM = "Step Mash Running";

// Was: PROGMEM const char *messages[]  = { msg0, on arduino 1.5.x
const char * const messages[] PROGMEM =  { msg0, 
                                    msg1,
                                    msg2,
                                    msg3,
                                    msg4,
                                    msg5,
                                    msg6,
                                    msg7,
                                    msg8,
                                    msg9,
                                    msg10,
                                    msg11,
                                    msg12,                                         
                                    msg13,
                                    msg14,
                                    msg15,
                                    msg16,
                                    msg17,
                                    NULL
                                  };
