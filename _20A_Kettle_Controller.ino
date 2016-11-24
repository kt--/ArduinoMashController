////
//// Copyright (C) 2016 Kingsley Turner, krt@krt.com.au
//// See LICENCE.txt for details, but licensed CC BY-NC-SA
////
#include <stdint.h>
#include <SPI.h>        // must include this here (or else IDE can't find it)
#include <EEPROM.h>


// Addresses of saved settings in the Arduino EEPROM
#define ADDRESS_HLT_TEMP  0
#define ADDRESS_HLT_CALI  (sizeof(float))
#define ADDRESS_MASH_TEMP (2*sizeof(float))
#define ADDRESS_MASH_CALI (3*sizeof(float))



#include <PDQ_GFX.h>             // PDQ: Core graphics library
#define ILI9341_CS_PIN      10   // <= /CS pin (chip-select, LOW to get attention of ILI9341, HIGH and it ignores SPI bus)
#define ILI9341_DC_PIN       9   // <= DC pin (1=data or 0=command indicator line) also called RS
#define ILI9341_RST_PIN      8   // <= RST pin (optional)
#define ILI9341_SAVE_SPCR    0   // <= 0/1 with 1 to save/restore AVR SPI control register (to "play nice" when other SPI use)
#include <PDQ_ILI9341.h>         // PDQ: Hardware-specific driver library
void paintLabel(unsigned int name_id, int cursor_x, int cursor_y, int text_size=2, int colour=ILI9341_CYAN);

PDQ_ILI9341 tft;      // PDQ: create LCD object (using pins in "PDQ_ILI9341_config.h")

#include "gui_button.h"
#include "extra_graphics.h"

//#include <Fonts/FreeSerif12pt7b.h>  // include fancy serif font
//#include <Fonts/FreeSans12pt7b.h> // include fancy sans-serif font

// Rotary Encoder (Interrupt Driven)
#define ENCODER_PIN_A       2 // must be pin2 for interrupt 
#define ENCODER_PIN_B       3 // must be pin3 for interrupt 
#define ENCODER_BUTTON_PIN  4
#define ENCODER_OPTIMIZE_INTERRUPTS
#define ENCODER_REVERSE_DIRECTION
#include <Encoder.h>
#define ENCODER_DEBOUNCE_DELAY 100
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);

// These are the readings from the sensors
char hlt_start_timestamp[17] = { 'now\0\0\0\0\0\0\0\0\0\0\0\0\0' };
 float hlt_temperature      = 999.0;
 float mash_temperature     = 999.0;
 float panel_temperature    = 999.0;
 float mash_set_temperature = 0.00;  // off
 float hlt_set_temperature  = 75.50;
 float hlt_calibration      = 0.0;   // used to adjust sensor
 float mash_calibration     = 0.0;
 float panel_calibration    = 0.0;
#define TEMPERATURE_MAX       100.0
#define TEMPERATURE_MIN         0.0
#define TEMPERATURE_DEFAULT    74.5



#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS    7  // Temperature Sensor is plugged into port 7 on the Arduino
#define TEMP_PRECISION 12  // 9,10,11,12 are useful resolutions
// The Dallas Temperature Sensor uses the OneWire protocol
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

////
//// These addresses need to be determined and configured for your sensor cables / devices.
//// These are for *my* sensors, you need the addresses for your sensors, these will not work
////
#undef LOG_SENSOR_ADDRESS  // Turn this on to log the current addresses
DeviceAddress hlt_sensor_address   = { 0x28, 0xF2, 0x5B, 0xCA, 0x03, 0x00, 0x00, 0x21 };
DeviceAddress mash_sensor_address  = { 0x28, 0x66, 0x46, 0xCA, 0x03, 0x00, 0x00, 0xDC };
DeviceAddress panel_sensor_address = { 0x28, 0xB5, 0x3A, 0xB3, 0x06, 0x00, 0x00, 0x03 };

// these are the last readings written to the screen (big numbers to ensure that first screen-update)
 float last_hlt_temperature      = 999.0;
 float last_mash_temperature     = 999.0;
 float last_panel_temperature    = 999.0;
 float last_mash_set_temperature = 999.0;
 float last_hlt_set_temperature  = 999.0;
#define TEMP_REQUEST_TIMEOUT 8000
unsigned long last_read_hlt_temp = 0;
unsigned long last_read_mash_temp = 0;
unsigned long last_read_panel_temp = 0;
unsigned long last_paint_heater_state = 0;
// Step mash timings and temperatures
unsigned long step_mash_run_start_time = 0;
unsigned long step_time[4] = {   0,   0,   0,   0 };
float         step_temp[4] = { 0.0, 0.0, 0.0, 0.0 };


// PID Controlled Relay On/Off
#include <PID_v1.h>
#define HLT_RELAY_PIN       6
#define MASH_RELAY_PIN        5
#define HEATER_RELAY_ON     1
#define HEATER_RELAY_OFF    0
boolean can_use_heaters = false;  // Don't turn the heaters on unless told to
double pid_hlt_input, pid_hlt_setpoint, pid_hlt_output;
double pid_mash_input, pid_mash_setpoint, pid_mash_output;
#define HEATER_WINDOW_SIZE 5000 // milliseconds
unsigned long heater_hlt_window_start_time = 0;
unsigned long heater_mash_window_start_time = 0;
// The Proportional constant relates the error (100-16) to the output.  
// I think if you want the heater full on if the temperature is 10 degrees low you probably want a P parameter of 500 instead of 2.
// The Integral constant is saving you some.  It adds up the errors over time to eliminate offset.  If the output is not responding fast enough this boosts the output.
// The Differential only takes action when the error value starts changing.  It's there to try to keep the temperature from overshooting.
// (500,0,0.45) works well, but creeps very slowly to target (never going over)
float Kp_hlt = 500.0;    //500; //2;
float Ki_hlt =   0.0;      //5;
float Kd_hlt =   0.45;   //0.1; //2;
PID hlt_pid(&pid_hlt_input, &pid_hlt_output, &pid_hlt_setpoint, Kp_hlt,Ki_hlt,Kd_hlt, DIRECT);

float Kp_mash = 500.0;    //500; //2;
float Ki_mash =   0.0;      //5;
float Kd_mash =   0.45;   //0.1; //2;
PID mash_pid(&pid_mash_input, &pid_mash_output, &pid_mash_setpoint, Kp_mash,Ki_mash,Kd_mash, DIRECT);
 unsigned int last_hlt_heater_state = 2;
 unsigned int last_mash_heater_state = 2;



// Screen Layout Parameters
#define GUI_WIDTH  320
#define GUI_HEIGHT 240
#define GUI_BUTTON_WIDTH   70
#define GUI_BUTTON_HEIGHT  32

// A bunch of RGB 5-6-5 bit 16bit colour values
#define DARK_BLUE  0x000d  // 0,0,107 packed into 5,6,5 16bit RGB
#define GOLDEN     0xfe60  // 255, 204, 0
#define WHITE      0xffff
#define CRIMSON    0xb085  // 181, 18, 40
#define MINT       0x8795
#define PINK       0xfd5e  // 253 171 246
#define LIGHTGREY  0xce59  // 200 200 200
#define MIDGREY    0x8410  // 128 128 128 
#define DARKGREY   0x3186  // 50 50 50 
#define SKYBLUE    0x5dff
#define SWAMPGREEN 0x9ee9
#define SPOTON     0x14e8  // 16 157 65
#define TOOHOT     0xe101

// These are a bunch of co-ordinate pirs to draw a 
// catroon-ish flame representing when heat is being
// applied to the Kettle or Mash
const unsigned int count_outer_coords = 50;
const unsigned int heater_flame_coords_outer[50] = { 11,40,  6,39,  3,37,  1,33,  0,29,  0,25,  2,20,  5,16, 
                                                      9,13, 10,11, 11, 8, 11, 6, 10, 3,  7, 0, 13, 5, 16, 9, 
                                                     20,13, 23,17, 25,22, 26,26, 26,29, 26,33, 23,36, 21,38, 
                                                     18,40 };  // 25 pairs
const unsigned int count_inner_coords = 32;
const unsigned int heater_flame_coords_inner[32] = { 11,40,  9,38,  8,36,  7,34,  8,30,  9,28, 12,26, 14,25, 
                                                     15,23, 17,26, 18,29, 19,33, 18,36, 17,39, 16,40, 11,40 };  // 16 pairs


  
  
#define TEXT_TINY   1
#define TEXT_NORMAL 2
#define TEXT_HUGE   4


// This large list of enumerated types just define a list of 
// button/edit-field IDs for the GUI menus.  The field entering & leaving
// is based on the order of these, and re-arranging them will break it

// All buttons and edit fields are this high.
#define GUI_BUTTON_ROW (GUI_HEIGHT-GUI_BUTTON_HEIGHT)

// There is one main menu, and three sub-menus.
GUIMenu main_menu(4);
GUIMenu hlt_menu(3);
GUIMenu mash_menu(11);
GUIMenu calib_menu(8);

enum
{
    MENU_MAIN_OVER=0,
    MENU_MAIN_HLT,
    MENU_MAIN_MASH,
    MENU_MAIN_CALIB
};
enum 
{
    SUBMENU_HLT_SETTEMP=0,
    SUBMENU_HLT_STARTTIME=1,
    SUBMENU_HLT_BACK=2
};    
enum 
{
    SUBMENU_MASH_SETTEMP=0,
    SUBMENU_MASH_START=1,
    SUBMENU_MASH_STEP1_TIME=2,
    SUBMENU_MASH_STEP1_TEMP=3,
    SUBMENU_MASH_STEP2_TIME=4,
    SUBMENU_MASH_STEP2_TEMP=5,
    SUBMENU_MASH_STEP3_TIME=6,
    SUBMENU_MASH_STEP3_TEMP=7,
    SUBMENU_MASH_STEP4_TIME=8,
    SUBMENU_MASH_STEP4_TEMP=9,
    SUBMENU_MASH_BACK=10
};    
enum 
{
    SUBMENU_CALIB_DELTA_HLT=0,
    SUBMENU_CALIB_DELTA_MASH=1,    
    SUBMENU_CALIB_CLOCK_YEAR=2,    
    SUBMENU_CALIB_CLOCK_MONTH=3,    
    SUBMENU_CALIB_CLOCK_DAY=4,    
    SUBMENU_CALIB_CLOCK_HOUR=5,    
    SUBMENU_CALIB_CLOCK_MINUTE=6,
    SUBMENU_CALIB_BACK=7
};

// Which control item has focus (meaning the rotary-encoder spinner might be adjusting it)
enum {
    FOCUS_MAIN_MENU=0, 
    FOCUS_MENU_OVERVIEW, 
    FOCUS_MENU_HLT,
    FOCUS_MENU_MASH,
    FOCUS_MENU_CALIBRATION,
    FOCUS_SUBMENU_HLT,
    FOCUS_SUBMENU_MASH,
    FOCUS_SUBMENU_CALIBRATION,
    FOCUS_SUBMENU_HLT_SETTEMP,
    FOCUS_SUBMENU_HLT_STARTTIME,
    FOCUS_SUBMENU_MASH_SETTEMP,
    FOCUS_SUBMENU_MASH_START,
    FOCUS_SUBMENU_MASH_STEP1_TIME,
    FOCUS_SUBMENU_MASH_STEP1_TEMP,    
    FOCUS_SUBMENU_MASH_STEP2_TIME,
    FOCUS_SUBMENU_MASH_STEP2_TEMP,    
    FOCUS_SUBMENU_MASH_STEP3_TIME,
    FOCUS_SUBMENU_MASH_STEP3_TEMP,    
    FOCUS_SUBMENU_MASH_STEP4_TIME,
    FOCUS_SUBMENU_MASH_STEP4_TEMP,    
    FOCUS_SUBMENU_MASH_BACK,    
    FOCUS_SUBMENU_CALI_HLT,
    FOCUS_SUBMENU_CALI_MASH,
    FOCUS_SUBMENU_CALI_CLOCK_YEAR,
    FOCUS_SUBMENU_CALI_CLOCK_MONTH,
    FOCUS_SUBMENU_CALI_CLOCK_DAY,
    FOCUS_SUBMENU_CALI_CLOCK_HOUR,
    FOCUS_SUBMENU_CALI_CLOCK_MINUTE,
    FOCUS_SUBMENU_CALI_BACK,
    
    FOCUS_NONE
};   

/*
// There probably isn't enough free memory to use this anymore
// But it was only used for debugging the menu logic anyway
const char *getFocusName(int x)
{
    switch(x)
    {
        case FOCUS_MAIN_MENU: return "MAIN_MENU";
        case FOCUS_MENU_OVERVIEW: return "MENU_OVERVIEW";
        case FOCUS_MENU_HLT: return "MENU_HLT";
        case FOCUS_MENU_MASH: return "MENU_MASH";        
        case FOCUS_MENU_CALIBRATION: return "MENU_CALIBRATION";
        case FOCUS_SUBMENU_HLT: return "SUBMENU_HLT";
        case FOCUS_SUBMENU_MASH: return "SUBMENU_MASH";
        case FOCUS_SUBMENU_CALIBRATION: return "SUBMENU_CALIBRATION";
        case FOCUS_SUBMENU_HLT_SETTEMP: return "SUBMENU_HLT_SETTEMP";
        case FOCUS_SUBMENU_HLT_STARTTIME: return "SUBMENU_HLT_STARTTIME";
        case FOCUS_SUBMENU_HLT_BACK: return "SUBMENU_HLT_BACK";    
        case FOCUS_SUBMENU_CALI_HLT: return "SUBMENU+CALI+HLT";    
        case FOCUS_SUBMENU_CALI_MASH: return "SUBMENU_CALI_MASH";    
        case FOCUS_SUBMENU_CALI_CLOCK_YEAR: return "SUBMENU_CLOCK_Y";    
        case FOCUS_SUBMENU_CALI_CLOCK_MONTH: return "SUBMENU_CLOCK_MO";    
        case FOCUS_SUBMENU_CALI_CLOCK_DAY: return "SUBMENU_CLOCK_D";    
        case FOCUS_SUBMENU_CALI_CLOCK_HOUR: return "SUBMENU_CLOCK_H";    
        case FOCUS_SUBMENU_CALI_CLOCK_MINUTE: return "SUBMENU_CLOCK_MI";    
        case FOCUS_SUBMENU_CALI_BACK: return "SUBMENU_CALI_BACK";    
    }
    return "Unknown";
}
*/

// Which GUI control is currently focused.
int last_menu_item_selected  = -1;
int last_hlt_item_selected   = -1;
int last_mash_item_selected  = -1;
int last_calib_item_selected = -1;

unsigned int previous_control_focus = FOCUS_MAIN_MENU;
unsigned int control_focus          = FOCUS_MAIN_MENU;

#define SCREEN_OVERVIEW    0
#define SCREEN_KETTLE      1
#define SCREEN_MASH        2
#define SCREEN_CALIBRATION 3

// This variable holds the ID of the current screen shown to the user
int current_screen = 0;


#include "string_table.h"
#include "utils.h"
#include <Wire.h>
#include "clock.h"
#include <stdlib.h>



// Clock
// The citcuit has a DS3231 battery-backed clock to maintain the
// clock during power-off.  The clock values are used for delayed
// starting of the water heater (HLT).
DS3231Clock real_time_clock;
unsigned long last_clock_paint_time = 999999;
long clock_year,clock_month,clock_day,clock_hour,clock_minute,clock_second;  // The edit-menu fields need longs.
int start_year, start_month, start_day, start_hour, start_minute;


// This is a general purpose buffer for building up string representation
// of numbers, and whatever else it can be found useful for.
char buffer[32];  // for ftoa(), clock-log etc.


void setup() 
{
    // Gentlemen, start your encoders
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);
    pinMode(ENCODER_BUTTON_PIN, INPUT_PULLUP);
    encoder.write(TEMPERATURE_DEFAULT * 2);

    // Setup the PID stuff
    heater_hlt_window_start_time = millis();
    pid_hlt_setpoint = (double)TEMPERATURE_DEFAULT;
    hlt_pid.SetOutputLimits(0, HEATER_WINDOW_SIZE);
    hlt_pid.SetMode(AUTOMATIC);
    pinMode(HLT_RELAY_PIN, OUTPUT);
    digitalWrite(HLT_RELAY_PIN,HEATER_RELAY_OFF);  // turn it off (Relay is avtive *LOW*)

    heater_mash_window_start_time = millis();
    pid_mash_setpoint = (double)TEMPERATURE_DEFAULT;
    mash_pid.SetOutputLimits(0, HEATER_WINDOW_SIZE);
    mash_pid.SetMode(AUTOMATIC);
    pinMode(MASH_RELAY_PIN, OUTPUT);
    digitalWrite(MASH_RELAY_PIN, HEATER_RELAY_OFF);  // turn it off (Relay is avtive *LOW*)
 
    // DO the little LCD Screen
    resetTFT();  
    tft.begin();   // initialise the TFT-LCD
    tft.setRotation(1);  // 90 degrees clockwise (landscape mode, pins left)
    //tft.fillScreen(ILI9341_BLACK);  // Paint it Black
    tft.fillRect(0, 0, 320, 240, ILI9341_BLACK);


    // DO the DS18B20 Temperature Sensors
    sensors.begin();                      // initialise the sensor library 
    sensors.setWaitForConversion(false);  // Don't wait for the temperature to be ready


    

#ifdef LOG_SENSOR_ADDRESS  
    // You will have to comment out a bunch of stuff for this piece of code to work.
    // The 2x calls to the drawPot() should be more than enough.
    Serial.begin(115200); 

    // Enable this #define to log your DS18B20 temperature sesnor addresses
    // The configure the addresses correctly in the code at the stop of the sketch
    Serial.println("\nLogging Sensor Addresses:");
    sensors.getAddress(hlt_sensor_address, 0);
    Serial.print("HLT DS18B20 Address is: "); 
    printDS18B20Address(hlt_sensor_address);
    
    sensors.getAddress(mash_sensor_address, 1);
    Serial.print("Mash HX DS18B20 Address is: "); 
    printDS18B20Address(mash_sensor_address);        
    
    sensors.getAddress(panel_sensor_address, 2);
    Serial.print("Control Panel DS18B20 Address is: "); 
    printDS18B20Address(panel_sensor_address);
#endif    

    // We expect 3 temperature sensors - HLT, Mash and interal control-box temperature
    if (sensors.getDeviceCount() < 3)
    {
        paintLabel(STR_SENSOR_FAIL, 60, 120, 2, ILI9341_RED);
        delay(8000);
    }

    // Set the temperature accuracy
    sensors.setResolution(hlt_sensor_address, TEMP_PRECISION);
    sensors.setResolution(mash_sensor_address, TEMP_PRECISION);        
    sensors.setResolution(panel_sensor_address, TEMP_PRECISION);
    // Ask the sensors to start taking a reading               
    sensors.requestTemperatures();
    

    // Use an interrupt to adjust the Heater relay
    // Run timer2 interrupt every 15 ms
    TCCR2A = 0;
    TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;
    //Timer2 Overflow Interrupt Enable
    TIMSK2 |= 1<<TOIE2;


    // Read the clock to pull the date-time
    Wire.begin();  
    real_time_clock.getDateTime(&clock_year,&clock_month,&clock_day,&clock_hour,&clock_minute,&clock_second);
    //getDateString(buffer, clock_year,clock_month,clock_day,clock_hour,clock_minute);
    //Serial.print("Clock Says: ");
    //Serial.println(buffer);
    
    // If the clock doesn't get enough power, or Wire.begin() not called, readings are 165,165,164,45,165 etc.
    /*
    if (clock_year > 100)
    {
        clock_year   = 16;
        clock_month  = 7;
        clock_day    = 28;
        clock_hour   = 8;
        clock_minute = 30;
    }*/
    strcpy(hlt_start_timestamp, "now");  // the timestamp is either 'now' or a date in the future.
  
    // Read the calibration info that might be stored in the EEPROM memory
    readFloatFromEEPROM(ADDRESS_HLT_TEMP,&hlt_set_temperature);
    readFloatFromEEPROM(ADDRESS_HLT_CALI,&hlt_calibration);
    //readFloatFromEEPROM(ADDRESS_MASH_TEMP,&mash_set_temperature);    // We don't want the mash to start automatically.
    readFloatFromEEPROM(ADDRESS_MASH_CALI,&mash_calibration);


    // Populate the GUI menus.  I have to wonder if this couldn't just be loaded in somehow.
    main_menu.setColours(DARK_BLUE, GOLDEN, GOLDEN, CRIMSON, WHITE, WHITE);
    main_menu.addMenuEntry(MENU_MAIN_OVER,  "OVER",   0, GUI_BUTTON_ROW, GUI_BUTTON_WIDTH, GUI_BUTTON_HEIGHT);
    main_menu.addMenuEntry(MENU_MAIN_HLT,   "HLT",   82, GUI_BUTTON_ROW, GUI_BUTTON_WIDTH, GUI_BUTTON_HEIGHT);
    main_menu.addMenuEntry(MENU_MAIN_MASH,  "MASH", 164, GUI_BUTTON_ROW, GUI_BUTTON_WIDTH, GUI_BUTTON_HEIGHT);
    main_menu.addMenuEntry(MENU_MAIN_CALIB, "CALI", 246, GUI_BUTTON_ROW, GUI_BUTTON_WIDTH, GUI_BUTTON_HEIGHT);
    main_menu.setHighlightedEntry(MENU_MAIN_OVER);
     
    hlt_menu.setColours(DARK_BLUE, GOLDEN, GOLDEN, CRIMSON, WHITE, WHITE);
    hlt_menu.addMenuEntry(SUBMENU_HLT_SETTEMP,    &hlt_set_temperature,  2, 110,  0,                        GUI_BUTTON_WIDTH+24,  GUI_BUTTON_HEIGHT);
    hlt_menu.addMenuEntry(SUBMENU_HLT_STARTTIME,   hlt_start_timestamp,     110,  5+GUI_BUTTON_HEIGHT,      GUI_BUTTON_WIDTH+140, GUI_BUTTON_HEIGHT);
    hlt_menu.addMenuEntry(SUBMENU_HLT_BACK,        "back",                  110,  20+(4*GUI_BUTTON_HEIGHT), GUI_BUTTON_WIDTH,     GUI_BUTTON_HEIGHT);
     
    mash_menu.setColours(DARK_BLUE, GOLDEN, GOLDEN, CRIMSON, WHITE, WHITE);
    mash_menu.addMenuEntry(SUBMENU_MASH_SETTEMP,     &mash_set_temperature,  2, 150,  0,                        GUI_BUTTON_WIDTH,     GUI_BUTTON_HEIGHT);
    mash_menu.addMenuEntry(SUBMENU_MASH_START,        "Run",                    150,  5+GUI_BUTTON_HEIGHT,      GUI_BUTTON_WIDTH,     GUI_BUTTON_HEIGHT);
    mash_menu.addMenuEntry(SUBMENU_MASH_STEP1_TIME,  (long*)&step_time[0],   0,   0,  10+(2*GUI_BUTTON_HEIGHT), GUI_BUTTON_WIDTH-24,  GUI_BUTTON_HEIGHT);
    mash_menu.addMenuEntry(SUBMENU_MASH_STEP1_TEMP,  (float*)&step_temp[0],  2,  65,  10+(2*GUI_BUTTON_HEIGHT), GUI_BUTTON_WIDTH,     GUI_BUTTON_HEIGHT);
    mash_menu.addMenuEntry(SUBMENU_MASH_STEP2_TIME,  (long*)&step_time[1],   0, 166,  10+(2*GUI_BUTTON_HEIGHT), GUI_BUTTON_WIDTH-24,  GUI_BUTTON_HEIGHT);
    mash_menu.addMenuEntry(SUBMENU_MASH_STEP2_TEMP,  (float*)&step_temp[1],  2, 231,  10+(2*GUI_BUTTON_HEIGHT), GUI_BUTTON_WIDTH,     GUI_BUTTON_HEIGHT);
    mash_menu.addMenuEntry(SUBMENU_MASH_STEP3_TIME,  (long*)&step_time[2],   0,   0,  15+(3*GUI_BUTTON_HEIGHT), GUI_BUTTON_WIDTH-24,  GUI_BUTTON_HEIGHT);
    mash_menu.addMenuEntry(SUBMENU_MASH_STEP3_TEMP,  (float*)&step_temp[2],  2,  65,  15+(3*GUI_BUTTON_HEIGHT), GUI_BUTTON_WIDTH,     GUI_BUTTON_HEIGHT);
    mash_menu.addMenuEntry(SUBMENU_MASH_STEP4_TIME,  (long*)&step_time[3],   0, 166,  15+(3*GUI_BUTTON_HEIGHT), GUI_BUTTON_WIDTH-24,  GUI_BUTTON_HEIGHT);
    mash_menu.addMenuEntry(SUBMENU_MASH_STEP4_TEMP,  (float*)&step_temp[3],  2, 231,  15+(3*GUI_BUTTON_HEIGHT), GUI_BUTTON_WIDTH,     GUI_BUTTON_HEIGHT);
    mash_menu.addMenuEntry(SUBMENU_MASH_BACK,        "back",                    150,  20+(4*GUI_BUTTON_HEIGHT), GUI_BUTTON_WIDTH,     GUI_BUTTON_HEIGHT);
    
    calib_menu.setColours(DARK_BLUE, GOLDEN, GOLDEN, CRIMSON, WHITE, WHITE);
    calib_menu.addMenuEntry(SUBMENU_CALIB_DELTA_HLT,    &hlt_calibration,      2, 150,   0,                       GUI_BUTTON_WIDTH+24, GUI_BUTTON_HEIGHT);
    calib_menu.addMenuEntry(SUBMENU_CALIB_DELTA_MASH,   &mash_calibration,     2, 150,   5+GUI_BUTTON_HEIGHT,     GUI_BUTTON_WIDTH+24, GUI_BUTTON_HEIGHT);
    calib_menu.addMenuEntry(SUBMENU_CALIB_CLOCK_YEAR,   (long*)&clock_year,    2, 150,  10+(2*GUI_BUTTON_HEIGHT), GUI_BUTTON_WIDTH-24, GUI_BUTTON_HEIGHT);
    calib_menu.addMenuEntry(SUBMENU_CALIB_CLOCK_MONTH,  (long*)&clock_month,   2, 200,  10+(2*GUI_BUTTON_HEIGHT), GUI_BUTTON_WIDTH-24, GUI_BUTTON_HEIGHT);
    calib_menu.addMenuEntry(SUBMENU_CALIB_CLOCK_DAY,    (long*)&clock_day,     2, 250,  10+(2*GUI_BUTTON_HEIGHT), GUI_BUTTON_WIDTH-24, GUI_BUTTON_HEIGHT);
    calib_menu.addMenuEntry(SUBMENU_CALIB_CLOCK_HOUR,   (long*)&clock_hour,    2, 150,  15+(3*GUI_BUTTON_HEIGHT), GUI_BUTTON_WIDTH-24, GUI_BUTTON_HEIGHT);
    calib_menu.addMenuEntry(SUBMENU_CALIB_CLOCK_MINUTE, (long*)&clock_minute,  2, 200,  15+(3*GUI_BUTTON_HEIGHT), GUI_BUTTON_WIDTH-24, GUI_BUTTON_HEIGHT);
    calib_menu.addMenuEntry(SUBMENU_CALIB_BACK,         "back",                   150,  20+(4*GUI_BUTTON_HEIGHT), GUI_BUTTON_WIDTH,    GUI_BUTTON_HEIGHT);
    calib_menu.setHighlightedEntry(SUBMENU_CALIB_DELTA_HLT);
   
}


// Timer Interrupt Handler
SIGNAL(TIMER2_OVF_vect)
{
    controlHeaters();
}



//
// Reads the temperature, and acts on the heaters
// Since this is *my* control box, and I run only a single power cable
// Only one heating element is allowed to run at once.  Priority is
// given to the Mash heater.
//
void controlHeaters()
{
    // Use the PID Algorithm to control the heating element
    unsigned long now = millis();

    // if it's not time to start the heaters - DON'T DO ANYTHING
    if (can_use_heaters == true)
    {
        //Serial.println("Heaters are allowed to RUN");
        
        // PID Control
        pid_mash_input     = mash_temperature;   // global
        pid_mash_setpoint  = mash_set_temperature;      // global
        mash_pid.Compute();
   
        // First the Mash Heat Exchange (HX)
        // turn the output pin on/off based on pid output
        if (now - heater_mash_window_start_time > HEATER_WINDOW_SIZE)
            heater_mash_window_start_time += HEATER_WINDOW_SIZE; //time to shift the Relay Window
        if (pid_mash_output > now - heater_mash_window_start_time)
        {
            if (last_mash_heater_state != HEATER_RELAY_ON)
            {
                //Serial.println("MASH heater - turning ON");
                        
                // Turn off the HLT heater - only one heater runs at once
                if (last_hlt_heater_state == HEATER_RELAY_ON)
                {
                    //Serial.print(" +  MASH heater - turning OFF");
                    digitalWrite(HLT_RELAY_PIN, HEATER_RELAY_OFF);
                    last_hlt_heater_state = HEATER_RELAY_OFF;
                }

                // Turn on the MASH heater
                digitalWrite(MASH_RELAY_PIN, HEATER_RELAY_ON);
                last_mash_heater_state = HEATER_RELAY_ON;
            }
        }
        else
        {
            if (last_mash_heater_state != HEATER_RELAY_OFF)
            {
                //Serial.println("MASH heater - turning off");
                digitalWrite(MASH_RELAY_PIN, HEATER_RELAY_OFF);
                last_mash_heater_state = HEATER_RELAY_OFF;
            }
        }
    
    
        pid_hlt_input    = hlt_temperature;         // global
        pid_hlt_setpoint = hlt_set_temperature;     // global
    
        // We do not turn the (huge) HLT heater on if the MASH heater is already on
        if (last_mash_heater_state == HEATER_RELAY_OFF)
        {
            // Second the HLT / Kettle
            // turn the output pin on/off based on pid output
            hlt_pid.Compute();
            if (now - heater_hlt_window_start_time > HEATER_WINDOW_SIZE)
                heater_hlt_window_start_time += HEATER_WINDOW_SIZE; //time to shift the Relay Window
                
            if (pid_hlt_output > now - heater_hlt_window_start_time)
            {
                if (last_hlt_heater_state != HEATER_RELAY_ON)
                {
                    //Serial.println("HLT heater - turning ON");
                    digitalWrite(HLT_RELAY_PIN, HEATER_RELAY_ON);
                    last_hlt_heater_state = HEATER_RELAY_ON;
                }
            }
            else
            {
                if (last_hlt_heater_state != HEATER_RELAY_OFF)
                {
                    //Serial.println("HLT heater - turning off");
                    digitalWrite(HLT_RELAY_PIN, HEATER_RELAY_OFF);
                    last_hlt_heater_state = HEATER_RELAY_OFF;
                }
            }
        }
    }
    else
    {
        // We're not allowed to use heaters, turn them OFF!
        digitalWrite(MASH_RELAY_PIN, HEATER_RELAY_OFF);
        last_mash_heater_state = HEATER_RELAY_OFF;        
        digitalWrite(HLT_RELAY_PIN, HEATER_RELAY_OFF);
        last_hlt_heater_state = HEATER_RELAY_OFF;
    }
}






// Loop the encoder value from 0 -> x -> 0
int encoderZeroConstrain(int x, int maxx)
{
    if (x < 0)
        x = maxx;
    else if (x > maxx)
        x = 0;
    encoder.write(x);
}

// Ensure the given value is within range, and also write
// it to the encoder.  The encoder position decides temperatures,
// menu positions and what-not.  So we often change input mode,
// and need to reset the position of the encoder for a new 
// input focus.
int encoderConstrain(int x, int minx, int maxx)
{
    x = constrain(x, minx, maxx);
    encoder.write(x);           
    return x;  
}

/// Like the arduino map() function, except it doesn't truncate the remainder,
/// which works much better for small menus since mapping 1.8 -> 1 is bad
/// (This is a much more computationally expensive function though)
int mapRound(int x, int in_min, int in_max, int out_min, int out_max)
{
    float X_delta   = x - in_min;
    float IN_min    = in_min;
    float IN_delta  = in_max - in_min;
    float OUT_min   = out_min;
    float OUT_delta = out_max - out_min;
    
    // Calculate with rounding
    // Normal map() is (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    return int (0.5 + (X_delta * OUT_delta / IN_delta + OUT_min));
}



// 
// Given a label's stringtable ID code, find it, read it in, 
void paintLabel(unsigned int name_id, int cursor_x, int cursor_y, int text_size, int colour)
{
    const char *name = (const char*)pgm_read_word(&(messages[name_id]));
    char ch;
      
    int i=0;
    while((ch = pgm_read_byte(name)))
    {
        buffer[i] = ch;
        name++;
        i++;
    }
    buffer[i] = '\0';        
        
    tft.setTextColor(colour);
    printTFT(&tft, cursor_x, cursor_y, text_size, buffer);
}


//
// Given a temperature reading, format it up for writing to the screen.
// Writing to the LCD is fairly slow, so we only re-draw the temperature if it
// has changed since last time.  We also change the colour if it's under, over, or
// near (within 1.25C) of the target temperature.
//
// When DS18B20 sensors fail, they return a -127.0 value.  We handle this error here too.
//
void paintReading(float current_value, float set_value, float *last_drawn_value, int cursor_x, int cursor_y, int text_size=2)
{
    // Paint the current temperature coloured for when it's under/at/over set temp.
    const int char_width = TEXT_NORMAL * 7;
    unsigned int temp_colour = SPOTON;
    if (set_value <= 1.0)
    {
        temp_colour = MIDGREY; // it's off - who cares what the temp is
    }
    else
    {
        if (set_value - current_value > 1.25) 
            temp_colour = SKYBLUE;  // too cold
        else if (current_value - set_value > 1.25)
            temp_colour = TOOHOT;  // too hot
    }

    // Only paint the temperature if it's changed        
    if (current_value != *last_drawn_value)
    {
        tft.setTextColor(ILI9341_BLACK);
        printNumberTFT(&tft, cursor_x, cursor_y, text_size, *last_drawn_value);  // erase
 
        // -127.0 means "no reading"
        if (current_value < -120.0)
        {
            tft.setTextColor(MIDGREY);
            printTFT(&tft, cursor_x, cursor_y, text_size, "--");  // "--" means "no reading".
        }
        else
        {
            tft.setTextColor(temp_colour);
            printNumberTFT(&tft, cursor_x, cursor_y, text_size, current_value);
            paintLabel(STR_DEGSYM, cursor_x-9+(char_width*5), cursor_y-2, TEXT_TINY, temp_colour); 
        }
        *last_drawn_value = current_value;   // remember what we painted     
    }        
}    


//
// Draws a stylised line-plot heater flame.
// This is used to indicate the heating element is currently operating.
// We also need to be able to un-draw the flame graphic too.
//
void drawHeaterFlame(int at_x, int at_y, boolean erase=false)
{
    unsigned int inner_colour = ILI9341_YELLOW;
    unsigned int outer_colour = ILI9341_RED;
    
    // To erase the flame, we just paint it black
    if (erase == true)
    {
        inner_colour = ILI9341_BLACK;
        outer_colour = ILI9341_BLACK;
    }
    
    for (unsigned int i=0; i<count_outer_coords-2; i+=2)
        tft.drawLine(at_x+heater_flame_coords_outer[i],   at_y+heater_flame_coords_outer[i+1], 
                     at_x+heater_flame_coords_outer[i+2], at_y+heater_flame_coords_outer[i+3], outer_colour);
    for (unsigned int i=0; i<count_inner_coords-2; i+=2)
        tft.drawLine(at_x+heater_flame_coords_inner[i],   at_y+heater_flame_coords_inner[i+1], 
                     at_x+heater_flame_coords_inner[i+2], at_y+heater_flame_coords_inner[i+3], inner_colour);
                     
    // we re-draw the flame a +1,+1 to make it a bit thicker - more bold on the GUI.
    at_x += 1;
    at_y += 1;
    for (unsigned int i=0; i<count_outer_coords-2; i+=2)
        tft.drawLine(at_x+heater_flame_coords_outer[i],   at_y+heater_flame_coords_outer[i+1], 
                     at_x+heater_flame_coords_outer[i+2], at_y+heater_flame_coords_outer[i+3], outer_colour);
    for (unsigned int i=0; i<count_inner_coords-2; i+=2)
        tft.drawLine(at_x+heater_flame_coords_inner[i],   at_y+heater_flame_coords_inner[i+1], 
                     at_x+heater_flame_coords_inner[i+2], at_y+heater_flame_coords_inner[i+3], inner_colour);

}


//
// This function started out as simply plotting the picture of a water pot on the LCD.
// Then it was also the most appropriate place to the set-point.  But this only draws
// static stuff, the flame icon, and readings are painted in the main loop since they change often.
//
void drawPot(int at_x, int at_y, float target_temp, unsigned int label_id)
{
    color_t colour = 0xffff;
    int     height = 100;
    // Width is dependant on the ellipse radii
    
    // The pot is two elipses, and two sides.
    tft.drawEllipse(at_x+50, at_y,       50, 20, colour);
    tft.drawEllipse(at_x+50, at_y+height, 50, 20, colour);
    tft.drawLine(at_x, at_y, at_x, at_y+height, colour);
    tft.drawLine(at_x+100, at_y, at_x+100, at_y+height, colour);    
    
    // Stripe the hidden-side of the pot (bottom back) as per a techinal drawing from 1985
    for (int i=0; i<11; i++)
        tft.fillRect(3+(at_x+(i*9)), at_y+(height-25), 5, 25,  ILI9341_BLACK); 
        
    // If this is the Mash, draw the Heat Exchange (Hx) coil
    // if (label_id == STR_MASH)
    // {    
    //     TODO - don't have much RAM left for this.
    // }
    
    // The name of the pot, eithe "HLT" or "Mash"
    paintLabel(label_id, at_x+40, at_y+height+25, TEXT_TINY, ILI9341_WHITE);
    
    int temp_x = at_x+20;
    int temp_y = at_y-20+(height/2);
    
    int subtitle_x_pos = temp_x+15;
    int subtitle_y_pos = temp_y+20;
    
    // If the target temperature is set to 0.0 (well, < 1.0), the device
    // is considered to be off.
    if (target_temp <= 1.0)
    {
        // pot is off
        paintLabel(STR_OFF, subtitle_x_pos, subtitle_y_pos, TEXT_TINY, MIDGREY); 
    }
    else
    {
        // pot is ON!
        tft.setTextColor(ILI9341_MAGENTA);
        printNumberTFT(&tft, subtitle_x_pos, subtitle_y_pos, TEXT_TINY, target_temp);
    }

    // The heater icon and temperature reading are drawn in the main loop, because we 
    // only re-paint the pot when the screen re-draws completely.
}






// this goes true to indicate we want to repaint the
// entire screen (like when the main-menu changes the screen)
boolean needs_background_repaint = true;   



void loop() 
{
    // What time is it?
    unsigned long now = millis();

    // We need to know which GUI Screen the user has activated
    int menu_item_selected, menu_item_mode;
    main_menu.getCurrentSelection(&menu_item_selected, &menu_item_mode);
    int calib_item_selected, calib_item_mode;
    calib_menu.getCurrentSelection(&calib_item_selected, &calib_item_mode);
    int hlt_item_selected, hlt_item_mode;
    hlt_menu.getCurrentSelection(&hlt_item_selected, &hlt_item_mode);
    int mash_item_selected, mash_item_mode;
    mash_menu.getCurrentSelection(&mash_item_selected, &mash_item_mode);
    

    ////
    //// Check the time to see if it's OK to turn the heaters on
    //// The user may have elected to turn the HLT on in the future
    //// (like 05:30 tomorrow morning)
    ////
    if (hlt_start_timestamp[0] != 'n' /*now*/)
    {
        // Don't do anything while we're twiddling the time setting though
        if (control_focus != FOCUS_SUBMENU_HLT_STARTTIME ||
            (control_focus == FOCUS_SUBMENU_HLT_STARTTIME && hlt_item_mode != MENU_SELECTION_ACTIVE))
        {
            // read the clock, and the set-time, but get them both as minutes-after (some point in the waaay past)
            real_time_clock.getDateTime(&clock_year,&clock_month,&clock_day,&clock_hour,&clock_minute,&clock_second);
            long clock_now = getDateMinutes(2000+(int)clock_year, clock_month, clock_day, clock_hour, clock_minute);
            long heater_start = getDateMinutes(start_year, start_month, start_day, start_hour, start_minute);
            
            // If the real-time clock has advanced past the start-time, let's kick-off
            // Otherwise just don't do anything right now.
            if (clock_now < heater_start)
            {
                can_use_heaters = false;
            }
            else
            {
                // ... and we're OFF!
                can_use_heaters = true;
                strcpy(hlt_start_timestamp, "now");
                
                needs_background_repaint = true;  // repaint over any messages/menus that may be on-screen
            }
        }
    }
    else
    {
        can_use_heaters = true;  // if no future-start time is set, it's OK to turn the heaters on
    }

    
    ///
    /// Handle all the encoder knob turning - this includes looking at what
    /// control is focused for the encoder's range, etc.
    ///
    
    // If the knob is twiddling a temperature range ~
    int raw_encoder_value = encoder.read();    
    
    // Depending on which control has focus, ensure the encoder value is sensible.
    int target_entry;
    switch (control_focus)
    {
        case FOCUS_MAIN_MENU:
            raw_encoder_value = encoderZeroConstrain(raw_encoder_value, 20);
            target_entry = mapRound(raw_encoder_value, 0, 20, 0, 3);  // 4 buttons (#0..#3), 20 clicks, so that's 5 doobies per button
            main_menu.setFocusedEntry(target_entry);
            break;
        case FOCUS_SUBMENU_HLT:
            raw_encoder_value = encoderZeroConstrain(raw_encoder_value, 15);
            target_entry = mapRound(raw_encoder_value, 0, 15, 0, 2);  // 3 buttons (#0..#2), 15 clicks, so that's 5 doobies per button
            hlt_menu.setFocusedEntry(target_entry);
            break;
        case FOCUS_SUBMENU_HLT_SETTEMP:
            raw_encoder_value = encoderConstrain(raw_encoder_value, TEMPERATURE_MIN, 2*TEMPERATURE_MAX);
            hlt_set_temperature = ((float)raw_encoder_value) / 2.0;  // divide by 2.0 to give 1-100 in 0.5 steps
            //hlt_menu.setMenuEntryValue(SUBMENU_HLT_SETTEMP, hlt_set_temperature);
            hlt_menu.paint(&tft, SUBMENU_HLT_SETTEMP);
            break;
        case FOCUS_SUBMENU_HLT_STARTTIME:
            if (raw_encoder_value <= 0)
            {
                strcpy(hlt_start_timestamp, "now");
                encoder.write(0);
                raw_encoder_value = 1;
            }
            else
            {
                // each encoder tick sets the time forward 10 minutes
                // So we need to render the time some amount from 'now'
                real_time_clock.getDateTime(&clock_year,&clock_month,&clock_day,&clock_hour,&clock_minute,&clock_second);
                if (clock_year > 100)
                {
                    strcpy(hlt_start_timestamp, "clock-failue");
                }
                else
                {
                    long offset = (long)raw_encoder_value * 15;
                    start_year  = 2000+clock_year;
                    start_month = clock_month;
                    start_day   = clock_day;
                    start_hour  = clock_hour;
                    start_minute= (15 * ((clock_minute) / 15));

                    addMinutesToTime(offset, &start_year, &start_month, &start_day, &start_hour, &start_minute);
                    getDateString(hlt_start_timestamp, start_year, start_month, start_day, start_hour, start_minute);
                }
            }
            hlt_menu.paint(&tft, SUBMENU_HLT_STARTTIME);
            break;
            
            
        case FOCUS_SUBMENU_MASH:
            raw_encoder_value = encoderZeroConstrain(raw_encoder_value, 55);
            target_entry = mapRound(raw_encoder_value, 0, 55, 0, 10);  // 11 buttons (#0..#10), 55 clicks, so that's 5 doobies per button
            mash_menu.setFocusedEntry(target_entry);
            break;
            
        case FOCUS_SUBMENU_MASH_SETTEMP:
        case FOCUS_SUBMENU_MASH_STEP1_TEMP:
        case FOCUS_SUBMENU_MASH_STEP2_TEMP:
        case FOCUS_SUBMENU_MASH_STEP3_TEMP:
        case FOCUS_SUBMENU_MASH_STEP4_TEMP:
        {
            raw_encoder_value = encoderConstrain(raw_encoder_value, TEMPERATURE_MIN, 2*TEMPERATURE_MAX);
            float set_temperature = ((float)raw_encoder_value) / 2.0;  // divide by 2.0 to give 1-100 in 0.5 steps
            int menu_focus = control_focus-FOCUS_SUBMENU_MASH_SETTEMP;
            mash_menu.paint(&tft, menu_focus);
            switch (control_focus)
            {
                case FOCUS_SUBMENU_MASH_SETTEMP:
                    mash_set_temperature = set_temperature;    
                    break;
                case FOCUS_SUBMENU_MASH_STEP1_TEMP:
                    step_temp[0] = set_temperature;
                    break;
                case FOCUS_SUBMENU_MASH_STEP2_TEMP:
                    step_temp[1] = set_temperature;
                    break;
                case FOCUS_SUBMENU_MASH_STEP3_TEMP:
                    step_temp[2] = set_temperature;
                    break;
                case FOCUS_SUBMENU_MASH_STEP4_TEMP:
                    step_temp[3] = set_temperature;
                    break;
            }
            break;
        }
        case FOCUS_SUBMENU_MASH_STEP1_TIME:
        case FOCUS_SUBMENU_MASH_STEP2_TIME:
        case FOCUS_SUBMENU_MASH_STEP3_TIME:
        case FOCUS_SUBMENU_MASH_STEP4_TIME:        
        {
            int menu_focus = control_focus-FOCUS_SUBMENU_MASH_SETTEMP;
            raw_encoder_value = encoderConstrain(raw_encoder_value, 0, 180);  // minutes
            mash_menu.paint(&tft, menu_focus);
            switch (control_focus)
            {
                case FOCUS_SUBMENU_MASH_STEP1_TIME:
                    step_time[0] = raw_encoder_value;
                    break;
                case FOCUS_SUBMENU_MASH_STEP2_TIME:
                    step_time[1] = raw_encoder_value;
                    break;
                case FOCUS_SUBMENU_MASH_STEP3_TIME:
                    step_time[2] = raw_encoder_value;
                    break;
                case FOCUS_SUBMENU_MASH_STEP4_TIME:
                    step_time[3] = raw_encoder_value;
                    break;
            }            
            break;
        }
            


        case FOCUS_SUBMENU_CALIBRATION:
            raw_encoder_value = encoderZeroConstrain(raw_encoder_value, 40);
            target_entry = mapRound(raw_encoder_value, 0, 40, 0, 7);  // 8 buttons (#0..#7), 40 clicks, so that's 5 doobies per button
            calib_menu.setFocusedEntry(target_entry);
            break;            
        case FOCUS_SUBMENU_CALI_HLT:
            hlt_calibration = ((float)raw_encoder_value) / 4.0;  // 0 -> 50 by 0.25
            calib_menu.paint(&tft, SUBMENU_CALIB_DELTA_HLT);
            break;
        case FOCUS_SUBMENU_CALI_MASH:
            mash_calibration = ((float)raw_encoder_value) / 4.0;  // 0 -> 50 by 0.25 steps
            calib_menu.paint(&tft, SUBMENU_CALIB_DELTA_MASH);            
            break;            
        case FOCUS_SUBMENU_CALI_CLOCK_YEAR:
            raw_encoder_value = encoderConstrain(raw_encoder_value, 10, 150);
            clock_year = raw_encoder_value;
            calib_menu.paint(&tft, SUBMENU_CALIB_CLOCK_YEAR);
            break;
        case FOCUS_SUBMENU_CALI_CLOCK_MONTH:
            raw_encoder_value = encoderConstrain(raw_encoder_value, 1, 12);
            clock_month = raw_encoder_value;
            calib_menu.paint(&tft, SUBMENU_CALIB_CLOCK_MONTH);
            break;
        case FOCUS_SUBMENU_CALI_CLOCK_DAY:
            raw_encoder_value = encoderConstrain(raw_encoder_value, 1, 31);
            clock_day = raw_encoder_value;
            calib_menu.paint(&tft, SUBMENU_CALIB_CLOCK_DAY);
            break;
        case FOCUS_SUBMENU_CALI_CLOCK_HOUR:
            raw_encoder_value = encoderConstrain(raw_encoder_value, 0, 23);
            clock_hour = raw_encoder_value;
            calib_menu.paint(&tft, SUBMENU_CALIB_CLOCK_HOUR);
            break;
        case FOCUS_SUBMENU_CALI_CLOCK_MINUTE:
            raw_encoder_value = encoderConstrain(raw_encoder_value, 0, 59);
            clock_minute = raw_encoder_value;
            calib_menu.paint(&tft, SUBMENU_CALIB_CLOCK_MINUTE);
            break;

        
    };  // switch (control_focus)
    


 
    ///
    /// Handle all of the incoming sensor data
    ///
    
    // Read Temperatures
    // We have told the sensor library to not wait for the temp reading, so we need to check
    // if the sensor has a reading ready for us.  If it is, we take it, and queue a new one
    // Sometimes the DS18B20's get muddled-up and don't ever report "Available", so re-request
    if (sensors.isConversionAvailable(hlt_sensor_address))     
    {
        hlt_temperature = sensors.getTempC(hlt_sensor_address);
        hlt_temperature += hlt_calibration;
        last_read_hlt_temp = now;            
        // Do we need to delay before doing this?
        sensors.requestTemperaturesByAddress(hlt_sensor_address);
    }
    else if (now - last_read_hlt_temp > TEMP_REQUEST_TIMEOUT)
    {
        // If here, the sensor has become muddled.  Re-requesting it usually fixes it
        // Supposedly this is caused by noisy current, and 0.1uF capacitor "near" the
        // sensor really helps.  But my sensors are sealed in a waterproof housing.
        sensors.requestTemperaturesByAddress(hlt_sensor_address); 
    }
        
    if (sensors.isConversionAvailable(mash_sensor_address))     
    {
        mash_temperature = sensors.getTempC(mash_sensor_address);
        mash_temperature += mash_calibration;
        last_read_mash_temp = now;
        // Do we need to delay before doing this?
        sensors.requestTemperaturesByAddress(mash_sensor_address);
    }
    else if (now - last_read_mash_temp > TEMP_REQUEST_TIMEOUT)
    {
        // If here, the sensor has become muddled.  Re-requesting it usually fixes it      
        sensors.requestTemperaturesByAddress(mash_sensor_address);
    }

    // We always want the control-panel internal temperature to make sure it's not too hot
    if (sensors.isConversionAvailable(panel_sensor_address))     
    {
        panel_temperature = sensors.getTempC(panel_sensor_address);
        panel_temperature += panel_calibration;
        last_read_panel_temp = now;
        // Do we need to delay before doing this?
        sensors.requestTemperaturesByAddress(panel_sensor_address);
    }
    else if (now - last_read_panel_temp > TEMP_REQUEST_TIMEOUT)
    {
        // If here, the sensor has become muddled.  Re-requesting it usually fixes it      
        sensors.requestTemperaturesByAddress(panel_sensor_address);
    }


    ////
    //// Process the Step-Mash, if it's running
    ////
    if (step_mash_run_start_time > 0)
    {
        unsigned long time_delta = now - step_mash_run_start_time;

        // convert to seconds
        time_delta = ((time_delta + 500) / 1000);  
        // convert to minutes
        time_delta = time_delta / 60;
        
        // Work out where along the mash we should be.
        // Mash step times are measures in minutes
        
        // Each step-time is a duration of minutes
        // So we loop through the steps summing the durations to 
        // see where that time-delta from start puts us.
        int current_step = 0;
        int cumulative_durations = 0;
        for (current_step=0; current_step < 4; current_step++)
        {
            cumulative_durations += step_time[current_step];
            if (time_delta <= cumulative_durations)
            {
                // we're in the <current_step> step, automatically set the step-temperature
                mash_set_temperature = step_temp[current_step];
                // if we're looking at the MASH menu, we need to repaint it because the set-point changed
                //mash_menu.setMenuEntryValue(SUBMENU_MASH_SETTEMP, mash_set_temperature);   
                if (current_screen == SCREEN_MASH)
                    mash_menu.paint(&tft);                 
                break;  
            }
        }
        // But!... if we have gone past the end of all steps, everything needs to be stopped
        if (current_step == 4)
        {
            step_mash_run_start_time = 0;    // stop step mash
            mash_set_temperature     = 0.0;  // stop the mash heat exchange too
            
            // if we're looking at the MASH menu, we need to repaint it because the set-point changed
            mash_menu.setMenuEntryLabel(SUBMENU_MASH_START, "Done");            
            //mash_menu.setMenuEntryValue(SUBMENU_MASH_SETTEMP, mash_set_temperature);   
            if (current_screen == SCREEN_MASH)            
                mash_menu.paint(&tft); 
            else if (current_screen == SCREEN_OVERVIEW)
                needs_background_repaint = true;  // remove message from overview screen
        }
    }

    
    ///
    /// Do everthing to render the static parts of the screen
    ///
    if (needs_background_repaint == true)
    {
        //tft.fillScreen(ILI9341_BLACK);  // Paint it Black 
        tft.fillRect(0, 0, 319, 239, ILI9341_BLACK);
        last_clock_paint_time = 0; // be sure to re-paint the clock too

        // Ensure all readings & calibrations are re-drawn when the whole screen changes
        // We only draw the temperatures when they appear to have changed otherwise
        last_hlt_temperature      = 999.0;
        last_mash_temperature     = 999.0;
        last_panel_temperature    = 999.0;
        last_hlt_set_temperature  = 999.0;
        last_mash_set_temperature = 999.0;

        tft.setTextSize(TEXT_NORMAL);
        main_menu.paint(&tft); 

        if (current_screen == SCREEN_OVERVIEW)
        {
#ifdef GUI_OVERVIEW             
            // This code is for the simple statistical overview screen.
            // It was there for so long, I can't bear to delete this code
            // and it takes up much less space then pot-drawings
            paintLabel(STR_HLT_STARTAT,  0,   0);
            tft.setCursor(120, 0);
            tft.setTextColor(ILI9341_YELLOW);             
            tft.print(hlt_start_timestamp);
            paintLabel(STR_HLT,      0,  20);
            paintLabel(STR_MASH,     0,  40);
            paintLabel(STR_PANEL,    0,  60);        
            paintLabel(STR_HLT_SET,  0,  80);        
            paintLabel(STR_MASH_SET, 0, 100);        
#else            
            // Draw the pots, and any step-mashing hints.
            if (step_mash_run_start_time > 0)
                paintLabel(STR_MASH_RUNNING, 60, 20, TEXT_NORMAL, ILI9341_YELLOW);                
            drawPot( 20,55, hlt_set_temperature, STR_HLT );  // HLT
            drawPot(180,55, mash_set_temperature, STR_MASH);  // MASH-Hx
#endif            
        }
        else if (current_screen == SCREEN_KETTLE)
        {
            paintLabel(STR_HLT_SET,     0,   5);   paintLabel(STR_DEGSYM, 206, 1, TEXT_TINY);       
            paintLabel(STR_HLT_STARTAT, 0,  45);
            hlt_menu.paint(&tft);       
        }
        else if (current_screen == SCREEN_MASH)
        {
            paintLabel(STR_MASH_SET,       0,  5);  paintLabel(STR_DEGSYM, 222,  1, TEXT_TINY);
            paintLabel(STR_MASH_STEPMASH,  0, 45);
            paintLabel(STR_M,             48, 90);  paintLabel(STR_DEGSYM, 136,  75, TEXT_TINY);
            paintLabel(STR_M,            214, 90);  paintLabel(STR_DEGSYM, 302,  75, TEXT_TINY);
            paintLabel(STR_M,             48,125);  paintLabel(STR_DEGSYM, 136, 110, TEXT_TINY);
            paintLabel(STR_M,            214,125);  paintLabel(STR_DEGSYM, 302, 110, TEXT_TINY);
            mash_menu.paint(&tft); 
        }
        else if (current_screen == SCREEN_CALIBRATION)
        {
            paintLabel(STR_HLT_DELTA,  0,  5);  paintLabel(STR_DEGSYM, 246,   1, TEXT_TINY);
            paintLabel(STR_MASH_DELTA, 0, 45);  paintLabel(STR_DEGSYM, 246,  36, TEXT_TINY);
            paintLabel(STR_DATE,       0, 80);      
            paintLabel(STR_TIME,       0,120);      
            calib_menu.paint(&tft);  
        }
        
        needs_background_repaint = false;
    }


    // Repaint the main-menu if it's changed
    if (menu_item_selected != last_menu_item_selected)
    {
        //drawMenuBottons(focused_menu_button, selected_menu_button);
        main_menu.paint(&tft);
        last_menu_item_selected = menu_item_selected;
    }
    
    

    // Paint the relevant non-static part of the screen given the selected item
    if (current_screen == SCREEN_OVERVIEW)
    {
#ifdef GUI_OVERVIEW      
        // Repaint the readings if they're changed
        paintReading(hlt_temperature,       &last_hlt_temperature,      hlt_set_temperature,  120,  20);
        paintReading(mash_temperature,      &last_mash_temperature,     mash_set_temperature, 120,  40);    
        paintReading(panel_temperature,     &last_panel_temperature,    panel_temperature,    120,  60);        
        paintReading(hlt_set_temperature,   &last_hlt_set_temperature,  hlt_set_temperature,  120,  80);    
        paintReading(mash_set_temperature,  &last_mash_set_temperature, mash_set_temperature, 120, 100);    
        
        // Draw the "Delayed Start" Message if the mash is waiting on the clock
        if (can_use_heaters == true)
            paintLabel(STR_DELAYED_START, 50, 150, 2, ILI9341_BLACK);
        else
            paintLabel(STR_DELAYED_START, 50, 150, 2, ILI9341_RED);
        
#else
        // If we're waiting until a certain time to start (i.e.: delayed start)
        // Put a note about it and that start-time on the screen.
        if (can_use_heaters == false)
        {
              paintLabel(STR_DELAYED_START, 80, 0, TEXT_NORMAL, ILI9341_RED);
              tft.setCursor(65, 20);
              tft.setTextColor(ILI9341_YELLOW);             
              tft.print(hlt_start_timestamp);            
        }

        paintReading(hlt_temperature,  hlt_set_temperature,  &last_hlt_temperature,   40, 85);
        paintReading(mash_temperature, mash_set_temperature, &last_mash_temperature, 200, 85);  
#endif        

        // Draw the heater state of the two elements
        if (now - last_paint_heater_state > 250)  // don't repaint faster than 1/4 second
        {
            if (last_hlt_heater_state == HEATER_RELAY_OFF)
                drawHeaterFlame( 55, 120, true);
            else
                drawHeaterFlame( 55, 120);
            
            if (last_mash_heater_state == HEATER_RELAY_OFF)
                drawHeaterFlame(215, 120, true);
            else
                drawHeaterFlame(215, 120);
                
            last_paint_heater_state = now;
        }

    }
    else if (current_screen == SCREEN_KETTLE)
    {
        if (hlt_item_selected != last_hlt_item_selected)
        {
            hlt_menu.paint(&tft);
            last_hlt_item_selected = hlt_item_selected;
        }
    }
    else if (current_screen == SCREEN_MASH)
    {
        if (mash_item_selected != last_mash_item_selected)
        {
            mash_menu.paint(&tft);
            last_mash_item_selected = mash_item_selected;
        }  
    }
    else if (current_screen == SCREEN_CALIBRATION)
    {
        //calib_menu.getCurrentSelection(&calib_item_selected, &calib_item_mode);
        if (calib_item_selected != last_calib_item_selected)
        {
            calib_menu.paint(&tft);
            last_calib_item_selected = calib_item_selected;
        }
    }


    // Is the panel overheating?  if so, paint a warning
    // Geeze, we should probably have a buzzer that sounds too
    if (panel_temperature > 50.0)
    {
        if (last_panel_temperature != panel_temperature)
        {
            tft.setTextColor(ILI9341_BLACK);
            printNumberTFT(&tft, 90, 20, TEXT_HUGE, last_panel_temperature);
        }
        paintLabel(STR_PANEL_WARN, 80, 0, TEXT_NORMAL, ILI9341_RED);
        printNumberTFT(&tft, 90, 20, TEXT_HUGE, panel_temperature);
        last_panel_temperature = panel_temperature;
    }
        
    
    // paint the time on the top-right screen every so often
    if (current_screen != SCREEN_CALIBRATION && now - last_clock_paint_time > 5000) 
    {
        real_time_clock.getDateTime(&clock_year,&clock_month,&clock_day,&clock_hour,&clock_minute,&clock_second);
        getDateString(buffer, clock_year, clock_month, clock_day, clock_hour, clock_minute);
        if (buffer[11] == '0')
            buffer[11] = ' ';

        int normal_time_y = 0;
        tft.fillRect(260, normal_time_y, 59, 14, ILI9341_BLACK);
        tft.setCursor(260, normal_time_y);            
        tft.setTextSize(TEXT_NORMAL);        
        tft.setTextColor(WHITE);
        tft.print(buffer+11);  // skip the date, just the time
        last_clock_paint_time = now;
    }

    ///
    /// Did we get a button press on the encoder?
    /// Handle all the button presses here.  
    /// Basically button presses are used to enter or exit a control item.
    ///
    if (digitalRead(ENCODER_BUTTON_PIN) == LOW)
    {
        delay(ENCODER_DEBOUNCE_DELAY);  // debounce the button
        if (digitalRead(ENCODER_BUTTON_PIN) == LOW)
        {
            ////
            //// The user has clicked the encoder button.
            //// If the focus was a submenu, that item needs to be activated.
            //// it may be another sub-menu, a "back" exit menu, an edit-field,
            //// or an edit field already focused that the user is finished with.
            ////
            switch(control_focus)
            {    
                // If any of the main menu buttons is activated
                // The main menu is active with one of the buttons highlighted (but NOT selected)   
                case FOCUS_MAIN_MENU:
                    // Some highlighted menu entry has been clicked -> activate it (ON)
                    // menu item should always be in 'Highlighted' state here (not activated)
                    // So we need to activate the menu item, and swap the screen
                    main_menu.activateCurrentEntry();
                    main_menu.paint(&tft);                            
                    switch(menu_item_selected)
                    {
                        case MENU_MAIN_OVER:
                            current_screen = SCREEN_OVERVIEW;
                            control_focus = FOCUS_MAIN_MENU;
                            // The Overview entry does not stay selected, since there's no reason to
                            main_menu.flashAndDeactivateCurrentEntry(&tft);
                            break;
                        case MENU_MAIN_HLT:
                            //real_time_clock.getDateTime(&clock_year,&clock_month,&clock_day,&clock_hour,&clock_minute,&clock_second);
                            current_screen = SCREEN_KETTLE;                            
                            control_focus = FOCUS_SUBMENU_HLT; // the focus is now the hlt sub-menu
                            encoder.write(15); // last option "back" in hlt sub-menu
                            break;
                        case MENU_MAIN_MASH:
                            current_screen = SCREEN_MASH;                                                        
                            control_focus = FOCUS_SUBMENU_MASH; // the focus is now the mash sub-menu
                            encoder.write(55); // last option "back" in mash sub-menu
                            
                            // Set the step-mash settings
                            // TODO? Do we need to do this at all?
                            break;
                        case MENU_MAIN_CALIB:
                            current_screen = SCREEN_CALIBRATION;
                            control_focus = FOCUS_SUBMENU_CALIBRATION; // the focus is now the calibration sub-menu
                            encoder.write(40); // last option "back" in calibration menu
                            
                            // Get the latest time for the clock settings
                            real_time_clock.getDateTime(&clock_year,&clock_month,&clock_day,&clock_hour,&clock_minute,&clock_second);
                            break;
                    }
                    needs_background_repaint = true;
                    break;

                case FOCUS_SUBMENU_HLT:
                    if (hlt_item_selected == SUBMENU_HLT_BACK)
                    {
                        encoder.write(5+3); // HLT on main-menu 
                        control_focus = FOCUS_MAIN_MENU;  // Jump out of the calibration screen
                        // Flash the button for a 1/2 second
                        hlt_menu.flashAndDeactivateCurrentEntry(&tft);
                        // And back to the main menu
                        main_menu.highlightCurrentEntry();  
                        main_menu.paint(&tft);  // only the menu (not entire screen) needs repainting.
                    }
                    else
                    {
                        switch(hlt_item_selected)
                        {
                            case SUBMENU_HLT_SETTEMP:
                            {
                                float menu_entry_value = hlt_menu.getMenuEntryValue(hlt_item_selected);
                                encoder.write(2.0 * menu_entry_value);
                                control_focus = FOCUS_SUBMENU_HLT_SETTEMP;                             
                                break;
                            }
                            case SUBMENU_HLT_STARTTIME:
                                encoder.write(0);
                                control_focus = FOCUS_SUBMENU_HLT_STARTTIME;                             
                                break;
                        }
                        hlt_menu.activateCurrentEntry();
                        hlt_menu.paint(&tft);                                                
                    }
                    break;

                case FOCUS_SUBMENU_MASH:
                    if (mash_item_selected == SUBMENU_MASH_BACK)
                    {
                        encoder.write(15); // MASH on main-menu 
                        control_focus = FOCUS_MAIN_MENU;  // Jump out of the calibration screen
                        // Flash the button for a 1/2 second
                        mash_menu.flashAndDeactivateCurrentEntry(&tft);
                        // And back to the main menu                         
                        main_menu.highlightCurrentEntry();  
                        main_menu.paint(&tft);  // only the menu (not entire screen) needs repainting.
                    }
                    else if (mash_item_selected == SUBMENU_MASH_START)
                    {
                        // This is a toggle button Run/Stop
                        // The Button says "Run" or "Stop"
                        mash_menu.activateCurrentEntry();  // Just highlight the button for a bit, so user knows it pressed
                        mash_menu.paint(&tft);                                                
                        const char *button_label = mash_menu.getMenuEntryLabel(SUBMENU_MASH_START);
                        if (button_label[0] == 'R' || button_label[0] == 'D')  // Run or Done
                        {
                            // Run
                            step_mash_run_start_time = millis();
                            mash_menu.setMenuEntryLabel(SUBMENU_MASH_START, "Stop");
                        }
                        else
                        {
                            // Stop
                            mash_menu.setMenuEntryLabel(SUBMENU_MASH_START, "Run");                            
                            step_mash_run_start_time = 0;                            
                        }
                        delay(500);
                        mash_menu.highlightCurrentEntry();  // De-activate
                        mash_menu.paint(&tft);
                    }
                    else
                    {
                        float menu_entry_value = mash_menu.getMenuEntryValue(mash_item_selected);
                        float factor = 2.0;
                        if (mash_item_selected == SUBMENU_MASH_STEP1_TIME || mash_item_selected == SUBMENU_MASH_STEP2_TIME || 
                            mash_item_selected == SUBMENU_MASH_STEP3_TIME || mash_item_selected == SUBMENU_MASH_STEP4_TIME)
                            factor = 1.0;  
                        encoder.write(factor * menu_entry_value);

                        int menu_id_to_focus_id_delta = FOCUS_SUBMENU_MASH_SETTEMP; // What do we add to the mash-menu ID to get the focus ID
                        control_focus = mash_item_selected + menu_id_to_focus_id_delta;
                        mash_menu.activateCurrentEntry();
                        mash_menu.paint(&tft);                                                
                    }
                    break;


                // Some calibration option of the CALI sub-menu has been clicked
                // Currently, none of them is activated
                case FOCUS_SUBMENU_CALIBRATION:
                    //Serial.println("CLICK Was in Calibration Submenu");
                    if (calib_item_selected == SUBMENU_CALIB_BACK)
                    {
                        encoder.write(20); // CALIB on main menu 
                        control_focus = FOCUS_MAIN_MENU;  // Jump out of the calibration screen
                        // Flash the "back" button for a 1/2 second
                        calib_menu.flashAndDeactivateCurrentEntry(&tft);
                        // And focus back to the main menu
                        main_menu.highlightCurrentEntry();  
                        main_menu.paint(&tft);  // only the menu (not entire screen) needs repainting.  
                        /*
                        Serial.println("Setting RTC");
                        Serial.print("Year = "); Serial.println((uint8_t)calib_menu.getMenuEntryValue(SUBMENU_CALIB_CLOCK_YEAR));
                        Serial.print("Month= "); Serial.println((uint8_t)calib_menu.getMenuEntryValue(SUBMENU_CALIB_CLOCK_MONTH));
                        Serial.print("Day  = "); Serial.println((uint8_t)calib_menu.getMenuEntryValue(SUBMENU_CALIB_CLOCK_DAY));                        
                        Serial.print("Hour = "); Serial.println((uint8_t)calib_menu.getMenuEntryValue(SUBMENU_CALIB_CLOCK_HOUR));                        
                        Serial.print("Minut= "); Serial.println((uint8_t)calib_menu.getMenuEntryValue(SUBMENU_CALIB_CLOCK_MINUTE));                        
                        */
                        
                        // Save the calibration info to the EEPROM memory
                        writeFloatToEEPROM(ADDRESS_HLT_TEMP, hlt_set_temperature);
                        writeFloatToEEPROM(ADDRESS_HLT_CALI, hlt_calibration);
                        //writeFloatToEEPROM(ADDRESS_MASH_TEMP, mash_set_temperature);    // We don't save this, sine we want the Mash OFF at power-on-reset
                        writeFloatToEEPROM(ADDRESS_MASH_CALI, mash_calibration);
                        
                        // Change the time remembered by the real-time clock
                        real_time_clock.setTime((byte)clock_year, (byte)clock_month, (byte)clock_day, (byte)0, (byte)clock_hour, (byte)clock_minute, (byte)0);
                    }
                    else
                    {
                        // Some calibration sub-menu item was activated
                        float menu_entry_value = calib_menu.getMenuEntryValue(calib_item_selected);
                        encoder.write(menu_entry_value);
                        switch (calib_item_selected)
                        {
                            case SUBMENU_CALIB_DELTA_HLT:
                                control_focus = FOCUS_SUBMENU_CALI_HLT;
                                encoder.write(4.0 * menu_entry_value);
                                break;
                            case SUBMENU_CALIB_DELTA_MASH:
                                control_focus = FOCUS_SUBMENU_CALI_MASH;
                                encoder.write(4.0 * menu_entry_value);
                                break;
                            case SUBMENU_CALIB_CLOCK_YEAR:
                                control_focus = FOCUS_SUBMENU_CALI_CLOCK_YEAR;
                                break;
                            case SUBMENU_CALIB_CLOCK_MONTH:
                                control_focus = FOCUS_SUBMENU_CALI_CLOCK_MONTH;
                                break;
                            case SUBMENU_CALIB_CLOCK_DAY:
                                control_focus = FOCUS_SUBMENU_CALI_CLOCK_DAY;
                                break;
                            case SUBMENU_CALIB_CLOCK_HOUR:
                                control_focus = FOCUS_SUBMENU_CALI_CLOCK_HOUR;
                                break;
                            case SUBMENU_CALIB_CLOCK_MINUTE:
                                control_focus = FOCUS_SUBMENU_CALI_CLOCK_MINUTE;
                                break;
                        }
                        calib_menu.activateCurrentEntry();
                        calib_menu.paint(&tft);                        
                    }
                    break;



                ////
                //// Here the user is exiting a sub-menu option, giving focus back to the 
                //// containing menu. 
                //// E.g.: going from a editing calibration item back to the calibration menu
                ////

                // The user is leaving a section on the HLT menu
                case FOCUS_SUBMENU_HLT_SETTEMP:
                case FOCUS_SUBMENU_HLT_STARTTIME:
                    //Serial.println("Leaving activated entry on HLT sub-menu");
                    encoder.write(2+ (5 * (control_focus-FOCUS_SUBMENU_HLT_SETTEMP)));  // reposition encoder to focus exit point
                    control_focus = FOCUS_SUBMENU_HLT;
                    hlt_menu.highlightCurrentEntry();  // De-activate
                    hlt_menu.paint(&tft);  // the HLT menu needs repainting.
                    break;

                case FOCUS_SUBMENU_MASH_SETTEMP:
                case FOCUS_SUBMENU_MASH_START:
                case FOCUS_SUBMENU_MASH_STEP1_TIME:
                case FOCUS_SUBMENU_MASH_STEP1_TEMP:
                case FOCUS_SUBMENU_MASH_STEP2_TIME:
                case FOCUS_SUBMENU_MASH_STEP2_TEMP:
                case FOCUS_SUBMENU_MASH_STEP3_TIME:
                case FOCUS_SUBMENU_MASH_STEP3_TEMP:
        	case FOCUS_SUBMENU_MASH_STEP4_TIME:
                case FOCUS_SUBMENU_MASH_STEP4_TEMP:
                    encoder.write(2+ (5 * (control_focus-FOCUS_SUBMENU_MASH_SETTEMP)));  // reposition encoder to focus exit point
                    control_focus = FOCUS_SUBMENU_MASH;
                    mash_menu.highlightCurrentEntry();  // De-activate
                    mash_menu.paint(&tft);  // the mash menu needs repainting.
                    break;

                    
                // The user is leaving a section on the calibration menu
                case FOCUS_SUBMENU_CALI_HLT:                                
                case FOCUS_SUBMENU_CALI_MASH:                
                case FOCUS_SUBMENU_CALI_CLOCK_YEAR:
                case FOCUS_SUBMENU_CALI_CLOCK_MONTH:
                case FOCUS_SUBMENU_CALI_CLOCK_DAY:                
                case FOCUS_SUBMENU_CALI_CLOCK_HOUR:
                case FOCUS_SUBMENU_CALI_CLOCK_MINUTE:
                    encoder.write(2+ (5 * (control_focus-FOCUS_SUBMENU_CALI_HLT)));  // reposition encoder to focus exit point
                    control_focus = FOCUS_SUBMENU_CALIBRATION;
                    calib_menu.highlightCurrentEntry();  // De-activate
                    calib_menu.paint(&tft);  // the calibration menu needs repainting.
                    break;


            } // switch (control_focus)     
        } // if encoder-button-pressed *still*
    } // if encoder-button-pressed
    
    
} // loop()


// That's it, stop reading now.
// Maybe preuse some of the support files
