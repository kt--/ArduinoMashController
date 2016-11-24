////
//// Copyright (C) 2016 Kingsley Turner, krt@krt.com.au
//// See LICENCE.txt for details, but licensed CC BY-NC-SA
////

#include "utils.h"                                           

#include <EEPROM.h>



// Copied this function from the Arduino forum.  
// ref: http://forum.arduino.cc/index.php?topic=44262.5
// I don't know if "skumlerud" is the original author.

// NOTE: precision of 0 still gives a .0 on the end
// NOTE: This function seems a bit dodgey, and I suspect there's
//       a bunch of stuff it doesn't handle.  I wouldn't use it
//       for anything super imprortant.  That said, I'm still using it.
//
char *ftoa(float f, char *str_buffer, int precision)
{
    long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
   
    char *ret = str_buffer;
    long heiltal = (long)f;
    if (f < 0 && heiltal == 0)
    {
        // KT 2016-10-31 If the (value > -0.5) and (value < 0), 
        // the result loses it's sign because <heiltal> rounds to 0.  
        // So put it back
        str_buffer[0] = '-';
        str_buffer++;
    }    
    itoa(heiltal, str_buffer, 10);
    while (*str_buffer != '\0') 
        str_buffer++;
    *str_buffer++ = '.';
    long desimal = abs((long)((f - heiltal) * p[precision]));
    itoa(desimal, str_buffer, 10) ;
    return ret;
}

/*
char *ftoa(float f, char *a, int precision)
{
    // This is probably better but takes an extra kilobyte over ftoa() above.
    return dtostrf(f,3,precision,a);
}
*/





// function to print a device address, assuming you have the
// RAM left to include Serial.h
void printDS18B20Address(uint8_t *device_address)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (device_address[i] < 16) 
        Serial.print("0x0");
    else
        Serial.print("0x");
    Serial.print(device_address[i], HEX);
    if (i < 7)
        Serial.print(", ");
  }
  Serial.println();
}



// Write floats to the Arduino EEPROM.
// we use this to remember settings over power cycles, etc.
void writeFloatToEEPROM(unsigned int address, float value)
{
    unsigned char *ptr = (unsigned char*)(&value);
    
    // Step through the individual bytes of <value>
    for (unsigned int i=0; i<sizeof(float); i++)
    {
        // Read the current EEPROM location's value,
        // and only change it if this new one is different
        // (since EEPROMs have a limited write-life)
        byte b = EEPROM.read(address);
        if (*ptr != b)
            EEPROM.write(address, *ptr);
        address++;
        ptr++;
    }
}

void readFloatFromEEPROM(unsigned int address, float *value)
{
    unsigned char *ptr = (unsigned char*)(value);
    
    for (unsigned int i=0; i<sizeof(float); i++)
    {
        // Read the location, copying it out into <value>
        *ptr = EEPROM.read(address);
        address++;
        ptr++;
    }
}


