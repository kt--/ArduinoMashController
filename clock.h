////
//// Copyright (C) 2016 Kingsley Turner, krt@krt.com.au
//// See LICENCE.txt for details, but licensed CC BY-NC-SA
////

#ifndef __DS3231_CLOCK_H__
#define __DS3231_CLOCK_H__

// Based on code by John Boxall at tronixstuff.com ~
// http://tronixstuff.com/2014/12/01/tutorial-using-ds1307-and-ds3231-real-time-clock-modules-with-arduino/


// Arduino Wiring:
// UNO:  SDA->A4
//       SCL->A5
// Mega: SDA->SDA20
//       SCL->SCL21


#include "Wire.h"

#define DS3231_I2C_ADDRESS 0x68

#define ARG_TYPE long  // Normally just a byte is enough for this, but my menu-classes need longs.  *Sigh*.

class DS3231Clock
{
public:
    void begin()
    {
        Wire.begin();
    }
    
    // Set the internal clock.
    // week_day: 1=Sunday ... 7=Saturday
    // hour: 0 ... 24
    // It seems OK to use a zero for the week-day.
    void setTime(byte year, byte month, byte day, byte week_day, byte hour, byte minute, byte second)
    {
        Wire.beginTransmission(DS3231_I2C_ADDRESS);
        Wire.write(0); // set next input to start at the seconds register
        Wire.write(decToBCD(second));
        Wire.write(decToBCD(minute));        
        Wire.write(decToBCD(hour));
        Wire.write(decToBCD(week_day));        
        Wire.write(decToBCD(day));        
        Wire.write(decToBCD(month));
        Wire.write(decToBCD(year));        
        Wire.endTransmission();
    }
    
    void getDateTime(ARG_TYPE *year, ARG_TYPE *month, ARG_TYPE *day, /*ARG_TYPE *week_day,*/ ARG_TYPE *hour, ARG_TYPE *minute, ARG_TYPE *second)
    {
        Wire.beginTransmission(DS3231_I2C_ADDRESS);
        Wire.write(0); // set next input to start at the seconds register
        Wire.endTransmission();
        Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
        *second   = (ARG_TYPE)BCDToDec(Wire.read() & 0x7f);
        *minute   = (ARG_TYPE)BCDToDec(Wire.read());
        *hour     = (ARG_TYPE)BCDToDec(Wire.read() & 0x3f);
        //*week_day = (ARG_TYPE)BCDToDec(Wire.read());
        Wire.read();
        *day      = (ARG_TYPE)BCDToDec(Wire.read());
        *month    = (ARG_TYPE)BCDToDec(Wire.read());
        *year     = (ARG_TYPE)BCDToDec(Wire.read());
    }

    void getTime(ARG_TYPE *hour, ARG_TYPE *minute, ARG_TYPE *second)
    {
        Wire.beginTransmission(DS3231_I2C_ADDRESS);
        Wire.write(0); // set next input to start at the seconds register
        Wire.endTransmission();
        Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
        *second   = (ARG_TYPE)BCDToDec(Wire.read() & 0x7f);
        *minute   = (ARG_TYPE)BCDToDec(Wire.read());
        *hour     = (ARG_TYPE)BCDToDec(Wire.read() & 0x3f);
        for (int i=0; i<4; i++)
            Wire.read();  // Consuming the remaining date bytes
    }


private:
    // Convert normal decimal numbers to binary coded decimal
    byte decToBCD(byte val)
    {
        return( (val/10*16) + (val%10) );
    }
    
    // Convert binary coded decimal to normal decimal numbers
    byte BCDToDec(byte val)
    {
        return( (val/16*10) + (val%16) );
    }
};


// Three general date/time functions
// Ported from C++11/C++14 to C by me.
// Ref: http://stackoverflow.com/questions/7960318/math-to-convert-seconds-since-1970-into-date-and-vice-versa


//// Returns number of days since civil 1970-01-01
//// Negative values indicate days prior to 1970-01-01
////    Y -> YYYY, e.g.: 2016
////    M -> 1 - 12 
////    D -> 1 - 31 (appropriate for <M>)
////
long daysFromCivil(long Y, long M, long D)
{
    long era;
    unsigned long yoe, doy, doe;

    Y -= M <= 2;
    era = (Y >= 0 ? Y : Y-399) / 400;
    yoe = (unsigned long)(Y - era * 400);            // [0, 399]
    doy = (153*(M + (M > 2 ? -3 : 9)) + 2)/5 + D-1;  // [0, 365]
    doe = yoe * 365 + yoe/4 - yoe/100 + doy;         // [0, 146096]
    return era * 146097 + (long)(doe) - 719468;
}

// Returns year, month (1-12), day (1-31)
void civilFromDays(long z, int *Y, int *M, int *D)
{
    long era, y,m,d;
    unsigned long doe, yoe, doy, mp;

    z += 719468;
    era = (z >= 0 ? z : z - 146096) / 146097;
    doe = (unsigned long)(z - era * 146097);                // [0, 146096]
    yoe = (doe - doe/1460 + doe/36524 - doe/146096) / 365;  // [0, 399]
    y = (long)(yoe)  + era * 400;
    doy = doe - (365*yoe + yoe/4 - yoe/100);                // [0, 365]
    mp = (5*doy + 2)/153;                                   // [0, 11]
    d = doy - (153*mp+2)/5 + 1;                             // [1, 31]
    m = mp + (mp < 10 ? 3 : -9);                            // [1, 12]

    *Y = (int)(y + (m <= 2));
    *M = (int)m;
    *D = (int)d;
}

// Returns day of week in civil calendar [0, 6] -> [Sun, Sat]
int weekdayFromDays(long z)
{
    return (z >= -4 ? (z+4) % 7 : (z+5) % 7 + 6);
}


//
// Given a YYYY-MM-DD HH:MM date-time, add <add_minutes> to the time, and return the 
// new date.  Handles leap-years, year rollover, etc. etc.  It's rather Nifty.
//
void addMinutesToTime(long add_minutes, int *year, int *month, int *day, int *hour, int *minute)
{
    // Get the minutes since 1970-01-01 00:00
    long long_hours = (long)*hour;
    long long_minutes = (long)*minute;
    long day_count_1970 = daysFromCivil((long)*year, (long)*month, (long)*day);

    long minute_count = (day_count_1970 * 24 * 60) + (long_hours * 60) + long_minutes + add_minutes;

    // Now convert that back into a date-time
    long new_day_count = minute_count / (24 * 60); // cut off the partial-day
    civilFromDays(new_day_count, year, month, day);

    long partial_day_minutes = minute_count - (new_day_count * 24 * 60);
    long hours_part = partial_day_minutes / 60;
    *hour =   (int)hours_part;
    *minute = (int)(partial_day_minutes - (hours_part * 60));
}


// Get the number of minutes between 1970-01-01 00:00 and 'now'
long getDateMinutes(int year, int month, int day, int hour, int minute)
{
    // Get the minutes since 1970-01-01 00:00
    long day_count_1970 = daysFromCivil((long)year, (long)month, (long)day);
    long minute_count = (day_count_1970 * 24 * 60) + ((long)hour * 60) + (long)minute;
    return minute_count;
}


// Format up a date-string.
// If you have plenty of RAM, nuke this junk and use the sprintf() version 
// But if you're on a 16k/32k Arduino, every byte counts.
char *getDateString(char *str_buffer/*[17+]*/, int year, int month, int day, int hour, int minute)
{
    // 0123456789abcdef
    // YYYY-MM-DD HH:MM

    // Sprintf() Vs -below- uses 1316 bytes more
    
    //sprintf(str_buffer, "%04d-%02d-%02d %02d:%02d", year, month, day, hour, minute);    

    if (year < 2000)
        year += 2000;
        
    itoa(year, str_buffer, 10);

    str_buffer[4] = '-';

    if (month < 10)
    {
        str_buffer[5]='0';
        itoa(month, &(str_buffer[6]), 10);
    }
    else
    {
        itoa(month, &(str_buffer[5]), 10);      
    }

    str_buffer[7] = '-';

    if (day < 10)
    {
        str_buffer[8]='0';
        itoa(day, &(str_buffer[9]), 10);
    }
    else
    {
        itoa(day, &(str_buffer[8]), 10);      
    }
    
    str_buffer[10] = ' ';

    if (hour < 10)
    {
        str_buffer[11]='0';
        itoa(hour, &(str_buffer[12]), 10);
    }
    else
    {
        itoa(hour, &(str_buffer[11]), 10);      
    }
    
    str_buffer[13]=':';

    if (minute < 10)
    {
        str_buffer[14]='0';
        itoa(minute, &(str_buffer[15]), 10);
    }
    else
    {
        itoa(minute, &(str_buffer[14]), 10);      
    }
  
    return str_buffer;
}

#endif //#ifndef __DS3231_CLOCK_H__
