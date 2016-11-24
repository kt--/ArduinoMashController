#ifndef __UTILS_H__
#define __UTILS_H__



void printDS18B20Address(uint8_t *device_address);

char *ftoa(float f, char *a, int precision);

void writeFloatToEEPROM(unsigned int address, float value);
void readFloatFromEEPROM(unsigned int address, float *value);



#endif //#ifndef __UTILS_H__
