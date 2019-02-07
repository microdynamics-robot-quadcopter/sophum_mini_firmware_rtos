#ifndef __ESP32_MODULE_MPU9250_H__
#define __ESP32_MODULE_MPU9250_H__

uint8_t MPU9250_getID(void);
bool MPU9250_testConnection(void);

bool MPU9250_getSelfTest(void);
bool MPU9250_evaluateSelfTest(float low, float high, float value, char* string);
void MPU9250_Init(void);

#endif


