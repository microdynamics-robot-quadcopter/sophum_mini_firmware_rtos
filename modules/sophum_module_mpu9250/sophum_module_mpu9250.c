#include <stdio.h>
#include "driver/i2c.h"
#include "sophum_driver_i2c.h"
#include "sophum_module_mpu9250.h"



/* private const variable */
static const uint16_t mpu9250_selftest_table[256] = {
  2620,2646,2672,2699,2726,2753,2781,2808, //7
  2837,2865,2894,2923,2952,2981,3011,3041, //15
  3072,3102,3133,3165,3196,3228,3261,3293, //23
  3326,3359,3393,3427,3461,3496,3531,3566, //31
  3602,3638,3674,3711,3748,3786,3823,3862, //39
  3900,3939,3979,4019,4059,4099,4140,4182, //47
  4224,4266,4308,4352,4395,4439,4483,4528, //55
  4574,4619,4665,4712,4759,4807,4855,4903, //63
  4953,5002,5052,5103,5154,5205,5257,5310, //71
  5363,5417,5471,5525,5581,5636,5693,5750, //79
  5807,5865,5924,5983,6043,6104,6165,6226, //87
  6289,6351,6415,6479,6544,6609,6675,6742, //95
  6810,6878,6946,7016,7086,7157,7229,7301, //103
  7374,7448,7522,7597,7673,7750,7828,7906, //111
  7985,8065,8145,8227,8309,8392,8476,8561, //119
  8647,8733,8820,8909,8998,9088,9178,9270,
  9363,9457,9551,9647,9743,9841,9939,10038,
  10139,10240,10343,10446,10550,10656,10763,10870,
  10979,11089,11200,11312,11425,11539,11654,11771,
  11889,12008,12128,12249,12371,12495,12620,12746,
  12874,13002,13132,13264,13396,13530,13666,13802,
  13940,14080,14221,14363,14506,14652,14798,14946,
  15096,15247,15399,15553,15709,15866,16024,16184,
  16346,16510,16675,16842,17010,17180,17352,17526,
  17701,17878,18057,18237,18420,18604,18790,18978,
  19167,19359,19553,19748,19946,20145,20347,20550,
  20756,20963,21173,21385,21598,21814,22033,22253,
  22475,22700,22927,23156,23388,23622,23858,24097,
  24338,24581,24827,25075,25326,25579,25835,26093,
  26354,26618,26884,27153,27424,27699,27976,28255,
  28538,28823,29112,29403,29697,29994,30294,30597,
  30903,31212,31524,31839,32157,32479,32804,33132
};


/* private operation */
static uint8_t MPU9250_getID(void);
static bool    MPU9250_testConnection(void);

static bool    MPU9250_getSelfTest(void);
static bool    MPU9250_evaluateSelfTest(float low, float high, float value, char* string);

static uint8_t MPU9250_getSampleRate(void);
static void    MPU9250_setSampleRate(uint8_t rate);

static uint8_t MPU9250_getExternalFrameSync(void);
static void    MPU9250_setExternalFrameSync(uint8_t sync);

static uint8_t MPU9250_getDLPFMode(void);
static void    MPU9250_setDLPFMode(uint8_t mode);

static uint8_t MPU9250_getFullScaleGyroRangeId(void);
static float   MPU9250_getFullScaleGyroDPL(void);
static void    MPU9250_setFullScaleGyroRange(uint8_t range);

static void    MPU9250_setGyroXSelfTest(bool enabled);
static void    MPU9250_setGyroYSelfTest(bool enabled);
static void    MPU9250_setGyroZSelfTest(bool enabled);

static bool    MPU9250_getAccelXSelfTest(void);
static void    MPU9250_setAccelXSelfTest(bool enabled);
static bool    MPU9250_getAccelYSelfTest(void);
static void    MPU9250_setAccelYSelfTest(bool enabled);
static bool    MPU9250_getAccelZSelfTest(void);
static void    MPU9250_setAccelZSelfTest(bool enabled);

static uint8_t MPU9250_getFullScaleAccelRangeId(void);
static float   MPU9250_getFullScaleAccelGPL(void);
static void    MPU9250_setFullScaleAccelRange(uint8_t range);

static void    MPU9250_setAccelDLPF(uint8_t range);
static uint8_t MPU9250_getDHPFMode(void);
static void    MPU9250_setDHPFMode(uint8_t bandwidth);

static bool    MPU9250_getTempFIFOEnabled(void);
static void    MPU9250_setTempFIFOEnabled(bool enabled);
static bool    MPU9250_getXGyroFIFOEnabled(void);
static void    MPU9250_setXGyroFIFOEnabled(bool enabled);
static bool    MPU9250_getYGyroFIFOEnabled(void);
static void    MPU9250_setYGyroFIFOEnabled(bool enabled);
static bool    MPU9250_getZGyroFIFOEnabled(void);
static void    MPU9250_setZGyroFIFOEnabled(bool enabled);
static bool    MPU9250_getAccelFIFOEnabled(void);
static void    MPU9250_setAccelFIFOEnabled(bool enabled);
static bool    MPU9250_getSlave2FIFOEnabled(void);
static void    MPU9250_setSlave2FIFOEnabled(bool enabled);
static bool    MPU9250_getSlave1FIFOEnabled(void);
static void    MPU9250_setSlave1FIFOEnabled(bool enabled);
static bool    MPU9250_getSlave0FIFOEnabled(void);
static void    MPU9250_setSlave0FIFOEnabled(bool enabled);

static bool    MPU9250_getMultiMasterEnabled(void);
static void    MPU9250_setMultiMasterEnabled(bool enabled);
static bool    MPU9250_getWaitForExternalSensorEnabled(void);
static void    MPU9250_setWaitForExternalSensorEnabled(bool enabled);
static bool    MPU9250_getSlave3FIFOEnabled(void);
static void    MPU9250_setSlave3FIFOEnabled(bool enabled);
static bool    MPU9250_getSlaveReadWriteTransitionEnabled(void);
static void    MPU9250_setSlaveReadWriteTransitionEnabled(bool enabled);
static uint8_t MPU9250_getMasterClockSpeed(void);
static void    MPU9250_setMasterClockSpeed(uint8_t speed);

static uint8_t MPU9250_getSlaveAddress(uint8_t num);
static void    MPU9250_setSlaveAddress(uint8_t num, uint8_t address);
static uint8_t MPU9250_getSlaveRegister(uint8_t num);
static void    MPU9250_setSlaveRegister(uint8_t num, uint8_t reg);
static bool    MPU9250_getSlaveEnabled(uint8_t num);
static void    MPU9250_setSlaveEnabled(uint8_t num, bool enabled);
static bool    MPU9250_getSlaveWordByteSwap(uint8_t num);
static void    MPU9250_setSlaveWordByteSwap(uint8_t num, bool enabled);
static bool    MPU9250_getSlaveWriteMode(uint8_t num);
static void    MPU9250_setSlaveWriteMode(uint8_t num, bool mode);
static bool    MPU9250_getSlaveWordGroupOffset(uint8_t num);
static void    MPU9250_setSlaveWordGroupOffset(uint8_t num, bool enabled);
static uint8_t MPU9250_getSlaveDataLength(uint8_t num);
static void    MPU9250_setSlaveDataLength(uint8_t num, uint8_t length);

static uint8_t MPU9250_getSlave4Address(void);
static void    MPU9250_setSlave4Address(uint8_t address);
static uint8_t MPU9250_getSlave4Register(void);
static void    MPU9250_setSlave4Register(uint8_t reg);
static void    MPU9250_setSlave4OutputByte(uint8_t data);
static bool    MPU9250_getSlave4Enabled(void);
static void    MPU9250_setSlave4Enabled(bool enabled);
static bool    MPU9250_getSlave4InterruptEnabled(void);
static void    MPU9250_setSlave4InterruptEnabled(bool enabled);
static bool    MPU9250_getSlave4WriteMode(void);
static void    MPU9250_setSlave4WriteMode(bool mode);
static uint8_t MPU9250_getSlave4MasterDelay(void);
static void    MPU9250_setSlave4MasterDelay(uint8_t delay);
static uint8_t MPU9250_getSlate4InputByte(void);

static bool    MPU9250_getPassthroughStatus(void);
static bool    MPU9250_getSlave4IsDone(void);
static bool    MPU9250_getLostArbitration(void);
static bool    MPU9250_getSlave4Nack(void);
static bool    MPU9250_getSlave3Nack(void);
static bool    MPU9250_getSlave2Nack(void);
static bool    MPU9250_getSlave1Nack(void);
static bool    MPU9250_getSlave0Nack(void);

static bool    MPU9250_getInterruptMode(void);
static void    MPU9250_setInterruptMode(bool mode);
static bool    MPU9250_getInterruptDrive(void);
static void    MPU9250_setInterruptDrive(bool drive);
static bool    MPU9250_getInterruptLatch(void);
static void    MPU9250_setInterruptLatch(bool latch);
static bool    MPU9250_getInterruptLatchClear(void);
static void    MPU9250_setInterruptLatchClear(bool clear);
static bool    MPU9250_getFSyncInterruptLevel(void);
static void    MPU9250_setFSyncInterruptLevel(bool level);
static bool    MPU9250_getFSyncInterruptEnabled(void);
static void    MPU9250_setFSyncInterruptEnabled(bool enabled);
static bool    MPU9250_getI2CBypassEnabled(void);
static void    MPU9250_setI2CBypassEnabled(bool enabled);
static bool    MPU9250_getClockOutputEnabled(void);
static void    MPU9250_setClockOutputEnabled(bool enabled);

static uint8_t MPU9250_getIntEnabled(void);
static void    MPU9250_setIntEnabled(uint8_t enabled);
static bool    MPU9250_getIntFreefallEnabled(void);
static void    MPU9250_setIntFreefallEnabled(bool enabled);
static bool    MPU9250_getIntMotionEnabled(void);
static void    MPU9250_setIntMotionEnabled(bool enabled);
static bool    MPU9250_getIntZeroMotionEnabled(void);
static void    MPU9250_setIntZeroMotionEnabled(bool enabled);
static bool    MPU9250_getIntFIFOBufferOverflowEnabled(void);
static void    MPU9250_setIntFIFOBufferOverflowEnabled(bool enabled);
static bool    MPU9250_getIntI2CMasterEnabled(void);
static void    MPU9250_setIntI2CMasterEnabled(bool enabled);
static bool    MPU9250_getIntDataReadyEnabled(void);
static void    MPU9250_setIntDataReadyEnabled(bool enabled);

static uint8_t MPU9250_getIntStatus(void);
static bool    MPU9250_getIntFreefallStatus(void);
static bool    MPU9250_getIntMotionStatus(void);
static bool    MPU9250_getIntZeroMotionStatus(void);
static bool    MPU9250_getIntFIFOBufferOverflowStatus(void);
static bool    MPU9250_getIntI2CMasterStatus(void);
static bool    MPU9250_getIntDataReadyStatus(void);

static void    MPU9250_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
static void    MPU9250_getAcceleration(int16_t* x, int16_t* y, int16_t* z);
static int16_t MPU9250_getAccelerationX(void);
static int16_t MPU9250_getAccelerationY(void);
static int16_t MPU9250_getAccelerationZ(void);
static int16_t MPU9250_getTemperature(void);
static void    MPU9250_getRotation(int16_t* x, int16_t* y, int16_t* z);
static int16_t MPU9250_getRotationX(void);
static int16_t MPU9250_getRotationY(void);
static int16_t MPU9250_getRotationZ(void);
static uint8_t MPU9250_getExternalSensorByte(int position);
static uint16_t MPU9250_getExternalSensorWord(int position);
static uint32_t MPU9250_getExternalSensorDWord(int position);

static bool    MPU9250_getXNegMotionDetected(void);
static bool    MPU9250_getXPosMotionDetected(void);
static bool    MPU9250_getYNegMotionDetected(void);
static bool    MPU9250_getYPosMotionDetected(void);
static bool    MPU9250_getZNegMotionDetected(void);
static bool    MPU9250_getZPosMotionDetected(void);
static bool    MPU9250_getZeroMotionDetected(void);

static void    MPU9250_setSlaveOutputByte(uint8_t num, uint8_t data);

static bool    MPU9250_getExternalShadowDelayEnabled(void);
static void    MPU9250_setExternalShadowDelayEnabled(bool enabled);
static bool    MPU9250_getSlaveDelayEnabled(uint8_t num);
static void    MPU9250_setSlaveDelayEnabled(uint8_t num, bool enabled);

static void    MPU9250_resetGyroscopePath(void);
static void    MPU9250_resetAccelerometerPath(void);
static void    MPU9250_resetTemperaturePath(void);

static uint8_t MPU9250_getAccelerometerPowerOnDelay(void);
static void    MPU9250_setAccelerometerPowerOnDelay(uint8_t delay);
static uint8_t MPU9250_getFreefallDetectionCounterDecrement(void);
static void    MPU9250_setFreefallDetectionCounterDecrement(uint8_t decrement);
static uint8_t MPU9250_getMotionDetectionCounterDecrement(void);
static void    MPU9250_setMotionDetectionCounterDecrement(uint8_t decrement);

static bool    MPU9250_getFIFOEnabled(void);
static void    MPU9250_setFIFOEnabled(bool enabled);
static bool    MPU9250_getI2CMasterModeEnabled(void);
static void    MPU9250_setI2CMasterModeEnabled(bool enabled);
static void    MPU9250_switchSPIEnabled(bool enabled);
static void    MPU9250_resetFIFO(void);
static void    MPU9250_resetI2CMaster(void);
static void    MPU9250_resetSensors(void);

static void    MPU9250_reset(void);
static bool    MPU9250_getSleepEnabled(void);
static void    MPU9250_setSleepEnabled(bool enabled);
static bool    MPU9250_getWakeCycleEnabled(void);
static void    MPU9250_setWakeCycleEnabled(bool enabled);
static bool    MPU9250_getTempSensorEnabled(void);
static void    MPU9250_setTempSensorEnabled(bool enabled);
static uint8_t MPU9250_getClockSource(void);
static void    MPU9250_setClockSource(uint8_t source);

static uint8_t MPU9250_getWakeFrequency(void);
static void    MPU9250_setWakeFrequency(uint8_t frequency);
static bool    MPU9250_getStandbyXAccelEnabled(void);
static void    MPU9250_setStandbyXAccelEnabled(bool enabled);
static bool    MPU9250_getStandbyYAccelEnabled(void);
static void    MPU9250_setStandbyYAccelEnabled(bool enabled);
static bool    MPU9250_getStandbyZAccelEnabled(void);
static void    MPU9250_setStandbyZAccelEnabled(bool enabled);
static bool    MPU9250_getStandbyXGyroEnabled(void);
static void    MPU9250_setStandbyXGyroEnabled(bool enabled);
static bool    MPU9250_getStandbyYGyroEnabled(void);
static void    MPU9250_setStandbyYGyroEnabled(bool enabled);
static bool    MPU9250_getStandbyZGyroEnabled(void);
static void    MPU9250_setStandbyZGyroEnabled(bool enabled);

static uint16_t MPU9250_getFIFOCount(void);
static uint8_t  MPU9250_getFIFOByte(void);
static void     MPU9250_setFIFOByte(uint8_t data);



/**
 * get MPU9250 device ID
 * @return TRUE if operation was successful, otherwise FALSE
 */
static uint8_t MPU9250_getID(void)
{
    bool ret;
    uint8_t mpu9250_id = 0x00;

    ret = I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_WHO_AM_I, &mpu9250_id);

    if(ret == false)
    {
        printf("MPU9250 get ID ERROR!!!\n");
    }
    else
    {
        // printf("The MPU9250 ID is %u\n", mpu9250_id);
    }
    return mpu9250_id;
}


/**
 * verify the I2C connection
 * @return TRUE if connection was successful, otherwise FALSE
 */
static bool MPU9250_testConnection(void)
{
    return MPU9250_getID() == 0x71; /* 0x71 is MPU9250 ID with AD0 = 0 */
}


/**
 * do a MPU9250 self test
 * @return TRUE if self test passed, otherwise FALSE
 */
static bool MPU9250_getSelfTest(void)
{
    uint8_t raw_data[6] = {0, 0, 0, 0, 0, 0};
    uint8_t old_config[5];
    uint8_t selftest_data[6];
    int32_t aver_gyro_data[3] = {0}, aver_acce_data[3] = {0};
    int32_t aver_gyro_selftest_data[3] = {0}, aver_acce_selftest_data[3] = {0};
    int32_t factory_trim[6];
    float gyro_diff[3], acce_diff[3];
    uint8_t FS = 0;

    /* save old config */
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_SMPLRT_DIV,    old_config);
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_CONFIG,        old_config + 1);
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_CONFIG,   old_config + 2);
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG2, old_config + 3);
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG,  old_config + 4);

    /* write test config */
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_SMPLRT_DIV, 0x00);    /* set gyro sample rate to 1 kHz */
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_CONFIG, 0x02);        /* set gyro sample rate to 1 kHz and DLPF to 92 Hz */
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_CONFIG, 1<<FS);  /* set full scale range for the gyro to 250 dps */
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG2, 0x02); /* set accelerometer rate to 1 kHz and bandwidth to 92 Hz */
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG, 1<<FS); /* set full scale range for the accelerometer to 2 g */

    for(int i = 1; i <= 200; i++)
    {
        I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, 6, raw_data);
        aver_acce_data[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);
        aver_acce_data[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
        aver_acce_data[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

        I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_XOUT_H, 6, raw_data);
        aver_gyro_data[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);
        aver_gyro_data[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
        aver_gyro_data[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);
    }

    for(int i = 0; i < 3; i++)
    {
        aver_acce_data[i] /= 200;
        aver_gyro_data[i] /= 200;
    }

    /* configure the acce and gyro for self-test */
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG, 0xE0); /* enable self test on all three axes and set acce range to +/- 2g */
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_CONFIG, 0xE0);  /* enable self test on all three axes and set gyro range to +/- 250dps */
    vTaskDelay(25); /* delay a while to let the device stabilize */

    for(int i = 1; i <= 200; i++)
    {
        I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, 6, raw_data);
        aver_acce_selftest_data[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);
        aver_acce_selftest_data[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
        aver_acce_selftest_data[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

        I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_XOUT_H, 6, raw_data);
        aver_gyro_selftest_data[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);
        aver_gyro_selftest_data[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
        aver_gyro_selftest_data[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);
    }

    for(int i = 0; i < 3; i++)
    {
        aver_acce_selftest_data[i] /= 200;
        aver_gyro_selftest_data[i] /= 200;
    }

    /* configure the gyro and accelerometer for normal operation */
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG, 0x00);
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_CONFIG, 0x00);
    vTaskDelay(25); /* delay a while to let the device stabilize */

    /* retrieve acce and gyro factory Self-Test Code from USR_Reg */
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_SELF_TEST_X_ACCEL, selftest_data);
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_SELF_TEST_Y_ACCEL, selftest_data + 1);
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_SELF_TEST_Z_ACCEL, selftest_data + 2);
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_SELF_TEST_X_GYRO,  selftest_data + 3);
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_SELF_TEST_Y_GYRO,  selftest_data + 4);
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_SELF_TEST_Z_GYRO,  selftest_data + 5);

    for(int i = 0; i < 6; i++)
    {
        if(selftest_data[i] != 0)
        {
            factory_trim[i] = mpu9250_selftest_table[selftest_data[i]-1];
        }
        else
        {
            factory_trim[i] = 0;
        }
    }

    /* report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response */
    /* to get percent, must multiply by 100 */
    for(int i = 0; i < 3; i++)
    {
        acce_diff[i] = 100.0F * ((float)((aver_acce_selftest_data[i] - aver_acce_data[i]) - factory_trim[i])) / factory_trim[i];
        gyro_diff[i] = 100.0F * ((float)((aver_gyro_selftest_data[i] - aver_gyro_data[i]) - factory_trim[i+3])) / factory_trim[i+3];
    }

    /* restore old config */
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_SMPLRT_DIV,    old_config[0]);
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_CONFIG,        old_config[1]);
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_CONFIG,   old_config[2]);
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG2, old_config[3]);
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG,  old_config[4]);

    if(MPU9250_evaluateSelfTest(MPU9250_SELF_TEST_GYRO_LOW, MPU9250_SELF_TEST_GYRO_HIGH, gyro_diff[0], "gyro x") &&
       MPU9250_evaluateSelfTest(MPU9250_SELF_TEST_GYRO_LOW, MPU9250_SELF_TEST_GYRO_HIGH, gyro_diff[1], "gyro y") &&
       MPU9250_evaluateSelfTest(MPU9250_SELF_TEST_GYRO_LOW, MPU9250_SELF_TEST_GYRO_HIGH, gyro_diff[2], "gyro z") &&
       MPU9250_evaluateSelfTest(MPU9250_SELF_TEST_ACCE_LOW, MPU9250_SELF_TEST_ACCE_HIGH, acce_diff[0], "acce x") &&
       MPU9250_evaluateSelfTest(MPU9250_SELF_TEST_ACCE_LOW, MPU9250_SELF_TEST_ACCE_HIGH, acce_diff[1], "acce y") &&
       MPU9250_evaluateSelfTest(MPU9250_SELF_TEST_ACCE_LOW, MPU9250_SELF_TEST_ACCE_HIGH, acce_diff[2], "acce z"))
    {
        return true;
    }
    else
    {
        return false;
    }
}


/**
 * evaluate the values from a MPU9250 self test
 * @param low   : The low limit of the self test
 * @param high  : The high limit of the self test
 * @param value : The value to compare with.
 * @param string: A pointer to a string describing the value.
 *
 * @return TRUE if self test within low - high limit, otherwise FALSE
 */
static bool MPU9250_evaluateSelfTest(float low, float high, float value, char* string)
{
    if(value < low || value > high)
    {
        printf("Self test %s [FAIL]. low: %0.2f, high: %0.2f, measured: %0.2f\n",
                string, (double)low, (double)high, (double)value);
        return false;
    }
    return true;
}


/**
 * get the gyroscope output(sample rate) rate divider
 * the sensor register output, FIFO output, DMP sampling, motion detection, zero
 * motion detection, and free fall detection are all based on the sample Rate
 *
 * the sample rate is generated by dividing the gyroscope output rate by
 * SMPLRT_DIV:
 *
 * sample rate = gyroscope output rate / (1 + SMPLRT_DIV)
 *
 * where gyroscope output rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
 * 7), and 1kHz when the DLPF is enabled (see Register 26).
 *
 * note: the accelerometer output rate is 1kHz, this means that for a sample
 * rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * for a diagram of the gyroscope and accelerometer signal paths, see Section 8
 * of the MPU-6000/MPU-6500 Product Specification document.
 *
 * @return current sample rate divider
 */
static uint8_t MPU9250_getSampleRate(void)
{
    return 1;
}

void MPU9250_Init(void)
{
    if(MPU9250_testConnection() == true)
    {
        printf("test connection SUCCESS!!!\n");
    }

    if(MPU9250_getSelfTest() == true)
    {
        printf("self test SUCCESS!!!\n");
    }
}