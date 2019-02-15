#include <stdio.h>
#include "driver/i2c.h"
#include "sophum_driver_i2c.h"
#include "sophum_module_mpu9250.h"


/* private const variable */
static const uint16_t mpu9250_selftest_table[256] = {
  2620,2646,2672,2699,2726,2753,2781,2808, /*7*/
  2837,2865,2894,2923,2952,2981,3011,3041, /*15*/
  3072,3102,3133,3165,3196,3228,3261,3293, /*23*/
  3326,3359,3393,3427,3461,3496,3531,3566, /*31*/
  3602,3638,3674,3711,3748,3786,3823,3862, /*39*/
  3900,3939,3979,4019,4059,4099,4140,4182, /*47*/
  4224,4266,4308,4352,4395,4439,4483,4528, /*55*/
  4574,4619,4665,4712,4759,4807,4855,4903, /*63*/
  4953,5002,5052,5103,5154,5205,5257,5310, /*71*/
  5363,5417,5471,5525,5581,5636,5693,5750, /*79*/
  5807,5865,5924,5983,6043,6104,6165,6226, /*87*/
  6289,6351,6415,6479,6544,6609,6675,6742, /*95*/
  6810,6878,6946,7016,7086,7157,7229,7301, /*103*/
  7374,7448,7522,7597,7673,7750,7828,7906, /*111*/
  7985,8065,8145,8227,8309,8392,8476,8561, /*119*/
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

/* private variable */
static uint8_t tmp_buf[16];

/* private operation */
static uint8_t MPU9250_getID(void);
static bool    MPU9250_testConnection(void);

static bool    MPU9250_getSelfTest(void);
static bool    MPU9250_evaluateSelfTest(float low, float high, float value, char* string);

/* MPU9250_SMPLRT_DIV register */
static uint8_t MPU9250_getSampleRate(void);
static void    MPU9250_setSampleRate(uint8_t rate);

/* MPU9250_CONFIG register */
static uint8_t MPU9250_getExternalFrameSync(void);
static void    MPU9250_setExternalFrameSync(uint8_t sync);

static uint8_t MPU9250_getDLPFMode(void);
static void    MPU9250_setDLPFMode(uint8_t mode);

/* MPU9250_ GYRO_CONFIG register */
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
// static uint8_t MPU9250_getDHPFMode(void);
// static void    MPU9250_setDHPFMode(uint8_t bandwidth);

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
// static bool    MPU9250_getClockOutputEnabled(void);
// static void    MPU9250_setClockOutputEnabled(bool enabled);

static uint8_t MPU9250_getIntEnabled(void);
static void    MPU9250_setIntEnabled(uint8_t enabled);
// static bool    MPU9250_getIntFreefallEnabled(void);
// static void    MPU9250_setIntFreefallEnabled(bool enabled);
static bool    MPU9250_getIntMotionEnabled(void);
static void    MPU9250_setIntMotionEnabled(bool enabled);
// static bool    MPU9250_getIntZeroMotionEnabled(void);
// static void    MPU9250_setIntZeroMotionEnabled(bool enabled);
static bool    MPU9250_getIntFIFOBufferOverflowEnabled(void);
static void    MPU9250_setIntFIFOBufferOverflowEnabled(bool enabled);
// static bool    MPU9250_getIntI2CMasterEnabled(void);
// static void    MPU9250_setIntI2CMasterEnabled(bool enabled);
static bool    MPU9250_getIntDataReadyEnabled(void);
static void    MPU9250_setIntDataReadyEnabled(bool enabled);

static uint8_t MPU9250_getIntStatus(void);
// static bool    MPU9250_getIntFreefallStatus(void);
static bool    MPU9250_getIntMotionStatus(void);
// static bool    MPU9250_getIntZeroMotionStatus(void);
static bool    MPU9250_getIntFIFOBufferOverflowStatus(void);
// static bool    MPU9250_getIntI2CMasterStatus(void);
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

// static bool    MPU9250_getXNegMotionDetected(void);
// static bool    MPU9250_getXPosMotionDetected(void);
// static bool    MPU9250_getYNegMotionDetected(void);
// static bool    MPU9250_getYPosMotionDetected(void);
// static bool    MPU9250_getZNegMotionDetected(void);
// static bool    MPU9250_getZPosMotionDetected(void);
// static bool    MPU9250_getZeroMotionDetected(void);

static void    MPU9250_setSlaveOutputByte(uint8_t num, uint8_t data);

static bool    MPU9250_getExternalShadowDelayEnabled(void);
static void    MPU9250_setExternalShadowDelayEnabled(bool enabled);
static bool    MPU9250_getSlaveDelayEnabled(uint8_t num);
static void    MPU9250_setSlaveDelayEnabled(uint8_t num, bool enabled);

static void    MPU9250_resetGyroscopePath(void);
static void    MPU9250_resetAccelerometerPath(void);
static void    MPU9250_resetTemperaturePath(void);

// static uint8_t MPU9250_getAccelerometerPowerOnDelay(void);
// static void    MPU9250_setAccelerometerPowerOnDelay(uint8_t delay);
// static uint8_t MPU9250_getFreefallDetectionCounterDecrement(void);
// static void    MPU9250_setFreefallDetectionCounterDecrement(uint8_t decrement);
// static uint8_t MPU9250_getMotionDetectionCounterDecrement(void);
// static void    MPU9250_setMotionDetectionCounterDecrement(uint8_t decrement);

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
// static bool    MPU9250_getTempSensorEnabled(void);
// static void    MPU9250_setTempSensorEnabled(bool enabled);
static uint8_t MPU9250_getClockSource(void);
static void    MPU9250_setClockSource(uint8_t source);

// static uint8_t MPU9250_getWakeFrequency(void);
// static void    MPU9250_setWakeFrequency(uint8_t frequency);
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
 * Get MPU9250 device ID
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
 * Verify the I2C connection
 * @return TRUE if connection was successful, otherwise FALSE
 */
static bool MPU9250_testConnection(void)
{
    return MPU9250_getID() == 0x71; /* 0x71 is MPU9250 ID with AD0 = 0 */
}


/**
 * Do a MPU9250 self test
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
 * Evaluate the values from a MPU9250 self test
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


/** Get the gyroscope output(sample rate) rate divider
 *
 * The sensor register output, FIFO output, DMP sampling, motion detection, zero
 * motion detection, and free fall detection are all based on the sample Rate.
 *
 * The sample rate is generated by dividing the gyroscope output rate by
 * SMPLRT_DIV:
 *
 * sample rate = gyroscope output rate / (1 + SMPLRT_DIV)
 *
 * Where gyroscope output rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
 * 7), and 1kHz when the DLPF is enabled (see Register 26)
 *
 * Note: The accelerometer output rate is 1kHz, this means that for a sample
 * rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * For a diagram of the gyroscope and accelerometer signal paths, see Section 8
 * of the MPU-6000/MPU-6500 Product Specification document.
 *
 * @return current sample rate divider
 */
static uint8_t MPU9250_getSampleRate(void)
{
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_SMPLRT_DIV, tmp_buf);
    return tmp_buf[0];
}


/**
 * Set gyro sample rate divider
 * @param rate: sample rate divider
 *
 * @return void
 */
static void MPU9250_setSampleRate(uint8_t rate)
{
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_SMPLRT_DIV, rate);
}


/** Get external FSYNC configuration
 *
 * Configures the external Frame Synchronization (FSYNC) pin sampling. An
 * external signal connected to the FSYNC pin can be sampled by configuring
 * EXT_SYNC_SET. Signal changes to the FSYNC pin are latched so that short
 * strobes may be captured. The latched FSYNC signal will be sampled at the
 * Sampling Rate, as defined in register 25. after sampling, the latch will
 * reset to the current FSYNC signal state.
 *
 * The sampled value will be reported in place of the least significant bit in
 * a sensor data register determined by the value of EXT_SYNC_SET according to
 * the following table.
 *
 * <pre>
 * EXT_SYNC_SET | FSYNC Bit Location
 * -------------+-------------------
 * 0            | Input disabled
 * 1            | TEMP_OUT_L[0]
 * 2            | GYRO_XOUT_L[0]
 * 3            | GYRO_YOUT_L[0]
 * 4            | GYRO_ZOUT_L[0]
 * 5            | ACCEL_XOUT_L[0]
 * 6            | ACCEL_YOUT_L[0]
 * 7            | ACCEL_ZOUT_L[0]
 * </pre>
 *
 * @return FSYNC configuration value
 */
static uint8_t MPU9250_getExternalFrameSync(void)
{
    I2C_readMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_CONFIG,
                      MPU9250_CONFIG_EXT_SYNC_SET_BIT, MPU9250_CONFIG_EXT_SYNC_SET_LEN,
                      tmp_buf);
    return tmp_buf[0];
}


/** Set external FSYNC configuration
 *
 * @param sync: FSYNC config value
 *
 * @return void
 */
static void MPU9250_setExternalFrameSync(uint8_t sync)
{
    I2C_writeMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_CONFIG,
                       MPU9250_CONFIG_EXT_SYNC_SET_BIT, MPU9250_CONFIG_EXT_SYNC_SET_LEN,
                       sync);
}


/** Get digital low-pass filter configuration
 *
 * The DLPF_CFG parameter sets the digital low pass filter configuration. It
 * also determines the internal sampling rate used by the device as shown in
 * the table below.
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * </pre>
 *
 * @return DLFP configuration
 */
static uint8_t MPU9250_getDLPFMode(void)
{
    I2C_readMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_CONFIG,
                      MPU9250_CONFIG_DLPF_CFG_BIT, MPU9250_CONFIG_DLPF_CFG_LEN,
                      tmp_buf);
    return tmp_buf[0];
}


/** Set Digital Low-Pass Filter configuration
 *
 * @param mode: DLPF config value
 *
 * @return void
 */
static void MPU9250_setDLPFMode(uint8_t mode)
{
    I2C_writeMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_CONFIG,
                       MPU9250_CONFIG_DLPF_CFG_BIT, MPU9250_CONFIG_DLPF_CFG_LEN,
                       mode);
}


/** Get full-scale gyroscope range id
 *
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return current full-scale gyroscope range setting
 */
static uint8_t MPU9250_getFullScaleGyroRangeId(void)
{
    I2C_readMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_CONFIG,
                      MPU9250_GYRO_CONFIG_GYRO_FS_SEL_BIT, MPU9250_GYRO_CONFIG_GYRO_FS_SEL_LEN,
                      tmp_buf);
    return tmp_buf[0];
}


/** Get full-scale gyroscope degrees per LSB
 *
 * @return float of current full-scale gyroscope setting as degrees per LSB
 */
static float MPU9250_getFullScaleGyroDPL(void)
{
    int32_t range_id;
    float range;

    range_id = MPU9250_getFullScaleGyroRangeId();
    switch(range_id)
    {
        case MPU9250_GYRO_CONFIG_GYRO_FS_SEL_250DPS:
            range = MPU9250_DEG_PER_LSB_250DPS;
        break;
        case MPU9250_GYRO_CONFIG_GYRO_FS_SEL_500DPS:
            range = MPU9250_DEG_PER_LSB_500DPS;
        break;
        case MPU9250_GYRO_CONFIG_GYRO_FS_SEL_1000DPS:
            range = MPU9250_DEG_PER_LSB_1000DPS;
        break;
        case MPU9250_GYRO_CONFIG_GYRO_FS_SEL_2000DPS:
            range = MPU9250_DEG_PER_LSB_2000DPS;
        break;
        default:
            range = MPU9250_DEG_PER_LSB_1000DPS;
        break;
    }
    return range;
}


/** Set full-scale gyroscope range
 *
 * @param range: full-scale gyroscope range value
 *
 * @return void
 */
static void MPU9250_setFullScaleGyroRange(uint8_t range)
{
    I2C_writeMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_CONFIG,
                       MPU9250_GYRO_CONFIG_GYRO_FS_SEL_BIT, MPU9250_GYRO_CONFIG_GYRO_FS_SEL_LEN,
                       range);
}


/** Set self-test enabled value for gyro X axis
 *
 * @param enabled: self-test enabled value
 *
 * @return void
 */
static void MPU9250_setGyroXSelfTest(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_CONFIG,
                    MPU9250_GYRO_CONFIG_XGYRO_CTEN_BIT, enabled);
}


/** Set self-test enabled value for gyro Y axis
 *
 * @param enabled: self-test enabled value
 *
 * @return void
 */
static void MPU9250_setGyroYSelfTest(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_CONFIG,
                    MPU9250_GYRO_CONFIG_YGYRO_CTEN_BIT, enabled);
}


/** Set self-test enabled value for gyro Z axis
 *
 * @param enabled: self-test enabled value
 *
 * @return void
 */
static void MPU9250_setGyroZSelfTest(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_CONFIG,
                    MPU9250_GYRO_CONFIG_ZGYRO_CTEN_BIT, enabled);
}


/** Get self-test enabled setting for acce X axis
 *
 * @return self-test enabled value
 */
static bool MPU9250_getAccelXSelfTest(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG,
                   MPU9250_ACCEL_CONFIG_AX_ST_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set self-test enabled setting for acce X axis
 *
 * @param enabled: self-test enabled value
 *
 * @return void
 */
static void MPU9250_setAccelXSelfTest(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG,
                    MPU9250_ACCEL_CONFIG_AX_ST_EN_BIT, enabled);
}


/** Get self-test enabled setting for acce Y axis
 *
 * @return self-test enabled value
 */
static bool MPU9250_getAccelYSelfTest(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG,
                   MPU9250_ACCEL_CONFIG_AY_ST_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set self-test enabled setting for acce Y axis
 *
 * @param enabled: self-test enabled value
 *
 * @return void
 */
static void MPU9250_setAccelYSelfTest(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG,
                    MPU9250_ACCEL_CONFIG_AY_ST_EN_BIT, enabled);
}


/** Get self-test enabled setting for acce Z axis
 *
 * @return Self-test enabled value
 */
static bool MPU9250_getAccelZSelfTest(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG,
                   MPU9250_ACCEL_CONFIG_AZ_ST_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set self-test enabled setting for acce Z axis
 *
 * @param enabled: self-test enabled value
 *
 * @return void
 */
static void MPU9250_setAccelZSelfTest(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG,
                    MPU9250_ACCEL_CONFIG_AZ_ST_EN_BIT, enabled);
}


/** Get full-scale accelerometer range
 *
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 */
static uint8_t MPU9250_getFullScaleAccelRangeId(void)
{
    I2C_readMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG,
                      MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_BIT, MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_LEN,
                      tmp_buf);
    return tmp_buf[0];
}


/** Get full-scale accelerometer G per LSB
 *
 * @return float of current full-scale accelerometer setting as G per LSB
 */
static float MPU9250_getFullScaleAccelGPL(void)
{
    int32_t range_id;
    float range;

    range_id = MPU9250_getFullScaleAccelRangeId();
    switch(range_id)
    {
        case MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_2G:
            range = MPU9250_G_PER_LSB_2G;
        break;
        case MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_4G:
            range = MPU9250_G_PER_LSB_4G;
        break;
        case MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_8G:
            range = MPU9250_G_PER_LSB_8G;
        break;
        case MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_16G:
            range = MPU9250_G_PER_LSB_16G;
        break;
        default:
            range = MPU9250_G_PER_LSB_8G;
        break;
    }
    return range;
}


/** Set full-scale accelerometer range
 *
 * @param range: full-scale accelerometer range setting
 *
 * @return void
 */
static void MPU9250_setFullScaleAccelRange(uint8_t range)
{
    I2C_writeMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG,
                       MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_BIT, MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_LEN,
                       range);
}


/** Set accelerometer Digital Low-Pass Filter
 *
 * @param range: DLPF setting
 *
 * @return void
 */
static void MPU9250_setAccelDLPF(uint8_t range)
{
    I2C_writeMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG2,
                       MPU9250_ACCEL_CONFIG2_A_DLPFCFG_BIT, MPU9250_ACCEL_CONFIG2_A_DLPFCFG_LEN,
                       range);
}


/** Get temperature FIFO enabled value
 *
 * When set to 1, this bit enables TEMP_OUT_H and TEMP_OUT_L (Registers 65 and
 * 66) to be written into the FIFO buffer.
 *
 * @return current temperature FIFO enabled value
 */
static bool MPU9250_getTempFIFOEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                   MPU9250_FIFO_EN_TEMP_OUT_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set temperature FIFO enabled value
 *
 * @param enabled: temperature FIFO enabled value
 *
 * @return void
 */
static void MPU9250_setTempFIFOEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                    MPU9250_FIFO_EN_TEMP_OUT_BIT, enabled);
}


/** Get gyroscope X-axis FIFO enabled value
 *
 * When set to 1, this bit enables GYRO_XOUT_H and GYRO_XOUT_L (Registers 67 and
 * 68) to be written into the FIFO buffer.
 *
 * @return current gyroscope X-axis FIFO enabled value
 */
static bool MPU9250_getXGyroFIFOEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                   MPU9250_FIFO_EN_GYRO_XOUT_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set gyroscope X-axis FIFO enabled value
 *
 * @param enabled: gyroscope X-axis FIFO enabled value
 *
 * @return void
 */
static void MPU9250_setXGyroFIFOEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                    MPU9250_FIFO_EN_GYRO_XOUT_BIT, enabled);
}


/** Get gyroscope Y-axis FIFO enabled value
 *
 * When set to 1, this bit enables GYRO_YOUT_H and GYRO_YOUT_L (Registers 69 and
 * 70) to be written into the FIFO buffer.
 *
 * @return current gyroscope Y-axis FIFO enabled value
 */
static bool MPU9250_getYGyroFIFOEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                   MPU9250_FIFO_EN_GYRO_YOUT_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set gyroscope Y-axis FIFO enabled value
 *
 * @param enabled: gyroscope Y-axis FIFO enabled value
 *
 * @return void
 */
static void MPU9250_setYGyroFIFOEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                    MPU9250_FIFO_EN_GYRO_YOUT_BIT, enabled);
}


/** Get gyroscope Z-axis FIFO enabled value
 *
 * When set to 1, this bit enables GYRO_ZOUT_H and GYRO_ZOUT_L (Registers 71 and
 * 72) to be written into the FIFO buffer.
 *
 * @return current gyroscope Z-axis FIFO enabled value
 */
static bool MPU9250_getZGyroFIFOEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                   MPU9250_FIFO_EN_GYRO_ZOUT_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set gyroscope Z-axis FIFO enabled value
 *
 * @param enabled: gyroscope Z-axis FIFO enabled value
 *
 * @return void
 */
static void MPU9250_setZGyroFIFOEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                    MPU9250_FIFO_EN_GYRO_ZOUT_BIT, enabled);
}


/** Get accelerometer FIFO enabled value
 *
 * When set to 1, this bit enables ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H,
 * ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L (Registers 59 to 64) to be
 * written into the FIFO buffer.
 *
 * @return current accelerometer FIFO enabled value
 */
static bool MPU9250_getAccelFIFOEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                   MPU9250_FIFO_EN_ACCEL_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set accelerometer FIFO enabled value
 *
 * @param enabled: accelerometer FIFO enabled value
 *
 * @return void
 */
static void MPU9250_setAccelFIFOEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                    MPU9250_FIFO_EN_ACCEL_BIT, enabled);
}


/** Get slave 2 FIFO enabled value
 *
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with slave 2 to be written into the FIFO buffer.
 *
 * @return current slave 2 FIFO enabled value
 */
static bool MPU9250_getSlave2FIFOEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                   MPU9250_FIFO_EN_SLV_2_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set slave 2 FIFO enabled value
 *
 * @param enabled: slave 2 FIFO enabled value
 *
 * @return void
 */
static void MPU9250_setSlave2FIFOEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                    MPU9250_FIFO_EN_SLV_2_BIT, enabled);
}


/** Get slave 1 FIFO enabled value
 *
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with slave 1 to be written into the FIFO buffer.
 *
 * @return current slave 1 FIFO enabled value
 */
static bool MPU9250_getSlave1FIFOEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                   MPU9250_FIFO_EN_SLV_1_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set slave 1 FIFO enabled value
 *
 * @param enabled: slave 1 FIFO enabled value
 *
 * @return void
 */
static void MPU9250_setSlave1FIFOEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                    MPU9250_FIFO_EN_SLV_1_BIT, enabled);
}


/** Get slave 0 FIFO enabled value
 *
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with slave 0 to be written into the FIFO buffer.
 *
 * @return current slave 0 FIFO enabled value
 */
static bool MPU9250_getSlave0FIFOEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                   MPU9250_FIFO_EN_SLV_0_BIT, tmp_buf);
    return tmp_buf[0];
}

/** Set slave 0 FIFO enabled value
 *
 * @param enabled: slave 0 FIFO enabled value
 *
 * @return void
 */
static void MPU9250_setSlave0FIFOEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                    MPU9250_FIFO_EN_SLV_0_BIT, enabled);
}


/** Get multi-master enabled value
 *
 * Multi-master capability allows multiple I2C masters to operate on the same
 * bus. In circuits where multi-master capability is required, set MULT_MST_EN
 * to 1. This will increase current drawn by approximately 30uA.
 *
 * In circuits where multi-master capability is required, the state of the I2C
 * bus must always be monitored by each separate I2C Master. Before an I2C
 * Master can assume arbitration of the bus, it must first confirm that no other
 * I2C Master has arbitration of the bus. When MULT_MST_EN is set to 1, the
 * MPU-60X0's bus arbitration detection logic is turned on, enabling it to
 * detect when the bus is available.
 *
 * @return current multi-master enabled value
 */
static bool MPU9250_getMultiMasterEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_CTRL,
                   MPU9250_I2C_MST_CTRL_MULT_MST_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set multi-master enabled value
 *
 * @param enabled: multi-master enabled value
 *
 * @return void
 */
static void MPU9250_setMultiMasterEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_CTRL,
                    MPU9250_I2C_MST_CTRL_MULT_MST_EN_BIT, enabled);
}


/** Get wait-for-external-sensor-data enabled value
 *
 * When the WAIT_FOR_ES bit is set to 1, the Data Ready interrupt will be
 * delayed until External Sensor data from the Slave Devices are loaded into the
 * EXT_SENS_DATA registers. This is used to ensure that both the internal sensor
 * data (i.e. from gyro and accel) and external sensor data have been loaded to
 * their respective data registers (i.e. the data is synced) when the Data Ready
 * interrupt is triggered.
 *
 * @return current wait-for-external-sensor-data enabled value
 */
static bool MPU9250_getWaitForExternalSensorEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_CTRL,
                   MPU9250_I2C_MST_CTRL_WAIT_FOR_ES_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set wait-for-external-sensor-data enabled value
 *
 * @param enabled: wait-for-external-sensor-data enabled value
 *
 * @return void
 */
static void MPU9250_setWaitForExternalSensorEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_CTRL,
                    MPU9250_I2C_MST_CTRL_WAIT_FOR_ES_BIT, enabled);
}


/** Get slave 3 FIFO enabled value
 *
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with slave 3 to be written into the FIFO buffer.
 *
 * @return current slave 3 FIFO enabled value
 */
static bool MPU9250_getSlave3FIFOEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_CTRL,
                   MPU9250_I2C_MST_CTRL_SLV_3_FIFO_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set slave 3 FIFO enabled value
 *
 * @param enabled: slave 3 FIFO enabled value
 *
 * @return void
 */
static void MPU9250_setSlave3FIFOEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_CTRL,
                    MPU9250_I2C_MST_CTRL_SLV_3_FIFO_EN_BIT, enabled);
}


/** Get slave read/write transition enabled value
 *
 * The I2C_MST_P_NSR bit configures the I2C Master's transition from one slave
 * read to the next slave read. If the bit equals 0, there will be a restart
 * between reads. If the bit equals 1, there will be a stop followed by a start
 * of the following read. When a write transaction follows a read transaction,
 * the stop followed by a start of the successive write will be always used.
 *
 * @return current slave read/write transition enabled value
 */
static bool MPU9250_getSlaveReadWriteTransitionEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_CTRL,
                   MPU9250_I2C_MST_CTRL_I2C_MST_P_NSR_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set slave read/write transition enabled value
 *
 * @param enabled: slave read/write transition enabled value
 *
 * @return void
 */
static void MPU9250_setSlaveReadWriteTransitionEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_CTRL,
                    MPU9250_I2C_MST_CTRL_I2C_MST_P_NSR_BIT, enabled);
}


/** Get I2C master clock speed
 *
 * I2C_MST_CLK is a 4 bit unsigned value which configures a divider on the
 * MPU-9250 internal 8MHz clock. It sets the I2C master clock speed according to
 * the following table:
 *
 * <pre>
 * I2C_MST_CLK | I2C Master Clock Speed | 8MHz Clock Divider
 * ------------+------------------------+-------------------
 * 0           | 348kHz                 | 23
 * 1           | 333kHz                 | 24
 * 2           | 320kHz                 | 25
 * 3           | 308kHz                 | 26
 * 4           | 296kHz                 | 27
 * 5           | 286kHz                 | 28
 * 6           | 276kHz                 | 29
 * 7           | 267kHz                 | 30
 * 8           | 258kHz                 | 31
 * 9           | 500kHz                 | 16
 * 10          | 471kHz                 | 17
 * 11          | 444kHz                 | 18
 * 12          | 421kHz                 | 19
 * 13          | 400kHz                 | 20
 * 14          | 381kHz                 | 21
 * 15          | 364kHz                 | 22
 * </pre>
 *
 * @return current I2C master clock speed
 */
static uint8_t MPU9250_getMasterClockSpeed(void)
{
    I2C_readMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_CTRL,
                      MPU9250_I2C_MST_CTRL_I2C_MST_CLK_BIT, MPU9250_I2C_MST_CTRL_I2C_MST_CLK_LEN,
                      tmp_buf);
    return tmp_buf[0];
}


/** Set I2C master clock speed
 *
 * @param speed: current I2C master clock speed
 *
 * @return void
 */
static void MPU9250_setMasterClockSpeed(uint8_t speed)
{
    I2C_writeMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_CTRL,
                       MPU9250_I2C_MST_CTRL_I2C_MST_CLK_BIT, MPU9250_I2C_MST_CTRL_I2C_MST_CLK_LEN,
                       speed);
}


/** Get the I2C address of the specified slave(0-3)
 *
 * Note that Bit 7(MSB) controls read/write mode. If Bit 7 is set, it's a read
 * operation, and if it is cleared, then it's a write operation. The remaining
 * bits (6-0) are the 7-bit device address of the slave device.
 *
 * In read mode, the result of the read is placed in the lowest available
 * EXT_SENS_DATA register. For further information regarding the allocation of
 * read results, please refer to the EXT_SENS_DATA register description(Registers 73 - 96).
 *
 * The MPU-9250 supports a total of five slaves, but slave 4 has unique
 * characteristics, and so it has its own functions (getSlave4* and setSlave4*).
 *
 * I2C data transactions are performed at the Sample Rate, as defined in
 * Register 25. The user is responsible for ensuring that I2C data transactions
 * to and from each enabled slave can be completed within a single period of the
 * Sample Rate.
 *
 * The I2C slave access rate can be reduced relative to the Sample Rate. This
 * reduced access rate is determined by I2C_MST_DLY (Register 52). Whether a
 * slave's access rate is reduced relative to the Sample Rate is determined by
 * I2C_MST_DELAY_CTRL(Register 103).
 *
 * The processing order for the slaves is fixed. The sequence followed for
 * processing the slaves is slave 0, slave 1, slave 2, slave 3 and slave 4. If a
 * particular Slave is disabled it will be skipped.
 *
 * Each slave can either be accessed at the sample rate or at a reduced sample
 * rate. In a case where some slaves are accessed at the Sample Rate and some
 * slaves are accessed at the reduced rate, the sequence of accessing the slaves
 * (slave 0 to slave 4) is still followed. However, the reduced rate slaves will
 * be skipped if their access rate dictates that they should not be accessed
 * during that particular cycle. For further information regarding the reduced
 * access rate, please refer to Register 52. Whether a slave is accessed at the
 * Sample Rate or at the reduced rate is determined by the Delay Enable bits in
 * Register 103.
 *
 * @param num: slave number(0-3)
 *
 * @return current address for specified slave
 */
static uint8_t MPU9250_getSlaveAddress(uint8_t num)
{
    if(num > 3) return 0;
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV0_ADDR + num * 3,
                    tmp_buf);
    return tmp_buf[0];
}


/** Set the I2C address of the specified slave(0-3)
 *
 * @param num : slave number(0-3)
 * @param addr: address for specified slave
 *
 * @return void
 */
static void MPU9250_setSlaveAddress(uint8_t num, uint8_t addr)
{
    if(num > 3) return;
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV0_ADDR + num * 3,
                     addr);
}


/** Get the active internal register for the specified slave(0-3)
 *
 * Read/write operations for this slave will be done to whatever internal
 * register address is stored in this MPU register.
 *
 * The MPU-9250 supports a total of five slaves, but Slave 4 has unique
 * characteristics, and so it has its own functions.
 *
 * @param num: slave number(0-3)
 *
 * @return current active register for specified slave
 */
static uint8_t MPU9250_getSlaveRegister(uint8_t num)
{
    if(num > 3) return 0;
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV0_REG + num * 3,
                    tmp_buf);
    return tmp_buf[0];
}


/** Set the active internal register for the specified slave(0-3)
 *
 * @param num: slave number(0-3)
 * @param reg: active register for specified slave
 *
 * @return void
 */
static void MPU9250_setSlaveRegister(uint8_t num, uint8_t reg)
{
    if(num > 3) return;
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV0_REG + num * 3,
                     reg);
}


/** Get the enabled value for the specified slave(0-3)
 *
 * When set to 1, this bit enables slave 0 for data transfer operations. When
 * cleared to 0, this bit disables slave 0 from data transfer operations.
 *
 * @param num: slave number(0-3)
 *
 * @return current enabled value for specified slave
 */
static bool MPU9250_getSlaveEnabled(uint8_t num)
{
    if(num > 3) return 0;
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV0_CTRL + num * 3,
                   MPU9250_I2C_SLV_CTRL_I2C_SLV_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set the enabled value for the specified slave(0-3)
 *
 * @param num    : slave number(0-3)
 * @param enabled: enabled value for specified slave
 *
 * @return void
 */
static void MPU9250_setSlaveEnabled(uint8_t num, bool enabled)
{
    if(num > 3) return;
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV0_CTRL + num * 3,
                    MPU9250_I2C_SLV_CTRL_I2C_SLV_EN_BIT, enabled);
}


/** Get word pair byte-swapping enabled for the specified slave(0-3)
 *
 * When set to 1, this bit enables byte swapping. When byte swapping is enabled,
 * the high and low bytes of a word pair are swapped. Please refer to
 * I2C_SLV0_GRP for the pairing convention of the word pairs. When cleared to 0,
 * bytes transferred to and from Slave 0 will be written to EXT_SENS_DATA
 * registers in the order they were transferred.
 *
 * @param num: slave number(0-3)
 *
 * @return current word pair byte-swapping enabled value for specified slave
 */
static bool MPU9250_getSlaveWordByteSwap(uint8_t num)
{
    if(num > 3) return 0;
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV0_CTRL + num * 3,
                   MPU9250_I2C_SLV_CTRL_I2C_SLV_BYTE_SW_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set word pair byte-swapping enabled for the specified slave(0-3)
 *
 * @param num    : slave number(0-3)
 * @param enabled: word pair byte-swapping enabled value for specified slave
 *
 * @return void
 */
static void MPU9250_setSlaveWordByteSwap(uint8_t num, bool enabled)
{
    if(num > 3) return;
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV0_CTRL + num * 3,
                    MPU9250_I2C_SLV_CTRL_I2C_SLV_BYTE_SW_BIT, enabled);
}

/** Get write mode for the specified slave(0-3)
 *
 * When set to 1, the transaction will read or write data only. When cleared to
 * 0, the transaction will write a register address prior to reading or writing
 * data. This should equal 0 when specifying the register address within the
 * slave device to/from which the ensuing data transaction will take place.
 *
 * @param num: slave number(0-3)
 *
 * @return current write mode for specified slave (0 = register address + data, 1 = data only)
 */
static bool MPU9250_getSlaveWriteMode(uint8_t num)
{
    if(num > 3) return 0;
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV0_CTRL + num * 3,
                   MPU9250_I2C_SLV_CTRL_I2C_SLV_REG_DIS_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set write mode for the specified slave(0-3)
 *
 * @param num : slave number(0-3)
 * @param mode: write mode for specified slave (0 = register address + data, 1 = data only)
 *
 * @return void
 */
static void MPU9250_setSlaveWriteMode(uint8_t num, bool mode)
{
    if(num > 3) return;
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV0_CTRL + num * 3,
                    MPU9250_I2C_SLV_CTRL_I2C_SLV_REG_DIS_BIT, mode);
}


/** Get word pair grouping order offset for the specified slave(0-3)
 *
 * This sets specifies the grouping order of word pairs received from registers.
 * When cleared to 0, bytes from register addresses 0 and 1, 2 and 3, etc (even,
 * then odd register addresses) are paired to form a word. When set to 1, bytes
 * from register addresses are paired 1 and 2, 3 and 4, etc. (odd, then even
 * register addresses) are paired to form a word.
 *
 * @param num: slave number(0-3)
 *
 * @return current word pair grouping order offset for specified slave
 */
static bool MPU9250_getSlaveWordGroupOffset(uint8_t num)
{
    if(num > 3) return 0;
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV0_CTRL + num * 3,
                   MPU9250_I2C_SLV_CTRL_I2C_SLV_GRP_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set word pair grouping order offset for the specified slave(0-3)
 *
 * @param num    : slave number(0-3)
 * @param enabled: word pair grouping order offset for specified slave
 *
 * @return void
 */
static void MPU9250_setSlaveWordGroupOffset(uint8_t num, bool enabled)
{
    if(num > 3) return;
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV0_CTRL + num * 3,
                    MPU9250_I2C_SLV_CTRL_I2C_SLV_GRP_BIT, enabled);
}



/** Get number of bytes to read for the specified slave(0-3)
 *
 * Specifies the number of bytes transferred to and from slave 0. Clearing this
 * bit to 0 is equivalent to disabling the register by writing 0 to I2C_SLV0_EN
 *
 * @param num: slave number(0-3)
 *
 * @return number of bytes to read for specified slave
 */
static uint8_t MPU9250_getSlaveDataLength(uint8_t num)
{
    if(num > 3) return 0;
    I2C_readMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV0_CTRL + num * 3,
                      MPU9250_I2C_SLV_CTRL_I2C_SLV_LENG_BIT, MPU9250_I2C_SLV_CTRL_I2C_SLV_LENG_LEN,
                      tmp_buf);
    return tmp_buf[0];
}


/** Set number of bytes to read for the specified slave(0-3)
 *
 * @param num: slave number(0-3)
 * @param len: number of bytes to read for specified slave
 *
 * @return void
 */
static void MPU9250_setSlaveDataLength(uint8_t num, uint8_t len)
{
    if(num > 3) return;
    I2C_writeMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV0_CTRL + num * 3,
                       MPU9250_I2C_SLV_CTRL_I2C_SLV_LENG_BIT, MPU9250_I2C_SLV_CTRL_I2C_SLV_LENG_LEN,
                       len);
}


/** Get the I2C address of slave 4
 *
 * Note that Bit 7 (MSB) controls read/write mode. If Bit 7 is set, it's a read
 * operation, and if it is cleared, then it's a write operation. The remaining
 * bits (6-0) are the 7-bit device address of the slave device.
 *
 * @return current address for slave 4
 */
static uint8_t MPU9250_getSlave4Address(void)
{
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV4_ADDR,
                    tmp_buf);
    return tmp_buf[0];
}


/** Set the I2C address of slave 4
 *
 * @param addr: address for slave 4
 *
 * @return void
 */
static void MPU9250_setSlave4Address(uint8_t addr)
{
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV4_ADDR,
                     addr);
}


/** Get the active internal register for the slave 4
 *
 * Read/write operations for this slave will be done to whatever internal
 * register address is stored in this MPU register.
 *
 * @return current active register for slave 4
 */
static uint8_t MPU9250_getSlave4Register(void)
{
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV4_REG,
                    tmp_buf);
    return tmp_buf[0];
}


/** Set the active internal register for slave 4
 *
 * @param reg: active register for slave 4
 *
 * @return void
 */
static void MPU9250_setSlave4Register(uint8_t reg)
{
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV4_REG,
                     reg);
}


/** Set new byte to write to slave 4
 *
 * This register stores the data to be written into the slave 4. If I2C_SLV4_RW
 * is set 1 (set to read), this register has no effect.
 *
 * @param data: byte to write to slave 4
 *
 * @return void
 */
static void MPU9250_setSlave4OutputByte(uint8_t data)
{
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV4_DO,
                     data);
}


/** Get the enabled value for the slave 4
 *
 * When set to 1, this bit enables slave 4 for data transfer operations. When
 * cleared to 0, this bit disables slave 4 from data transfer operations.
 *
 * @return current enabled value for slave 4
 */
static bool MPU9250_getSlave4Enabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV4_CTRL,
                   MPU9250_I2C_SLV4_CTRL_I2C_SLV4_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set the enabled value for slave 4
 *
 * @param enabled: enabled value for slave 4
 *
 * @return void
 */
static void MPU9250_setSlave4Enabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV4_CTRL,
                    MPU9250_I2C_SLV4_CTRL_I2C_SLV4_EN_BIT, enabled);
}


/** Get the enabled value for slave 4 transaction interrupts
 *
 * When set to 1, this bit enables the generation of an interrupt signal upon
 * completion of a slave 4 transaction. When cleared to 0, this bit disables the
 * generation of an interrupt signal upon completion of a slave 4 transaction.
 * The interrupt status can be observed in Register 54.
 *
 * @return current enabled value for slave 4 transaction interrupts
 */
static bool MPU9250_getSlave4InterruptEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV4_CTRL,
                   MPU9250_I2C_SLV4_CTRL_I2C_SLV4_DONE_INT_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set the enabled value for Slave 4 transaction interrupts
 *
 * @param enabled: enabled value for Slave 4 transaction interrupts
 *
 * @return void
 */
static void MPU9250_setSlave4InterruptEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV4_CTRL,
                    MPU9250_I2C_SLV4_CTRL_I2C_SLV4_DONE_INT_EN_BIT, enabled);
}


/** Get write mode for slave 4
 *
 * When set to 1, the transaction will read or write data only. When cleared to
 * 0, the transaction will write a register address prior to reading or writing
 * data. This should equal 0 when specifying the register address within the
 * slave device to/from which the ensuing data transaction will take place.
 *
 * @return current write mode for slave 4 (0 = register address + data, 1 = data only)
 */
static bool MPU9250_getSlave4WriteMode(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV4_CTRL,
                   MPU9250_I2C_SLV4_CTRL_I2C_SLV4_REG_DIS_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set write mode for the slave 4
 *
 * @param mode: write mode for slave 4(0 = register address + data, 1 = data only)
 *
 * @return void
 */
static void MPU9250_setSlave4WriteMode(bool mode)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV4_CTRL,
                    MPU9250_I2C_SLV4_CTRL_I2C_SLV4_REG_DIS_BIT, mode);
}


/** Get slave 4 master delay value
 *
 * This configures the reduced access rate of I2C slaves relative to the Sample
 * Rate. When a slave's access rate is decreased relative to the Sample Rate,
 * the slave is accessed every:
 *
 *     1 / (1 + I2C_MST_DLY) samples
 *
 * This base Sample Rate in turn is determined by SMPLRT_DIV (register 25) and
 * DLPF_CFG (register 26). Whether a slave's access rate is reduced relative to
 * the Sample Rate is determined by I2C_MST_DELAY_CTRL (register 103). For
 * further information regarding the Sample Rate, please refer to register 25.
 *
 * @return current slave 4 master delay value
 */
static uint8_t MPU9250_getSlave4MasterDelay(void)
{
    I2C_readMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV4_CTRL,
                      MPU9250_I2C_SLV4_CTRL_I2C_SLV4_MST_DLY_BIT, MPU9250_I2C_SLV4_CTRL_I2C_SLV4_MST_DLY_LEN,
                      tmp_buf);
    return tmp_buf[0];
}


/** Set slave 4 master delay value
 *
 * @param delay: slave 4 master delay value
 *
 * @return void
 */
static void MPU9250_setSlave4MasterDelay(uint8_t delay)
{
    I2C_writeMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV4_CTRL,
                       MPU9250_I2C_SLV4_CTRL_I2C_SLV4_MST_DLY_BIT, MPU9250_I2C_SLV4_CTRL_I2C_SLV4_MST_DLY_LEN,
                       delay);    
}


/** Get last available byte read from slave 4
 *
 * This register stores the data read from slave 4. This field is populated
 * after a read transaction.
 *
 * @return last available byte read from to slave 4
 */
static uint8_t MPU9250_getSlate4InputByte(void)
{
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV4_DI,
                    tmp_buf);
    return tmp_buf[0];    
}

/** Get FSYNC interrupt status
 *
 * This bit reflects the status of the FSYNC interrupt from an external device
 * into the MPU-60X0. This is used as a way to pass an external interrupt
 * through the MPU-60X0 to the host application processor. When set to 1, this
 * bit will cause an interrupt if FSYNC_INT_EN is asserted in INT_PIN_CFG
 * (Register 55).
 *
 * @return FSYNC interrupt status
 */
static bool MPU9250_getPassthroughStatus(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_STATUS,
                   MPU9250_I2C_MST_STATUS_PASS_THROUGH_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Get slave 4 transaction done status
 *
 * Automatically sets to 1 when a slave 4 transaction has completed. This
 * triggers an interrupt if the I2C_MST_INT_EN bit in the INT_ENABLE register
 * (Register 56) is asserted and if the SLV_4_DONE_INT bit is asserted in the
 * I2C_SLV4_CTRL register (Register 52).
 *
 * @return slave 4 transaction done status
 */
static bool MPU9250_getSlave4IsDone(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_STATUS,
                   MPU9250_I2C_MST_STATUS_I2C_SLV4_DONE_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Get master arbitration lost status
 *
 * This bit automatically sets to 1 when the I2C Master has lost arbitration of
 * the auxiliary I2C bus (an error condition). This triggers an interrupt if the
 * I2C_MST_INT_EN bit in the INT_ENABLE register (Register 56) is asserted.
 *
 * @return master arbitration lost status
 */
static bool MPU9250_getLostArbitration(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_STATUS,
                   MPU9250_I2C_MST_STATUS_I2C_LOST_ARB_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Get slave 4 NACK status
 *
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with slave 4. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 *
 * @return slave 4 NACK interrupt status
 */
static bool MPU9250_getSlave4Nack(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_STATUS,
                   MPU9250_I2C_MST_STATUS_I2C_SLV4_NACK_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Get slave 3 NACK status
 *
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with slave 3. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 *
 * @return slave 3 NACK interrupt status
 */
static bool MPU9250_getSlave3Nack(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_STATUS,
                   MPU9250_I2C_MST_STATUS_I2C_SLV3_NACK_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Get slave 2 NACK status
 *
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with slave 2. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 *
 * @return slave 2 NACK interrupt status
 */
static bool MPU9250_getSlave2Nack(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_STATUS,
                   MPU9250_I2C_MST_STATUS_I2C_SLV2_NACK_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Get slave 1 NACK status
 *
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with slave 1. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 *
 * @return slave 1 NACK interrupt status
 */
static bool MPU9250_getSlave1Nack(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_STATUS,
                   MPU9250_I2C_MST_STATUS_I2C_SLV1_NACK_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Get slave 0 NACK status
 *
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with slave 0. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 *
 * @return slave 0 NACK interrupt status
 */
static bool MPU9250_getSlave0Nack(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_STATUS,
                   MPU9250_I2C_MST_STATUS_I2C_SLV0_NACK_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Get interrupt logic level mode
 *
 * Will be set 0 for active-high, 1 for active-low
 *
 * @return current interrupt mode(0=active-high, 1=active-low)
 */
static bool MPU9250_getInterruptMode(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_PIN_CFG,
                   MPU9250_INT_PIN_CFG_ACTL_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set interrupt logic level mode
 *
 * @param mode: interrupt mode(0=active-high, 1=active-low)
 *
 * @return void
 */
static void MPU9250_setInterruptMode(bool mode)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_PIN_CFG,
                    MPU9250_INT_PIN_CFG_ACTL_BIT, mode);
}


/**  Get interrupt drive mode
 *
 * Will be set 0 for push-pull, 1 for open-drain
 *
 * @return current interrupt drive mode(0=push-pull, 1=open-drain)
 */
static bool MPU9250_getInterruptDrive(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_PIN_CFG,
                   MPU9250_INT_PIN_CFG_OPEN_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set interrupt drive mode
 *
 * @param drive: interrupt drive mode(0=push-pull, 1=open-drain)
 *
 * @return void
 */
static void MPU9250_setInterruptDrive(bool drive)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_PIN_CFG,
                    MPU9250_INT_PIN_CFG_OPEN_BIT, drive);
}


/** Get interrupt latch mode
 *
 * Will be set 0 for 50us-pulse, 1 for latch-until-int-cleared
 *
 * @return current latch mode(0=50us-pulse, 1=latch-until-int-cleared)
 */
static bool MPU9250_getInterruptLatch(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_PIN_CFG,
                   MPU9250_INT_PIN_CFG_LATCH_INT_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set interrupt latch mode
 *
 * @param latch: latch mode(0=50us-pulse, 1=latch-until-int-cleared)
 *
 * @return void
 */
static void MPU9250_setInterruptLatch(bool latch)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_PIN_CFG,
                    MPU9250_INT_PIN_CFG_LATCH_INT_EN_BIT, latch);
}


/** Get interrupt latch clear mode
 *
 * Will be set 0 for status-read-only, 1 for any-register-read
 *
 * @return current latch clear mode(0=status-read-only, 1=any-register-read)
 */
static bool MPU9250_getInterruptLatchClear(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_PIN_CFG,
                   MPU9250_INT_PIN_CFG_INT_ANYRD_2CLEAR_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set interrupt latch clear mode
 *
 * @param clear: latch clear mode(0=status-read-only, 1=any-register-read)
 *
 * @return void
 */
static void MPU9250_setInterruptLatchClear(bool clear)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_PIN_CFG,
                    MPU9250_INT_PIN_CFG_INT_ANYRD_2CLEAR_BIT, clear);
}


/** Get FSYNC interrupt logic level mode
 *
 * @return current FSYNC interrupt mode(0=active-high, 1=active-low)
 */
static bool MPU9250_getFSyncInterruptLevel(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_PIN_CFG,
                   MPU9250_INT_PIN_CFG_ACTL_FSYNC_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set FSYNC interrupt logic level mode
 *
 * @param mode: FSYNC interrupt mode(0=active-high, 1=active-low)
 *
 * @return void
 */
static void MPU9250_setFSyncInterruptLevel(bool level)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_PIN_CFG,
                    MPU9250_INT_PIN_CFG_ACTL_FSYNC_BIT, level);
}


/** Get FSYNC pin interrupt enabled setting
 *
 * Will be set 0 for disabled, 1 for enabled
 *
 * @return current interrupt enabled setting
 */
static bool MPU9250_getFSyncInterruptEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_PIN_CFG,
                   MPU9250_INT_PIN_CFG_FSYNC_INT_MODE_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set FSYNC pin interrupt enabled setting
 *
 * @param enabled: FSYNC pin interrupt enabled setting
 *
 * @return void
 */    
static void MPU9250_setFSyncInterruptEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_PIN_CFG,
                    MPU9250_INT_PIN_CFG_FSYNC_INT_MODE_EN_BIT, enabled);
}


/** Get I2C bypass enabled status
 *
 * When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
 * 0, the host application processor will be able to directly access the
 * auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
 * application processor will not be able to directly access the auxiliary I2C
 * bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
 * bit[5]).
 *
 * @return current I2C bypass enabled status
 */
static bool MPU9250_getI2CBypassEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_PIN_CFG,
                   MPU9250_INT_PIN_CFG_BYPASS_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set I2C bypass enabled status
 *
 * When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
 * 0, the host application processor will be able to directly access the
 * auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
 * application processor will not be able to directly access the auxiliary I2C
 * bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
 * bit[5]).
 *
 * @param enabled: I2C bypass enabled status
 *
 * @return void
 */
static void MPU9250_setI2CBypassEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_PIN_CFG,
                    MPU9250_INT_PIN_CFG_BYPASS_EN_BIT, enabled);
}


/** Get full interrupt enabled status
 *
 * Full register byte for all interrupts, for quick reading. Each bit will be
 * set 0 for disabled, 1 for enabled.
 *
 * @return current interrupt enabled status
 **/
static uint8_t MPU9250_getIntEnabled(void)
{
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_ENABLE,
                    tmp_buf);
    return tmp_buf[0];
}


/** Set full interrupt enabled status
 *
 * Full register byte for all interrupts, for quick reading. Each bit should be
 * set 0 for disabled, 1 for enabled.
 *
 * @param enabled: interrupt enabled status
 *
 * @return void
 **/
static void MPU9250_setIntEnabled(uint8_t enabled)
{
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_ENABLE,
                     enabled);
}


/** Get Motion Detection interrupt enabled status
 *
 * Will be set 0 for disabled, 1 for enabled
 *
 * @return current interrupt enabled status
 **/
static bool MPU9250_getIntMotionEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_ENABLE,
                   MPU9250_INT_ENABLE_WOM_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set Motion Detection interrupt enabled status
 *
 * @param enabled: interrupt enabled status
 *
 * @return void
 **/
static void MPU9250_setIntMotionEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_ENABLE,
                    MPU9250_INT_ENABLE_WOM_EN_BIT, enabled);
}


/** Get FIFO Buffer Overflow interrupt enabled status
 *
 * Will be set 0 for disabled, 1 for enabled
 *
 * @return current interrupt enabled status
 **/
static bool MPU9250_getIntFIFOBufferOverflowEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_ENABLE,
                   MPU9250_INT_ENABLE_FIFO_OVERFLOW_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set FIFO Buffer Overflow interrupt enabled status
 *
 * @param enabled: interrupt enabled status
 *
 * @return void
 **/
static void MPU9250_setIntFIFOBufferOverflowEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_ENABLE,
                    MPU9250_INT_ENABLE_FIFO_OVERFLOW_EN_BIT, enabled);
}


/** Get Data Ready interrupt enabled setting
 *
 * This event occurs each time a write operation to all of the sensor registers
 * has been completed. Will be set 0 for disabled, 1 for enabled.
 *
 * @return current interrupt enabled status
 */
static bool MPU9250_getIntDataReadyEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_ENABLE,
                   MPU9250_INT_ENABLE_RAW_RDY_EN_BIT, tmp_buf);
    return tmp_buf[0];
}

/** Set Data Ready interrupt enabled status
 *
 * @param enabled: interrupt enabled status
 *
 * @return void
 */
static void MPU9250_setIntDataReadyEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_ENABLE,
                    MPU9250_INT_ENABLE_RAW_RDY_EN_BIT, enabled);
}


/** Get full set of interrupt status bits
 *
 * These bits clear to 0 after the register has been read. Very useful
 * for getting multiple INT statuses, since each single bit read clears
 * all of them because it has to read the whole byte.
 *
 * @return current interrupt status
 */
static uint8_t MPU9250_getIntStatus(void)
{
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_STATUS,
                    tmp_buf);
    return tmp_buf[0];
}


/** Get Motion Detection interrupt status
 *
 * This bit automatically sets to 1 when a Motion Detection interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 *
 * @return current interrupt status
 */
static bool MPU9250_getIntMotionStatus(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_STATUS,
                   MPU9250_INT_STATUS_WOM_INT_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Get FIFO Buffer Overflow interrupt status
 *
 * This bit automatically sets to 1 when a Free Fall interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 *
 * @return current interrupt status
 */
static bool MPU9250_getIntFIFOBufferOverflowStatus(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_STATUS,
                   MPU9250_INT_STATUS_FIFO_OVERFLOW_INT_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Get Data Ready interrupt status
 *
 * This bit automatically sets to 1 when a Data Ready interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 *
 * @return current interrupt status
 */
static bool MPU9250_getIntDataReadyStatus(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_INT_STATUS,
                   MPU9250_INT_STATUS_RAW_RDY_INT_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Get raw 6-axis motion sensor readings (accel/gyro)
 *
 * Retrieves all currently available motion sensor values
 *
 * @param ax: 16-bit signed integer container for accelerometer X-axis value
 * @param ay: 16-bit signed integer container for accelerometer Y-axis value
 * @param az: 16-bit signed integer container for accelerometer Z-axis value
 * @param gx: 16-bit signed integer container for gyroscope X-axis value
 * @param gy: 16-bit signed integer container for gyroscope Y-axis value
 * @param gz: 16-bit signed integer container for gyroscope Z-axis value
 *
 * @return void
 */
static void MPU9250_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
    I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_XOUT_H,
                       14, tmp_buf);

    *ax = (((int16_t)tmp_buf[0])  << 8) | tmp_buf[1];
    *ay = (((int16_t)tmp_buf[2])  << 8) | tmp_buf[3];
    *az = (((int16_t)tmp_buf[4])  << 8) | tmp_buf[5];
    *gx = (((int16_t)tmp_buf[8])  << 8) | tmp_buf[9];
    *gy = (((int16_t)tmp_buf[10]) << 8) | tmp_buf[11];
    *gz = (((int16_t)tmp_buf[12]) << 8) | tmp_buf[13];
}


/** Get 3-axis accelerometer readings
 *
 * These registers store the most recent accelerometer measurements.
 * Accelerometer measurements are written to these registers at the Sample Rate
 * as defined in Register 25.
 *
 * The accelerometer measurement registers, along with the temperature
 * measurement registers, gyroscope measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 *
 * The data within the accelerometer sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
 * (Register 28). For each full scale setting, the accelerometers' sensitivity
 * per LSB in ACCEL_xOUT is shown in the table below:
 *
 * <pre>
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg
 * </pre>
 *
 * @param x: 16-bit signed integer container for X-axis acceleration
 * @param y: 16-bit signed integer container for Y-axis acceleration
 * @param z: 16-bit signed integer container for Z-axis acceleration
 *
 * @return void
 */
static void MPU9250_getAcceleration(int16_t* x, int16_t* y, int16_t* z)
{
    I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_XOUT_H,
                       6, tmp_buf);

    *x = (((int16_t)tmp_buf[0]) << 8) | tmp_buf[1];
    *y = (((int16_t)tmp_buf[2]) << 8) | tmp_buf[3];
    *z = (((int16_t)tmp_buf[4]) << 8) | tmp_buf[5];
}


/** Get X-axis accelerometer reading
 *
 * @return X-axis acceleration measurement in 16-bit 2's complement format
 */
static int16_t MPU9250_getAccelerationX(void)
{
    I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_XOUT_H,
                       2, tmp_buf);
    return (((int16_t)tmp_buf[0]) << 8) | tmp_buf[1];
}


/** Get Y-axis accelerometer reading
 *
 * @return Y-axis acceleration measurement in 16-bit 2's complement format
 */
static int16_t MPU9250_getAccelerationY(void)
{
    I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_YOUT_H,
                       2, tmp_buf);
    return (((int16_t)tmp_buf[0]) << 8) | tmp_buf[1];
}


/** Get Z-axis accelerometer reading
 *
 * @return Z-axis acceleration measurement in 16-bit 2's complement format
 */
static int16_t MPU9250_getAccelerationZ(void)
{
    I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_ZOUT_H,
                       2, tmp_buf);
    return (((int16_t)tmp_buf[0]) << 8) | tmp_buf[1];
}

/** Get current internal temperature
 *
 * @return Temperature reading in 16-bit 2's complement format
 */
static int16_t MPU9250_getTemperature(void)
{
    I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_TEMP_OUT_H,
                       2, tmp_buf);
    return (((int16_t)tmp_buf[0]) << 8) | tmp_buf[1];
}

/** Get 3-axis gyroscope readings
 *
 * These gyroscope measurement registers, along with the accelerometer
 * measurement registers, temperature measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 * The data within the gyroscope sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
 * (Register 27). For each full scale setting, the gyroscopes' sensitivity per
 * LSB in GYRO_xOUT is shown in the table below:
 *
 * <pre>
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 * </pre>
 *
 * @param x: 16-bit signed integer container for X-axis rotation
 * @param y: 16-bit signed integer container for Y-axis rotation
 * @param z: 16-bit signed integer container for Z-axis rotation
 *
 * @return void
 */
static void MPU9250_getRotation(int16_t* x, int16_t* y, int16_t* z)
{
    I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_XOUT_H,
                       6, tmp_buf);

    *x = (((int16_t)tmp_buf[0]) << 8) | tmp_buf[1];
    *y = (((int16_t)tmp_buf[2]) << 8) | tmp_buf[3];
    *z = (((int16_t)tmp_buf[4]) << 8) | tmp_buf[5];
}


/** Get X-axis gyroscope reading
 *
 * @return X-axis rotation measurement in 16-bit 2's complement format
 */
static int16_t MPU9250_getRotationX(void)
{
    I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_XOUT_H,
                       2, tmp_buf);
    return (((int16_t)tmp_buf[0]) << 8) | tmp_buf[1];
}

/** Get Y-axis gyroscope reading
 *
 * @return Y-axis rotation measurement in 16-bit 2's complement format
 */
static int16_t MPU9250_getRotationY(void)
{
    I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_YOUT_H,
                       2, tmp_buf);
    return (((int16_t)tmp_buf[0]) << 8) | tmp_buf[1];
}


/** Get Z-axis gyroscope reading
 *
 * @return Z-axis rotation measurement in 16-bit 2's complement format
 */
static int16_t MPU9250_getRotationZ(void)
{
    I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_ZOUT_H,
                       2, tmp_buf);
    return (((int16_t)tmp_buf[0]) << 8) | tmp_buf[1];
}


/** Read single byte from external sensor data register
 *
 * These registers store data read from external sensors by the Slave 0, 1, 2,
 * and 3 on the auxiliary I2C interface. Data read by Slave 4 is stored in
 * I2C_SLV4_DI (Register 53).
 *
 * External sensor data is written to these registers at the Sample Rate as
 * defined in Register 25. This access rate can be reduced by using the Slave
 * Delay Enable registers (Register 103).
 *
 * External sensor data registers, along with the gyroscope measurement
 * registers, accelerometer measurement registers, and temperature measurement
 * registers, are composed of two sets of registers: an internal register set
 * and a user-facing read register set.
 *
 * The data within the external sensors' internal register set is always updated
 * at the Sample Rate (or the reduced access rate) whenever the serial interface
 * is idle. This guarantees that a burst read of sensor registers will read
 * measurements from the same sampling instant. Note that if burst reads are not
 * used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Data is placed in these external sensor data registers according to
 * I2C_SLV0_CTRL, I2C_SLV1_CTRL, I2C_SLV2_CTRL, and I2C_SLV3_CTRL (Registers 39,
 * 42, 45, and 48). When more than zero bytes are read (I2C_SLVx_LEN > 0) from
 * an enabled slave (I2C_SLVx_EN = 1), the slave is read at the Sample Rate (as
 * defined in Register 25) or delayed rate (if specified in Register 52 and
 * 103). During each Sample cycle, slave reads are performed in order of Slave
 * number. If all slaves are enabled with more than zero bytes to be read, the
 * order will be Slave 0, followed by Slave 1, Slave 2, and Slave 3.
 *
 * Each enabled slave will have EXT_SENS_DATA registers associated with it by
 * number of bytes read (I2C_SLVx_LEN) in order of slave number, starting from
 * EXT_SENS_DATA_00. Note that this means enabling or disabling a slave may
 * change the higher numbered slaves' associated registers. Furthermore, if
 * fewer total bytes are being read from the external sensors as a result of
 * such a change, then the data remaining in the registers which no longer have
 * an associated slave device (i.e. high numbered registers) will remain in
 * these previously allocated registers unless reset.
 *
 * If the sum of the read lengths of all SLVx transactions exceed the number of
 * available EXT_SENS_DATA registers, the excess bytes will be dropped. There
 * are 24 EXT_SENS_DATA registers and hence the total read lengths between all
 * the slaves cannot be greater than 24 or some bytes will be lost.
 *
 * Note: Slave 4's behavior is distinct from that of Slaves 0-3. For further
 * information regarding the characteristics of Slave 4, please refer to
 * Registers 49 to 53.
 *
 * EXAMPLE:
 * Suppose that Slave 0 is enabled with 4 bytes to be read (I2C_SLV0_EN = 1 and
 * I2C_SLV0_LEN = 4) while Slave 1 is enabled with 2 bytes to be read so that
 * I2C_SLV1_EN = 1 and I2C_SLV1_LEN = 2. In such a situation, EXT_SENS_DATA _00
 * through _03 will be associated with Slave 0, while EXT_SENS_DATA _04 and 05
 * will be associated with Slave 1. If Slave 2 is enabled as well, registers
 * starting from EXT_SENS_DATA_06 will be allocated to Slave 2.
 *
 * If Slave 2 is disabled while Slave 3 is enabled in this same situation, then
 * registers starting from EXT_SENS_DATA_06 will be allocated to Slave 3
 * instead.
 *
 * REGISTER ALLOCATION FOR DYNAMIC DISABLE VS. NORMAL DISABLE:
 * If a slave is disabled at any time, the space initially allocated to the
 * slave in the EXT_SENS_DATA register, will remain associated with that slave.
 * This is to avoid dynamic adjustment of the register allocation.
 *
 * The allocation of the EXT_SENS_DATA registers is recomputed only when (1) all
 * slaves are disabled, or (2) the I2C_MST_RST bit is set (Register 106).
 *
 * This above is also true if one of the slaves gets NACKed and stops
 * functioning.
 *
 * @param pos: starting position(0-23)
 *
 * @return byte read from register
 */
static uint8_t MPU9250_getExternalSensorByte(int pos)
{
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_EXT_SENS_DATA_00 + pos,
                    tmp_buf);
    return tmp_buf[0];
}


/** Read word(2 bytes) from external sensor data registers
 *
 * @param pos: starting position(0-21)
 *
 * @return word read from register
 */
static uint16_t MPU9250_getExternalSensorWord(int pos)
{
    I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_EXT_SENS_DATA_00 + pos,
                       2, tmp_buf);
    return (((uint16_t) tmp_buf[0]) << 8) | tmp_buf[1];
}

/** Read double word(4 bytes) from external sensor data registers
 *
 * @param pos: Starting position(0-20)
 *
 * @return double word read from registers
 */
static uint32_t MPU9250_getExternalSensorDWord(int pos)
{
    I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_EXT_SENS_DATA_00 + pos,
                       4, tmp_buf);
    return (((uint32_t) tmp_buf[0]) << 24) | (((uint32_t) tmp_buf[1]) << 16)
         | (((uint32_t) tmp_buf[2]) << 8)  | tmp_buf[3];
}


/** Write byte to Data Output container for specified slave
 *
 * This register holds the output data written into Slave when Slave is set to
 * write mode. For further information regarding Slave control, please
 * refer to Registers 37 to 39 and immediately following.
 *
 * @param num : slave number(0-3)
 * @param data: byte to write
 *
 * @return void
 */
static void MPU9250_setSlaveOutputByte(uint8_t num, uint8_t data)
{
    if(num > 3) return;
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_SLV0_DO + num,
                     data);
}


/** Get external data shadow delay enabled status
 *
 * This register is used to specify the timing of external sensor data
 * shadowing. When DELAY_ES_SHADOW is set to 1, shadowing of external
 * sensor data is delayed until all data has been received.
 *
 * @return current external data shadow delay enabled status
 */
static bool MPU9250_getExternalShadowDelayEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_DELAY_CTRL,
                   MPU9250_I2C_MST_DELAY_CTRL_DELAY_ES_SHADOW_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set external data shadow delay enabled status
 *
 * @param enabled: external data shadow delay enabled status
 *
 * @return void
 */
static void MPU9250_setExternalShadowDelayEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_DELAY_CTRL,
                    MPU9250_I2C_MST_DELAY_CTRL_DELAY_ES_SHADOW_BIT, enabled);
}


/** Get slave delay enabled status
 *
 * When a particular slave delay is enabled, the rate of access for the that
 * slave device is reduced. When a slave's access rate is decreased relative to
 * the Sample Rate, the slave is accessed every:
 *
 *     1 / (1 + I2C_MST_DLY) Samples
 *
 * This base Sample Rate in turn is determined by SMPLRT_DIV (register  * 25)
 * and DLPF_CFG (register 26).
 *
 * For further information regarding I2C_MST_DLY, please refer to register 52.
 * For further information regarding the Sample Rate, please refer to register 25.
 *
 * @param num: slave number(0-4)
 *
 * @return current slave delay enabled status
 */
static bool MPU9250_getSlaveDelayEnabled(uint8_t num)
{
    if(num > 4) return 0;
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_DELAY_CTRL,
                   num, tmp_buf);
    return tmp_buf[0];
}


/** Set slave delay enabled status
 *
 * @param num     : slave number(0-4)
 * @param enabled : slave delay enabled status
 *
 * @return void
 */
static void MPU9250_setSlaveDelayEnabled(uint8_t num, bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_DELAY_CTRL,
                    num, enabled);
}



/** Reset gyroscope signal path
 *
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 *
 * @return void
 */
static void MPU9250_resetGyroscopePath(void)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_SIGNAL_PATH_RESET,
                    MPU9250_SIGNAL_PATH_RESET_GYRO_RST_BIT, 1);
}


/** Reset accelerometer signal path
 *
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 *
 * @return void
 */
static void MPU9250_resetAccelerometerPath(void)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_SIGNAL_PATH_RESET,
                    MPU9250_SIGNAL_PATH_RESET_ACCE_RST_BIT, 1);
}


/** Reset temperature sensor signal path
 *
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 *
 * @return void
 */
static void MPU9250_resetTemperaturePath(void)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_SIGNAL_PATH_RESET,
                    MPU9250_SIGNAL_PATH_RESET_TEMP_RST_BIT, 1);
}


/** Get FIFO enabled status
 *
 * When this bit is set to 0, the FIFO buffer is disabled. The FIFO buffer
 * cannot be written to or read from while disabled. The FIFO buffer's state
 * does not change unless the MPU-60X0 is power cycled.
 *
 * @return current FIFO enabled status
 */
static bool MPU9250_getFIFOEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_USER_CTRL,
                   MPU9250_USER_CTRL_FIFO_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set FIFO enabled status
 *
 * @param enabled: FIFO enabled status
 *
 * @return void
 */
static void MPU9250_setFIFOEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_USER_CTRL,
                    MPU9250_USER_CTRL_FIFO_EN_BIT, enabled);
}


/** Get I2C Master Mode enabled status
 *
 * When this mode is enabled, the MPU-60X0 acts as the I2C Master to the
 * external sensor slave devices on the auxiliary I2C bus. When this bit is
 * cleared to 0, the auxiliary I2C bus lines (AUX_DA and AUX_CL) are logically
 * driven by the primary I2C bus (SDA and SCL). This is a precondition to
 * enabling Bypass Mode. For further information regarding Bypass Mode, please
 * refer to Register 55.
 *
 * @return current I2C Master Mode enabled status
 */
static bool MPU9250_getI2CMasterModeEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_USER_CTRL,
                   MPU9250_USER_CTRL_I2C_MST_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set I2C Master Mode enabled status
 *
 * @param enabled: I2C Master Mode enabled status
 *
 * @return void
 */
static void MPU9250_setI2CMasterModeEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_USER_CTRL,
                    MPU9250_USER_CTRL_I2C_MST_EN_BIT, enabled);
}


/** Switch from I2C to SPI mode(MPU-6000 only)
 *
 * If this is set, the primary SPI interface will be enabled in place of the
 * disabled primary I2C interface.
 *
 * @param enabled: Switch from I2C to SPI mode enabled status
 *
 * @return void
 */
static void MPU9250_switchSPIEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_USER_CTRL,
                    MPU9250_USER_CTRL_I2C_IF_DIS_BIT, enabled);
}


/** Reset the FIFO
 *
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 *
 * @return void
 */
static void MPU9250_resetFIFO(void)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_USER_CTRL,
                    MPU9250_USER_CTRL_FIFO_RST_BIT, 1);
}


/** Reset the I2C Master
 *
 * This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0.
 * This bit automatically clears to 0 after the reset has been triggered.
 *
 * @return void
 */
static void MPU9250_resetI2CMaster(void)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_USER_CTRL,
                    MPU9250_USER_CTRL_I2C_MST_RST_BIT, 1);
}


/** Reset all sensor registers and signal paths
 *
 * When set to 1, this bit resets the signal paths for all sensors (gyroscopes,
 * accelerometers, and temperature sensor). This operation will also clear the
 * sensor registers. This bit automatically clears to 0 after the reset has been
 * triggered.
 *
 * When resetting only the signal path (and not the sensor registers), please
 * use Register 104, SIGNAL_PATH_RESET.
 *
 * @return void
 */
static void MPU9250_resetSensors(void)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_USER_CTRL,
                    MPU9250_USER_CTRL_SIG_COND_RST_BIT, 1);
}


/** Trigger a full device reset
 *
 * A small delay of ~50ms may be desirable after triggering a reset.
 *
 * @return void
 */
static void MPU9250_reset(void)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_PWR_MGMT_1,
                    MPU9250_PWR_MGMT_1_H_RESET_BIT, 1);
}


/** Get sleep mode status
 *
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 *
 * @return current sleep mode enabled status
 */
static bool MPU9250_getSleepEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_PWR_MGMT_1,
                   MPU9250_PWR_MGMT_1_SLEEP_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set sleep mode status
 *
 * @param enabled: sleep mode enabled status
 *
 * @return void
 */
static void MPU9250_setSleepEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_PWR_MGMT_1,
                    MPU9250_PWR_MGMT_1_SLEEP_BIT, enabled);
}


/** Get wake cycle enabled status
 *
 * When this bit is set to 1 and SLEEP is disabled, the MPU-60X0 will cycle
 * between sleep mode and waking up to take a single sample of data from active
 * sensors at a rate determined by LP_WAKE_CTRL (register 108).
 *
 * @return current sleep mode enabled status
 */
static bool MPU9250_getWakeCycleEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_PWR_MGMT_1,
                   MPU9250_PWR_MGMT_1_CYCLE_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set wake cycle enabled status
 *
 * @param enabled: wake cycle enabled status
 *
 * @return void
 */
static void MPU9250_setWakeCycleEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_PWR_MGMT_1,
                    MPU9250_PWR_MGMT_1_CYCLE_BIT, enabled);
}


/** Get clock source setting
 *
 * @return current clock source setting
 */
static uint8_t MPU9250_getClockSource(void)
{
    I2C_readMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_PWR_MGMT_1,
                      MPU9250_PWR_MGMT_1_CLKSEL_BIT, MPU9250_PWR_MGMT_1_CLKSEL_LEN,
                      tmp_buf);
    return tmp_buf[0];
}


/** Set clock source setting
 *
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source: clock source setting
 *
 * @return void
 */
static void MPU9250_setClockSource(uint8_t source)
{
    I2C_writeMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_PWR_MGMT_1,
                       MPU9250_PWR_MGMT_1_CLKSEL_BIT, MPU9250_PWR_MGMT_1_CLKSEL_LEN,
                       source);
}


/** Get X-axis accelerometer standby enabled status
 *
 * If enabled, the X-axis will not gather or report data (or use power).
 *
 * @return current X-axis standby enabled status
 */
static bool MPU9250_getStandbyXAccelEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_PWR_MGMT_2,
                   MPU9250_PWR_MGMT_2_DISABLE_XA_BIT, tmp_buf);
    return tmp_buf[0];
}

/** Set X-axis accelerometer standby enabled status
 *
 * @param enabled: X-axis standby enabled status
 *
 * @return void
 */
static void MPU9250_setStandbyXAccelEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_PWR_MGMT_2,
                    MPU9250_PWR_MGMT_2_DISABLE_XA_BIT, enabled);
}


/** Get Y-axis accelerometer standby enabled status
 *
 * If enabled, the Y-axis will not gather or report data (or use power)
 *
 * @return current Y-axis standby enabled status
 */
static bool MPU9250_getStandbyYAccelEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_PWR_MGMT_2,
                   MPU9250_PWR_MGMT_2_DISABLE_YA_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set Y-axis accelerometer standby enabled status
 *
 * @param enabled: Y-axis standby enabled status
 *
 * @return void
 */
static void MPU9250_setStandbyYAccelEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_PWR_MGMT_2,
                    MPU9250_PWR_MGMT_2_DISABLE_YA_BIT, enabled);
}


/** Get Z-axis accelerometer standby enabled status
 *
 * If enabled, the Z-axis will not gather or report data (or use power)
 *
 * @return current Z-axis standby enabled status
 */
static bool MPU9250_getStandbyZAccelEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_PWR_MGMT_2,
                   MPU9250_PWR_MGMT_2_DISABLE_ZA_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set Z-axis accelerometer standby enabled status
 *
 * @param enabled: Z-axis standby enabled status
 *
 * @return void
 */
static void MPU9250_setStandbyZAccelEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_PWR_MGMT_2,
                    MPU9250_PWR_MGMT_2_DISABLE_ZA_BIT, enabled);
}


/** Get X-axis gyroscope standby enabled status
 *
 * If enabled, the X-axis will not gather or report data (or use power)
 *
 * @return current X-axis standby enabled status
 */
static bool MPU9250_getStandbyXGyroEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_PWR_MGMT_2,
                   MPU9250_PWR_MGMT_2_DISABLE_XG_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set X-axis gyroscope standby enabled status
 *
 * @param enabled: X-axis standby enabled status
 *
 * @return void
 */
static void MPU9250_setStandbyXGyroEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_PWR_MGMT_2,
                    MPU9250_PWR_MGMT_2_DISABLE_XG_BIT, enabled);
}


/** Get Y-axis gyroscope standby enabled status
 *
 * If enabled, the Y-axis will not gather or report data (or use power)
 *
 * @return current Y-axis standby enabled status
 */
static bool MPU9250_getStandbyYGyroEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_PWR_MGMT_2,
                   MPU9250_PWR_MGMT_2_DISABLE_YG_BIT, tmp_buf);
    return tmp_buf[0];
}


/** Set Y-axis gyroscope standby enabled status
 *
 * @param enabled: Y-axis standby enabled status
 *
 * @return void
 */
static void MPU9250_setStandbyYGyroEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_PWR_MGMT_2,
                    MPU9250_PWR_MGMT_2_DISABLE_YG_BIT, enabled);
}


/** Get Z-axis gyroscope standby enabled status
 *
 * If enabled, the Z-axis will not gather or report data (or use power)
 *
 * @return current Z-axis standby enabled status
 */
static bool MPU9250_getStandbyZGyroEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_PWR_MGMT_2,
                   MPU9250_PWR_MGMT_2_DISABLE_ZG_BIT, tmp_buf);
    return tmp_buf[0];
}

/** Set Z-axis gyroscope standby enabled status
 *
 * @param enabled: Z-axis standby enabled status
 *
 * @return void
 */
static void MPU9250_setStandbyZGyroEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_PWR_MGMT_2,
                    MPU9250_PWR_MGMT_2_DISABLE_ZG_BIT, enabled);
}


/** Get current FIFO buffer size
 *
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 *
 * @return current FIFO buffer size
 */
static uint16_t MPU9250_getFIFOCount(void)
{
    I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_COUNTH,
                       2, tmp_buf);
    return (((uint16_t) tmp_buf[0]) << 8) | tmp_buf[1];
}


/** Get byte from FIFO buffer
 *
 * This register is used to read and write data from the FIFO buffer. Data is
 * written to the FIFO in order of register number (from lowest to highest). If
 * all the FIFO enable flags (see below) are enabled and all External Sensor
 * Data registers (Registers 73 to 96) are associated with a Slave device, the
 * contents of registers 59 through 96 will be written in order at the Sample
 * Rate.
 *
 * The contents of the sensor data registers (Registers 59 to 96) are written
 * into the FIFO buffer when their corresponding FIFO enable flags are set to 1
 * in FIFO_EN (Register 35). An additional flag for the sensor data registers
 * associated with I2C Slave 3 can be found in I2C_MST_CTRL (Register 36).
 *
 * If the FIFO buffer has overflowed, the status bit FIFO_OFLOW_INT is
 * automatically set to 1. This bit is located in INT_STATUS (Register 58).
 * When the FIFO buffer has overflowed, the oldest data will be lost and new
 * data will be written to the FIFO.
 *
 * If the FIFO buffer is empty, reading this register will return the last byte
 * that was previously read from the FIFO until new data is available. The user
 * should check FIFO_COUNT to ensure that the FIFO buffer is not read when
 * empty.
 *
 * @return byte from FIFO buffer
 */
static uint8_t MPU9250_getFIFOByte(void)
{
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_R_W,
                    tmp_buf);
    return tmp_buf[0];
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