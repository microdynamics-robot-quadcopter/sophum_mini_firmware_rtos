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

/* private variable */
static uint8_t tmp_buf[14];

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


/**
 * Get the gyroscope output(sample rate) rate divider
 * The sensor register output, FIFO output, DMP sampling, motion detection, zero
 * motion detection, and free fall detection are all based on the sample Rate
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


/**
 * Get external FSYNC configuration
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
                      MPU9250_CFG_EXT_SYNC_SET_BIT, MPU9250_CFG_EXT_SYNC_SET_LEN,
                      tmp_buf);
    return tmp_buf[0];
}


/**
 * Set external FSYNC configuration
 * @param sync: FSYNC config value
 *
 * @return void
 */
static void MPU9250_setExternalFrameSync(uint8_t sync)
{
    I2C_writeMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_CONFIG,
                       MPU9250_CFG_EXT_SYNC_SET_BIT, MPU9250_CFG_EXT_SYNC_SET_LEN,
                       sync);
}


/**
 * Get digital low-pass filter configuration
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
                      MPU9250_CFG_DLPF_CFG_BIT, MPU9250_CFG_DLPF_CFG_LEN,
                      tmp_buf);
    return tmp_buf[0];
}


/**
 * Set Digital Low-Pass Filter configuration
 * @param mode: DLPF config value
 *
 * @return void
 */
static void MPU9250_setDLPFMode(uint8_t mode)
{
    I2C_writeMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_CONFIG,
                       MPU9250_CFG_DLPF_CFG_BIT, MPU9250_CFG_DLPF_CFG_LEN,
                       mode);
}


/**
 * Get full-scale gyroscope range id
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
                      MPU9250_GYROCFG_FS_SEL_BIT, MPU9250_GYROCFG_FS_SEL_LEN,
                      tmp_buf);
    return tmp_buf[0];
}


/**
 * Get full-scale gyroscope degrees per LSB
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
        case MPU9250_GYROCFG_FS_250dps:
            range = MPU9250_DEG_PER_LSB_250dps;
        break;
        case MPU9250_GYROCFG_FS_500dps:
            range = MPU9250_DEG_PER_LSB_500dps;
        break;
        case MPU9250_GYROCFG_FS_1000dps:
            range = MPU9250_DEG_PER_LSB_1000dps;
        break;
        case MPU9250_GYROCFG_FS_2000dps:
            range = MPU9250_DEG_PER_LSB_2000dps;
        break;
        default:
            range = MPU9250_DEG_PER_LSB_1000dps;
        break;
    }
    return range;
}


/**
 * Set full-scale gyroscope range
 * @param range: full-scale gyroscope range value
 *
 * @return void
 */
static void MPU9250_setFullScaleGyroRange(uint8_t range)
{
    I2C_writeMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_CONFIG,
                       MPU9250_GYROCFG_FS_SEL_BIT, MPU9250_GYROCFG_FS_SEL_LEN,
                       range);
}


/**
 * Set self-test enabled value for gyro X axis
 * @param enabled: Self-test enabled value
 *
 * @return void
 */
static void MPU9250_setGyroXSelfTest(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_CONFIG,
                     MPU9250_GYROCFG_XG_SELF_TEST_EN_BIT, enabled);
}


/**
 * Set self-test enabled value for gyro Y axis
 * @param enabled: Self-test enabled value
 *
 * @return void
 */
static void MPU9250_setGyroYSelfTest(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_CONFIG,
                    MPU9250_GYROCFG_YG_SELF_TEST_EN_BIT, enabled);
}


/**
 * Set self-test enabled value for gyro Z axis
 * @param enabled: Self-test enabled value
 *
 * @return void
 */
static void MPU9250_setGyroZSelfTest(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_GYRO_CONFIG,
                     MPU9250_GYROCFG_ZG_SELF_TEST_EN_BIT, enabled);
}


/**
 * Get self-test enabled setting for acce X axis
 *
 * @return Self-test enabled value
 */
static bool MPU9250_getAccelXSelfTest(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG,
                   MPU9250_ACCECFG_XA_SELF_TEST_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/**
 * Set self-test enabled setting for acce X axis
 * @param enabled: Self-test enabled value
 *
 * @return void
 */
static void MPU9250_setAccelXSelfTest(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG,
                    MPU9250_ACCECFG_XA_SELF_TEST_EN_BIT, enabled);
}


/**
 * Get self-test enabled setting for acce Y axis
 *
 * @return Self-test enabled value
 */
static bool MPU9250_getAccelYSelfTest(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG,
                   MPU9250_ACCECFG_YA_SELF_TEST_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/**
 * Set self-test enabled setting for acce Y axis
 * @param enabled: Self-test enabled value
 *
 * @return void
 */
static void MPU9250_setAccelYSelfTest(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG,
                    MPU9250_ACCECFG_YA_SELF_TEST_EN_BIT, enabled);
}


/**
 * Get self-test enabled setting for acce Z axis
 *
 * @return Self-test enabled value
 */
static bool MPU9250_getAccelZSelfTest(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG,
                   MPU9250_ACCECFG_ZA_SELF_TEST_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/**
 * Set self-test enabled setting for acce Z axis
 * @param enabled: Self-test enabled value
 *
 * @return void
 */
static void MPU9250_setAccelZSelfTest(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG,
                    MPU9250_ACCECFG_ZA_SELF_TEST_EN_BIT, enabled);
}


/**
 * Get full-scale accelerometer range
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below
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
                      MPU9250_ACCECFG_AFS_SEL_BIT, MPU9250_ACCECFG_AFS_SEL_LEN,
                      tmp_buf);
    return tmp_buf[0];
}


/**
 * Get full-scale accelerometer G per LSB
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
        case MPU9250_ACCECFG_FS_2g:
            range = MPU9250_G_PER_LSB_2g;
        break;
        case MPU9250_ACCECFG_FS_4g:
            range = MPU9250_G_PER_LSB_4g;
        break;
        case MPU9250_ACCECFG_FS_8g:
            range = MPU9250_G_PER_LSB_8g;
        break;
        case MPU9250_ACCECFG_FS_16g:
            range = MPU9250_G_PER_LSB_16g;
        break;
        default:
            range = MPU9250_G_PER_LSB_8g;
        break;
    }
    return range;
}


/**
 * Set full-scale accelerometer range
 * @param range: full-scale accelerometer range setting
 *
 * @return void
 */
static void MPU9250_setFullScaleAccelRange(uint8_t range)
{
    I2C_writeMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG,
                       MPU9250_ACCECFG_AFS_SEL_BIT, MPU9250_ACCECFG_AFS_SEL_LEN,
                       range);
}


/**
 * Set accelerometer Digital Low-Pass Filter
 * @param range: DLPF setting
 *
 * @return void
 */
static void MPU9250_setAccelDLPF(uint8_t range)
{
    I2C_writeMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_ACCEL_CONFIG2,
                       MPU9250_ACCECFG2_DLPF_BIT, MPU9250_ACCECFG2_DLPF_LEN, range);
}


/**
 * Get temperature FIFO enabled value
 * When set to 1, this bit enables TEMP_OUT_H and TEMP_OUT_L (Registers 65 and
 * 66) to be written into the FIFO buffer
 *
 * @return current temperature FIFO enabled value
 */
static bool MPU9250_getTempFIFOEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                   MPU9250_FIFO_EN_TEMP_BIT, tmp_buf);
    return tmp_buf[0];
}


/**
 * Set temperature FIFO enabled value
 * @param enabled: temperature FIFO enabled value
 *
 * @return void
 */
static void MPU9250_setTempFIFOEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                    MPU9250_FIFO_EN_TEMP_BIT, enabled);
}


/**
 * Get gyroscope X-axis FIFO enabled value
 * When set to 1, this bit enables GYRO_XOUT_H and GYRO_XOUT_L (Registers 67 and
 * 68) to be written into the FIFO buffer
 *
 * @return current gyroscope X-axis FIFO enabled value
 */
static bool MPU9250_getXGyroFIFOEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                   MPU9250_FIFO_EN_XG_BIT, tmp_buf);
    return tmp_buf[0];
}


/**
 * Set gyroscope X-axis FIFO enabled value
 * @param enabled: gyroscope X-axis FIFO enabled value
 *
 * @return void
 */
static void MPU9250_setXGyroFIFOEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                    MPU9250_FIFO_EN_XG_BIT, enabled);
}


/**
 * Get gyroscope Y-axis FIFO enabled value
 * When set to 1, this bit enables GYRO_YOUT_H and GYRO_YOUT_L (Registers 69 and
 * 70) to be written into the FIFO buffer
 *
 * @return current gyroscope Y-axis FIFO enabled value
 */
static bool MPU9250_getYGyroFIFOEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                   MPU9250_FIFO_EN_YG_BIT, tmp_buf);
    return tmp_buf[0];
}


/**
 * Set gyroscope Y-axis FIFO enabled value
 * @param enabled: gyroscope Y-axis FIFO enabled value
 *
 * @return void
 */
static void MPU9250_setYGyroFIFOEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                    MPU9250_FIFO_EN_YG_BIT, enabled);
}


/**
 * Get gyroscope Z-axis FIFO enabled value
 * When set to 1, this bit enables GYRO_ZOUT_H and GYRO_ZOUT_L (Registers 71 and
 * 72) to be written into the FIFO buffer
 *
 * @return current gyroscope Z-axis FIFO enabled value
 */
static bool MPU9250_getZGyroFIFOEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                   MPU9250_FIFO_EN_ZG_BIT, tmp_buf);
    return tmp_buf[0];
}


/**
 * Set gyroscope Z-axis FIFO enabled value
 * @param enabled: gyroscope Z-axis FIFO enabled value
 *
 * @return void
 */
static void MPU9250_setZGyroFIFOEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                    MPU9250_FIFO_EN_ZG_BIT, enabled);
}


/**
 * Get accelerometer FIFO enabled value
 * When set to 1, this bit enables ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H,
 * ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L (Registers 59 to 64) to be
 * written into the FIFO buffer
 *
 * @return current accelerometer FIFO enabled value
 */
static bool MPU9250_getAccelFIFOEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                   MPU9250_FIFO_EN_ACCE_BIT, tmp_buf);
    return tmp_buf[0];
}


/**
 * Set accelerometer FIFO enabled value
 * @param enabled: accelerometer FIFO enabled value
 *
 * @return void
 */
static void MPU9250_setAccelFIFOEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                    MPU9250_FIFO_EN_ACCE_BIT, enabled);
}


/**
 * Get Slave 2 FIFO enabled value
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 2 to be written into the FIFO buffer
 *
 * @return current Slave 2 FIFO enabled value
 */
static bool MPU9250_getSlave2FIFOEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                   MPU9250_FIFO_EN_SLV2_BIT, tmp_buf);
    return tmp_buf[0];
}


/**
 * Set Slave 2 FIFO enabled value
 * @param enabled: Slave 2 FIFO enabled value
 *
 * @return void
 */
static void MPU9250_setSlave2FIFOEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                    MPU9250_FIFO_EN_SLV2_BIT, enabled);
}


/**
 * Get Slave 1 FIFO enabled value
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 1 to be written into the FIFO buffer
 *
 * @return current Slave 1 FIFO enabled value
 */
static bool MPU9250_getSlave1FIFOEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                   MPU9250_FIFO_EN_SLV1_BIT, tmp_buf);
    return tmp_buf[0];
}


/**
 * Set Slave 1 FIFO enabled value
 * @param enabled: Slave 1 FIFO enabled value
 *
 * @return void
 */
static void MPU9250_setSlave1FIFOEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                    MPU9250_FIFO_EN_SLV1_BIT, enabled);
}


/**
 * Get Slave 0 FIFO enabled value
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 0 to be written into the FIFO buffer
 *
 * @return current Slave 0 FIFO enabled value
 */
static bool MPU9250_getSlave0FIFOEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                   MPU9250_FIFO_EN_SLV0_BIT, tmp_buf);
    return tmp_buf[0];
}

/**
 * Set Slave 0 FIFO enabled value
 * @param enabled: Slave 0 FIFO enabled value
 *
 * @return void
 */
static void MPU9250_setSlave0FIFOEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_FIFO_EN,
                    MPU9250_FIFO_EN_SLV0_BIT, enabled);
}


/**
 * Get multi-master enabled value
 * Multi-master capability allows multiple I2C masters to operate on the same
 * bus. In circuits where multi-master capability is required, set MULT_MST_EN
 * to 1. This will increase current drawn by approximately 30uA
 *
 * In circuits where multi-master capability is required, the state of the I2C
 * bus must always be monitored by each separate I2C Master. Before an I2C
 * Master can assume arbitration of the bus, it must first confirm that no other
 * I2C Master has arbitration of the bus. When MULT_MST_EN is set to 1, the
 * MPU-60X0's bus arbitration detection logic is turned on, enabling it to
 * detect when the bus is available
 *
 * @return current multi-master enabled value
 */
static bool MPU9250_getMultiMasterEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_CTRL,
                   MPU9250_MULT_MST_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/**
 * Set multi-master enabled value
 * @param enabled: multi-master enabled value
 *
 * @return void
 */
static void MPU9250_setMultiMasterEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_CTRL,
                    MPU9250_MULT_MST_EN_BIT, enabled);
}


/**
 * Get wait-for-external-sensor-data enabled value
 * When the WAIT_FOR_ES bit is set to 1, the Data Ready interrupt will be
 * delayed until External Sensor data from the Slave Devices are loaded into the
 * EXT_SENS_DATA registers. This is used to ensure that both the internal sensor
 * data (i.e. from gyro and accel) and external sensor data have been loaded to
 * their respective data registers (i.e. the data is synced) when the Data Ready
 * interrupt is triggered
 *
 * @return current wait-for-external-sensor-data enabled value
 */
static bool MPU9250_getWaitForExternalSensorEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_CTRL,
                   MPU9250_WAIT_FOR_ES_BIT, tmp_buf);
    return tmp_buf[0];
}


/**
 * Set wait-for-external-sensor-data enabled value
 * @param enabled: wait-for-external-sensor-data enabled value
 *
 * @return void
 */
static void MPU9250_setWaitForExternalSensorEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_CTRL,
                    MPU9250_WAIT_FOR_ES_BIT, enabled);
}


/**
 * Get Slave 3 FIFO enabled value
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 3 to be written into the FIFO buffer
 *
 * @return current Slave 3 FIFO enabled value
 */
static bool MPU9250_getSlave3FIFOEnabled(void)
{
    I2C_readOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_CTRL,
                   MPU9250_SLV3_FIFO_EN_BIT, tmp_buf);
    return tmp_buf[0];
}


/**
 * Set Slave 3 FIFO enabled value
 * @param enabled: Slave 3 FIFO enabled value
 *
 * @return void
 */
static void MPU9250_setSlave3FIFOEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_CTRL,
                    MPU9250_SLV3_FIFO_EN_BIT, enabled);
}


/**
 * Get slave read/write transition enabled value
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
                   MPU9250_I2C_MST_P_NSR_BIT, tmp_buf);
    return tmp_buf[0];
}


/**
 * Set slave read/write transition enabled value
 * @param enabled: slave read/write transition enabled value
 *
 * @return void
 */
static void MPU9250_setSlaveReadWriteTransitionEnabled(bool enabled)
{
    I2C_writeOneBit(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_CTRL,
                    MPU9250_I2C_MST_P_NSR_BIT, enabled);
}


/**
 * Get I2C master clock speed
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
 * @return Current I2C master clock speed
 */
static uint8_t MPU9250_getMasterClockSpeed(void)
{
    I2C_readMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_CTRL,
                      MPU9250_I2C_MST_CLK_BIT, MPU9250_I2C_MST_CLK_LEN,
                      tmp_buf);
    return tmp_buf[0];
}


/**
 * Set I2C master clock speed
 * @param speed: current I2C master clock speed
 *
 * @return void
 */
static void MPU9250_setMasterClockSpeed(uint8_t speed)
{
    I2C_writeMultiBits(I2C_NUM1_MASTER_PORT_GPIO, MPU9250_ADDR, MPU9250_I2C_MST_CTRL,
                       MPU9250_I2C_MST_CLK_BIT, MPU9250_I2C_MST_CLK_LEN,
                       speed);
}



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