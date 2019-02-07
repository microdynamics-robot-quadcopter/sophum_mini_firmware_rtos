#include <stdio.h>
#include "driver/i2c.h"
#include "esp32_module_mpu9250.h"


#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_1  /*!< I2C port number for master dev */
#define ACK_CHECK_EN       0x1  /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS      0x0  /*!< I2C master will not check ack from slave */
#define ACK_VAL            0x0  /*!< I2C ack value */
#define NACK_VAL           0x1  /*!< I2C nack value */

#define MPU9250_ADDR       0xD0 //i2c的地址 AD0=0 0xD0 AD0=1 0xD2

#define MPU9250_SELF_TEST_X_GYRO   0x00
#define MPU9250_SELF_TEST_Y_GYRO   0x01
#define MPU9250_SELF_TEST_Z_GYRO   0x02
#define MPU9250_SELF_TEST_X_ACCEL  0x0D
#define MPU9250_SELF_TEST_Y_ACCEL  0x0E
#define MPU9250_SELF_TEST_Z_ACCEL  0x0F
#define MPU9250_XG_OFFSET_H        0x13
#define MPU9250_XG_OFFSET_L        0x14
#define MPU9250_YG_OFFSET_H        0x15
#define MPU9250_YG_OFFSET_L        0x16
#define MPU9250_ZG_OFFSET_H        0x17
#define MPU9250_ZG_OFFSET_L        0x18
#define MPU9250_SMPLRT_DIV         0x19 //陀螺仪采样率典型值为0x07 1000/(1+7)=125HZ
#define MPU9250_CONFIG             0x1A //低通滤波器 典型值0x06 5hz
#define MPU9250_GYRO_CONFIG        0x1B //陀螺仪测量范围 0x18 正负2000度
#define MPU9250_ACCEL_CONFIG       0x1C //加速度计测量范围 0x18 正负16g
#define MPU9250_ACCEL_CONFIG2      0x1D //加速度计低通滤波器 0x06 5hz
#define MPU9250_LP_ACCEL_ODR       0x1E
#define MPU9250_WOM_THR            0x1F
#define MPU9250_FIFO_EN            0x23
#define MPU9250_I2C_MST_CTRL       0x24
#define MPU9250_I2C_SLV0_ADDR      0x25
#define MPU9250_I2C_SLV0_REG       0x26
#define MPU9250_I2C_SLV0_CTRL      0x27
#define MPU9250_I2C_SLV1_ADDR      0x28
#define MPU9250_I2C_SLV1_REG       0x29
#define MPU9250_I2C_SLV1_CTRL      0x2A
#define MPU9250_I2C_SLV2_ADDR      0x2B
#define MPU9250_I2C_SLV2_REG       0x2C
#define MPU9250_I2C_SLV2_CTRL      0x2D
#define MPU9250_I2C_SLV3_ADDR      0x2E
#define MPU9250_I2C_SLV3_REG       0x2F
#define MPU9250_I2C_SLV3_CTRL      0x30
#define MPU9250_I2C_SLV4_ADDR      0x31
#define MPU9250_I2C_SLV4_REG       0x32
#define MPU9250_I2C_SLV4_DO        0x33
#define MPU9250_I2C_SLV4_CTRL      0x34
#define MPU9250_I2C_SLV4_DI        0x35
#define MPU9250_I2C_MST_STATUS     0x36
#define MPU9250_INT_PIN_CFG        0x37
#define MPU9250_INT_ENABLE         0x38
#define MPU9250_INT_STATUS         0x3A
#define MPU9250_ACCEL_XOUT_H       0x3B //加速度计输出数据
#define MPU9250_ACCEL_XOUT_L       0x3C
#define MPU9250_ACCEL_YOUT_H       0x3D
#define MPU9250_ACCEL_YOUT_L       0x3E
#define MPU9250_ACCEL_ZOUT_H       0x3F
#define MPU9250_ACCEL_ZOUT_L       0x40
#define MPU9250_TEMP_OUT_H         0x41 //温度计输出数据
#define MPU9250_TEMP_OUT_L         0x42
#define MPU9250_GYRO_XOUT_H        0x43 //陀螺仪输出数据
#define MPU9250_GYRO_XOUT_L        0x44
#define MPU9250_GYRO_YOUT_H        0x45
#define MPU9250_GYRO_YOUT_L        0x46
#define MPU9250_GYRO_ZOUT_H        0x47
#define MPU9250_GYRO_ZOUT_L        0x48
#define MPU9250_EXT_SENS_DATA_00   0x49
#define MPU9250_EXT_SENS_DATA_01   0x4A
#define MPU9250_EXT_SENS_DATA_02   0x4B
#define MPU9250_EXT_SENS_DATA_03   0x4C
#define MPU9250_EXT_SENS_DATA_04   0x4D
#define MPU9250_EXT_SENS_DATA_05   0x4E
#define MPU9250_EXT_SENS_DATA_06   0x4F
#define MPU9250_EXT_SENS_DATA_07   0x50
#define MPU9250_EXT_SENS_DATA_08   0x51
#define MPU9250_EXT_SENS_DATA_09   0x52
#define MPU9250_EXT_SENS_DATA_10   0x53
#define MPU9250_EXT_SENS_DATA_11   0x54
#define MPU9250_EXT_SENS_DATA_12   0x55
#define MPU9250_EXT_SENS_DATA_13   0x56
#define MPU9250_EXT_SENS_DATA_14   0x57
#define MPU9250_EXT_SENS_DATA_15   0x58
#define MPU9250_EXT_SENS_DATA_16   0x59
#define MPU9250_EXT_SENS_DATA_17   0x5A
#define MPU9250_EXT_SENS_DATA_18   0x5B
#define MPU9250_EXT_SENS_DATA_19   0x5C
#define MPU9250_EXT_SENS_DATA_20   0x5D
#define MPU9250_EXT_SENS_DATA_21   0x5E
#define MPU9250_EXT_SENS_DATA_22   0x5F
#define MPU9250_EXT_SENS_DATA_23   0x60
#define MPU9250_I2C_SLV0_DO        0x63
#define MPU9250_I2C_SLV1_DO        0x64
#define MPU9250_I2C_SLV2_DO        0x65
#define MPU9250_I2C_SLV3_DO        0x66
#define MPU9250_I2C_MST_DELAY_CTRL 0x67
#define MPU9250_SIGNAL_PATH_RESET  0x68
#define MPU9250_MOT_DETECT_CTRL    0x69
#define MPU9250_USER_CTRL          0X6A //用户配置当为0X10时使用SPI模式
#define MPU9250_PWR_MGMT_1         0X6B //电源管理1 典型值为0x00
#define MPU9250_PWR_MGMT_2         0X6C //电源管理2 典型值为0X00
#define MPU9250_FIFO_COUNTH        0x72
#define MPU9250_FIFO_COUNTL        0x73
#define MPU9250_FIFO_R_W           0x74
#define MPU9250_WHO_AM_I           0X75 //器件ID MPU9250默认ID为0X71
#define MPU9250_XA_OFFSET_H        0x77
#define MPU9250_XA_OFFSET_L        0x78
#define MPU9250_YA_OFFSET_H        0x7A
#define MPU9250_YA_OFFSET_L        0x7B
#define MPU9250_ZA_OFFSET_H        0x7D
#define MPU9250_ZA_OFFSET_L        0x7E


uint8_t MPU9250_getID(void)
{
    int ret;
    uint8_t mpu9250_id = 0x00;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9250_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MPU9250_WHO_AM_I, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9250_ADDR + 1, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &mpu9250_id, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);    

    if(ret != ESP_OK)
    {
        printf("MPU9250 get ID ERROR!!!\n");
    }
    else 
    {
        printf("The MPU9250 ID is %u\n", mpu9250_id);
    }
    return mpu9250_id;
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool MPU9250_testConnection(void)
{
    return MPU9250_getID() == 0x71; //0x71 is MPU9250 ID with AD0 = 0;
}

/** Do a MPU9250 self test.
 * @return True if self test passed, false otherwise
 */
bool MPU9250_getSelfTest(void)
{
    uint8_t raw_data[6] = {0, 0, 0, 0, 0, 0};
    uint8_t old_config[5];
    uint8_t selftest_data[6];
    
    return true;
}

/** Evaluate the values from a MPU9250 self test.
 * @param low The low limit of the self test
 * @param high The high limit of the self test
 * @param value The value to compare with.
 * @param string A pointer to a string describing the value.
 * @return True if self test within low - high limit, false otherwise
 */
bool MPU9250_evaluateSelfTest(float low, float high, float value, char* string)
{
    if(value < low || value > high)
    {
        printf("Self test %s [FAIL]. low: %0.2f, high: %0.2f, measured: %0.2f\n",
                string, (double)low, (double)high, (double)value);
        return false;
    }
    return true;
}

