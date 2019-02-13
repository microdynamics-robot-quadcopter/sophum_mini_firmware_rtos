#ifndef __SOPHUM_DRIVER_I2C_H__
#define __SOPHUM_DRIVER_I2C_H__

#define I2C_NUM0_MASTER_SCL_GPIO       25        /* gpio number for I2C num0 master clock */
#define I2C_NUM0_MASTER_SDA_GPIO       26        /* gpio number for I2C num0 master data  */
#define I2C_NUM0_MASTER_PORT_GPIO      I2C_NUM_0 /* I2C port number for master dev */
#define I2C_NUM0_MASTER_TX_BUF_DISABLE 0         /* I2C num0 master do not need buffer */
#define I2C_NUM0_MASTER_RX_BUF_DISABLE 0         /* I2C num0 master do not need buffer */
#define I2C_NUM0_MASTER_FREQ_HZ        100000    /* I2C num0 master clock frequency(100k) */

#define I2C_NUM1_MASTER_SCL_GPIO       18        /* gpio number for I2C num1 master clock */
#define I2C_NUM1_MASTER_SDA_GPIO       19        /* gpio number for I2C num1 master data  */
#define I2C_NUM1_MASTER_PORT_GPIO      I2C_NUM_1 /* I2C port number for master dev */
#define I2C_NUM1_MASTER_TX_BUF_DISABLE 0         /* I2C num1 master do not need buffer */
#define I2C_NUM1_MASTER_RX_BUF_DISABLE 0         /* I2C num1 master do not need buffer */
#define I2C_NUM1_MASTER_FREQ_HZ        100000    /* I2C num1 master clock frequency(100k) */

#define MASTER_EN_CHECK_ACK            0x1       /* I2C master will check ack from slave */
#define MASTER_DIS_CHECK_ACK           0x0       /* I2C master will not check ack from slave */
#define MASTER_ACK                     0x0       /* I2C master ack value for each byte read */
#define MASTER_NACK                    0x1       /* I2C master nack value for each byte read */
#define MASTER_LAST_NACK               0x2       /* I2C master nack value for the last byte */


/** Initialize an I2C peripheral master mode driver
 *
 * @param i2c_port: The I2C port number(I2C_NUM_0 or I2C_NUM_1)
 * @param scl_pin : The scl pin
 * @param sda_pin : The sda pin
 * @param clk_frq : The clock speed
 *
 * @return TRUE if write was successful, otherwise FALSE
 */
extern bool I2C_MasterInit(uint8_t i2c_port, uint8_t scl_pin,
                           uint8_t sda_pin, uint32_t clk_frq);

/** Write only register address to an I2C peripheral(note: such as reset operation)
 *
 * @param i2c_port: The I2C port number(I2C_NUM_0 or I2C_NUM_1)
 * @param dev_addr: The device address to write to
 * @param reg_addr: The internal address to write from, I2CDEV_NO_MEM_ADDR if none
 *
 * @return TRUE if write was successful, otherwise FALSE
 */
extern bool I2C_writeZeroByte(uint8_t i2c_port, uint8_t dev_addr,
                              uint8_t reg_addr);

/** Write one byte to an I2C peripheral
 *
 * @param i2c_port: The I2C port number(I2C_NUM_0 or I2C_NUM_1)
 * @param dev_addr: The device address to write to
 * @param reg_addr: The internal address to write from, I2CDEV_NO_MEM_ADDR if none
 * @param data    : The byte to write
 *
 * @return TRUE if write was successful, otherwise FALSE
 */
extern bool I2C_writeOneByte(uint8_t i2c_port, uint8_t dev_addr,
                             uint8_t reg_addr, uint8_t data);

/** Write mulit bytes to an I2C peripheral
 *
 * @param i2c_port: The I2C port number(I2C_NUM_0 or I2C_NUM_1)
 * @param dev_addr: The device address to write to
 * @param reg_addr: The internal address to write to, I2CDEV_NO_MEM_ADDR if none
 * @param byte_num: Number of bytes to read
 * @param data    : Pointer to a buffer to read the data from that will be written
 *
 * @return TRUE if write was successful, otherwise FALSE
 */
extern bool I2C_writeMultiBytes(uint8_t i2c_port, uint8_t dev_addr,
                                uint8_t reg_addr, uint16_t byte_num,
                                uint8_t* data);

/** Write one bit to an I2C peripheral
 *
 * @param i2c_port: The I2C port number(I2C_NUM_0 or I2C_NUM_1)
 * @param dev_addr: The device address to write to
 * @param reg_addr: The internal address to write to, I2CDEV_NO_MEM_ADDR if none
 * @param bit_id  : The bit id(0 - 7)to write
 * @param data    : The bit to write
 *
 * @return TRUE if read was successful, otherwise FALSE
 */
extern bool I2C_writeOneBit(uint8_t i2c_port, uint8_t dev_addr,
                            uint8_t reg_addr, uint8_t bit_id,
                            uint8_t data);

/** Write up to 8 bits to an I2C peripheral(note: start bit is MSB)
 *
 * @param i2c_port : The I2C port number(I2C_NUM_0 or I2C_NUM_1)
 * @param dev_addr : The device address to write to
 * @param reg_addr : The internal address to write to, I2CDEV_NO_MEM_ADDR if none
 * @param bit_start: The bit to start from(0 - 7)
 * @param bit_len  : The number of bits to write(1 - 8)
 * @param data     : The byte containing the bits to write
 *
 * @return TRUE if read was successful, otherwise FALSE
 */
extern bool I2C_writeMultiBits(uint8_t i2c_port, uint8_t dev_addr,
                               uint8_t reg_addr, uint8_t bit_start,
                               uint8_t bit_len, uint8_t data);

/** Read one byte from an I2C peripheral
 *
 * @param i2c_port: The I2C port number(I2C_NUM_0 or I2C_NUM_1)
 * @param dev_addr: The device address to read from
 * @param reg_addr: The internal address to read from, I2CDEV_NO_MEM_ADDR if none
 * @param data    : Pointer to a buffer to read the data to
 *
 * @return TRUE if read was successful, otherwise FALSE
 */
extern bool I2C_readOneByte(uint8_t i2c_port, uint8_t dev_addr,
                            uint8_t reg_addr, uint8_t* data);

/** Read mulit bytes from an I2C peripheral
 *
 * @param i2c_port: The I2C port number(I2C_NUM_0 or I2C_NUM_1)
 * @param dev_addr: The device address to read from
 * @param reg_addr: The internal address to read from, I2CDEV_NO_MEM_ADDR if none
 * @param byte_num: Number of bytes to read
 * @param data    : Pointer to a buffer to read the data to
 *
 * @return TRUE if read was successful, otherwise FALSE
 */
extern bool I2C_readMultiBytes(uint8_t i2c_port, uint8_t dev_addr,
                               uint8_t reg_addr, uint16_t byte_num,
                               uint8_t* data);

/** Read one bit from an I2C peripheral
 *
 * @param i2c_port: The I2C port number(I2C_NUM_0 or I2C_NUM_1)
 * @param dev_addr: The device address to read from
 * @param reg_addr: The internal address to read from, I2CDEV_NO_MEM_ADDR if none
 * @param bit_id  : The bit id(0 - 7) to read
 * @param data    : Pointer to a buffer to read the data to
 *
 * @return TRUE if read was successful, otherwise FALSE
 */
extern bool I2C_readOneBit(uint8_t i2c_port, uint8_t dev_addr,
                           uint8_t reg_addr, uint8_t bit_id,
                           uint8_t* data);

/** Read up to 8 bits from an I2C peripheral(note: start bit is MSB)
 *
 * @param i2c_port : The I2C port number(I2C_NUM_0 or I2C_NUM_1)
 * @param dev_addr : The device address to read from
 * @param reg_addr : The internal address to read from, I2CDEV_NO_MEM_ADDR if none
 * @param bit_start: The bit to start from(0 - 7)
 * @param bit_len  : The number of bits to read(1 - 8)
 * @param data     : Pointer to a buffer to read the data to
 *
 * @return TRUE if read was successful, otherwise FALSE
 */
extern bool I2C_readMultiBits(uint8_t i2c_port, uint8_t dev_addr,
                              uint8_t reg_addr, uint8_t bit_start,
                              uint8_t bit_len, uint8_t* data);

#endif


