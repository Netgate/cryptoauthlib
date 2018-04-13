/**
 * \file
 * \brief ATCA Hardware abstraction layer for I2C on FreeBSD.
 *
 * \copyright (c) 2018 Rubicon Communications, LLC (Netgte).
 *
 */

#ifndef HAL_FREEBSD_I2C_H_
#define HAL_FREEBSD_I2C_H_

/**
 * \brief This enumeration lists flags for I2C read or write addressing.
 */
enum i2c_read_write_flag
{
    I2C_WRITE = (uint8_t)0x00,  //!< write command flag
    I2C_READ  = (uint8_t)0x01   //!< read command flag
};

/**
 * \brief This is the hal_data for ATCA HAL.
 */
typedef struct atcaI2Cmaster
{
    const char *dev;
    int      fd;
    int      ref_ct;
    //! for conveniences during interface release phase
    int bus_index;
} ATCAI2CMaster_t;

#endif /* HAL_FREEBSD_I2C_H_ */
