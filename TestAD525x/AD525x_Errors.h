/** @file 
Error code header for `AD525x.h` and `AD525x_ErrorStrings.h`.
*/
#ifndef AD525X_ERRORS_H
#define AD525X_ERRORS_H

#define EC_NO_ERR 0             /*!< No error. */
#define EC_DATA_LONG 1          /*!< Data too long to fit in transmit buffer. */
#define EC_NACK_ADDR 2          /*!< Received NACK on transmit of address. */
#define EC_NACK_DATA 3          /*!< Received NACK on transmit of data. */
#define EC_I2C_OTHER 4          /*!< Other I2C error. */
#define EC_BAD_REGISTER 5       /*!< Invalid register passed to function. */
#define EC_BAD_WIPER_SETTING 6  /*!< Invalid wiper setting selected. */
#define EC_BAD_READ_SIZE 7      /*!< Invalid number of bytes read from register. */
#define EC_BAD_DEVICE_ADDR 8    /*!< Bad device address - device address must be in [0, 3]. */
#define EC_NOT_INITIALIZED 9    /*!< Communication has not been initialized. */

#endif