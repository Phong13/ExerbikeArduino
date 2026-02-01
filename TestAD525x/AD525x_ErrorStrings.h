/** @file 
Header file for retrieving human-readable strings corresponding to the error codes defined in 
`AD525x_Errors.h`.
*/
#ifndef AD525X_ERRSTRINGS_H
#define AD525X_ERRSTRINGS_H

#include <stdint.h>                      // Required to use uint8_t

#define EC_NO_ERR_str "No error."
#define EC_DATA_LONG_str "Data too long to fit in transmit buffer."
#define EC_NACK_ADDR_str "Received NACK on transmit of address."
#define EC_NACK_DATA_str "Received NACK on transmit of data."
#define EC_I2C_OTHER_str "Other I2C error."
#define EC_BAD_REGISTER_str "Invalid register passed to function."
#define EC_BAD_WIPER_SETTING_str "Invalid wiper setting."
#define EC_BAD_READ_SIZE_str "Invalid number of bytes read from register."
#define EC_BAD_DEVICE_ADDR_str "Bad device address - device address must be in [0, 3]."
#define EC_NOT_INITIALIZED_str "Communication has not been initialized."

#define EC_UNKNOWN_ERR_str "Unknown error."

const char *AD525xErrorStringHandler(uint8_t err_code);

#endif