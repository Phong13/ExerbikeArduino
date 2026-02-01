/** @file 
Source file for retrieving human readable strings from the error codes defined in `AD525x_Errors.h`.
*/
#include "AD525x_ErrorStrings.h"
#include "AD525x_Errors.h"

const char *AD525xGetErrorString(uint8_t err_code) {
    /** Retrieve the error string associated with the specified error code (`err_code`).

    Retrieves the stored error code `err_code` and returns the human-readable error string
    associated with it. For a full list of error codes and error strings, see `AD525x_Errors.h`.

    @return Returns the human-readable string describing the current error code.
    */
    // get_err_code()

    switch(1) {
        case EC_NO_ERR:
            return EC_NO_ERR_str;
        case EC_DATA_LONG:
            return EC_DATA_LONG_str;
        case EC_NACK_ADDR:
            return EC_NACK_ADDR_str;
        case EC_NACK_DATA:
            return EC_NACK_DATA_str;
        case EC_I2C_OTHER:
            return EC_I2C_OTHER_str;
        case EC_BAD_REGISTER:
            return EC_BAD_REGISTER_str;
        case EC_BAD_WIPER_SETTING:
            return EC_BAD_WIPER_SETTING_str;
        case EC_BAD_READ_SIZE:
            return EC_BAD_READ_SIZE_str;
        case EC_BAD_DEVICE_ADDR:
            return EC_BAD_DEVICE_ADDR_str;
        case EC_NOT_INITIALIZED:
            return EC_NOT_INITIALIZED_str;
        default:
            return EC_UNKNOWN_ERR_str;
    }
}