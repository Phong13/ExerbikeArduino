/** @file 
Class file for AD5253/AD5254 digital potentiometer Arduino library.
*/

#include "AD525x.h"
#include "AD525x_Errors.h"
#include <Arduino.h>

uint8_t AD525x::initialize(uint8_t AD_addr) {
    /** Initialize the potentiometer - pass `(AD1<<1 | AD0)` to AD_addr to set the device address.

    Starts I2C communications with the specified device (specified via the `AD1` and `AD0` pins on
    the device itself -  high = 1, low = 0). This two-bit input parameter is used to construct the
    full 7-bit I2C address.

    If an invalid address is specified, `err_code` is set to `EC_BAD_DEVICE_ADDR`. This can be
    queried via `get_err_code()`.

    @param[in] AD_addr The two bit user-specified address of the device with which you are 
                       communicating. Should be (AD1<<1 | AD0). 

    @return Returns 0 on no error or the error code on error.
    */

    // Error is raised in the event of an invalid address.
    if(AD_addr > AD525x::max_AD_addr) {
        initialized = false;
        err_code = EC_BAD_DEVICE_ADDR;
        return err_code;
    }

    dev_addr = AD525x::base_I2C_addr | AD_addr;

    Wire.begin();                               // Start I2C communications.

    initialized = true;
    return 0;
}

uint8_t AD525x::write_RDAC(uint8_t RDAC, uint8_t value) {
    /** Write value to RDAC register specified by RDAC. RDAC registers are zero-based index.

    Writes the specified value (`value`) to the RDAC register with RDAC address `RDAC`. There are 
    four RDAC registers, addressed using 2-bits on a zero-based index.

    @param[in] RDAC     The address of one of the 4 RDAC registers (0-3), representing the 4 
                        potentiometers in the IC.
    @param[in] value    The wiper value to set the specified RDAC. This value must be in the span
                        [0, `max_val`].

    @return Returns 0 on no error, otherwise returns an error code and sets the `err_code` parameter
            (queried via `get_err_code()`). I2C errors are raised indirectly via a call to 
            `write_data()`. This function also raises the following error codes:
            - \c `EC_NOT_INITIALIZED`: Raised of the potentiometer object is not initialized.
            - \c `EC_BAD_REGISTER`: Raised if the supplied RDAC register exceeds the maximum value 
                                   (3).
            - \c `EC_BAD_WIPER_SETTING`: Raised if the wiper setting (`value`) eceeds the maximum 
                                        value (63 for AD5253(), 255 for AD5254())
    */

    // Errors: Object must be initialized and a valid RDAC must be passed.
    if(!initialized) {  return (err_code = EC_NOT_INITIALIZED); }
    if(RDAC > AD525x::max_RDAC_register) {  return (err_code = EC_BAD_REGISTER); }

    if (value > this->get_max_val()) {
        err_code = EC_BAD_WIPER_SETTING;
        return err_code;
    }

    uint8_t instr_addr = AD525x::RDAC_register | RDAC;
    err_code = write_data(instr_addr, value);
    return err_code;
}

uint8_t AD525x::read_RDAC(uint8_t RDAC) {
    /** Read the wiper setting from the specified RDAC register.

    Reads the current wiper setting of the RDAC register specified by `RDAC`. On error, this sets 
    error code and returns 0. Because 0 is a valid wiper setting, always check the error code via
    `get_error_code()` before trusting a 0 return value for this function. In addition to I2C errors
    raised by `read_data_byte()`, this function also raises `EC_BAD_REGISTER` if the supplied RDAC 
    register exceeds the maximum value (3).

    @param[in] RDAC     The address of one of the 4 RDAC registers (0-3), representing the 4 
                        potentiometers in the IC.

    @return Returns the wiper value or 0 on error (0 is also a valid wiper value).  I2C errors are
            raised indirectly via a call to `read_data_byte()`. This function also raises the 
            following error codes:
            - \c `EC_NOT_INITIALIZED`: Raised of the potentiometer object is not initialized.
            - \c `EC_BAD_REGISTER`: Raised if the supplied RDAC register exceeds the maximum value 
                                   (3).
    */
    if(!initialized) {
        err_code = EC_NOT_INITIALIZED;
        return 0;
    }

    if(RDAC > AD525x::max_RDAC_register) {
        err_code = EC_BAD_REGISTER;
        return 0;
    }

    uint8_t instr_addr = AD525x::RDAC_register | RDAC;

    uint8_t rv = read_data_byte(instr_addr);
    if(get_err_code() != 0) {
        return 0;       // Err code set in read_data already.
    }

    return rv;
}

uint8_t AD525x::write_EEMEM(uint8_t reg, uint8_t value) {
    /**   Write to the EEMEM non-volatile memory register. 

    Write to the EEMEM non-volatile memory register. The first 4 registers (0, 1, 2, 3) are the 
    RDAC storage registers. The remainder can store user data. There are 16 total registers.

    @param reg The register to which to write the EEMEM data. The first 4 are reserved for RDAC in
               that certain commands (such as `CMD_RESTORE_RDAC`) will automatically push/pull the
               RDAC values to/from these registers, though setting the RDAC-reserved EEMEM registers
               will not immediately set the RDAC registers.
    @param value The value to store in the register.

    @returns Returns 0 on no error. Raises I2C errors indirectly via a call to `write_data`. Also
             raises:
            - \c `EC_NOT_INITIALIZED`: Raised of the potentiometer object is not initialized.
            - \c `EC_BAD_WIPER_SETTING`: Register selected is an RDAC register and value exceeds
                                         maximum allowed RDAC value.
            - \c `EC_BAD_REGISTER`: An invalid register address was provided.
    */

    if (!initialized) { return (err_code =  EC_NOT_INITIALIZED);  }     // Must be initialized

    if (reg <= AD525x::max_RDAC_register && value < this->get_max_val()) {
        // Fairly sure the max value only applies to the RDAC registers, not the EEMEM.
       return (err_code = EC_BAD_WIPER_SETTING);
    }

    if(reg > AD525x::max_EEMEM_register) {  return (err_code = EC_BAD_REGISTER); }

    uint8_t instr_addr = AD525x::EEMEM_register | reg;

    err_code = write_data(instr_addr, value);
    return err_code;
}

uint8_t AD525x::read_EEMEM(uint8_t reg) {
    /** Read EEMEM value from register.

    Reads the value stored in the EEMEM value with the specified register address (`reg`). There are
    16 EEMEM registers, and the first 4 are reserved for storing RDAC values, in the sense that
    certain commands (e.g. `CMD_RESTORE_RDAC`) push/pull the values to/from these registers.

    On error, this sets the error code and returns 0. Since 0 is a valid wiper value, check
    `get_err_code()` if this returns 0. This raises I2C errors indirectly from a call to 
    `read_data_byte()`, and raises `EC_BAD_REGISTER` if an invalid register is passed to `reg`.
    
    @param[in] reg The EEMEM register whose value you want to query [0-15].

    @return Returns the value stored in the specified register or 0 on error. Check `get_err_code()`
            if return value is 0 to determine if an error has occurred. This raises I2C errors via
            a call to `read_data_byte()`, and also raises the following errors directly:

            - \c `EC_NOT_INITIALIZED`: Raised of the potentiometer object is not initialized.
            - \c `EC_BAD_REGISTER`: An invalid register address was provided.
    */

    if (!initialized) {
        err_code = EC_NOT_INITIALIZED;
        return 0;
    }

    if (reg > AD525x::max_EEMEM_register) {
        err_code = EC_BAD_REGISTER;
        return 0;
    }

    uint8_t instr_addr = AD525x::EEMEM_register | reg;

    uint8_t rv = read_data_byte(instr_addr);
    if (get_err_code() != 0) {
        return 0;       // Err code set in read_data already.
    }

    return rv;
}

float AD525x::read_tolerance(uint8_t RDAC) {
    /** Reads the RAB tolerance, written at the factory, in percentage (signed float).

    Reads the RAB tolerance programmed into the device at the factory for the specified RDAC 
    (`RDAC`), returning the percent tolerance as a signed float. These values will vary from device
    to device and register to register.

    This returns 0.0 on error, but since 0.0 is a valid output, in the event of a 0.0 return value,
    check `get_err_code()` to determine if an error has occurred. This raises I2C errors indirectly
    via calls to `read_data_byte()`, and raises `EC_BAD_REGISTER` if the provided `RDAC` value
    exceeds the maximum (3).

    @param[in] RDAC The RDAC register whose tolerance you would like to query.

    @return Returns the RAB tolerance set at the factory and stored in read-only memory on the chip,
            represented as a signed float as a percentage of the total device resistance. On error,
            returns 0 and sets err_code to a non-zero value. Check err_code in the event of a
            non-zero return value. Raises I2C errors indirectly through a call to `read_data_byte()`
            and directly raises:
            - \c `EC_NOT_INITIALIZED` Raised if the object has not been initialized.
            - \c `EC_BAD_REGISTER` Raised if an invalid register is passed to the function.
    */

    if (!initialized) {
        err_code = EC_NOT_INITIALIZED;
        return 0;
    }

    if (RDAC > AD525x::max_RDAC_register) {
        err_code = EC_BAD_REGISTER;
        return 0;
    }

    uint8_t sign_mask = 0x80;

    // Shift RDAC by 1, low bit is integer / decimal.
    uint8_t instr_addr = AD525x::Tolerance_register | (RDAC << 1);
    uint8_t instr_addr_int = instr_addr | AD525x::Tol_int;
    uint8_t instr_addr_dec = instr_addr | AD525x::Tol_dec;

    // 8-bit signed integer
    int8_t tol_int_data = (int8_t)read_data_byte(instr_addr_int);
    if(err_code) {
        return 0;
    }

    // Fractional portion - 8 bits - interpret as (value*2e-8)
    uint8_t tol_dec_data = read_data_byte(instr_addr_dec);
    if(err_code) {
        return 0;
    }

    // Convert to signed float - there may be a better way to do this, but this seems OK.
    float output;
    output = float(tol_int_data);
    
    float frac_portion = float(tol_dec_data)/256.0 * ((output < 0)?-1:1);
    return output + frac_portion;
}

//
//  Device commands
//

uint8_t AD525x::reset_device() {
    /** Return the device to idle state.

    @return Returns 0 on no error, otherwise returns the error code. In addition to errors raised
            indirectly through a call to `write_cmd()`, this also raises:
            - \c `EC_NOT_INITIALIZED`: Raised if communication has not been initialized.
    */
    if (!initialized) { return (err_code = EC_NOT_INITIALIZED); }

    return write_cmd(AD525x::CMD_NOP);
}

uint8_t AD525x::restore_RDAC(uint8_t RDAC) {
    /** Restore the wiper value for RDAC register `RDAC` from the EEMEM registers.
    
    Restores the wiper value for the specified `RDAC` register from the corresponding EEMEM 
    register.

    @param[in] RDAC The 2-bit register address specifying the RDAC to restore. [0-3]

    @return Returns 0 on no error, otherwise returns the error code. In addition to errors raised
            indirectly through a call to `write_cmd()`, this also raises:
            - \c `EC_NOT_INITIALIZED`: Raised if communication has not been initialized.
            - \c `EC_BAD_REGISTER`: Raised if `RDAC` exceeds 3.
    */

    if (!initialized) { return (err_code = EC_NOT_INITIALIZED); }
    if (RDAC > AD525x::max_RDAC_register) { return (err_code = EC_BAD_REGISTER); }

    return write_cmd(AD525x::CMD_Restore_RDAC | RDAC);
}

uint8_t AD525x::restore_all_RDAC() {
    /** Restores the wiper value for all RDAC registers from their corresponding EEMEM registers.

    @return Returns 0 on no error, otherwise returns the error code, I2C errors are raised 
            indirectly through a call to `write_cmd()`, and `EC_NOT_INITIALIZED` is raised if the
            object has not been initialized.
    */
    if (!initialized) { return (err_code = EC_NOT_INITIALIZED); }

    return write_cmd(AD525x::CMD_Restore_All_RDAC);
}

uint8_t AD525x::store_RDAC(uint8_t RDAC) {
    /** Store the current RDAC value for RDAC register `RDAC` in the EEMEM registers.
    
    Stores the wiper value for the specified `RDAC` register in the relevant EEMEM register.

    @param[in] RDAC The 2-bit register address specifying the RDAC to store. [0-3]

   @return Returns 0 on no error, otherwise returns the error code. In addition to errors raised
            indirectly through a call to `write_cmd()`, this also raises:
            - \c `EC_NOT_INITIALIZED`: Raised if communication has not been initialized.
            - \c `EC_BAD_REGISTER`: Raised if `RDAC` exceeds 3.
     */

    if (!initialized) { return (err_code = EC_NOT_INITIALIZED); }
    if (RDAC > AD525x::max_RDAC_register) { return (err_code = EC_BAD_REGISTER); }

    return write_cmd(AD525x::CMD_Store_RDAC | RDAC);
}

uint8_t AD525x::decrement_RDAC(uint8_t RDAC) {
    /** Decrements the current RDAC wiper value by 1.

    Takes the current wiper value for the potentiometer specified by `RDAC` and decrements its
    value by 1.

    @param[in] RDAC The 2-bit register address of the RDAC to decrement [0-3]

   @return Returns 0 on no error, otherwise returns the error code. In addition to errors raised
            indirectly through a call to `write_cmd()`, this also raises:
            - \c `EC_NOT_INITIALIZED`: Raised if communication has not been initialized.
            - \c `EC_BAD_REGISTER`: Raised if `RDAC` exceeds 3.
    */

    if (!initialized) { return (err_code = EC_NOT_INITIALIZED); }
    if (RDAC > AD525x::max_RDAC_register) { return (err_code = EC_BAD_REGISTER); }

    return write_cmd(AD525x::CMD_Dec_RDAC_step | RDAC);   
}

uint8_t AD525x::increment_RDAC(uint8_t RDAC) {
    /** Increments the current RDAC wiper value by 1.

    Takes the current wiper value for the potentiometer specified by `RDAC` and increments its
    value by 1.

    @param[in] RDAC The 2-bit register address of the RDAC to increment [0-3]

   @return Returns 0 on no error, otherwise returns the error code. In addition to errors raised
            indirectly through a call to `write_cmd()`, this also raises:
            - \c `EC_NOT_INITIALIZED`: Raised if communication has not been initialized.
            - \c `EC_BAD_REGISTER`: Raised if `RDAC` exceeds 3.
    */

    if (!initialized) { return (err_code = EC_NOT_INITIALIZED); }
    if (RDAC > AD525x::max_RDAC_register) { return (err_code = EC_BAD_REGISTER); }

    return write_cmd(AD525x::CMD_Inc_RDAC_step | RDAC);   
}

uint8_t AD525x::decrement_RDAC_6dB(uint8_t RDAC) {
    /** Decrements the current RDAC wiper value for the specified RDAC by 6dB

    Takes the current RDAC wiper value for the potentiometer specified by `RDAC` and decrements it
    by 6dB (i.e. cuts it in half).

    @param[in] RDAC The 2-bit register address of the RDAC to decrement [0-3]

   @return Returns 0 on no error, otherwise returns the error code. In addition to errors raised
            indirectly through a call to `write_cmd()`, this also raises:
            - \c `EC_NOT_INITIALIZED`: Raised if communication has not been initialized.
            - \c `EC_BAD_REGISTER`: Raised if `RDAC` exceeds 3.
    */

    if (!initialized) { return (err_code = EC_NOT_INITIALIZED); }
    if (RDAC > AD525x::max_RDAC_register) { return (err_code = EC_BAD_REGISTER); }

    return write_cmd(AD525x::CMD_Dec_RDAC_6dB | RDAC);   
}

uint8_t AD525x::increment_RDAC_6dB(uint8_t RDAC) {
    /** Increments the current RDAC wiper value for the specified RDAC by 6dB

    Takes the current RDAC wiper value for the potentiometer specified by `RDAC` and increments it
    by 6dB (i.e. doubles it)

    @param[in] RDAC The 2-bit register address of the RDAC to increment [0-3]

   @return Returns 0 on no error, otherwise returns the error code. In addition to errors raised
            indirectly through a call to `write_cmd()`, this also raises:
            - \c `EC_NOT_INITIALIZED`: Raised if communication has not been initialized.
            - \c `EC_BAD_REGISTER`: Raised if `RDAC` exceeds 3.
    */

    if (!initialized) { return (err_code = EC_NOT_INITIALIZED); }
    if (RDAC > AD525x::max_RDAC_register) { return (err_code = EC_BAD_REGISTER); }

    return write_cmd(AD525x::CMD_Inc_RDAC_6dB | RDAC);   
}

uint8_t AD525x::decrement_all_RDAC() {
    /** Decrements the current wiper value for all RDAC potentiometers by 1.

    Takes the current wiper value for all RDAC potentiometers and decrements them by 1.

   @return Returns 0 on no error, otherwise returns the error code. In addition to errors raised
            indirectly through a call to `write_cmd()`, this also raises:
            - \c `EC_NOT_INITIALIZED`: Raised if communication has not been initialized.
    */
    if (!initialized) { return (err_code = EC_NOT_INITIALIZED); }

    return write_cmd(AD525x::CMD_Dec_All_RDAC_step);   
}

uint8_t AD525x::increment_all_RDAC() {
    /** Increments the current wiper value for all RDAC potentiometers by 1.

    Takes the current wiper value for all RDAC potentiometers and increments them by 1.

   @return Returns 0 on no error, otherwise returns the error code. In addition to errors raised
            indirectly through a call to `write_cmd()`, this also raises:
            - \c `EC_NOT_INITIALIZED`: Raised if communication has not been initialized.
    */
    if (!initialized) { return (err_code = EC_NOT_INITIALIZED); }

    return write_cmd(AD525x::CMD_Inc_All_RDAC_step);   
}

uint8_t AD525x::decrement_all_RDAC_6dB() {
    /** Decrements the current wiper value for all RDAC potentiometers by 6dB

    Takes the current wiper value for all RDAC potentiometers and decrements them by 6dB 
    (i.e. cuts them in half).

   @return Returns 0 on no error, otherwise returns the error code. In addition to errors raised
            indirectly through a call to `write_cmd()`, this also raises:
            - \c `EC_NOT_INITIALIZED`: Raised if communication has not been initialized.
    */
    if (!initialized) { return (err_code = EC_NOT_INITIALIZED); }

    return write_cmd(AD525x::CMD_Dec_All_RDAC_6dB);   
}

uint8_t AD525x::increment_all_RDAC_6dB() {
    /** Increments the current wiper value for all RDAC potentiometers by 6dB

    Takes the current wiper value for all RDAC potentiometers and increments them by 6dB 
    (i.e. doubles them).

   @return Returns 0 on no error, otherwise returns the error code. In addition to errors raised
            indirectly through a call to `write_cmd()`, this also raises:
            - \c `EC_NOT_INITIALIZED`: Raised if communication has not been initialized.
    */
    if (!initialized) { return (err_code = EC_NOT_INITIALIZED); }

    return write_cmd(AD525x::CMD_Inc_All_RDAC_6dB);   
}

//
// Getter function for maximum value - not implemented for AD525x
//

uint8_t AD5253::get_max_val() {
    /** Retrieve the maximum value of the wiper.

    The AD5253 and AD5254 differ only in their maximum wiper value, [0, 64) for AD5253 and [0, 256) 
    for AD5254. This function is an interface that should return the maximum wiper value when called
    from the child class objects. See `AD525x::get_max_val()`.

    @return Returns the value stored in `AD5253::max_val`. (63)
    */

    return AD5253::max_val;
}

uint8_t AD5254::get_max_val() {
    /** Retrieve the maximum value of the wiper.

    The AD5253 and AD5254 differ only in their maximum wiper value, [0, 64) for AD5253 and [0, 256) 
    for AD5254. This function is an interface that should return the maximum wiper value when called
    from the child class objects. See `AD525x::get_max_val()`.

    @return Returns the value stored in `AD5254::max_val`. (255)
    */

    return AD5254::max_val;
}

//
// Error handling
//

uint8_t AD525x::get_err_code() {
    /** Retrieve the error code stored in a private variable.

    Retrieves the error code stored in the private `err_code` variable. If non-zero, use 
    `AD525xGetErrorString()` from `AD525x_ErrorStrings.h` to retrieve the human-readable error
    string, or see  `AD525x_Errors.h` for details.

    @return Returns the error code set in the current object. Non-zero value is an error.
    */
    return err_code;
}

//
// Private functions: General I2C communications.
//
uint8_t AD525x::write_cmd(uint8_t cmd_register) {
    /** Trigger a command in the command register (no data)

    This triggers a command in the command register (e.g. an address which takes no data). This is
    a private function, called by exposed functions hard-coded with the command register addresses.

    @param cmd_register The address of the command register (e.g. `CMD_NOP`, `CMD_Store_RDAC`)

    @return Returns 0 on no error, otherwise returns I2C errors:
            - \c `EC_NO_ERR`: No error.
            - \c `EC_DATA_LONG`: Data too long to fit in transmit buffer
            - \c `EC_NACK_ADDR`: Received NACK on transmit of address.
            - \c `EC_NACK_DATA`: Received NACK on transmit of data.
            - \c `EC_I2C_OTHER`: Other I2C error.
    */
    Wire.beginTransmission(dev_addr);
    Wire.write(cmd_register);

    err_code = Wire.endTransmission();
    return err_code;

}

uint8_t AD525x::write_data(uint8_t register_addr, uint8_t data) {
    /** Writes data to the specified register address.

    This is a private function, called by specific-use functions such as `write_RDAC()` and 
    `write_EEMEM()` to write data (specified by `data`) into the register specified by 
    `register_addr` using the Wire.h library.

    @param register_addr The register address to query.
    @param data The data to write to the specified address.

    @return Returns 0 on no error, otherwise returns I2C errors:
            - \c `EC_NO_ERR`: No error.
            - \c `EC_DATA_LONG`: Data too long to fit in transmit buffer
            - \c `EC_NACK_ADDR`: Received NACK on transmit of address.
            - \c `EC_NACK_DATA`: Received NACK on transmit of data.
            - \c `EC_I2C_OTHER`: Other I2C error.
    */
    Wire.beginTransmission(dev_addr);
    Wire.write(register_addr);
    Wire.write(data);
    err_code = Wire.endTransmission();
    return err_code;
}

uint8_t * AD525x::read_data(uint8_t register_addr, uint8_t length) {
    /** Reads data of length `length` from register  `register_addr`
    
    This is a private function, called by specific-use functions such as `read_RDAC()` and 
    `read_EEMEM()` to read a data array of length `length` (in bytes) from the register specified by
    `register_addr`.

    @param register_addr The address of the register to read from.
    @param length The length of the data stored in the register.

    @return On success, returns a pointer to an array of `uint8_t` unsigned bytes retrieved from the
            register. The returned array has length `length`, which in most cases will be 1. On 
            error, this function returns a `NULL` pointer and sets `err_code` (query 
            `get_err_code()` to get the value of this variable) to one of the I2C errors:
            - \c `EC_NO_ERR`: No error.
            - \c `EC_DATA_LONG`: Data too long to fit in transmit buffer
            - \c `EC_NACK_ADDR`: Received NACK on transmit of address.
            - \c `EC_NACK_DATA`: Received NACK on transmit of data.
            - \c `EC_I2C_OTHER`: Other I2C error.
    */
    
    uint8_t error = 0;

    Wire.beginTransmission(dev_addr);
    Wire.write(register_addr);
    err_code = Wire.endTransmission();
    if(err_code > 0) {
        return NULL;
    }

    Wire.beginTransmission(dev_addr);
    uint8_t n_bytes = Wire.requestFrom(dev_addr, length);

    if(n_bytes != length) {
        err_code = EC_BAD_READ_SIZE;
        return NULL;
    }

    uint8_t buff[length];
    if(Wire.available() == length) {
        for(int i = 0; i < length; i++) {
            buff[i] = Wire.read();
        }
    }

    err_code = Wire.endTransmission();
    if(err_code > 0) {
        return NULL;
    }

    return buff;
}

uint8_t AD525x::read_data_byte(uint8_t register_addr) {
    /** Reads a single byte from the specified register. Convenience wrapper for `read_data()`.

    This reads a single byte from the register specified at `register_addr` via a call to
    `read_data()`. In most cases, only a single byte is stored at the specified register address.

    @param[in] register_addr The register address from which to read a single byte.
    
    @return Returns the requested value. If there is an error, returns 0 and `err_code` (retrieved
    via `get_err_code()`) is set to a non-zero value. This is a simple wrapper for `read_data()`, so
    it raises only the errors raised by that function.
    */

    uint8_t *rv = read_data(register_addr, 1);
    if(rv == NULL) {
        return 0;       // Err code set in read_data already.
    }

    return rv[0];
}