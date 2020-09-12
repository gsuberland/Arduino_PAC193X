#ifndef PAC193X_ARDUINO_LIBRARY_h
#define PAC193X_ARDUINO_LIBRARY_h


#include "Arduino.h"
#include "Wire.h"


#ifdef PAC193X_BIG_ENDIAN_PLATFORM
#define PAC193X_NEED_ENDIAN_SWAP false
#else
#define PAC193X_NEED_ENDIAN_SWAP true
#endif


/* Returns a NotConfigured status if isConfigured is set to false. */
#define PAC193X_RETURN_IF_NOT_CONFIGURED { if (!this->isConfigured) return PAC193X_STATUS::NotConfigured; }
/* Checks if isConfigured is set to false, and if so sets the status parameter value to NotConfigured and returns. */
#define PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED { if (!this->isConfigured) { if (status != nullptr) { *status = PAC193X_STATUS::NotConfigured; } return 0; } }
#define PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED_RV(returnVal) { if (!this->isConfigured) { if (status != nullptr) { *status = PAC193X_STATUS::NotConfigured; } return returnVal; } }


#define PAC193X_REFRESH_WAIT_TIME_US    1050    /* This value represents the amount of time, in microseconds, that must be waited between issuing a refresh command and attempting to read from a value register. */

/*      Register                        Addr    Description */
#define PAC193X_REFRESH_CMD_ADDR        0x00    /* REFRESH command. Write any byte value to this address to execute a REFRESH command.
                                                   The REFRESH command performs a measurement acquisition and resets all accumulator & counts. This command takes 1ms to take effect. */
#define PAC193X_CTRL_ADDR               0x01    /* Control bitfield (see datasheet for full details):
                                                 * [7:6]   SAMPLE RATE:    00b = 1024/s, 01b = 256/s, 10b = 64/s, 11b = 8/s
                                                 * [5]     SLEEP:          Sleep mode. When set to 1, device enters sleep mode (stops sampling) after REFRESH command. Disabled by default.
                                                 * [4]     SINGLE:         Single-shot measurement mode. When set to 1, sampling only occurs when REFRESH command issued. Disabled by default.
                                                 * [3]     ALERT PIN:      Enable (1) or disable (0) the SLOW/~ALERT pin. When enabled, the ALERT CC and OVF ALERT bits specify function. Disabled by default.
                                                 * [2]     ALERT CC:       Conversion complete alert. When set to 1 and ALERT pin is enabled (bit 3), asserts the ALERT pin for 5us at end of each conversion cycle. Disabled by default.
                                                 * [1]     OVF ALERT:      Overflow alert. When set to 1 and ALERT pin is enabled (bit 3), the ALERT pin is asserted when an accumulator overflows. This has precedence over bit 2. Disabled by default.
                                                 * [0]     OVF:            Overflow indicator. This bit is set when an accumulator or accumulator counter overflows. This state is cleared by REFRESH and REFRESH_G commands.
                                                */
#define PAC193X_ACC_COUNT_ADDR          0x02    /* Accumulator count: 24-bit value, incremented every time a power result has been summed by the accumulator. */
#define PAC193X_VPOWER1_ACC_ADDR        0x03    /* (SENSE1) Power accumulator: 48-bit value (signed/unsigned depending on polar/unipolar Vbus/Vsense setup), sum of the power values accumulated so far. */
#define PAC193X_VPOWER2_ACC_ADDR        0x04    /* (SENSE2) See above */
#define PAC193X_VPOWER3_ACC_ADDR        0x05    /* (SENSE3) See above */
#define PAC193X_VPOWER4_ACC_ADDR        0x06    /* (SENSE4) See above */
#define PAC193X_VBUS1_ADDR              0x07    /* (SENSE1) Last Vbus reading: 16-bit value (signed/unsigned depending on polar/unipolar Vbus/Vsense setup), most recent value from Vbus sampling. */
#define PAC193X_VBUS2_ADDR              0x08    /* (SENSE2) See above */
#define PAC193X_VBUS3_ADDR              0x09    /* (SENSE3) See above */
#define PAC193X_VBUS4_ADDR              0x0A    /* (SENSE4) See above */
#define PAC193X_VSENSE1_ADDR            0x0B    /* (SENSE1) Last Vsense reading: 16-bit value (signed/unsigned depending on polar/unipolar Vbus/Vsense setup), most recent value from Vsense sampling. */
#define PAC193X_VSENSE2_ADDR            0x0C    /* (SENSE2) See above */
#define PAC193X_VSENSE3_ADDR            0x0D    /* (SENSE3) See above */
#define PAC193X_VSENSE4_ADDR            0x0E    /* (SENSE4) See above */
#define PAC193X_VBUS1_AVG_ADDR          0x0F    /* (SENSE1) Average Vbus reading: 16-bit value (signed/unsigned depending on polar/unipolar Vbus/Vsense setup), average of last 8 Vbus samples. */
#define PAC193X_VBUS2_AVG_ADDR          0x10    /* (SENSE2) See above */
#define PAC193X_VBUS3_AVG_ADDR          0x11    /* (SENSE3) See above */
#define PAC193X_VBUS4_AVG_ADDR          0x12    /* (SENSE4) See above */
#define PAC193X_VSENSE1_AVG_ADDR        0x13    /* (SENSE1) Average Vsense reading: 16-bit value (signed/unsigned depending on polar/unipolar Vbus/Vsense setup), average of last 8 Vsense samples. */
#define PAC193X_VSENSE2_AVG_ADDR        0x14    /* (SENSE2) See above */
#define PAC193X_VSENSE3_AVG_ADDR        0x15    /* (SENSE3) See above */
#define PAC193X_VSENSE4_AVG_ADDR        0x16    /* (SENSE4) See above */
#define PAC193X_VPOWER1_ADDR            0x17    /* (SENSE1) Power reading: 24-bit value (signed if either Vbus or Vsense are signed, otherwise unsigned), product of Vbus and Vsesne with improved internal precision. NOTE: Top 3 bits are always zero, take care when doing two's complement. */
#define PAC193X_VPOWER2_ADDR            0x18    /* (SENSE2) See above */
#define PAC193X_VPOWER3_ADDR            0x19    /* (SENSE3) See above */
#define PAC193X_VPOWER4_ADDR            0x1A    /* (SENSE4) See above */
#define PAC193X_CHANNEL_DIS_ADDR        0x1C    /* Channel disable and SMBUS config bitfield:
                                                 * [7-4]    CHn_OFF:    Channel disables for CH1 through CH4. Setting a 1 disables the channel. These are set to 1 in the factory for channels that do not exist in PAC1931/2/3.
                                                 * [3]      TIMEOUT:    Timeout enable bit. If set, the SMBus timeout feature is enabled. Disabled by default.
                                                 * [2]      BYTE COUNT: When set to 1, causes byte count data to be included in the SMBus Block Read command for each register read. Disabled by default.
                                                 * [1]      NO SKIP:    Controls auto-increment on the read address pointer. 0 skips over registers for inactive (disabled) channels. 1 does not skip.
                                                 * [0]      Unimplemented. Always zero.
                                                */
#define PAC193X_NEG_PWR_ADDR            0x1D    /* Negative power measurement config bitfield:
                                                 * [7-4]    CHn_BIDI:   Enable/disable bidirectional (bipolar) measurement of current for CH1 through CH4. When enabled, this channel's current measurements will be returned as two's complement signed integers.
                                                                        When enabled, Vsense has -100mV to +100mV range. When disabled, Vsense has 0mV to +100mV range.
                                                 * [3-0]    CHn_BIDV:   Enable/disable bidirectional (bipolar) measurement of voltage for CH1 through CH4. When enabled, this channel's voltage measurements will be returned as two's complement signed integers.
                                                                        When enabled, Vbus has -32V to +32V range. When disabled, Vbus has 0V to +32V range.
                                                */
#define PAC193X_REFRESH_G_CMD_ADDR      0x1E    /* REFRESH_G command. Write any byte value to this address to execute a REFRESH_G command. 
                                                   The REFRESH_G command is identical to the REFRESH command but is intended to be used with the GENERAL CALL command. This command takes 1ms to take effect. */
#define PAC193X_REFRESH_V_CMD_ADDR      0x1F    /* REFRESH_V command. Write any byte value to this address to execute a REFRESH_V command. 
                                                   The REFRESH_V command is similar to the REFRESH command but it does not reset accumulators or counts. This command takes 1ms to take effect. */
#define PAC193X_SLOW_ADDR               0x20    /* SLOW control and status bitfield:
                                                 * [7]      SLOW:       This bit is set to 0 if the SLOW pin is pulled low externally, or 1 if it is pulled high.
                                                 * [6]      SLOW-LH:    Set to 1 if the SLOW pin has transitioned low to high since the last REFRESH command.
                                                 * [5]      SLOW-HL:    Set to 1 if the SLOW pin has transitioned high to low since the last REFRESH command.
                                                 * [4]      R_RISE:     When set to 1, enables a limited REFRESH function on the rising edge of the SLOW pin.
                                                 * [3]      R_V_RISE:   When set to 1, enables a limited REFRESH_V function on the rising edge of the SLOW pin.
                                                 * [2]      R_FALL:     When set to 1, enables a limited REFRESH function on the falling edge of the SLOW pin.
                                                 * [1]      R_V_FALL:   When set to 1, enables a limited REFRESH_V function on the falling edge of the SLOW pin.
                                                 * [0]      POR:        Power on reset status bit.
                                                */
#define PAC193X_CTRL_ACT_ADDR           0x21    /* Actual control bitfield. This is a read-only copy of the CTRL register that represents the values that are currently in use. */
#define PAC193X_CHANNEL_DIS_ACT_ADDR    0x22    /* Actual channel disable. This is a read-only copy of the CHANNEL_DIS register that represents the values that are currently in use. */
#define PAC193X_NEG_PWR_ACT_ADDR        0x23    /* Actual negative power measurement config. This is a read-only copy of the NEG_PWR register that represents the values that are currently in use. */
#define PAC193X_CTRL_LAT_ADDR           0x24    /* Previous control bitfield. This is a read-only copy of the value of the CTRL register that was in use before the last REFRESH, REFRESH_V, or REFRESH_G command. */
#define PAC193X_CHANNEL_DIS_LAT_ADDR    0x25    /* Previous channel disable. This is a read-only copy of the value of the CHANNEL_DIS register that was in use before the last REFRESH, REFRESH_V, or REFRESH_G command. */
#define PAC193X_NEG_PWR_LAT_ADDR        0x26    /* Previous negative power measurement config. This is a read-only copy of the value of the NEG_PWR register that was in use before the last REFRESH, REFRESH_V, or REFRESH_G command. */
#define PAC193X_PRODUCT_ID_ADDR         0xFD    /* Product ID register: 01010000b for PAC1931, 01010001b for PAC1932, 01010010b for PAC1933, 01010011b for PAC1934. */
#define PAC193X_MANUFACTURER_ID_ADDR    0xFE    /* Manufacturer ID register: Returns a constant value of 0x5D */
#define PAC193X_REVISION_ID_ADDR        0xFF    /* Revision ID register: Silicon die revision. Should return 3, as per Rev.E (Aug 2019) datasheet. */


/* Manufacturer ID for Microchip */
#define PAC193X_MANUFACTURER_ID_MICROCHIP 0x5D


/* Product ID for PAC1931 */
#define PAC1931_PRODUCT_ID      0b01010000
/* Product ID for PAC1932 */
#define PAC1932_PRODUCT_ID      0b01010001
/* Product ID for PAC1933 */
#define PAC1933_PRODUCT_ID      0b01010010
/* Product ID for PAC1934 */
#define PAC1934_PRODUCT_ID      0b01010011

/* PAC193X product IDs */
enum PAC193X_PRODUCT : uint8_t
{
    PAC1931 = PAC1931_PRODUCT_ID,
    PAC1932 = PAC1932_PRODUCT_ID,
    PAC1933 = PAC1933_PRODUCT_ID,
    PAC1934 = PAC1934_PRODUCT_ID,
    VALUE_MIN = PAC1931,
    VALUE_MAX = PAC1934,
    INVALID = 0xFF
};


// macro to build the CTRL register bitfield value from components
#define PAC193X_MAKE_CTRL_REG(ctrl_sample_rate, ctrl_sleep, ctrl_sing, ctrl_alert_pin, ctrl_alert_cc, ctrl_ovf_alert) \
    ((uint8_t)( \
        (((uint8_t)ctrl_sample_rate & 0b11) << 6) | \
        (((ctrl_sleep == 0) ? 0 : 1) << 5) | \
        (((ctrl_sing == 0) ? 0 : 1) << 4) | \
        (((ctrl_alert_pin == 0) ? 0 : 1) << 3) | \
        (((ctrl_alert_cc == 0) ? 0 : 1) << 2) | \
        (((ctrl_ovf_alert == 0) ? 0 : 1) << 1) | \
    ))


// macro to build the SLOW register bitfield value from components
#define PAC193X_MAKE_SLOW_REG(slow_rise, slow_fall, slow_refresh_rise, slow_refresh_fall) \
    ((uint8_t)( \
        (((slow_rise == 0) ? 0 : 1) << 4) | \
        (((slow_fall == 0) ? 0 : 1) << 2) | \
        (((slow_refresh_rise == 0) ? 0 : 1) << 3) | \
        (((slow_refresh_fall == 0) ? 0 : 1) << 1) \
    ))


enum PAC193X_STATUS : uint8_t
{
    OK,

    InvalidDeviceAddress,
    InvalidResistorValue,
    InvalidSampleRate,
    InvalidChannelIndex,
    InvalidArgument,

    NotConfigured,

    UnknownManufacturerID,
    UnknownProductID,

    WriteError,
    ReadError,
    ReadTimeout,

    WaitingForRefresh,

    ChannelDisabled,
};

#define PAC193X_STATUS_OK(sv) (sv == PAC193X_STATUS::OK)


enum PAC193X_SAMPLE_RATE : uint8_t
{
    RATE_1024   = 0b00,
    RATE_256    = 0b01,
    RATE_64     = 0b10,
    RATE_8      = 0b11,
    VALUE_MIN   = RATE_1024,
    VALUE_MAX   = RATE_8,
};


#define PAC193X_ADDRESS_MASK_ZEROS ((uint8_t)0b11000001)    /* This mask represents which bits MUST be 0 in the device address. The top 2 bits are always zero, and the bottom bit is a R/W status bit that should not be used. */
#define PAC193X_ADDRESS_MASK_ONES  ((uint8_t)0b00100000)    /* This mask represents which bits MUST be 1 in the device address. Bit 5 must be set to 1 as per the datasheet. */


const uint8_t PAC193X_VBUS_CHANNELS[4] = { PAC193X_VBUS1_ADDR, PAC193X_VBUS2_ADDR, PAC193X_VBUS3_ADDR, PAC193X_VBUS4_ADDR };
const uint8_t PAC193X_VBUS_AVG_CHANNELS[4] = { PAC193X_VBUS1_AVG_ADDR, PAC193X_VBUS2_AVG_ADDR, PAC193X_VBUS3_AVG_ADDR, PAC193X_VBUS4_AVG_ADDR };
const uint8_t PAC193X_VSENSE_CHANNELS[4] = { PAC193X_VSENSE1_ADDR, PAC193X_VSENSE2_ADDR, PAC193X_VSENSE3_ADDR, PAC193X_VSENSE4_ADDR };
const uint8_t PAC193X_VSENSE_AVG_CHANNELS[4] = { PAC193X_VSENSE1_AVG_ADDR, PAC193X_VSENSE2_AVG_ADDR, PAC193X_VSENSE3_AVG_ADDR, PAC193X_VSENSE4_AVG_ADDR };
const uint8_t PAC193X_VPOWER_CHANNELS[4] = { PAC193X_VPOWER1_ADDR, PAC193X_VPOWER2_ADDR, PAC193X_VPOWER3_ADDR, PAC193X_VPOWER4_ADDR };


class PAC193X
{   
    public:
        PAC193X();
        ~PAC193X();

        PAC193X_STATUS begin(uint8_t address, uint32_t shuntResistanceMicroOhm, PAC193X_SAMPLE_RATE sampleRate);

        /* Returns the product ID from the PRODUCT_ID register. The status of the command is returned via the status parameter if null is not passed. */
        uint8_t getProductID(PAC193X_STATUS* status);
        /* Returns the product type as a PAC193X_PRODUCT enum member. This is identical to getProductID, but it returns an enum instead. The status of the command is returned via the status parameter if null is not passed. */
        PAC193X_PRODUCT getProduct(PAC193X_STATUS* status);
        /* Returns the manufacturer ID from the MANUFACTURER_ID register. The status of the command is returned via the status parameter if null is not passed. */
        uint8_t getManufacturerID(PAC193X_STATUS* status);
        /* Returns the part revision ID from the REVISION_ID register. The status of the command is returned via the status parameter if null is not passed. */
        uint8_t getRevisionID(PAC193X_STATUS* status);

        /* Returns the latest voltage measured on the specified channel (0-3). The status of the command is returned via the status parameter if null is not passed. */
        double getVoltageLast(uint8_t channelIndex, PAC193X_STATUS* status);
        /* Returns the rolling average voltage (over 8 readings) measured on the specified channel (0-3). The status of the command is returned via the status parameter if null is not passed. */
        double getVoltageAverage(uint8_t channelIndex, PAC193X_STATUS* status);
        /* Returns the latest current measured on the specified channel (0-3). The status of the command is returned via the status parameter if null is not passed. */
        double getCurrentLast(uint8_t channelIndex, PAC193X_STATUS* status);
        /* Returns the rolling average current (over 8 readings) measured on the specified channel (0-3). The status of the command is returned via the status parameter if null is not passed. */
        double getCurrentAverage(uint8_t channelIndex, PAC193X_STATUS* status);
        /* Returns an accurate power value (voltage multiplied by current) measured on the specified channel (0-3). The status of the command is returned via the status parameter if null is not passed. */
        double getPower(uint8_t channelIndex, PAC193X_STATUS* status);
        /* Returns the accumulated power for the specified channel (0-3) since last reset or overflow. If the overflow parameter is not null, it is used to return the status of the accumulator overflow flag. The status of the command is returned via the status parameter if null is not passed. */
        double getPowerAccumulated(uint8_t channelIndex, bool* overflow, PAC193X_STATUS* status);
        /* Returns the number of times a reading has been added to the accumulator, for all channels, since last reset or overflow. */
        uint32_t getAccumulatorCount(PAC193X_STATUS* status);

        /* Begin a refresh operation without resetting accumulators or counts. This corresponds to the REFRESH_V command. Returns a WaitingForRefresh status if a refresh was already pending. */
        PAC193X_STATUS refreshAsync();
        /* Begin a refresh operation and reset accumulators & counts. This corresponds to the REFRESH command. Returns a WaitingForRefresh status if a refresh was already pending. */
        PAC193X_STATUS refreshAndResetAsync();
        /* Blocks until the refresh operation completes. */
        void waitForRefresh();

        /* Perform a refresh operation without resetting accumulators or counts. This corresponds to the REFRESH_V command. This function blocks until the refresh operation has completed. Returns a WaitingForRefresh status if a refresh was already pending. */
        PAC193X_STATUS refreshSync();
        /* Perform a refresh operation and reset accumulators & counts. This corresponds to the REFRESH command. This function blocks until the refresh operation has completed. Returns a WaitingForRefresh status if a refresh was already pending. */
        PAC193X_STATUS refreshAndResetSync();

        /* Returns the number of physical channels for this particular device, based on the product ID. */
        uint8_t getPhysicalChannelCount();
        /* Returns true if the channel at the specified index (0-3) is enabled, otherwise false. The status of the command is returned via the status parameter if null is not passed. */
        bool isChannelEnabled(uint8_t index, PAC193X_STATUS* status);
        /* Sets whether or not the channel at the specified index (0-3) is enabled. */
        PAC193X_STATUS setChannelEnabled(uint8_t index, bool enable);

    private:
        bool isConfigured;
        uint8_t deviceAddress;
        uint32_t shuntResistance;
        uint32_t timeOfLastRefreshCommand;

        /* Returns true if the device address if of the correct format (see PAC193X_ADDRESS_MASK_xxx defines and datasheet for info) */
        bool ValidateDeviceAddress(uint8_t address);
        /* Returns true if the given shunt resistance value is within a sensible range. */
        bool ValidateShuntResistance(uint32_t resistance);
        /* Returns true if the sample rate is a known, defined value. */
        bool ValidateSampleRate(PAC193X_SAMPLE_RATE sampleRate);

        /* Reads the manufacturer ID from the chip and returns true if it is valid. */
        bool ValidateManufacturer();
        /* Reads the product ID from the chip and returns true if it is valid. */
        bool ValidateProduct();

        PAC193X_STATUS ReadRegister(uint8_t registerAddress, uint8_t byteCount, uint8_t* buffer, bool reverse);
        PAC193X_STATUS WriteRegister(uint8_t registerAddress, uint8_t value);

        /* Register read helpers */
        uint8_t Read8(uint8_t registerAddress, PAC193X_STATUS* status);
        uint16_t Read16(uint8_t registerAddress, PAC193X_STATUS* status);
        int16_t ReadSigned16(uint8_t registerAddress, PAC193X_STATUS* status);
        uint32_t Read24(uint8_t registerAddress, PAC193X_STATUS* status);
        int32_t ReadSigned24(uint8_t registerAddress, PAC193X_STATUS* status);
        uint32_t Read32(uint8_t registerAddress, PAC193X_STATUS* status);
        int32_t ReadSigned32(uint8_t registerAddress, PAC193X_STATUS* status);
        uint64_t Read48(uint8_t registerAddress, PAC193X_STATUS* status);
        int64_t ReadSigned48(uint8_t registerAddress, PAC193X_STATUS* status);
        uint64_t Read64(uint8_t registerAddress, PAC193X_STATUS* status);
        int64_t ReadSigned64(uint8_t registerAddress, PAC193X_STATUS* status);

        /* Returns true if a refrehs operation is currently pending, otherwise false. */
        bool IsRefreshPending();
        /* Blocks and waits for the last REFRESH[_V|_G] command to complete. The wait time is configured via PAC193X_REFRESH_WAIT_TIME_US (default is 1.05ms) and is tracked via timeOfLastRefreshCommand. */
        void WaitForRefresh();

        PAC193X_STATUS Command_REFRESH_Internal(uint8_t commandAddress);
        PAC193X_STATUS Command_REFRESH();
        PAC193X_STATUS Command_REFRESH_V();
        PAC193X_STATUS Command_REFRESH_G();

        PAC193X_STATUS MeasureInternal(bool resetAccumulators);
};


#endif
