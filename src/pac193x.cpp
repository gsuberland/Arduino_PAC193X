/* 
 * Arduino library for Microchip PAC193X series power monitors.
 * Copyright (c) 2020 Graham Sutherland
 * 
 * This software is released under MIT license.
 * Please see the LICENSE file for the full text.
 *
 */

#include "PAC193X.h"
#include <math.h>


PAC193X::PAC193X()
{
    this->deviceAddress = 0;
    for (int n = 0; n < PAC193X_MAX_CHANNELS; n++)
    {
        this->shuntResistances[n] = 0;
    }
    this->isConfigured = false;
}


PAC193X::~PAC193X()
{
    isConfigured = false;
}


PAC193X_STATUS PAC193X::begin(uint8_t address, uint32_t shuntResistanceMicroOhm, PAC193X_SAMPLE_RATE sampleRate = PAC193X_SAMPLE_RATE::RATE_1024)
{
    uint32_t resistances[PAC193X_MAX_CHANNELS];
    for (int n = 0; n < PAC193X_MAX_CHANNELS; n++)
    {
        this->shuntResistances[n] = shuntResistanceMicroOhm;
    }
    return begin(address, resistances, sampleRate);
}


PAC193X_STATUS PAC193X::begin(uint8_t address, uint32_t shuntResistancesMicroOhm[PAC193X_MAX_CHANNELS], PAC193X_SAMPLE_RATE sampleRate = PAC193X_SAMPLE_RATE::RATE_1024)
{
    if (!ValidateDeviceAddress(address))
        return PAC193X_STATUS::InvalidDeviceAddress;

    for (int n = 0; n < PAC193X_MAX_CHANNELS; n++)
    {
        if (!ValidateShuntResistance(shuntResistancesMicroOhm[n]))
            return PAC193X_STATUS::InvalidResistorValue;
    }

    if (!ValidateSampleRate(sampleRate))
        return PAC193X_STATUS::InvalidSampleRate;

    this->deviceAddress = address;
    for (int n = 0; n < PAC193X_MAX_CHANNELS; n++)
    {
        this->shuntResistances[n] = shuntResistancesMicroOhm[n];
    }
    
    this->isConfigured = true;

    return PAC193X_STATUS::NoError;
}


uint8_t PAC193X::getProductID(PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    return Read8(PAC193X_PRODUCT_ID_ADDR, status);
}


PAC193X_PRODUCT PAC193X::getProduct(PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED_RV(PAC193X_PRODUCT::INVALID);

    return (PAC193X_PRODUCT)Read8(PAC193X_PRODUCT_ID_ADDR, status);
}


uint8_t PAC193X::getManufacturerID(PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    return Read8(PAC193X_MANUFACTURER_ID_ADDR, status);
}


uint8_t PAC193X::getRevisionID(PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    return Read8(PAC193X_REVISION_ID_ADDR, status);
}


double PAC193X::getVoltageLast(uint8_t channelIndex, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    return ReadVoltageResult(channelIndex, PAC193X_VBUS_CHANNELS, status);
}


double PAC193X::getVoltageAverage(uint8_t channelIndex, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    return ReadVoltageResult(channelIndex, PAC193X_VBUS_AVG_CHANNELS, status);
}


double PAC193X::getCurrentLast(uint8_t channelIndex, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    return ReadCurrentResult(channelIndex, PAC193X_VSENSE_CHANNELS, status);
}


double PAC193X::getCurrentAverage(uint8_t channelIndex, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    return ReadCurrentResult(channelIndex, PAC193X_VSENSE_AVG_CHANNELS, status);
}


double PAC193X::getPower(uint8_t channelIndex, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    if (channelIndex > PAC193X_MAX_CHANNELS - 1)
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::InvalidChannelIndex);
        return 0;
    }
    if (!isChannelEnabled(channelIndex, status))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::ChannelDisabled);
        return 0;
    }
    if (IsRefreshPending())
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::WaitingForRefresh);
        return 0;
    }

    // power polarity is bipolar if either the voltage or current measurements are configured as bipolar.
    // otherwise power is unipolar.

    // find out if the channel voltage measurement is bipolar (i.e. signed)
    PAC193X_STATUS voltageBipolarStatus;
    bool voltageIsBipolar = IsChannelCurrentBipolar(channelIndex, &voltageBipolarStatus);
    if (!PAC193X_STATUS_OK(voltageBipolarStatus))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(voltageBipolarStatus);
        return 0;
    }

    // find out if the channel current measurement is bipolar (i.e. signed)
    PAC193X_STATUS currentBipolarStatus;
    bool currentIsBipolar = IsChannelCurrentBipolar(channelIndex, &currentBipolarStatus);
    if (!PAC193X_STATUS_OK(currentBipolarStatus))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(currentBipolarStatus);
        return 0;
    }

    bool powerIsBipolar = voltageIsBipolar | currentIsBipolar;

    // get the min/max voltage values
    double voltageMin, voltageMax;
    PAC193X_STATUS voltageRangeStatus = getChannelVoltageRange(channelIndex, &voltageMin, &voltageMax);
    if (!PAC193X_STATUS_OK(voltageRangeStatus))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(voltageRangeStatus);
        return 0;
    }
    
    // get the min/max current values
    double currentMin, currentMax;
    PAC193X_STATUS currentRangeStatus = getChannelCurrentRange(channelIndex, &currentMin, &currentMax);
    if (!PAC193X_STATUS_OK(currentRangeStatus))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(currentRangeStatus);
        return 0;
    }

    double powerMin = voltageMin * currentMin;
    double powerMax = voltageMax * currentMax;

    const int32_t INT28_MAX =  134217727L;
    const int32_t INT28_MIN = -134217728L;
    const uint32_t UINT28_MAX = 268435456UL;

    uint8_t powerRegisterAddr = PAC193X_VPOWER_CHANNELS[channelIndex];

    // get the power reading, using signed or unsigned reads (depending on polarity)
    double power;
    if (powerIsBipolar)
    {
        // read the register as a signed 28-bit integer
        PAC193X_STATUS readStatus;
        int32_t rawReading = ReadSigned28(powerRegisterAddr, &readStatus);
        if (!PAC193X_STATUS_OK(readStatus))
        {
            PAC193X_SET_STATUS_IF_NOT_NULL(readStatus);
            return 0;
        }
        // convert the raw value to a reading.
        // note that the negative scaling is sublty different to the positive scaling, because the range is +134,217,727 (2^27-1) to -134,217,728 (-2^27).
        if (rawReading >= 0)
            power = rawReading * (powerMax / INT28_MAX);
        else
            power = rawReading * (powerMin / INT28_MIN);
    }
    else
    {
        // read the register as an unsigned 28-bit integer
        PAC193X_STATUS readStatus;
        uint32_t rawReading = Read28(powerRegisterAddr, &readStatus);
        if (!PAC193X_STATUS_OK(readStatus))
        {
            PAC193X_SET_STATUS_IF_NOT_NULL(readStatus);
            return 0;
        }
        // convert the raw value to a power value.
        double readingRange = powerMin - powerMax;
        power = powerMin + (rawReading * (readingRange / UINT28_MAX));
    }

    PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::NoError);
    return power;
}


double PAC193X::getPowerAccumulated(uint8_t channelIndex, bool* overflow, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    if (channelIndex > PAC193X_MAX_CHANNELS - 1)
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::InvalidChannelIndex);
        return 0;
    }
    if (!isChannelEnabled(channelIndex, status))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::ChannelDisabled);
        return 0;
    }
    if (IsRefreshPending())
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::WaitingForRefresh);
        return 0;
    }

    // check the OVF flag and return it in overflow
    if (overflow != nullptr)
    {
        PAC193X_STATUS readCtrlStatus;
        uint8_t ctrlActReg = Read8(PAC193X_CTRL_ACT_ADDR, &readCtrlStatus);
        if (!PAC193X_STATUS_OK(readCtrlStatus))
        {
            PAC193X_SET_STATUS_IF_NOT_NULL(readCtrlStatus);
            return 0;
        }
        // OVF flag is in the LSB of the CTRL_ACT register.
        *overflow = (ctrlActReg & 1) == 1;
    }

    // power accumulator polarity is bipolar if either the voltage or current measurements are configured as bipolar.
    // otherwise power is unipolar.

    // find out if the channel voltage measurement is bipolar (i.e. signed)
    PAC193X_STATUS voltageBipolarStatus;
    bool voltageIsBipolar = IsChannelCurrentBipolar(channelIndex, &voltageBipolarStatus);
    if (!PAC193X_STATUS_OK(voltageBipolarStatus))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(voltageBipolarStatus);
        return 0;
    }

    // find out if the channel current measurement is bipolar (i.e. signed)
    PAC193X_STATUS currentBipolarStatus;
    bool currentIsBipolar = IsChannelCurrentBipolar(channelIndex, &currentBipolarStatus);
    if (!PAC193X_STATUS_OK(currentBipolarStatus))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(currentBipolarStatus);
        return 0;
    }

    bool powerIsBipolar = voltageIsBipolar | currentIsBipolar;

    // get the min/max voltage values
    double voltageMin, voltageMax;
    PAC193X_STATUS voltageRangeStatus = getChannelVoltageRange(channelIndex, &voltageMin, &voltageMax);
    if (!PAC193X_STATUS_OK(voltageRangeStatus))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(voltageRangeStatus);
        return 0;
    }
    
    // get the min/max current values
    double currentMin, currentMax;
    PAC193X_STATUS currentRangeStatus = getChannelCurrentRange(channelIndex, &currentMin, &currentMax);
    if (!PAC193X_STATUS_OK(currentRangeStatus))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(currentRangeStatus);
        return 0;
    }

    double powerMin = voltageMin * currentMin;
    double powerMax = voltageMax * currentMax;

    // the numbers here are actually scaled as if 28-bit integers, because the register is an accumulator.
    const int32_t INT28_MAX =  134217727L;
    const int32_t INT28_MIN = -134217728L;
    const uint32_t UINT28_MAX = 268435456UL;
    /*const int64_t INT48_MAX =  140737488355327LL;
    const int64_t INT48_MIN = -140737488355328LL;
    const uint64_t UINT48_MAX = 281474976710656ULL;*/

    // figure out which power accumulator register we need
    uint8_t powerAccRegisterAddr = PAC193X_VPOWER_ACC_CHANNELS[channelIndex];

    // get the power reading, using signed or unsigned reads (depending on polarity)
    double power;
    if (powerIsBipolar)
    {
        // read the register as a signed 28-bit integer
        PAC193X_STATUS readStatus;
        int64_t rawReading = ReadSigned28(powerAccRegisterAddr, &readStatus);
        if (!PAC193X_STATUS_OK(readStatus))
        {
            PAC193X_SET_STATUS_IF_NOT_NULL(readStatus);
            return 0;
        }
        // convert the raw value to a reading.
        // note that the negative scaling is sublty different to the positive scaling, because the range is +134,217,727 (2^27-1) to -134,217,728 (-2^27).
        if (rawReading >= 0)
            power = rawReading * (powerMax / INT28_MAX);
        else
            power = rawReading * (powerMin / INT28_MIN);
    }
    else
    {
        // read the register as an unsigned 48-bit integer
        PAC193X_STATUS readStatus;
        uint64_t rawReading = Read48(powerAccRegisterAddr, &readStatus);
        if (!PAC193X_STATUS_OK(readStatus))
        {
            PAC193X_SET_STATUS_IF_NOT_NULL(readStatus);
            return 0;
        }
        // convert the raw value to a power value.
        double readingRange = powerMin - powerMax;
        power = powerMin + (rawReading * (readingRange / UINT28_MAX));
    }

    PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::NoError);
    return power;
}


PAC193X_STATUS PAC193X::refreshAsync()
{
    PAC193X_RETURN_IF_NOT_CONFIGURED;

    return Command_REFRESH_V();
}


PAC193X_STATUS PAC193X::refreshAndResetAsync()
{
    PAC193X_RETURN_IF_NOT_CONFIGURED;
    
    return Command_REFRESH();
}


void PAC193X::waitForRefresh()
{
    WaitForRefresh();
}


PAC193X_STATUS PAC193X::refreshSync()
{
    PAC193X_RETURN_IF_NOT_CONFIGURED;

    PAC193X_STATUS status = Command_REFRESH_V();
    if (!PAC193X_STATUS_OK(status))
        return status;
    WaitForRefresh();
}


PAC193X_STATUS PAC193X::refreshAndResetSync()
{
    PAC193X_RETURN_IF_NOT_CONFIGURED;
    
    PAC193X_STATUS status = Command_REFRESH();
    if (!PAC193X_STATUS_OK(status))
        return status;
    WaitForRefresh();
}


bool PAC193X::isChannelEnabled(uint8_t channelIndex, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;
    
    if (channelIndex > PAC193X_MAX_CHANNELS - 1)
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::InvalidChannelIndex);
        return false;
    }

    // channel disable register is a bitmask: [CH1_OFF, CH2_OFF, CH3_OFF, CH4_OFF, U, U, U, U]
    // use the CHANNEL_DIS_ACT register to get the actual (i.e. currently in-use) state of the channel
    PAC193X_STATUS readStatus;
    uint8_t channelDisableBits = Read8(PAC193X_CHANNEL_DIS_ACT_ADDR, &readStatus);
    if (!PAC193X_STATUS_OK(readStatus))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(readStatus);
        return false;
    }

    PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::NoError);
    
    uint8_t channelBitPos = 4 + channelIndex;
    return (channelDisableBits & (1 << channelBitPos)) != 0;
}


PAC193X_CHANNEL_POLARITY PAC193X::getChannelVoltagePolarity(uint8_t channelIndex, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED_RV(PAC193X_CHANNEL_POLARITY::ERROR);

    if (channelIndex > PAC193X_MAX_CHANNELS - 1)
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::InvalidChannelIndex);
        return PAC193X_CHANNEL_POLARITY::ERROR;
    }

    // check the channel's voltage polarity. return an error state if the check fails.
    PAC193X_STATUS polarityStatus;
    bool isBipolar = IsChannelVoltageBipolar(channelIndex, &polarityStatus);
    if (!PAC193X_STATUS_OK(polarityStatus))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(polarityStatus);
        return PAC193X_CHANNEL_POLARITY::ERROR;
    }

    PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::NoError);

    // return the appropriate polarity value
    return isBipolar ? PAC193X_CHANNEL_POLARITY::BIPOLAR : PAC193X_CHANNEL_POLARITY::UNIPOLAR;
}


PAC193X_CHANNEL_POLARITY PAC193X::getChannelCurrentPolarity(uint8_t channelIndex, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED_RV(PAC193X_CHANNEL_POLARITY::ERROR);

    if (channelIndex > PAC193X_MAX_CHANNELS - 1)
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::InvalidChannelIndex);
        return PAC193X_CHANNEL_POLARITY::ERROR;
    }

    // check the channel's current polarity. return an error state if the check fails.
    PAC193X_STATUS polarityStatus;
    bool isBipolar = IsChannelCurrentBipolar(channelIndex, &polarityStatus);
    if (!PAC193X_STATUS_OK(polarityStatus))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(polarityStatus);
        return PAC193X_CHANNEL_POLARITY::ERROR;
    }

    PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::NoError);

    // return the appropriate polarity value
    return isBipolar ? PAC193X_CHANNEL_POLARITY::BIPOLAR : PAC193X_CHANNEL_POLARITY::UNIPOLAR;
}


PAC193X_STATUS PAC193X::getChannelVoltageRange(uint8_t channelIndex, double* voltageMin, double* voltageMax)
{
    PAC193X_RETURN_IF_NOT_CONFIGURED;

    if (channelIndex > PAC193X_MAX_CHANNELS - 1)
    {
        return PAC193X_STATUS::InvalidChannelIndex;
    }

    // voltageMin/voltageMax are not optional.
    if (voltageMin == nullptr || voltageMax == nullptr)
    {
        return PAC193X_STATUS::InvalidArgument;
    }

    // get the polarity. return an error state if the check fails.
    PAC193X_STATUS polarityStatus;
    PAC193X_CHANNEL_POLARITY polarity = getChannelCurrentPolarity(channelIndex, &polarityStatus);
    if (!PAC193X_STATUS_OK(polarityStatus))
    {
        return polarityStatus;
    }

    // set the min/max based on the polarity.
    switch (polarity)
    {
        case PAC193X_CHANNEL_POLARITY::UNIPOLAR:
            *voltageMin = PAC193X_UNIPOLAR_VOLTAGE_MIN;
            *voltageMax = PAC193X_UNIPOLAR_VOLTAGE_MAX;
            break;
        case PAC193X_CHANNEL_POLARITY::BIPOLAR:
            *voltageMin = PAC193X_BIPOLAR_VOLTAGE_MIN;
            *voltageMax = PAC193X_BIPOLAR_VOLTAGE_MAX;
            break;
        default:
            return PAC193X_STATUS::InternalError;
    }

    return PAC193X_STATUS::NoError;
}


PAC193X_STATUS PAC193X::getChannelCurrentRange(uint8_t channelIndex, double* currentMin, double* currentMax)
{
    PAC193X_RETURN_IF_NOT_CONFIGURED;

    if (channelIndex > PAC193X_MAX_CHANNELS - 1)
    {
        return PAC193X_STATUS::InvalidChannelIndex;
    }

    // currentMin/currentMax are not optional.
    if (currentMin == nullptr || currentMax == nullptr)
    {
        return PAC193X_STATUS::InvalidArgument;
    }

    // get the polarity. return an error state if the check fails.
    PAC193X_STATUS polarityStatus;
    PAC193X_CHANNEL_POLARITY polarity = getChannelCurrentPolarity(channelIndex, &polarityStatus);
    if (!PAC193X_STATUS_OK(polarityStatus))
    {
        return polarityStatus;
    }

    // set the min/max based on the polarity.
    // this uses I=sqrt(V/R), but requires multiplication by the sign (-1 or 1) in order to produce correct values for negative voltages.
    switch (polarity)
    {
        case PAC193X_CHANNEL_POLARITY::UNIPOLAR:
            *currentMin = (signbit(PAC193X_UNIPOLAR_CURRENT_MIN) ? -1.0 : 1.0) *
                            sqrt(PAC193X_UNIPOLAR_CURRENT_MIN / (this->shuntResistances[channelIndex] / 1000000.0));
            *currentMax = (signbit(PAC193X_UNIPOLAR_CURRENT_MAX) ? -1.0 : 1.0) *
                            sqrt(PAC193X_UNIPOLAR_CURRENT_MAX / (this->shuntResistances[channelIndex] / 1000000.0));
            break;
        case PAC193X_CHANNEL_POLARITY::BIPOLAR:
            *currentMin = (signbit(PAC193X_BIPOLAR_CURRENT_MIN) ? -1.0 : 1.0) *
                            sqrt(PAC193X_BIPOLAR_CURRENT_MIN / (this->shuntResistances[channelIndex] / 1000000.0));
            *currentMax = (signbit(PAC193X_BIPOLAR_CURRENT_MAX) ? -1.0 : 1.0) *
                            sqrt(PAC193X_BIPOLAR_CURRENT_MAX / (this->shuntResistances[channelIndex] / 1000000.0));
            break;
        default:
            return PAC193X_STATUS::InternalError;
    }

    return PAC193X_STATUS::NoError;
}


PAC193X_STATUS PAC193X::setChannelEnabled(uint8_t channelIndex, bool enable)
{
    PAC193X_RETURN_IF_NOT_CONFIGURED;

    if (channelIndex > PAC193X_MAX_CHANNELS - 1)
    {
        return PAC193X_STATUS::InvalidChannelIndex;
    }

    // read the current state from the CHANNEL_DIS register (i.e. current CHANNEL DIS state plus any existing pending changes)
    // channel disable register is a bitmask: [CH1_OFF, CH2_OFF, CH3_OFF, CH4_OFF, U, U, U, U]
    PAC193X_STATUS readStatus;
    uint8_t channelDisableBits = Read8(PAC193X_CHANNEL_DIS_ADDR, &readStatus);
    if (!PAC193X_STATUS_OK(readStatus))
    {
        return readStatus;
    }

    uint8_t channelBitPos = 4 + channelIndex;

    // set or clear the channel disable bit for this channel
    uint8_t newChannelDisableBits = channelDisableBits;
    if (enable)
    {
        // clear the disable bit for this channel
        newChannelDisableBits &= ~(1 << channelBitPos);
    }
    else
    {
        // set the disable bit for this channel
        newChannelDisableBits |= (1 << channelBitPos);
    }

    // write the modified disable bit back to the CHANNEL_DIS register
    PAC193X_STATUS writeStatus = WriteRegister(PAC193X_CHANNEL_DIS_ADDR, newChannelDisableBits);
    return writeStatus;
}


/* ======== PRIVATE METHODS ======== */


PAC193X_STATUS PAC193X::WriteRegister(uint8_t registerAddress, uint8_t value)
{
    PAC193X_RETURN_IF_NOT_CONFIGURED;

    Wire.beginTransmission(this->deviceAddress);

    Wire.write(registerAddress);
    Wire.write(value);

    if (Wire.endTransmission() != 0)
        return PAC193X_STATUS::WriteError;
    
    return PAC193X_STATUS::NoError;
}


PAC193X_STATUS PAC193X::ReadRegister(uint8_t registerAddress, uint8_t byteCount, uint8_t* buffer, bool reverse = false)
{
    PAC193X_RETURN_IF_NOT_CONFIGURED;

    if (byteCount == 0 || buffer == nullptr)
        return PAC193X_STATUS::InvalidArgument;
    
    Wire.beginTransmission(this->deviceAddress);
    Wire.write(registerAddress);
    if (Wire.endTransmission() != 0)
        return PAC193X_STATUS::WriteError;

    uint8_t bytesRemaining = byteCount;

    if (reverse)
        buffer += byteCount -  1;
    
    Wire.beginTransmission(this->deviceAddress);
	Wire.requestFrom(this->deviceAddress, byteCount);
    while (Wire.available() && (bytesRemaining > 0))
    {
        *buffer = (uint8_t)Wire.read();

        if (reverse)
            buffer--;
        else
            buffer++;
        
        bytesRemaining--;
    }
    if (Wire.endTransmission() != 0)
        return PAC193X_STATUS::WriteError;
    
    return PAC193X_STATUS::NoError;
}


uint8_t PAC193X::Read8(uint8_t registerAddress, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    uint8_t value = 0;
    PAC193X_STATUS readStatus = ReadRegister(registerAddress, sizeof(uint8_t), &value);
    PAC193X_SET_STATUS_IF_NOT_NULL(readStatus);
    return value;
}


uint16_t PAC193X::Read16(uint8_t registerAddress, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    uint16_t value = 0;
    PAC193X_STATUS readStatus = ReadRegister(registerAddress, sizeof(uint16_t), reinterpret_cast<uint8_t*>(&value), PAC193X_NEED_ENDIAN_SWAP);
    PAC193X_SET_STATUS_IF_NOT_NULL(readStatus);
    return value;
}


int16_t PAC193X::ReadSigned16(uint8_t registerAddress, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    return static_cast<int16_t>(Read16(registerAddress, status));
}


uint32_t PAC193X::Read28(uint8_t registerAddress, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;
    
    return Read32(registerAddress, status) >> 4;
}


int32_t PAC193X::ReadSigned28(uint8_t registerAddress, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;
    
    return static_cast<int32_t>(Read32(registerAddress, status)) >> 4;
}


uint32_t PAC193X::Read32(uint8_t registerAddress, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    uint32_t value = 0;
    PAC193X_STATUS readStatus = ReadRegister(registerAddress, sizeof(uint32_t), reinterpret_cast<uint8_t*>(&value), PAC193X_NEED_ENDIAN_SWAP);
    PAC193X_SET_STATUS_IF_NOT_NULL(readStatus);
    return value;
}


int32_t PAC193X::ReadSigned32(uint8_t registerAddress, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    return static_cast<int32_t>(Read32(registerAddress, status));
}


uint64_t PAC193X::Read48(uint8_t registerAddress, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    return Read64(registerAddress, status) >> 16;
}


int64_t PAC193X::ReadSigned48(uint8_t registerAddress, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;
    
    return static_cast<int64_t>(Read64(registerAddress, status)) >> 16;
}


uint64_t PAC193X::Read64(uint8_t registerAddress, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    uint64_t value = 0;
    PAC193X_STATUS readStatus = ReadRegister(registerAddress, sizeof(uint64_t), reinterpret_cast<uint8_t*>(&value), PAC193X_NEED_ENDIAN_SWAP);
    PAC193X_SET_STATUS_IF_NOT_NULL(readStatus);
    return value;
}


int64_t PAC193X::ReadSigned64(uint8_t registerAddress, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    return static_cast<int64_t>(Read64(registerAddress, status));
}


double PAC193X::ReadVoltageResult(uint8_t channelIndex, const uint8_t* channelRegisterMap, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    if (channelIndex > PAC193X_MAX_CHANNELS - 1)
    {
        return PAC193X_STATUS::InvalidChannelIndex;
    }
    if (!isChannelEnabled(channelIndex, status))
    {
        return PAC193X_STATUS::ChannelDisabled;
    }
    if (IsRefreshPending())
    {
        return PAC193X_STATUS::WaitingForRefresh;
    }

    // find out if the channel is bipolar (i.e. signed)
    PAC193X_STATUS bipolarStatus;
    bool isBipolar = IsChannelVoltageBipolar(channelIndex, &bipolarStatus);
    if (!PAC193X_STATUS_OK(bipolarStatus))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(bipolarStatus);
        return 0;
    }

    // get the min/max values
    double voltageMin, voltageMax;
    PAC193X_STATUS voltageRangeStatus = getChannelVoltageRange(channelIndex, &voltageMin, &voltageMax);
    if (!PAC193X_STATUS_OK(voltageRangeStatus))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(voltageRangeStatus);
        return 0;
    }

    // get the register associated with the channel index
    uint8_t registerAddr = channelRegisterMap[channelIndex];

    // read the register value and translate it to a voltage representation
    PAC193X_STATUS readStatus;
    double voltage = ReadResultRegister(registerAddr, isBipolar, voltageMin, voltageMax, &readStatus);
    if (!PAC193X_STATUS_OK(readStatus))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(readStatus);
        return 0;
    }

    PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::NoError);
    return voltage;
}


double PAC193X::ReadCurrentResult(uint8_t channelIndex, const uint8_t* channelRegisterMap, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    if (channelIndex > PAC193X_MAX_CHANNELS - 1)
    {
        return PAC193X_STATUS::InvalidChannelIndex;
    }
    if (!isChannelEnabled(channelIndex, status))
    {
        return PAC193X_STATUS::ChannelDisabled;
    }
    if (IsRefreshPending())
    {
        return PAC193X_STATUS::WaitingForRefresh;
    }

    // find out if the channel is bipolar (i.e. signed)
    PAC193X_STATUS bipolarStatus;
    bool isBipolar = IsChannelCurrentBipolar(channelIndex, &bipolarStatus);
    if (!PAC193X_STATUS_OK(bipolarStatus))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(bipolarStatus);
        return 0;
    }

    // get the min/max values
    double currentMin, currentMax;
    PAC193X_STATUS currentRangeStatus = getChannelCurrentRange(channelIndex, &currentMin, &currentMax);
    if (!PAC193X_STATUS_OK(currentRangeStatus))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(currentRangeStatus);
        return 0;
    }

    // get the register associated with the channel index
    uint8_t registerAddr = channelRegisterMap[channelIndex];

    // read the register value and translate it to a voltage representation
    PAC193X_STATUS readStatus;
    double voltage = ReadResultRegister(registerAddr, isBipolar, currentMin, currentMax, &readStatus);
    if (!PAC193X_STATUS_OK(readStatus))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(readStatus);
        return 0;
    }

    PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::NoError);
    return voltage;
}


double PAC193X::ReadResultRegister(uint8_t registerAddress, bool bipolar, double min, double max, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    if (bipolar && (min >= 0 || max <= 0))
    {
        // set internal error - min/max values must be negative and positive respectively.
        PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::InternalError);
        return 0;
    }

    if (!bipolar && (min < 0 || max <= 0))
    {
        // set internal error - min/max values should be 0 or greater and greater than zero respectively.
        PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::InternalError);
        return 0;
    }

    if (max < min)
    {
        // set internal error - wut? max must exceed min.
        PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::InternalError);
        return 0;
    }

    // get the reading, using signed or unsigned reads (depending on polarity)
    double reading;
    if (bipolar)
    {
        // read the register as a signed 16-bit integer
        PAC193X_STATUS readStatus;
        int16_t rawReading = ReadSigned16(registerAddress, &readStatus);
        if (!PAC193X_STATUS_OK(readStatus))
        {
            PAC193X_SET_STATUS_IF_NOT_NULL(readStatus);
            return 0;
        }
        // convert the raw value to a reading.
        // note that the negative scaling is sublty different to the positive scaling, because the range is +32767 to -32768.
        if (rawReading >= 0)
            reading = rawReading * (max / INT16_MAX);
        else
            reading = rawReading * (min / INT16_MIN);
    }
    else
    {
        // read the register as an unsigned 16-bit integer
        PAC193X_STATUS readStatus;
        uint16_t rawReading = Read16(registerAddress, &readStatus);
        if (!PAC193X_STATUS_OK(readStatus))
        {
            PAC193X_SET_STATUS_IF_NOT_NULL(readStatus);
            return 0;
        }
        // convert the raw value to a voltage.
        double readingRange = min - max;
        reading = min + (rawReading * (readingRange / UINT16_MAX));
    }

    PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::NoError);
    return reading;
}


bool PAC193X::IsChannelVoltageBipolar(uint8_t channelIndex, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    // validate channel index
    if (channelIndex > PAC193X_MAX_CHANNELS - 1)
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::InvalidChannelIndex);
        return false;
    }

    // read the NEG PWR ACT register to get the actually-configured measurement state
    PAC193X_STATUS readStatus;
    uint8_t negPwrAct = Read8(PAC193X_NEG_PWR_ACT_ADDR, &readStatus);
    if (!PAC193X_STATUS_OK(readStatus))
    {
        return readStatus;
    }

    PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::NoError);

    // bi-directional voltage config is in the bottom 4 bits
    return (negPwrAct & (1 << channelIndex)) != 0;
}


bool PAC193X::IsChannelCurrentBipolar(uint8_t channelIndex, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    // validate channel index
    if (channelIndex > PAC193X_MAX_CHANNELS - 1)
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::InvalidChannelIndex);
        return false;
    }

    // read the NEG PWR ACT register to get the actually-configured measurement state
    PAC193X_STATUS readStatus;
    uint8_t negPwrAct = Read8(PAC193X_NEG_PWR_ACT_ADDR, &readStatus);
    if (!PAC193X_STATUS_OK(readStatus))
    {
        return readStatus;
    }

    PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::NoError);

    // bi-directional current config is in the top 4 bits
    return (negPwrAct & (1 << (channelIndex + 4))) != 0;
}


bool PAC193X::ValidateDeviceAddress(uint8_t address)
{
    if ((address & PAC193X_ADDRESS_MASK_ZEROS) != 0)
        return false;
    
    if ((address & PAC193X_ADDRESS_MASK_ONES) != PAC193X_ADDRESS_MASK_ONES)
        return false;
    
    return true;
}


bool PAC193X::ValidateShuntResistance(uint32_t resistance)
{
    return (resistance > 0) && (resistance < UINT32_MAX);
}


bool PAC193X::ValidateSampleRate(PAC193X_SAMPLE_RATE sampleRate)
{
    return (sampleRate >= PAC193X_SAMPLE_RATE::PAC193X_SAMPLE_RATE_VALUE_MIN) && 
            (sampleRate <= PAC193X_SAMPLE_RATE::PAC193X_SAMPLE_RATE_VALUE_MAX);
}


bool PAC193X::IsRefreshPending()
{
    return (micros() - this->timeOfLastRefreshCommand) < PAC193X_REFRESH_WAIT_TIME_US;
}


void PAC193X::WaitForRefresh()
{
    while (IsRefreshPending()) ;
}


PAC193X_STATUS PAC193X::Command_REFRESH_Internal(uint8_t commandAddress)
{
    PAC193X_RETURN_IF_NOT_CONFIGURED;

    if (IsRefreshPending())
        return PAC193X_STATUS::WaitingForRefresh;

    PAC193X_STATUS status = WriteRegister(commandAddress, 1);
    if (PAC193X_STATUS_OK(status))
    {
        this->timeOfLastRefreshCommand = micros();
    }
}


PAC193X_STATUS PAC193X::Command_REFRESH()
{
    return Command_REFRESH_Internal(PAC193X_REFRESH_CMD_ADDR);
}


PAC193X_STATUS PAC193X::Command_REFRESH_V()
{
    return Command_REFRESH_Internal(PAC193X_REFRESH_V_CMD_ADDR);
}


PAC193X_STATUS PAC193X::Command_REFRESH_G()
{
    return Command_REFRESH_Internal(PAC193X_REFRESH_G_CMD_ADDR);
}
