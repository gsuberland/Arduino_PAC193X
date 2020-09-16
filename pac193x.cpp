/* 
 * Arduino library for Microchip PAC193X series power monitors.
 * Copyright (c) 2020 Graham Sutherland
 * 
 * This software is released under MIT license.
 * Please see the LICENSE file for the full text.
 *
 */

#include "pac193x.h"


PAC193X::PAC193X()
{
    this->deviceAddress = 0;
    this->shuntResistance = 0;
    this->isConfigured = false;
}


PAC193X::~PAC193X()
{
    isConfigured = false;
}


PAC193X_STATUS PAC193X::begin(uint8_t address, uint32_t shuntResistanceMicroOhm, PAC193X_SAMPLE_RATE sampleRate = PAC193X_SAMPLE_RATE::RATE_1024)
{
    if (!ValidateDeviceAddress(address))
        return PAC193X_STATUS::InvalidDeviceAddress;

    if (!ValidateShuntResistance(shuntResistanceMicroOhm))
        return PAC193X_STATUS::InvalidResistorValue;

    if (!ValidateSampleRate(sampleRate))
        return PAC193X_STATUS::InvalidSampleRate;

    this->deviceAddress = address;
    this->shuntResistance = shuntResistanceMicroOhm;
    
    this->isConfigured = true;

    return PAC193X_STATUS::OK;
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
    if (channelIndex > 3)
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


bool PAC193X::isChannelEnabled(uint8_t index, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;
    
    if (index > 3)
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::InvalidChannelIndex);
        return false;
    }

    // channel disable register is a bitmask: [CH1_OFF, CH2_OFF, CH3_OFF, CH4_OFF, U, U, U, U]
    PAC193X_STATUS readStatus;
    uint8_t channelDisableBits = Read8(PAC193X_CHANNEL_DIS_ACT_ADDR, &readStatus);
    if (!PAC193X_STATUS_OK(readStatus))
    {
        PAC193X_SET_STATUS_IF_NOT_NULL(readStatus);
        return false;
    }

    PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::OK);
    
    uint8_t channelBitPos = 4 + index;
    return (channelDisableBits & (1 << channelBitPos)) != 0;
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
    
    return PAC193X_STATUS::OK;
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
    
    return PAC193X_STATUS::OK;
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


uint32_t PAC193X::Read24(uint8_t registerAddress, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;
    
    return Read32(registerAddress, status) >> 8;
}


int32_t PAC193X::ReadSigned24(uint8_t registerAddress, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;
    
    return static_cast<int32_t>(Read32(registerAddress, status)) >> 8;
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


bool PAC193X::IsChannelVoltageBipolar(uint8_t channelIndex, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    // validate channel index
    if (channelIndex > 3)
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

    PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::OK);

    // bi-directional voltage config is in the bottom 4 bits
    return (negPwrAct & (1 << channelIndex)) != 0;
}


bool PAC193X::IsChannelCurrentBipolar(uint8_t channelIndex, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    // validate channel index
    if (channelIndex > 3)
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

    PAC193X_SET_STATUS_IF_NOT_NULL(PAC193X_STATUS::OK);

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
    return (sampleRate >= PAC193X_SAMPLE_RATE::VALUE_MIN) && (sampleRate <= PAC193X_SAMPLE_RATE::VALUE_MAX);
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
