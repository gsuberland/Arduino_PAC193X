#include "pac193x.h"


PAC193X::PAC193X()
{
    deviceAddress = 0;
    shuntResistance = 0;
    isConfigured = false;
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
    if (status != nullptr)
        *status = readStatus;
    return value;
}


uint16_t PAC193X::Read16(uint8_t registerAddress, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    uint16_t value = 0;
    PAC193X_STATUS readStatus = ReadRegister(registerAddress, sizeof(uint16_t), reinterpret_cast<uint8_t*>(&value), PAC193X_NEED_ENDIAN_SWAP);
    if (status != nullptr)
        *status = readStatus;
    return value;
}


uint32_t PAC193X::Read24(uint8_t registerAddress, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;
    
    return Read32(registerAddress, status) >> 8;
}


uint32_t PAC193X::Read32(uint8_t registerAddress, PAC193X_STATUS* status)
{
    PAC193X_RETURN_WITH_PARAM_IF_NOT_CONFIGURED;

    uint32_t value = 0;
    PAC193X_STATUS readStatus = ReadRegister(registerAddress, sizeof(uint32_t), reinterpret_cast<uint8_t*>(&value), PAC193X_NEED_ENDIAN_SWAP);
    if (status != nullptr)
        *status = readStatus;
    return value;
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
    return (micros() - timeOfLastRefreshCommand) < PAC193X_REFRESH_WAIT_TIME_US;
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
        timeOfLastRefreshCommand = micros();
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
