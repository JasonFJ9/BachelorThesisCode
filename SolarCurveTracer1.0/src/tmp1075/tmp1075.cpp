/*
 * Project Name: Library for the LM75A temperature sensor by NXP and Texas Instruments.
 * File: LM75A.cpp
 * Description: library Source file
 * Author: Gavin Lyons.
 * IDE:  Rpi=PICo rp2040 C++
 * Created Sep 2022
 * Description: See URL for full details.
 * URL: https://github.com/gavinlyonsrepo/RPI_PICO_projects_list
 */

#include "pico/stdlib.h"
#include "../include/tmp1075/tmp1075.hpp"

#include <stdio.h>

// Constructor
// Param1 : I2C address (7-bit)
// Param2 : I2C instance of port IC20 or I2C1
// Param3 : I2C Data pin
// Param4 : I2C Clock pin
// Param5 : I2C Bus Clock speed in KHz. Typically 100-400
LIB_TMP1075::LIB_TMP1075(uint8_t i2cAddress, i2c_inst_t* i2c_type, uint8_t SDApin, uint8_t SCLKpin, uint16_t CLKspeed)
{
    _i2cAddress = i2cAddress;
    _SClkPin = SCLKpin;
    _SDataPin = SDApin;
    _CLKSpeed = CLKspeed;
     i2c = i2c_type; 
}

// Power management

// Func Desc : enter Shutdown mode : 4 μA (Typical) current draw.
void LIB_TMP1075::shutdown()
{
    uint8_t config = read8bitRegister(TMP1075_REGISTER_CONFIG);
    write8bitRegister(TMP1075_REGISTER_CONFIG, (config & 0b11111110) | 0b1);
}

// Func Desc : exit Shutdown mode for  Operating  mode: 280 μA (Typical)
void LIB_TMP1075::wakeup()
{
    uint8_t config = read8bitRegister(TMP1075_REGISTER_CONFIG);
    write8bitRegister(TMP1075_REGISTER_CONFIG, config & 0b11111110);
}
// Fucn Desc get power mode
// Return bool
// 1 = shutdown
// 0 operating mode
bool LIB_TMP1075::isShutdown()
{
    return (read8bitRegister(TMP1075_REGISTER_CONFIG) & 0b1) == 1;	
}


// Temperature functions

// Get temperature Celsius
// returns float
float LIB_TMP1075::getTemperature()
{
    uint16_t result;
    if (!read16bitRegister(TMP1075_REGISTER_TEMP, result))
    {
        return TMP1075_INVALID_TEMPERATURE;
    }
    return (float)result / 256.0f;
}

// Get temperature Farenheit(
// returns float
float LIB_TMP1075::getTemperatureInFarenheit()
{
    return getTemperature() * 1.8f + 32.0f;
}

// Configuration functions

float LIB_TMP1075::getHysterisisTemperature()
{
    uint16_t result;
    if (!read16bitRegister(TMP1075_REGISTER_THYST, result))
    {
        return TMP1075_INVALID_TEMPERATURE;
    }
    return (float)result / 256.0f;
}

FaultQueueValue LIB_TMP1075::getFaultQueueValue()
{
    return (FaultQueueValue)(read8bitRegister(TMP1075_REGISTER_CONFIG) & 0b00011000);
}

float LIB_TMP1075::getOSTripTemperature()
{
    uint16_t result;
    if (!read16bitRegister(TMP1075_REGISTER_TOS, result))
    {
        return TMP1075_INVALID_TEMPERATURE;
    }
    return (float)result / 256.0f;
}

OsPolarity LIB_TMP1075::getOsPolarity()
{
    return (OsPolarity)(read8bitRegister(TMP1075_REGISTER_CONFIG) & 0b100);
}

DeviceMode LIB_TMP1075::getDeviceMode()
{
    return (DeviceMode)(read8bitRegister(TMP1075_REGISTER_CONFIG) & 0b010);
}

void LIB_TMP1075::setHysterisisTemperature(float temperature)
{
    write16bitRegister(TMP1075_REGISTER_THYST, temperature * 256);
}

void LIB_TMP1075::setOsTripTemperature(float temperature)
{
    write16bitRegister(TMP1075_REGISTER_TOS, temperature * 256);
}

void LIB_TMP1075::setFaultQueueValue(FaultQueueValue value)
{
    uint8_t config = read8bitRegister(TMP1075_REGISTER_CONFIG);
    write8bitRegister(TMP1075_REGISTER_CONFIG, (config & 0b11100111) | value);
}

void LIB_TMP1075::setOsPolarity(OsPolarity polarity)
{
    uint8_t config = read8bitRegister(TMP1075_REGISTER_CONFIG);
    write8bitRegister(TMP1075_REGISTER_CONFIG, (config & 0b11111011) | polarity);
}

void LIB_TMP1075::setDeviceMode(DeviceMode deviceMode)
{
    uint8_t config = read8bitRegister(TMP1075_REGISTER_CONFIG);
    write8bitRegister(TMP1075_REGISTER_CONFIG, (config & 0b11111101) | deviceMode);
}


// Function Desc Is the Device connected
// Writes Reads and writes  the Config register
// Returns 1 for success 0 if not.
bool LIB_TMP1075::isConnected()
{
    uint8_t oldValue = read8bitRegister(TMP1075_REGISTER_CONFIG);
    write8bitRegister(TMP1075_REGISTER_CONFIG, 0x0f);
    uint8_t newValue = read8bitRegister(TMP1075_REGISTER_CONFIG);
    write8bitRegister(TMP1075_REGISTER_CONFIG, oldValue);
    return newValue == 0x0f;
}

// Func Desc :  reads config register
// Returns Byte with values of config register
uint8_t LIB_TMP1075::getConfig()
{
    return read8bitRegister(TMP1075_REGISTER_CONFIG);
}

// Func Desc :  reads value of product ID register
float LIB_TMP1075::getProdId()
{
    uint8_t value = read8bitRegister(TMP1075_REGISTER_PRODID);
    return (float)(value >> 4) + (value & 0x0F) / 10.0f;
} 

// *************** Private function  ************************

uint8_t LIB_TMP1075::read8bitRegister(const uint8_t reg)
{
    uint8_t result;
    uint8_t BufTx[1];
    BufTx[0] = reg;

    return_value = i2c_write_blocking(i2c, _i2cAddress, BufTx, 1 , true);
    if (return_value < 1)
    {
        return 0xFF;
    }

    return_value =  i2c_read_blocking(i2c, _i2cAddress, &result, 1, false); 	
    if (return_value < 1)
    {
        return 0xFF;
    }
    return result;
}

bool LIB_TMP1075::read16bitRegister(const uint8_t reg, uint16_t& response)
{
    uint8_t bufRX[3];
    uint8_t bufTX[1];
    bufTX[0] = reg;

    return_value = i2c_write_blocking(i2c, _i2cAddress, bufTX, 1 , true);
    if (return_value < 1)
    {
        return false;
    }
    //printf("bufRX1: %d , %d , %d \n",bufRX[0],bufRX[1],bufRX[2]);
    //printf("bufTX1: %d \n",bufTX[0]);

    return_value =  i2c_read_blocking(i2c, _i2cAddress, bufRX, 2, false); 
    if (return_value < 1)
    {
        return false;
    }
    //printf("bufRX2: %d , %d , %d \n",bufRX[0],bufRX[1],bufRX[2]);
    //printf("bufTX2: %d \n",bufTX[0]);

    response = bufRX[0] << 8 | bufRX[1];
    //printf("response: %d \n",response);
    return true;
}

bool LIB_TMP1075::write16bitRegister(const uint8_t reg, const uint16_t value)
{
    uint8_t bufTX[3];
    bufTX[0] = reg;
    bufTX[1] = value >> 8;
    bufTX[2] = value;
        
    return_value = i2c_write_blocking(i2c, _i2cAddress, bufTX, 3 , false);
    if (return_value < 1)
    {
        return false;
    }

    return true;
}

bool LIB_TMP1075::write8bitRegister(const uint8_t reg, const uint8_t value)
{
    uint8_t bufTX[2];
    bufTX[0] = reg;
    bufTX[1] = value;

    return_value = i2c_write_blocking(i2c, _i2cAddress, bufTX, 2 ,false);
    if (return_value < 1)
    {
        return false;
    }
    return true;
}

void LIB_TMP1075::initTMP1075()
{
    gpio_set_function(_SDataPin, GPIO_FUNC_I2C);
    gpio_set_function(_SClkPin, GPIO_FUNC_I2C);
    gpio_pull_up(_SDataPin);
    gpio_pull_up(_SClkPin);
    i2c_init(i2c, _CLKSpeed * 1000);
}

void LIB_TMP1075::deinitTMP1075()
{
    gpio_set_function(_SDataPin, GPIO_FUNC_NULL);
    gpio_set_function(_SClkPin, GPIO_FUNC_NULL);
    i2c_deinit(i2c); 	
}