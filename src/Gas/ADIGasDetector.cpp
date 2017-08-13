/*
* ADIGasDetector.cpp
*
* Created: 08/08/2016 20:50:20
*  Author: searobin
*/
#include <Wire.h>
#include "ADIGasDetector.h"
#include "ADIGasDetectorReg.h"


ADIGasDetector::ADIGasDetector(int mask) : _address(ADI_GAS_DETECTOR_ADDRESS)
{
    _reostate    = 0;
    _temperature = 0;
    _sensorsMask = mask;

    for (int i = 0; i < ADI_MAX_SENSOR_NUM; ++i){
        _gas[i] = 0;
    }
}


bool ADIGasDetector::begin(void)
{
    volatile uint8_t data = 0;

    data = readRegister(_address, WHO_AM_I);
    if (data == WHO_AM_I_RETURN){
        return true;
    } 
    return false;
}



bool
ADIGasDetector::activate(void)
{
    uint8_t data;

    data = readRegister(_address, CTRL_REG1);
    data |= POWER_UP;
    data |= ODR0_SET;
    writeRegister(_address, CTRL_REG1, data);

    return true;
}


bool ADIGasDetector::deactivate(void)
{
    uint8_t data;

    data = readRegister(_address, CTRL_REG1);
    data &= ~POWER_UP;
    writeRegister(_address, CTRL_REG1, data);
    return true;
}

bool
ADIGasDetector::writeRawRheostate(uint16_t value)
{
    uint8_t data;

    data = value & 0xFF;         // LSB
    writeRegister(_address, REOST_W_L_REG, data);
    data = (value >> 8) & 0x3; // MSB
    writeRegister(_address, REOST_W_H_REG, data);

    return true;
}

uint16_t ADIGasDetector::readRawRheostate(void)
{
    volatile uint16_t   data = 0;
    volatile unsigned char  read = 0;
    float         f_rheostate = 0.0;

    read = readRegister(_address, REOST_L_REG);
    data = read;      // LSB

    read = readRegister(_address, REOST_H_REG);
    data |= read << 8; // MSB
    _reostate = data;
    // Decode reostate
    if (!_reostate) {
       f_rheostate = 120.0;
    } else {
       f_rheostate = (20000.0/1024.0)*_reostate;
    }        
    // _reostate  = f_rheostate;  // reostate 

    return _reostate;
}

/* Notes for conversion of 13 bit temperature: 
 * bit 15 = sign , bits 0-3 to be removed, 14-4 eventually two complement
 * Positive Temperature = ADC Code (dec)/16
 * Negative Temperature = (ADC Code (dec) - 8192)/16
 */
uint16_t
ADIGasDetector::readRawTemperature(void)
{
    volatile unsigned int   data = 0;
    volatile unsigned char  read = 0;

        read = readRegister(_address, TEMP_L_REG);
        data = read;      // LSB

        read = readRegister(_address, TEMP_H_REG);
        data |= read << 8; // MSB

        _temperature = data;
      return _temperature;
}


/* Notes for conversion of 13 bit temperature: 
 * bit 15 = sign , bits 0-3 to be removed, 14-4 eventually two complement
 * Positive Temperature = ADC Code (dec)/16
 * Negative Temperature = (ADC Code (dec) - 8192)/16
 */
float
ADIGasDetector::readTemperature(void)
{
    volatile unsigned int   data = 0;
    volatile unsigned char  read = 0;
    volatile int   t_temp = 0.0;
    volatile float f_temp = 0.0;
    uint16_t ui16Temp = 0;
    
    ui16Temp = readRawTemperature();

    // Decode Temperature
    if (ui16Temp & 0x8000) {
       // negative number
       ui16Temp = ui16Temp >> 3;
       f_temp = (ui16Temp - 8192) / 16.0;
    } else {
       ui16Temp = ui16Temp >> 3;
       f_temp = ui16Temp / 16.0;            
    }
    // temp in Celsius degree
    return f_temp;
}



long
ADIGasDetector::readRawCO2(void)
{
    long          data   = 0;
    unsigned char read   = 0;
   
    if (!(_sensorsMask & ADI_CO2_SENSOR_MASK)) {
        return -1;
    }
    read = readRegister(_address, CO2_X_H_REG);
    data = read << 24;  // MSB H
    delay(100);
    read = readRegister(_address, CO2_X_L_REG);
    data |= read  << 16;     // MSB L
    delay(100);
    read = readRegister(_address, CO2_H_REG);
    data |= read << 8;  // LSB H
    delay(100);
    read = readRegister(_address, CO2_L_REG);
    data |= read ;     // LSB L
    delay(100);

    _gas[ADI_CO2_SENSOR] = data;

    return  _gas[ADI_CO2_SENSOR];
}



// Read a single byte from addressToRead and return it as a byte
byte ADIGasDetector::readRegister(byte slaveAddress, byte regToRead)
{
    Wire.beginTransmission(slaveAddress);
    Wire.write(regToRead);
    Wire.endTransmission();

    delay(1);
    Wire.requestFrom(slaveAddress, 1); //Ask for 1 byte, once done, bus is released by default

    return Wire.read(); //Return this one byte
}

// Writes a single byte (dataToWrite) into regToWrite
bool ADIGasDetector::writeRegister(byte slaveAddress, byte regToWrite, byte dataToWrite)
{
    Wire.beginTransmission(slaveAddress);

    if (!Wire.write(regToWrite)) {
        return false;
    }
    if (!Wire.write(dataToWrite)) {
        return false;
    }

    uint8_t errorNo = Wire.endTransmission(); //Stop transmitting
    return (errorNo == 0);
}



ADIGasDetector adiGas;
