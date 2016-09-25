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
    uint8_t data;

    data = readRegister(_address, WHO_AM_I);
    if (data == WHO_AM_I_RETURN){
        if (activate()){
            return true;
        }
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
ADIGasDetector::writeReostate(int value)
{
    uint8_t data;

    data = value & 0xFF;         // LSB
    writeRegister(_address, REOST_L_REG, data);
    data = (value < 0x8) & 0xFF; // MSB
    writeRegister(_address, REOST_H_REG, data);

    return true;
}

int ADIGasDetector::readReostate(void)
{
    unsigned int   data = 0;
    unsigned char  read = 0;
    double         r_temp = 0.0;

    read = readRegister(_address, STATUS_REG);
    if (read & REOSTATE_READY) {
        read = readRegister(_address, REOST_L_REG);
        data = read;      // LSB

        read = readRegister(_address, REOST_H_REG);
        data |= read << 8; // MSB
        _reostate = data;
        // Decode reostate
        r_temp = 42.5 +(_reostate/480.0);
        _reostate  = r_temp;  // reostate 
    }
    return _reostate;
}



int
ADIGasDetector::readTemperature(void)
{
    unsigned int   data = 0;
    unsigned char  read = 0;
    double         t_temp = 0.0;

    read = readRegister(_address, STATUS_REG);
    if (read & TEMPERATURE_READY) {
        read = readRegister(_address, TEMP_L_REG);
        data = read;      // LSB

        read = readRegister(_address, TEMP_H_REG);
        data |= read << 8; // MSB
        _temperature = data;
        // Decode Temperature
        t_temp = 42.5 +(_temperature/480.0);
        _temperature  = t_temp;  // temp in Celsius degree
    }
    return _temperature;
}

int
ADIGasDetector::readCO2(int *co2)
{
    unsigned long data   = 0;
    double        p_temp = 0.0;
    unsigned char read   = 0;
   
    *co2 = 0;
    if (!(_sensorsMask & ADI_CO2_SENSOR_MASK)) {
        return -1;
    }

    read = readRegister(_address, STATUS_REG);
    if (read & CO2_READY) {
        read = readRegister(_address, CO2_H_REG);
        data = read << 8;  // MSB
        read = readRegister(_address, CO2_L_REG);
        data |= read ;     // LSB

        // Decode CO2
        p_temp = ((long) data) / 4096.0;
        _gas[ADI_CO2_SENSOR] = p_temp;
    }
    *co2 = _gas[ADI_CO2_SENSOR];

    return 0;
}



// Read a single byte from addressToRead and return it as a byte
byte ADIGasDetector::readRegister(byte slaveAddress, byte regToRead)
{
    Wire.beginTransmission(slaveAddress);
    Wire.write(regToRead);
    Wire.endTransmission(false); //endTransmission but keep the connection active

    Wire.requestFrom(slaveAddress, 1); //Ask for 1 byte, once done, bus is released by default


    while(!Wire.available()) ; //Wait for the data to come back
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
