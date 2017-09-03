/*
 * ADIGasDetector.h
 *
 * Created: 08/08/2016 20:50:30
 * Author: searobin
 */


#ifndef ADI_GAS_DETECTOR_H_
#define ADI_GAS_DETECTOR_H_

#include <Arduino.h>

typedef enum {
    ADI_CO2_SENSOR,
    ADI_SO_SENSOR,
    ADI_MAX_SENSOR_NUM 
} AdiSensorType;

#define ADI_CO2_SENSOR_MASK  (1 << ADI_CO2_SENSOR)
#define ADI_SO_SENSOR_MASK   (1 <<  ADI_SO_SENSOR)

class ADIGasDetector
{
private:
    uint16_t _sensorsMask;
    uint8_t _address;
    byte readRegister(byte slaveAddress, byte regToRead);
    bool writeRegister(byte slaveAddress, byte regToWrite, byte dataToWrite);
    byte readRegisterIndex(byte slaveAddress, byte regToRead, byte index);
public:
    ADIGasDetector(int mask = ADI_CO2_SENSOR_MASK);
    bool begin(void);
    bool activate(void);
    bool deactivate(void);

    bool writeRawRheostate(uint16_t value);
    uint16_t  readRawRheostate(void);
    float  readTemperature(void);
    uint16_t readRawTemperature(void);
    long  readRawCO2();
    uint8_t readFwVersion(void);

    uint16_t readRheostateMemory(uint8_t index);
    uint16_t storeRheostateInMemory();
    uint8_t  readRheostateFreeMemory(void);

protected:
    long    _gas[ADI_MAX_SENSOR_NUM];
    int     _reostate;
    int16_t _temperature;
};


extern ADIGasDetector adiGas;

#endif /* ADI_GAS_DETECTOR_H_ */
