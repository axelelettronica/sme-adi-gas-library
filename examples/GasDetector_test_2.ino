/*
 * CO-OX gas detector.ino
 *
 * Example of CO-OX reading through the ADI Gas Detector Shield
 *
 * Created: 8/8/2016 10:32:11 PM
 *  Author: searobin
 */

#include <Arduino.h>
#include <Wire.h>
#include <ADIGasDetector.h>


float calculateFeedbackResistor(void);
uint16_t setResistorValue(float resistor);

//  Gas Sensor Variables
uint16_t ui16sensorRange = 2000;      //value is in units (PPM)
uint16_t ui16sensitivity =   65;     //value is in units (nA/ppm)

// AD5270 variables
float fResistorValue = 0;
uint16_t ui16RdacWord = 0;


// Main variables
double fAdcVoltage = 0;
float fConcentration = 0;


// the setup function runs once when you press reset or power the board
void setup() {

    
    //Initiate the Wire library and join the I2C bus
    Wire.begin();

    SerialUSB.begin(115200);
    // Waiting for the USB serial connection
    while (!SerialUSB) {;}
     SerialUSB.println("Init Serial done");  
    adiGas.begin();

    SerialUSB.print("FwVersion = ");
    SerialUSB.println(adiGas.readFwVersion(), HEX);

    uint16_t reostate = adiGas.readRawRheostate();
    SerialUSB.print("Starting Rehostate Value = ");
    SerialUSB.println(reostate, HEX);
        
    // set digipot value
    fResistorValue = calculateFeedbackResistor();
    ui16RdacWord = setResistorValue(fResistorValue);
    adiGas.writeRawRheostate(ui16RdacWord);
  
     // uint16_t readRheostateMemory(uint8_t index);
    //uint16_t storeRheostateInMemory();

    SerialUSB.print("Free Rhostate Memory positions = ");
    SerialUSB.println(adiGas.readRheostateFreeMemory());
    //SerialUSB.print("Read Rhostate position 0 = ");
    //SerialUSB.println(adiGas.readRheostateMemory(1));
    
    //AD5270.writeAd5270 (HI_Z_PREP, 0x8001);  // Putting Rheostat into high Z mode on SDO line
    //AD5270.writeAd5270 (HI_Z, 0x0000);
        
    SerialUSB.print("Setting Rehostate Value = ");
    SerialUSB.print(fResistorValue);
    SerialUSB.print(" Ohm");
    SerialUSB.print("    (Word = ");
    SerialUSB.print(ui16RdacWord, HEX);
    SerialUSB.println(")");
    //SerialUSB.print("Storing for the next time Rhostate = ");
    
    //SerialUSB.println(adiGas.storeRheostateInMemory());
}

// the loop function runs over and over again forever
void loop() {

    long ui16Adcdata = 0;
    uint16_t reostate = 0;
    float fTemp = 0;
    digitalWrite(PIN_LED, HIGH); // turn the LED on
    delay(100);
    
    SerialUSB.println();
    SerialUSB.println();
    SerialUSB.println("reading temp ...");
    fTemp = adiGas.readTemperature();
    SerialUSB.print("Temperature: ");
    SerialUSB.print(fTemp);
    SerialUSB.println(" Celsius");

    reostate = adiGas.readRawRheostate();

    if (reostate != ui16RdacWord) {
        SerialUSB.print("Reostate Value = differs from the one configured!");
        SerialUSB.print("Read: ");
        SerialUSB.print(reostate);
        SerialUSB.print("        Configured: ");
        SerialUSB.println(ui16RdacWord);
    }    

    ui16Adcdata = adiGas.readRawCO2();
    
    //fAdcVoltage = ((ui16Adcdata / pow(2,15))-1)*1.2;    // Formula for input voltage using bipolar configuration
    fAdcVoltage = (((double(ui16Adcdata)) / pow(2,28)))*1.2;    // Formula for input voltage using bipolar configuration
    
    fConcentration = (abs(fAdcVoltage)/ (ui16RdacWord*(20000.0/1024.0))) / (ui16sensitivity*pow(10,-9));
    SerialUSB.print("ADC Data Reg value = ");
    SerialUSB.println(ui16Adcdata, HEX);
    SerialUSB.print("Sensor Voltage value uV = ");
    SerialUSB.println((1000000*fAdcVoltage));

    SerialUSB.print("Carbon Monoxide (CO) concentration = ");
    SerialUSB.print(fConcentration,2);  //display gas concentration
    SerialUSB.println(" PPM");


    digitalWrite(PIN_LED, LOW); // turn the LED off
    delay(2000);                // wait for a second

}



float calculateFeedbackResistor(void)
{
  float fFeedback = 0;
  fFeedback = 1.2 / (ui16sensorRange * ui16sensitivity * pow(10,-9));     //1.2 is the Vref of the circuit
  return fFeedback;
}

uint16_t setResistorValue(float resistor)
{
  uint16_t ui16RdacCode = 0;
  ui16RdacCode = int(resistor / (20000/1024));
  return ui16RdacCode;
}
