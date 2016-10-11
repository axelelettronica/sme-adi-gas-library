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
#include <SmeSFX.h>

bool debug = false;

/******************     Section Gas Detector     *****************/

float calculateFeedbackResistor(void);
uint16_t setResistorValue(float resistor);

//  Gas Sensor Variables
uint16_t ui16sensorRange = 2000;      //value is in units (PPM)
uint16_t ui16sensitivity =  65;     //value is in units (nA/ppm)

// AD5270 variables
float fResistorValue = 0;

uint16_t ui16RdacWord = 0;
uint16_t ui16Temp = 0;
int32_t i32Adcdata = 0;


uint16_t ui16Reostate = 0;
float fTemp = 0;

    
// Main variables
float fAdcVoltage = 0;
float fConcentration = 0;

void collectGasShieldData(void)
{
    if(debug)SerialUSB.println();
    if(debug)SerialUSB.println();
    if(debug)SerialUSB.print("Raw Temperature: ");
    ui16Temp = adiGas.readRawTemperature();
    delay (100);
    if(debug)SerialUSB.print(ui16Temp);
    if(debug)SerialUSB.println("");
    
    ui16Reostate = adiGas.readRawReostate();
    delay (100);
    if ((debug) && (ui16Reostate != ui16RdacWord)) {
        SerialUSB.print("Reostate Value = differs from the one configured!");
        SerialUSB.print("Read: ");
        SerialUSB.print(ui16Reostate);
        SerialUSB.print("        Configured: ");
        SerialUSB.println(ui16RdacWord);
    }    

    i32Adcdata = adiGas.readRawCO2();
    delay (100);
    //fAdcVoltage = ((i32Adcdata / pow(2,15))-1)*1.2;    // Formula for input voltage using bipolar configuration
    float fVolts   = (1.2 / 268435456);      // Internal reference, calculate lsb size in volts
    fAdcVoltage = (i32Adcdata * fVolts);   // Calculate ADC result in volts
    
    fConcentration = (abs(fAdcVoltage)/ (ui16RdacWord*(20000/1024))) / (ui16sensitivity*pow(10,-9));
    if(debug) {
        SerialUSB.print("ADC Data Reg value = ");
        SerialUSB.println(i32Adcdata, HEX);
        SerialUSB.print("Sensor Voltage value x1000 = ");
        SerialUSB.println((fAdcVoltage*1000));

        SerialUSB.print("Carbon Monoxide (CO) concentration = ");
        SerialUSB.print(fConcentration,2);  //display gas concentration
        SerialUSB.print(" PPM  = ");
  
        SerialUSB.print((fConcentration*1.23),2);  //display gas concentration
        SerialUSB.println(" mg/m3");
    }
}

/******************     Section Sigfox     *****************/

char helloMsg[12]= {1,0,0,0,0,0,0,0,0,0,0,0};

bool messageSent;

bool getSfxAnswer(void)
{
    bool received = false; 
    if (sfxAntenna.getSfxMode() == sfxDataMode) {

        switch (sfxAntenna.sfxDataAcknoledge()) {
        case SFX_DATA_ACK_START:
                    if(debug) SerialUSB.println("Waiting Answer");
        break;

        case SFX_DATA_ACK_PROCESSING:
                    if(debug)SerialUSB.print('.');
        break;

        case SFX_DATA_ACK_OK:
                   // ledGreenLight(HIGH);
                    if(debug)SerialUSB.println(' ');
                    if(debug)SerialUSB.println("Answer OK :) :) :) :)");
                    received = true;
        break;

        case SFX_DATA_ACK_KO:
                   // ledRedLight(HIGH);
                    if(debug)SerialUSB.println(' ');
                    if(debug)SerialUSB.println("Answer KO :( :( :( :(");
                    received = true;
        break;
        }
     }
     return received;
 }


/***********************************************************/

// the setup function runs once when you press reset or power the board
void setup() {

    
    //Initiate the Wire library and join the I2C bus
    Wire.begin();
    
    adiGas.begin();
    SerialUSB.begin(115200);
     
    // Waiting for the USB serial connection
    if(debug) while (!SerialUSB) {;}
    
    // set digipot value
    fResistorValue = calculateFeedbackResistor();
    ui16RdacWord = setResistorValue(fResistorValue);
    adiGas.writeRawReostate(ui16RdacWord);
  
  
    //AD5270.writeAd5270 (HI_Z_PREP, 0x8001);  // Putting Rheostat into high Z mode on SDO line
    //AD5270.writeAd5270 (HI_Z, 0x0000);
    if(debug) {    
    SerialUSB.print("Setting Rehostate Value = ");
    SerialUSB.print(fResistorValue);
    SerialUSB.print(" Ohm");
    SerialUSB.print("    (Word = ");
    SerialUSB.print(ui16RdacWord);
    SerialUSB.println(")");
    }

    sfxAntenna.begin();
    int initFinish=1;

    if(debug)SerialUSB.println("SFX in Command mode");
    sfxAntenna.setSfxConfigurationMode(); // enter in configuration Mode

    do {
        uint8_t answerReady = sfxAntenna.hasSfxAnswer();
        if (answerReady){
            switch (initFinish){
            case 1:                                
                if(debug)SerialUSB.println("SFX in Data mode");
                sfxAntenna.setSfxDataMode();
                initFinish++;
                break;

            case 2:
                initFinish++; // exit
                break;
            }
        }
    } while (initFinish!=3);

    if(debug)SerialUSB.println("sending Data to SigFox Network ...");
}

// the loop function runs over and over again forever
void loop() 
{
    bool received = false;

    digitalWrite(PIN_LED, HIGH); // turn the LED on
    delay(100);

    collectGasShieldData();

    // Rehostate MSB
    helloMsg[4] = ((ui16RdacWord >> 8) & 0xFF);
    // Rehostate LSB
    helloMsg[5] = ui16RdacWord & 0xFF;
    // Temperature MSB
    helloMsg[6] = ((ui16Temp >> 8) & 0xFF);
    // Temperature LSB
    helloMsg[7] = ui16Temp & 0xFF;
    // Gas MSB Value
    helloMsg[8] = ((i32Adcdata >> 24) & 0xFF);
    // Gas LSB Value
    helloMsg[9] = ((i32Adcdata >> 16) & 0xFF);
    // Gas MSB Value
    helloMsg[10] = ((i32Adcdata >> 8) & 0xFF);
    // Gas LSB Value
    helloMsg[11] = i32Adcdata & 0xFF;
    
    if (debug) {
        SerialUSB.print("Data: ");
        for (int i = 0; i < 12; ++i) {
            if (helloMsg[i] < 16) SerialUSB.print("0");
            SerialUSB.print(helloMsg[i], HEX);
            if (i%2) SerialUSB.print(" ");
        }
    }
    
    // send Hello on the air
    sfxAntenna.sfxSendData(helloMsg, 12 /*strlen((char*)helloMsg)*/);

    while (!received) {
        if (sfxAntenna.hasSfxAnswer()) {
            received = getSfxAnswer();
        }
    }
    digitalWrite(PIN_LED, LOW); // turn the LED off
    delay(1200000);                // wait for 20 minutes  
    // delay(120000);                // wait for 2 minutes  
}

/***********************************************/

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
