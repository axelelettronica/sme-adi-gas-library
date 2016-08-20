/*
 * Temperature.ino
 *
 * Example of Temperature reading 
 * In this sketch the temperature is read from the Gas Detector sheild
 *
 * Created: 8/8/2016 10:32:11 PM
 *  Author: searobin
 */

#include <Wire.h>

#include <ADIGasDetector.h>
#include <Arduino.h>


// the setup function runs once when you press reset or power the board
void setup() {
    //Initiate the Wire library and join the I2C bus
    Wire.begin();
    
    adiGas.begin();
    SerialUSB.begin(115200);
}

// the loop function runs over and over again forever
void loop() {

    int data = 0;

    digitalWrite(PIN_LED, HIGH); // turn the LED on
    delay(100);

    data = adiGas.readTemperature();
    SerialUSB.print("Temperature: ");
    SerialUSB.print(data);
    SerialUSB.println(" celsius");


    digitalWrite(PIN_LED, LOW); // turn the LED off
    delay(1000);                // wait for a second

}
