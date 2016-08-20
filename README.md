# Analog Devices Gas Detector shield 
----

This is a library for the ADI Gas Detector shield board.

The ADI Gas Detector is a mkr1000-format shield mounting a ADuCM2361 microcontroller interacting with:

- ADT7320UCPZ-R2 temperature sensor
- AD5270BRMZ-20 digita rheostat

The Digital rheostat is used to calibrate the Gas Sensor circuitry depending on the mounted sensor.

The ADuCM2361 has a preloaded firmware managing the Temperature and the Gas sensors and providing the
related management to the Arduino boards torhugh the I2C bus using this library.
 
* [ AD ADT7320UCPZ-R2 Data Sheet ](http://www.analog.com/media/en/technical-documentation/data-sheets/ADT7320.pdf)
* [ AD AD5270BRMZ-20 Data Sheet ](http://www.analog.com/media/en/technical-documentation/data-sheets/AD5270_5271.pdf)

Host control and result reading is performed using a I2C interface, no extra pin are required.

It was principally designed to work with the ADI board, but could
be easily adapted and used on every Arduino and Arduino Certified boards.

Written by Seve <seve@axelelettronica.it>.

## Repository Contents
-------------------
* **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE. 
* **/src** - Source files for the library (.cpp, .h).
* **library.properties** - General library properties for the Arduino package manager.

## Releases
---
#### v1.0.0 First Release

## Documentation
--------------

* **[Installing an Arduino Library Guide](http://www.arduino.cc/en/Guide/Libraries#toc3)** - How to install a Adi library on the Arduino IDE using the Library Manager


## License Information
-------------------

Copyright (c) Amel Technology. All right reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

