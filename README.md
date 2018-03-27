# Electronic Level

Arduino code for an electronic level designed to fit ontop of a bubble level. 
Provides a readout of the degrees off of the X and Y axis on the OLED screen. 
Below the readout it will say either "Stable" or "Unstable". 
A "Stable" readout indicates that the angle being displayed has been calculated to an accuracy of +/- .125 deg. 
An "Unstable" readout indicates that the angle being calculated by your arduino is changing by more than +/- .125 deg over the previous 200 calculations.

## Getting Started
To implement this on your own device just connect the Arduino and all of the components according to the "#Wiring" section below
once wired upload Electronic_Level.ino to your Arduino.


## Required Libraries

SPI.h

Wire.h

i2cdevlib.h < https://github.com/jrowberg/i2cdevlib >

Kalman.h  < https://github.com/TKJElectronics/KalmanFilter >

Adafruit_GFX.h  < https://github.com/adafruit/Adafruit-GFX-Library >

Adafruit_SSD1306.h < https://github.com/adafruit/Adafruit_SSD1306 >

## Wiring 
| MPU6050       | Arduino       | 
| ------------- |--------------:| 
| Vcc           | +5v           | 
| GND           | GND           |  
| SCL           | A5            |    
| SDA           | A4            |
| AD0           | GND           |
| Int           | D2            |


| OLED    | Arduino     |
|---------|------------:|
| GND     | GND         |
| Vcc     | +5v         |
| SCL     | A5          |
| SDA     | A4          |


## Authors
Austin Wohlert - eenginerd.com

## License
This project has no license currently

## Acknowledgments 

**Kristian Lauszus:**

The Kalman and Complimentary filter code along with angle calculations used in my Electronic Level project is just a modified version of Kristian Lauszus' example sketch in the Kalman Filter library.   

**Jeff Rowberg:**

Accelerometer and gyroscope data collected using the MPU-6050 is collected using Jeff Rowberg's I2Cdev and MPU6050 libraries. 
