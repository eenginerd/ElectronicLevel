# ElectronicLevel

Arduino code for an electronic level designed to fit ontop of a bubble level. 
Once uploaded to an Arduino wired according to the "Wiring" section below a readout of the degrees off of the X and Y axis will be displayed on the OLED screen. 
In addition, below the readout will be either "Stable" or "Unstable". 
Once you see the "Stable" readout you can then trust that the angle being displayed on the OLED has been calculated to an accuracy of +/- .125 deg. 

## Getting Started
To implement this on your own device just connect the Arduino and all of the components according to the "#Wiring" section below
once wired upload Electronic_Level.ino to your Arduino.


## Prerequisites
Libraries needed:
i2cdevlib < https://github.com/jrowberg/i2cdevlib >

Kalman Filter Library  < https://github.com/TKJElectronics/KalmanFilter >

SPI

Wire

Adafruit_GFX  < https://github.com/adafruit/Adafruit-GFX-Library >

Adafruit_SSD1306 < https://github.com/adafruit/Adafruit_SSD1306 >

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

Kristian Lauszus:
This project is based around Kristian Lauszus' sketch which is included as an example in the Kalman Filter library. 

Jeff Rowberg:
Accelerometer and gyroscope data collected using the MPU-6050 is collected using Jeff Rowberg's I2Cdev and MPU6050 libraries. 
