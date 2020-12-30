Driver for the Adafruit as7262 Sensor
---
The AS7262 class provides an interface to some of the color-finding capabilities of the Adafruit as7262 sensor. It uses I2C communications to interface to the RoboRIO.
 
This sensor has 6 integrated visible light sensing channels for red, orange, yellow, green, blue, and violet. These channels can be read via the I2C bus as either raw 16-bit values or calibrated floating-point values. There is also an on-board temperature sensor that can be used to read the temperature of the chip, and a powerful LED flash to reflect light off objects for better color detection.

This code is inspired by:
 <ul>
  <li>The KauaiLabs navX Robotics Navigation Sensor library</li>
  <li>This is ported from  <a href="https://github.com/adafruit/Adafruit_AS726x">Adafruit AS726x Library for Arduino</a>
 </ul>
 
<p> More <a href="https://www.adafruit.com/product/3779">adafruit.com resources</a>.