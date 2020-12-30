Driver for the STMicroelectronics VL53L0X Sensor
---
The VL53L0X class provides an interface to some of the range-finding capabilities of the STMicroelectronics VL53L0X sensor. It uses I2C communications to interface to the RoboRIO.
 
The VL53L0X is a laser-ranging sensor, which measures the range to a target an 
object up to 2 m away. It uses time-of-flight measurements of infrared pulses for ranging,
allowing it to give accurate results independent of the target’s color and surface.

This code is inspired by:
 <ul>
  <li>The KauaiLabs navX Robotics Navigation Sensor library</li>
  <li>FRC team 5461 V.E.R.N.'s <a href="https://github.com/FRC-Team-Vern/VL53L0X_Example">java port</a>
  of the <a href="https://github.com/pololu/vl53l0x-arduino">VL53L0X library for Arduino</a>.</li>
 </ul>
 
<p> More <a href="https://www.pololu.com/product/2490/resources">pololu.com resources</a>.