Team 4911's 2020 Codebase
===

## Setup Instructions

### General
1. Clone this repo
1. Run `./gradlew` to download gradle and needed FRC/Vendor libraries
1. Run `./gradlew tasks` to see available options
1. Enjoy!

### Visual Studio Code (Official IDE)
1. Get the WPILib extension for easiest use from the VSCode Marketplace - Requires Java 11 or greater
1. In [`.vscode/settings.json`](.vscode/settings.json), set the User Setting, `java.home`, to the correct directory pointing to your JDK 11 directory

### IntelliJ
1. Run `./gradlew idea`
1. Open the `robot-2020.ipr` file with IntelliJ

### Basic Gradle Commands
* Run `./gradlew deploy` to deploy to the robot in Terminal (*nix) or Powershell (Windows)
* Run `./gradlew build` to build the code.  Use the `--info` flag for more details
* Run `./gradlew assemble` to build the code without running all unit tests.  Use the `--info` flag for more details
* Run `./gradlew test` to run all of the JUnit tests

## Projects
We break our code up into robot code and libraries.

* [src](src) contains the 2020 robot code and utilities.
* [cheesylib](cheesylib) written by Team 254 for the 2018-2019 season.
* [madtownlib](madtownlib) written by Team 1323 for the 2019 season based on Team 254 code for the 2018 season.
* [cyberlib](cyberlib) written by Team 4911 for the 2020 season.
* [as7262](as7262) Driver for the Adafruit as726x 6 channel color sensor.
* [vl53l0x](vl53l0x) Driver for the STMicroelectronics VL53L0X time of flight laser-ranging sensor.

## [cheesylib](cheesylib)
cheesylib is code that is shared between all robots. It was written by Team 254  Cheesey Poofs from season 2018-2019.  
It also includes additional rotation and translation support needed for sverve drive modules courtesy of Team 1323 MadTown Robotics.

## [madtownlib](madtownlib)
madtownlib is code for swerve drive modules courtesy of Team 1323 MadTown Robotics from season 2019.

## [cyberlib](cyberlib)
cyberlib is utility code for io controllers, annotations, logging, etc.
