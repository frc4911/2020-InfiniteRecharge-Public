# CheesyLib

This is 4911's copy of team 254's 2018 FRC robot code written in Java and  based off of WPILib's Java control system and refactored as a library.

The code is divided into several packages, each responsible for a different aspect of the robot function.  Additional information about each specific class can be found in that class' Java file.

## Code Highlights

- Path following with a nonlinear feedback controller and splines.

	To control autonomous driving, the robot utilizes a [nonlinear feedback controller](../src/main/java/com/team4911/frc2019/planners/DriveMotionPlanner.java#L263) and drives paths constructed of [quintic Hermite splines](src/main/java/com/team254/lib/spline/QuinticHermiteSpline.java).

## Package Functions
- com.team254.lib.auto

	Defines the classes used to create and execute autonomous routines.  Contains the `actions` packages.
	
- com.team254.lib.auto.actions

	Defines the classes to implement actions used during the autonomous period, which all share a common interface, [`Action`](src/main/java/com/team254/lib/auto/actions/Action.java) (also in this package). Examples driving a trajectory, or moving the elevator.  Action routines interact with Subsystems, which interact with the hardware.

- com.team254.lib.drivers

    Contains a set of custom classes for TalonSRXs.
	
- com.team254.lib.geometry

    Contains a set of classes that represent various geometric entities.

- com.team254.lib.loops

	Loops are routines that run periodically on the robot, such as calculating robot pose, processing vision feedback, or updating subsystems. All loops implement the `Loop` interface and are handled (started, stopped, added) by the `Looper` class, which runs at 100-200 Hz.
    
	The `Robot` class typically implements two main loopers: `mEnabledLooper`, which runs all loops when the robot is enabled, and `mDisabledLooper` which can be used to run Subsystem loops when robot is disabled, e.g. to process auto mode selections.  Be careful not to 

- com.team254.lib.physics

    Contains classes that model DC motor transmissions and differential drive characterization.
	
- com.team254.lib.subsystems
	
	Subsystems are consolidated into one central class per Subsystem, all of which extend the Subsystem abstract class. Subsystems implement state machines for control by implementing [Loop](src/main/java/com/team254/lib/loops/Loop.java).
	Each Subsystem is also a singleton, meaning that there is only one instance of each. To modify a Subsystem, one would get the instance of the Subsystem and change its state. The `Subsystem` class works on setting the desired state.
	
	The `Robot` class creates a [SubsystemManager](src/main/java/com/team254/lib/subsystems/SubsystemManager.java) with the Subsystems it is to manage.  It's responsible for calling each Subsystem's loop methods.

- com.team254.lib.spline

    Contains the code for generating and optimizing splines.

- com.team254.lib.trajectory

    Contains classes for following and storing trajectories.

- com.team254.lib.trajectory.timing

	Contains classes for fitting trajectories with time profiles.

- com.team254.lib.util

    A collection of assorted utilities classes used in the robot code. Check each file for more information.
