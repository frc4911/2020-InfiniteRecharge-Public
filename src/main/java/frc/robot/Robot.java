/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import frc.robot.auto.AutoModeExecuter;
import frc.robot.auto.AutoModeSelector;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.ColorReader;
import frc.robot.subsystems.Donger;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.JSticks;
import frc.robot.subsystems.PanelManipulator;
import frc.robot.subsystems.RobotStateEstimator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Limelights.CollectwardsLimelight;
import frc.robot.subsystems.Limelights.ShootwardsLimelight;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.loops.Looper;
import com.team254.lib.subsystems.SubsystemManager;
import com.team254.lib.util.CrashTracker;


import cyberlib.utils.RobotName;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {

	@SuppressWarnings("unused")
	private RobotName robotName = new RobotName(Constants.kRobot1Name);
	private SubsystemManager mSubsystems;
	private Superstructure mSuperstructure;
	private Swerve mSwerve;
	private Indexer mIndexer;
	private Collector mCollector;
	private Shooter mShooter;
	private Climber mClimber;
	private Donger mDonger;
	private RobotStateEstimator mRobotStateEstimator;
	@SuppressWarnings("unused")
	private PanelManipulator mPanelManipulator;
	@SuppressWarnings("unused")
	private CollectwardsLimelight mCollectwardsLimelight;
	private ShootwardsLimelight mShootwardsLimelight;
	private JSticks mJStick;
	@SuppressWarnings("unused")
	private ColorReader mColorReader;
	private final double mLoopPeriod = .005;
	private Looper mSubsystemLooper = new Looper(mLoopPeriod,Thread.NORM_PRIORITY+1);

	private RobotState robotState;
    private String mClassName;

	private AutoModeExecuter mAutoModeExecuter;
	private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
	private AutoModeSelector mAutoModeSelector;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		mClassName = this.getClass().getSimpleName();
		LiveWindow.setEnabled(false);
		LiveWindow.disableAllTelemetry();
		// if any subsystems are missing then comment next two
		// references

		robotState = RobotState.getInstance(mClassName);
		// // note superstructure turns on most subsystems
		mSuperstructure = Superstructure.getInstance(mClassName);
		mSwerve = Swerve.getInstance(mClassName);
		mIndexer = Indexer.getInstance(mClassName);
		mCollector = Collector.getInstance(mClassName);
		mDonger = Donger.getInstance(mClassName);
		mShooter = Shooter.getInstance(mClassName);
		mClimber = Climber.getInstance(mClassName);
		mShootwardsLimelight = ShootwardsLimelight.getInstance(mClassName);
		// mCollectwardsLimelight = CollectwardsLimelight.getInstance(mClassName);
		mRobotStateEstimator = RobotStateEstimator.getInstance(mClassName);
		// mColorReader = ColorReader.getInstance(mClassName);
		mJStick = JSticks.getInstance(mClassName);
		// mPanelManipulator = PanelManipulator.getInstance(mClassName);

		mSubsystems = SubsystemManager.getInstance(mClassName);
		mSubsystems.initializeSubsystemManager( (int)(mLoopPeriod*1000),
				Arrays.asList(
						mJStick,
						mSuperstructure,
						mSwerve,
						mIndexer,
						mCollector,
						mDonger,
						mShooter,
						mClimber,
						mShootwardsLimelight,
						mRobotStateEstimator
						// mCollectwardsLimelight,
						// mColorReader
						// mPanelManipulator
						)
				);

		// ask each subsystem to register itself
		mSubsystems.registerEnabledLoops(mSubsystemLooper);

		if (mSwerve != null) {
			mSwerve.zeroSensors();
			mSwerve.zeroSensors(new Pose2d());

			// robotState.feignVisionTargets();
			// mSwerve.startTracking(Constants.kDiskTargetHeight, new Translation2d(-6.0,
			// 0.0), true, new Rotation2d());
			mSwerve.stop();
		}

		mAutoModeSelector = new AutoModeSelector();
		mTrajectoryGenerator.generateTrajectories();
	}

	public void allPeriodic() {
		// if no pigeon comment next line
		// Pigeon.getInstance().outputToSmartDashboard();
	}

	public void autoConfig() {
		if (mSwerve != null) {
			mSwerve.zeroSensors();
			mSwerve.setNominalDriveOutput(0.0);
			mSwerve.requireModuleConfiguration();
			mSwerve.set10VoltRotationMode(true);
		}
	}

	public void teleopConfig() {
		if (mSwerve != null) {
			mSwerve.setNominalDriveOutput(0.0);
			mSwerve.set10VoltRotationMode(false);
		}
	}

	@Override
	public void autonomousInit() {
		System.out.println("AutonomousInit");
		try {
			autoConfig();

			mSubsystemLooper.stop();
			mSubsystemLooper.start();

			if (mSwerve != null) {
				mSwerve.setCarpetDirection(robotState.onStandardCarpet());
			}

			mAutoModeExecuter.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		allPeriodic();
	}

	@Override
	public void teleopInit() {
	try {
			mSubsystemLooper.stop();
			mSubsystemLooper.start();
			teleopConfig();
			robotState.enableXTarget(false);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		try {
			// teleopRoutines();
			// allPeriodic();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledInit() {
		try {
			System.gc();
			if (mAutoModeExecuter != null) {
				mAutoModeExecuter.stop();
			}

			mAutoModeExecuter = new AutoModeExecuter();

			mSubsystemLooper.stop();
			mSubsystemLooper.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic() {
		try {
			mAutoModeExecuter.setAutoMode(mAutoModeSelector.getSelectedAutoMode());
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {
	}

	@Override
	public void testPeriodic() {
	}
}