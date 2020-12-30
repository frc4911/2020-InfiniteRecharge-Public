package frc.robot.subsystems;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.subsystems.Subsystem;

import cyberlib.io.CW;
import cyberlib.io.LogitechPS4;
import cyberlib.io.Turnigy;
import cyberlib.io.Xbox;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class JSticks extends Subsystem {

    public enum SystemState {
        READINGBUTTONS,
    }

    public enum WantedState {
        READBUTTONS,
    }

    private SystemState mSystemState = SystemState.READINGBUTTONS;
    private WantedState mWantedState = WantedState.READBUTTONS;
    public PeriodicIO mPeriodicIO;
    @SuppressWarnings("unused")
    private boolean mStateChanged;
    private final boolean mLoggingEnabled = true; // used to disable logging for this subsystem only
    private Turnigy mDriver;
    private Xbox mOperator;
    private LogitechPS4 mTester;
    private final double mDeadBand = 0.05; // for the turnigy (driver) swerve controls
    private String mPrevGameState = "";
	private Superstructure mSuperstructure = null;
    private Swerve mSwerve = null;
    private ColorReader mColorReader = null;
    private final int mDefaultSchedDelta = 100; // axis updated every 100 msec
    @SuppressWarnings("unused")
    private int mListIndex;

    private static String sClassName;
    private static int sInstanceCount;
    private static JSticks sInstance = null;
    public  static JSticks getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new JSticks(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private JSticks(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mDriver = new Turnigy();
        mOperator = new Xbox();
        mPeriodicIO = new PeriodicIO();
        mSuperstructure = Superstructure.getInstance(sClassName);
        mSwerve = Swerve.getInstance(sClassName);
        // mTester = new LogitechPS4();
        if (mTester != null){
            mColorReader = ColorReader.getInstance(sClassName);
        }
    }

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(Phase phase) {
            synchronized (JSticks.this) {
                mSystemState = SystemState.READINGBUTTONS;
                mWantedState = WantedState.READBUTTONS;
                mStateChanged = true;
                System.out.println(sClassName + " state " + mSystemState);
                switch (phase) {
                    case DISABLED:
                        mPeriodicIO.schedDeltaDesired = 0; // goto sleep
                        break;
                    default:
                        mPeriodicIO.schedDeltaDesired = mDefaultSchedDelta;
                        break;
                }
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (JSticks.this) {
                SystemState newState;
                switch (mSystemState) {
                case READINGBUTTONS:
                default:
                    newState = handleReadingButtons();
                    break;
                }

                if (newState != mSystemState) {
                    System.out.println(sClassName + " state " + mSystemState + " to " + newState + " (" + timestamp + ")");
                    mSystemState = newState;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    private SystemState handleReadingButtons() {
        teleopRoutines();
        testRoutines();
        
        return defaultStateTransfer();
    }

    public void teleopRoutines() {
		Superstructure.WantedState currentState = mSuperstructure.getWantedState();
		Superstructure.WantedState previousState = currentState;

		double swerveYInput = mPeriodicIO.drRightStickX_Translate;
		double swerveXInput = mPeriodicIO.drRightStickY_Translate;
		double swerveRotationInput = mPeriodicIO.drLeftStickX_Rotate;

		if (!mPeriodicIO.drRightToggleDown_SHOOT) {
			mSwerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, mPeriodicIO.drLeftToggleDown_RobotOrient, false);
		}

		if (mPeriodicIO.drMidButton_ResetIMU) {
			mSwerve.temporarilyDisableHeadingController();
			mSwerve.zeroSensors(Constants.kRobotStartingPose);
			mSwerve.resetAveragedDirection();
		}

		if (mPeriodicIO.opXButton_IdleShooter) {
            mSuperstructure.setShooterHoldSpeed(0.0);
		}

		if (currentState == Superstructure.WantedState.CLIMB) {
			mSuperstructure.setClimbOpenLoop(mPeriodicIO.opLeftStickY_ClimbSpeed);
		}

        currentState = activeBtnIsReleased(currentState);
		if (currentState == Superstructure.WantedState.HOLD) {
			if (mPeriodicIO.drRightToggleDown_SHOOT) {
				mSuperstructure.setWantedState(Superstructure.WantedState.SHOOT);
			} else if (mPeriodicIO.opRightTrigger_COLLECT) {
				mSuperstructure.setWantedState(Superstructure.WantedState.COLLECT);
			} else if (mPeriodicIO.opLeftBumper_CLIMB) {
				mSuperstructure.setWantedState(Superstructure.WantedState.CLIMB);
			} else if (mPeriodicIO.opPOV0_MANUAL10) {
				mSuperstructure.setManualShootDistance(10);
			} else if (mPeriodicIO.opPOV90_MANUAL15) {
				mSuperstructure.setManualShootDistance(15);
			} else if (mPeriodicIO.opPOV180_MANUAL20) {
				mSuperstructure.setManualShootDistance(20);
			} else if (mPeriodicIO.opPOV270_MANUAL25) {
				mSuperstructure.setManualShootDistance(25);
			} else if (mPeriodicIO.opLeftTrigger_CLEARBALLS) {
				mSuperstructure.setWantedState(Superstructure.WantedState.CLEAR_BALLS);
			} else if (previousState != currentState) {
				mSuperstructure.setWantedState(Superstructure.WantedState.HOLD);
			}
		}
	}

	private Superstructure.WantedState activeBtnIsReleased(Superstructure.WantedState currentState) {
		switch (currentState) {
			case SHOOT:
				return !mPeriodicIO.drRightToggleDown_SHOOT ? Superstructure.WantedState.HOLD : currentState;
			case COLLECT:
				return !mPeriodicIO.opRightTrigger_COLLECT ? Superstructure.WantedState.HOLD : currentState;
			case CLIMB:
				return !mPeriodicIO.opLeftBumper_CLIMB ? Superstructure.WantedState.HOLD : currentState;
			case MANUAL_SHOOT:
				if (!mPeriodicIO.opPOV0_MANUAL10 && !mPeriodicIO.opPOV90_MANUAL15 && !mPeriodicIO.opPOV180_MANUAL20 && !mPeriodicIO.opPOV270_MANUAL25) {
					return Superstructure.WantedState.HOLD;
				}
				return currentState;
			case CLEAR_BALLS:
				return !mPeriodicIO.opLeftTrigger_CLEARBALLS ? Superstructure.WantedState.HOLD : currentState;
			default:
                return Superstructure.WantedState.HOLD;
        }        
    }
    
    private void testRoutines(){
        if (mTester != null){
            if(mPeriodicIO.tsPOV0_READRED){
                mColorReader.setWantedState(ColorReader.WantedState.READ_RED);
            }
            else if(mPeriodicIO.tsPOV90_READGREEN){
                mColorReader.setWantedState(ColorReader.WantedState.READ_GREEN);
            }
            else if(mPeriodicIO.tsPOV180_READBLUE){
                mColorReader.setWantedState(ColorReader.WantedState.READ_BLUE);
            }
            else if(mPeriodicIO.tsPOV270_READCLEAR){
                mColorReader.setWantedState(ColorReader.WantedState.READ_CLEAR);
            }
            else if(mPeriodicIO.tsABUTTON_READALL){
                mColorReader.setWantedState(ColorReader.WantedState.READ_ALL);
            }
            else if(mPeriodicIO.tsBBUTTON_STARTSCAN){
                mColorReader.setWantedState(ColorReader.WantedState.SCAN);
            }
            else if(mPeriodicIO.tsXBUTTON_STOPSCAN){
                mColorReader.setWantedState(ColorReader.WantedState.HOLD);
            }
        }

    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
        case READBUTTONS:
        default:
            return SystemState.READINGBUTTONS;
        }
    }

    public synchronized void setWantedState(WantedState state) {
        mWantedState = state;
    }

    @Override
    public void stop() {
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        mListIndex = enabledLooper.register(mLoop);
    }

    @Override
    public String getLogHeaders() {
        if (mLoggingEnabled) {
            return sClassName+".date,"+
                   sClassName+".time,"+
                   sClassName+".alliance,"+
                   sClassName+".matchType,"+
                   sClassName+".matchNumber,"+
                   sClassName+".gameMessage,"+
                   sClassName+".eventName,"+
                   sClassName+".replayNumber,"+
                   sClassName+".gameState,"+
                   sClassName+".matchTime,"+
                   sClassName+".batteryVoltage,"+
                   sClassName+".batteryCurrent,"+
                   sClassName+".opRightBumper,"+
                   sClassName+".drRightStickX_Translate,"+
                   sClassName+".drRightStickY_Translate,"+
                   sClassName+".drLeftStickX_Rotate,"+
                   sClassName+".drMidButton_ResetIMU,"+
                   sClassName+".drRightToggleDown_SHOOT,"+
                   sClassName+".drLeftToggleDown_RobotOrient,"+
                   sClassName+".opXButton_IdleShooter,"+
                   sClassName+".opAButton,"+
                   sClassName+".opBButton,"+
                   sClassName+".opStartButton,"+
                   sClassName+".opRightTrigger_COLLECT,"+
                   sClassName+".opLeftTrigger_CLEARBALLS,"+
                   sClassName+".opLeftBumper_CLIMB,"+
                   sClassName+".opLeftStickY_ClimbSpeed,"+
                   sClassName+".opPOV0_MANUAL10,"+
                   sClassName+".opPOV90_MANUAL15,"+
                   sClassName+".opPOV180_MANUAL20,"+
                   sClassName+".opPOV270_MANUAL25,"+
                   sClassName+".schedDeltaDesired,"+
                   sClassName+".schedDeltaActual,"+
                   sClassName+".schedDuration";

        }
        return null;
    }

    private String generateLogValues(boolean telemetry) {

        String values;
        
        if (telemetry){
                
            // optional
            if (mPeriodicIO.ds.isDisabled()) {
                mPeriodicIO.gameState = "Disabled";
            } 
            else if (mPeriodicIO.ds.isOperatorControl()) {
                mPeriodicIO.gameState = "Teleop";
            }
            else if (mPeriodicIO.ds.isAutonomous()) {
                mPeriodicIO.gameState = "Autonomous"; 
            } 
            else if (mPeriodicIO.ds.isTest()) {
                mPeriodicIO.gameState = "Test";
            }

            if (!mPeriodicIO.gameState.equals(mPrevGameState)) {
                mPrevGameState = mPeriodicIO.gameState;
                mPeriodicIO.alliance = mPeriodicIO.ds.getAlliance().toString();
                mPeriodicIO.matchType = mPeriodicIO.ds.getMatchType().toString();
                mPeriodicIO.matchNumber = mPeriodicIO.ds.getMatchNumber();
                mPeriodicIO.eventName = mPeriodicIO.ds.getEventName();
                mPeriodicIO.replayNumber = mPeriodicIO.ds.getReplayNumber();
            }
            mPeriodicIO.batteryVoltage = mPeriodicIO.pdp.getVoltage();
            mPeriodicIO.batteryCurrent = mPeriodicIO.pdp.getTotalCurrent();

            values = ""+mPeriodicIO.date + "," +
                        mPeriodicIO.time + "," + 
                        mPeriodicIO.alliance + "," + 
                        mPeriodicIO.matchType + "," + 
                        mPeriodicIO.matchNumber + "," + 
                        mPeriodicIO.gameMessage + "," + 
                        mPeriodicIO.eventName + "," + 
                        mPeriodicIO.replayNumber + "," + 
                        mPeriodicIO.gameState + "," + 
                        mPeriodicIO.matchTime + "," + 
                        mPeriodicIO.batteryVoltage + "," + 
                        mPeriodicIO.batteryCurrent;
        }
        else {
            values = ""+/*mPeriodicIO.date + */"," +
                        /*mPeriodicIO.time + */"," + 
                        /*mPeriodicIO.alliance + */"," + 
                        /*mPeriodicIO.matchType + */"," + 
                        /*mPeriodicIO.matchNumber + */"," + 
                        /*mPeriodicIO.gameMessage + */"," + 
                        /*mPeriodicIO.eventName + */"," + 
                        /*mPeriodicIO.replayNumber + */"," + 
                        /*mPeriodicIO.gameState + */"," + 
                        /*mPeriodicIO.matchTime + */"," + 
                        /*mPeriodicIO.batteryVoltage + */"," 
                        /*mPeriodicIO.batteryCurrent*/;
        }
        values += "," + mPeriodicIO.opRightBumper + "," + 
                        mPeriodicIO.drRightStickX_Translate + "," + 
                        mPeriodicIO.drRightStickY_Translate + "," + 
                        mPeriodicIO.drLeftStickX_Rotate + "," + 
                        mPeriodicIO.drMidButton_ResetIMU + "," + 
                        mPeriodicIO.drRightToggleDown_SHOOT + "," + 
                        mPeriodicIO.drLeftToggleDown_RobotOrient + "," + 
                        mPeriodicIO.opXButton_IdleShooter + "," + 
                        mPeriodicIO.opAButton + "," + 
                        mPeriodicIO.opBButton + "," + 
                        mPeriodicIO.opStartButton + "," + 
                        mPeriodicIO.opRightTrigger_COLLECT + "," + 
                        mPeriodicIO.opLeftTrigger_CLEARBALLS + "," + 
                        mPeriodicIO.opLeftBumper_CLIMB + "," + 
                        mPeriodicIO.opLeftStickY_ClimbSpeed + "," + 
                        mPeriodicIO.opPOV0_MANUAL10 + "," + 
                        mPeriodicIO.opPOV90_MANUAL15 + "," + 
                        mPeriodicIO.opPOV180_MANUAL20 + "," + 
                        mPeriodicIO.opPOV270_MANUAL25;

        if(telemetry){
            values+= ","+
            /*mPeriodicIO.schedDeltaDesired+*/","+
            /*mPeriodicIO.schedDeltaActual+*/","
            /*mPeriodicIO.schedDuration*/;
        }
        else {
            
            mPeriodicIO.schedDuration = Timer.getFPGATimestamp() - mPeriodicIO.lastSchedStart;
            values+= ","+
            mPeriodicIO.schedDeltaDesired+","+
            mPeriodicIO.schedDeltaActual+","+
            mPeriodicIO.schedDuration;
        }
        
        return values;
    }

    @Override
    public String getLogValues(boolean telemetry) {
        if (mLoggingEnabled) {
            return generateLogValues(telemetry);
        }
        return null;
    }

    @Override
    public void readPeriodicInputs() {
        double now                   = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart   = now;

        mPeriodicIO.drMidButton_ResetIMU = mDriver.getButton(Turnigy.MID_BUTTON, CW.PRESSED_EDGE);
        mPeriodicIO.opXButton_IdleShooter = mOperator.getButton(Xbox.X_BUTTON, CW.PRESSED_EDGE);
        if(mTester != null){
            mPeriodicIO.tsPOV0_READRED = mTester.getButton(LogitechPS4.POV0_0, CW.PRESSED_EDGE);
            mPeriodicIO.tsPOV90_READGREEN = mTester.getButton(LogitechPS4.POV0_90, CW.PRESSED_EDGE);
            mPeriodicIO.tsPOV180_READBLUE = mTester.getButton(LogitechPS4.POV0_180, CW.PRESSED_EDGE);
            mPeriodicIO.tsPOV270_READCLEAR = mTester.getButton(LogitechPS4.POV0_270, CW.PRESSED_EDGE);
            mPeriodicIO.tsABUTTON_READALL = mTester.getButton(LogitechPS4.A_BUTTON, CW.PRESSED_EDGE);
            mPeriodicIO.tsBBUTTON_STARTSCAN = mTester.getButton(LogitechPS4.B_BUTTON, CW.PRESSED_EDGE);
            mPeriodicIO.tsXBUTTON_STOPSCAN = mTester.getButton(LogitechPS4.X_BUTTON, CW.PRESSED_EDGE);
        }

        // // button levels and raw values can be read every loop
        mPeriodicIO.drRightStickX_Translate = mDriver.getRaw(Turnigy.RIGHT_STICK_X, mDeadBand);
        mPeriodicIO.drRightStickY_Translate = mDriver.getRaw(Turnigy.RIGHT_STICK_Y, mDeadBand);
        mPeriodicIO.drLeftStickX_Rotate = mDriver.getRaw(Turnigy.LEFT_STICK_X, mDeadBand);
        mPeriodicIO.opLeftStickY_ClimbSpeed = mOperator.getRaw(Xbox.LEFT_STICK_Y, .08);
        mPeriodicIO.drRightToggleDown_SHOOT = mDriver.getToggle(Turnigy.RIGHT_TOGGLE, Turnigy.TOGGLE_DOWN);
        mPeriodicIO.drLeftToggleDown_RobotOrient = mDriver.getToggle(Turnigy.LEFT_TOGGLE, Turnigy.TOGGLE_DOWN);
        mPeriodicIO.opRightTrigger_COLLECT = mOperator.getButton(Xbox.RIGHT_TRIGGER, CW.PRESSED_LEVEL);
        mPeriodicIO.opLeftTrigger_CLEARBALLS = mOperator.getButton(Xbox.LEFT_TRIGGER, CW.PRESSED_LEVEL);
        mPeriodicIO.opLeftBumper_CLIMB = mOperator.getButton(Xbox.LEFT_BUMPER, CW.PRESSED_LEVEL);
        mPeriodicIO.opRightBumper = mOperator.getButton(Xbox.RIGHT_BUMPER, CW.PRESSED_LEVEL);
        mPeriodicIO.opPOV0_MANUAL10 = mOperator.getButton(Xbox.POV0_0, CW.PRESSED_LEVEL);
        mPeriodicIO.opPOV90_MANUAL15 = mOperator.getButton(Xbox.POV0_90, CW.PRESSED_LEVEL);
        mPeriodicIO.opPOV180_MANUAL20 = mOperator.getButton(Xbox.POV0_180, CW.PRESSED_LEVEL);
        mPeriodicIO.opPOV270_MANUAL25 = mOperator.getButton(Xbox.POV0_270, CW.PRESSED_LEVEL);

        mPeriodicIO.gameMessage = mPeriodicIO.ds.getGameSpecificMessage();
        mPeriodicIO.matchTime = mPeriodicIO.ds.getMatchTime();
    }

    @Override
    public void writePeriodicOutputs() {
    }

    @Override
    public int whenRunAgain () {
        return mPeriodicIO.schedDeltaDesired;
    }

    @Override
    public void outputTelemetry() {
    }

    public static class PeriodicIO {
        // LOGGING
        private LocalDateTime myDateObj = LocalDateTime.now();
        public String date = myDateObj.format(DateTimeFormatter.ofPattern("dd-MM-yyyy"));
        public String time = myDateObj.format(DateTimeFormatter.ofPattern("HH:mm:ss"));
        public  int    schedDeltaDesired;
        public  double schedDeltaActual;
        public  double schedDuration;
        private double lastSchedStart;
  
        // driverStation...
        private DriverStation ds = DriverStation.getInstance();
        public String alliance;
        public String matchType;
        public int matchNumber;
        public String gameMessage;
        public String eventName;
        public int replayNumber;
        public String gameState;
        public double matchTime;

        private PowerDistributionPanel pdp = new PowerDistributionPanel();
        public double batteryVoltage;
        public double batteryCurrent;

        // INPUTS
        public boolean opRightBumper; // manual shoot, clear faults
        public double drRightStickX_Translate; // drive
        public double drRightStickY_Translate; // drive
        public double drLeftStickX_Rotate;  // drive
        public boolean drMidButton_ResetIMU;  // reset direction
        public boolean drRightToggleDown_SHOOT; // shoot
        public boolean drLeftToggleDown_RobotOrient; // field/robot oriented
        public boolean opXButton_IdleShooter;        // WoF open loop
        public boolean opAButton;        // WoF
        public boolean opBButton;        // WoF
        public boolean opStartButton;        // WoF
        public boolean opRightTrigger_COLLECT;   // collect
        public boolean opLeftTrigger_CLEARBALLS;    // clear balls
        public boolean opLeftBumper_CLIMB;     // climb pto
        public double opLeftStickY_ClimbSpeed;      // climb
        public boolean opPOV0_MANUAL10;
        public boolean opPOV90_MANUAL15;
        public boolean opPOV180_MANUAL20;
        public boolean opPOV270_MANUAL25;

        public boolean tsPOV0_READRED;
        public boolean tsPOV90_READGREEN;
        public boolean tsPOV180_READBLUE;
        public boolean tsPOV270_READCLEAR;
        public boolean tsABUTTON_READALL;
        public boolean tsBBUTTON_STARTSCAN;
        public boolean tsXBUTTON_STOPSCAN;

    }
}