package frc.robot.subsystems;

import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.subsystems.Subsystem;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.states.LEDState;
import frc.robot.states.TimedLEDState;

/**
 * A subsystem for LED managing state and effects.
 */
public class LED extends Subsystem {
    private static final double kClimbingBlinkDuration = 0.5; // In sec
    // private static final double kWantsCargoBlinkDuration = 0.075; // In sec // TODO: bring back when needed
    private static final double kFaultBlinkDuration = 0.25; // In sec

    private LEDCanifier mLEDCanifier;
    private SystemState mSystemState = SystemState.DISPLAYING_SHOOT;
    private WantedAction mWantedAction = WantedAction.DISPLAY_SHOOT;
    private final boolean mLoggingEnabled = false;              // used to disable logging for this subsystem only
    public PeriodicIO mPeriodicIO;

    // List subsystems that have LED state

    // private boolean mFaultsEnabled = false; // TODO: bring back when needed
    private boolean mErrorCondition = false;
    private boolean mBlinking = false;

    private LEDState mDesiredLEDState = new LEDState(0.0, 0.0, 0.0);
    private TimedLEDState mActiveLEDState = TimedLEDState.StaticLEDState.kStaticOff;

    private static String sClassName;
    private static int sInstanceCount;
    private static LED sInstance = null;
    public  static LED getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new LED(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private LED(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mLEDCanifier = LEDCanifier.getInstance(sClassName);
        mPeriodicIO = new PeriodicIO();
    }

   public synchronized void setWantedAction(WantedAction wantedAction, boolean isBlinking) {
        mWantedAction = wantedAction;
        mBlinking = isBlinking;
    }

    private Loop loop = new Loop() {
            double stateStartTime;

            @Override
            public void onStart(Phase phase) {
                System.out.println(sClassName + " state " + mSystemState);
                stateStartTime = Timer.getFPGATimestamp();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (LED.this) {
                    SystemState newState = getStateTransition();

                    if (mSystemState != newState) {
                        System.out.println(sClassName + " state " + mSystemState + " to " + newState + " (" + timestamp + ")");
                        mSystemState = newState;
                        stateStartTime = timestamp;
                    }

                    double timeInState = timestamp - stateStartTime;

                    switch (mSystemState) {
                        case DISPLAYING_FAULT:
                            setFaultLEDCommand(timeInState);
                            break;
                        case DISPLAYING_CLIMB:
                            setClimbLEDCommand(timeInState);
                            break;
                        case DISPLAYING_SHOOT:
                        case DISPLAYING_INDEXER_LOADED:
                        case DISPLAYING_INDEXER_UNJAMMING:
                        case DISPLAYING_COLLECTOR_DEPLOYED:
                        case DISPLAYING_COLLECTOR_UNJAMMING:
                        case DISPLAYING_DRIVE:
                        case DISPLAYING_AUTO_START:
                            mActiveLEDState.getCurrentLEDState(mDesiredLEDState, timeInState);
                            break;
                        default:
                            System.out.println("Fell through on LED commands: " + mSystemState);
                            break;
                    }
                    
                    mLEDCanifier.setLEDColor(mDesiredLEDState.red, mDesiredLEDState.green, mDesiredLEDState.blue);
                }
            }

            @Override
            public void onStop(double timestamp) {
            }
        };
    

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

    public void readPeriodicInputs() {
        mPeriodicIO.systemState = mSystemState;
    }


    private void setFaultLEDCommand(double timeInState) {
        // Blink red.
        if ((int) (timeInState / kFaultBlinkDuration) % 2 == 0) {
            mDesiredLEDState.copyFrom(LEDState.kFault);
        } else {
            mDesiredLEDState.copyFrom(LEDState.kOff);
        }
    }

    private void setClimbLEDCommand(double timeInState) {
        // Blink orange
        if ((int) (timeInState / kClimbingBlinkDuration) % 2 == 0) {
            mDesiredLEDState.copyFrom(LEDState.kClimbing);
        } else {
            mDesiredLEDState.copyFrom(LEDState.kOff);
        }
    }

    private SystemState getStateTransition() {
        // if (mFaultsEnabled && !mSwerve.hasBeenZeroed()) {
        // return SystemState.DISPLAYING_FAULT;
        // }
        if (mErrorCondition) {
            return SystemState.DISPLAYING_FAULT;
        }

        switch (mWantedAction) {
            case DISPLAY_CLIMB:
                return SystemState.DISPLAYING_CLIMB;
            case DISPLAY_SHOOT:
                mActiveLEDState = mBlinking ? TimedLEDState.BlinkingLEDState.kShoot : TimedLEDState.StaticLEDState.kShoot;
                return SystemState.DISPLAYING_SHOOT;
            case DISPLAY_INDEXER_LOADED:
                mActiveLEDState = mBlinking ? TimedLEDState.BlinkingLEDState.kIndexerLoaded : TimedLEDState.StaticLEDState.kIndexerLoaded;
                return SystemState.DISPLAYING_INDEXER_LOADED;
            case DISPLAY_INDEXER_UNJAMMING:
                mActiveLEDState = mBlinking ? TimedLEDState.BlinkingLEDState.kIndexerUnjamming : TimedLEDState.StaticLEDState.kIndexerLoaded;
                return SystemState.DISPLAYING_INDEXER_LOADED;
            case DISPLAY_COLLECTOR_DEPLOYED:
                mActiveLEDState = mBlinking ? TimedLEDState.BlinkingLEDState.kCollectorDeployed: TimedLEDState.StaticLEDState.kCollectorDeployed;
                return SystemState.DISPLAYING_COLLECTOR_DEPLOYED;
            case DISPLAY_COLLECTOR_UNJAMMING:
                mActiveLEDState = mBlinking ? TimedLEDState.BlinkingLEDState.kCollectorUnjamming: TimedLEDState.StaticLEDState.kCollectorDeployed;
                return SystemState.DISPLAYING_COLLECTOR_DEPLOYED;
            case  DISPLAY_DRIVE:
                mActiveLEDState = mBlinking ? TimedLEDState.BlinkingLEDState.kDrive: TimedLEDState.StaticLEDState.kDrive;
                return SystemState.DISPLAYING_DRIVE;
            case DISPLAY_AUTO_START:
                mActiveLEDState = TimedLEDState.StaticLEDState.kAutoStart;
            default:
                System.out.println("Fell through on LED wanted action check: " + mWantedAction);
                return SystemState.DISPLAYING_SHOOT;
        }
    }

    // TODO: bring back when needed
    // public synchronized void setEnableFaults(boolean enable) {
    //     mFaultsEnabled = enable;
    // }

    boolean testRunOnce = false;
    public boolean checkSystem() {
        if (testRunOnce) {
            return true;
        }
        // TODO:  Re-enable tests for  2020 states
//
//        System.out.println("Climb - blink orange for 0.5s");
//        this.setWantedAction(WantedAction.DISPLAY_CLIMB);
//        runLoop(3.0);
//
//        // Test intake sequence:  Intaking -> Blinking Has Cargo -> Solid Has Cargo
//        System.out.println("Intaking - solid green for 0.5s");
//        setIntakeLEDState(TimedLEDState.StaticLEDState.kIntaking);
//        this.setWantedAction(WantedAction.DISPLAY_BEAK_DOWN);
//        runLoop(3.0);
//
//        // Test intake sequence:  Intaking -> Blinking Has Cargo -> Solid Has Cargo
//        System.out.println("Intaking (has cargo) - blinking blue for 0.075s");
//        setIntakeLEDState(TimedLEDState.BlinkingLEDState.kHasCargo);
//        this.setWantedAction(WantedAction.DISPLAY_BEAK_DOWN);
//        runLoop(3.0);
//
//        // Test intake sequence:  Intaking -> Blinking Has Cargo -> Solid Has Cargo
//        System.out.println("Intaking (has cargo) - solid blue");
//        setIntakeLEDState(TimedLEDState.StaticLEDState.kHasCargo);
//        this.setWantedAction(WantedAction.DISPLAY_BEAK_DOWN);
//        runLoop(3.0);
//
//        // Test wants Cargo sequence
//        System.out.println("Wants Cargo - blinking orange");
//        this.setWantedAction(WantedAction.DISPLAY_HAS_CARGO);
//        runLoop(3.0);
//
//        // Test Fault LED
//        System.out.println("Wants Cargo - blinking red");
//        mErrorCondition = true;
//        runLoop(3.0);
//        mErrorCondition = false;
//
//        setIntakeLEDState(TimedLEDState.StaticLEDState.kStaticOff);
//        this.setWantedAction(WantedAction.DISPLAY_BEAK_DOWN);
//        runLoop(3.0);
//
//        mDesiredLEDState.blue = 0;
//        mDesiredLEDState.red= 0;
//        mDesiredLEDState.green = 0;
//
//        testRunOnce = true;
        return true;
    }

    @Override
    public String getLogHeaders() {
        if (mLoggingEnabled){
            return null;
        }
        return null;
    }

    @Override
    public String getLogValues(boolean telemetry) {
        if (mLoggingEnabled){
            return "code needed";
        }
        return null;
    }

    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {
    }

    public enum WantedAction {
        DISPLAY_CLIMB,
        DISPLAY_SHOOT,
        DISPLAY_INDEXER_LOADED,
        DISPLAY_INDEXER_UNJAMMING,
        DISPLAY_COLLECTOR_DEPLOYED,
        DISPLAY_COLLECTOR_UNJAMMING,
        DISPLAY_DRIVE,
        DISPLAY_AUTO_START
    }

    private enum SystemState {
        DISPLAYING_FAULT,
        DISPLAYING_CLIMB,
        DISPLAYING_SHOOT,
        DISPLAYING_INDEXER_LOADED,
        DISPLAYING_INDEXER_UNJAMMING,
        DISPLAYING_COLLECTOR_DEPLOYED,
        DISPLAYING_COLLECTOR_UNJAMMING,
        DISPLAYING_DRIVE,
        DISPLAYING_AUTO_START
    }

    // TODO: bring back when needed
    // private void runLoop(double duration) {
    //     double endTime = Timer.getFPGATimestamp() + duration;
    //     double timeStamp;

    //     do {
    //         timeStamp = Timer.getFPGATimestamp();
    //         loop.onLoop(timeStamp );
    //         LEDCanifier.getInstance().readPeriodicInputs();
    //         LEDCanifier.getInstance().writePeriodicOutputs();
    //         Timer.delay(Constants.kLooperDt);
    //     } while (timeStamp < endTime);
    // }

    public static class PeriodicIO {
        // LOGGING
        public SystemState systemState;

        // INPUTS

        //OUTPUTS
    }
}
