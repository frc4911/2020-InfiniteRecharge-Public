package frc.robot.subsystems;

import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.subsystems.Subsystem;
import frc.robot.RobotState;

public class RobotStateEstimator extends Subsystem {

    public enum SystemState {
        ESTIMATING,
    }

    public enum WantedState {
        ESTIMATE,
    }

    private SystemState   mSystemState;
    private WantedState   mWantedState;
    private PeriodicIO    mPeriodicIO;
    @SuppressWarnings("unused")
    private boolean       mStateChanged;
    private final boolean mLoggingEnabled = true;                    // used to disable logging for this subsystem only
    private static int    mDefaultSchedDelta = 20;
    RobotState            robotState;// = RobotState.getInstance();
    Swerve                mSwerve;
    @SuppressWarnings("unused")
    private int mListIndex;

    private static String sClassName;
    private static int sInstanceCount;
    private static RobotStateEstimator sInstance = null;
    public  static RobotStateEstimator getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new RobotStateEstimator(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private RobotStateEstimator(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mPeriodicIO = new PeriodicIO();
        mSwerve = Swerve.getInstance(sClassName);
        robotState = RobotState.getInstance(sClassName);
    }
    
    private Loop mLoop = new Loop() {
        @Override
        public void onStart(Phase phase) {
            synchronized(RobotStateEstimator.this) {
                mSystemState = SystemState.ESTIMATING;
                mWantedState = WantedState.ESTIMATE;
                mStateChanged = true;
                System.out.println(sClassName + " state " + mSystemState);
                mPeriodicIO.schedDeltaDesired = mDefaultSchedDelta;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized(RobotStateEstimator.this) {
                SystemState newState;
                switch(mSystemState) {
                    case ESTIMATING:
                    default:
					newState = handleEstimating(timestamp);
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

    private SystemState handleEstimating(double timestamp) {
		robotState.addFieldToVehicleObservation(timestamp, mSwerve.getPose());
        return defaultStateTransfer();
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case ESTIMATE:
            default:
                return SystemState.ESTIMATING;
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
        if (mLoggingEnabled){
			return  sClassName+".systemState,"+
					sClassName+".schedDeltaDesired";
        }

        return null;
    }

    private String generateLogValues(boolean telemetry){
		String values = ""+ mSystemState+","+
							mPeriodicIO.schedDeltaDesired;
        return values;
    }

    @Override
    public String getLogValues(boolean telemetry) {
        if (mLoggingEnabled){
            return generateLogValues(telemetry);
        }

        return null;
    }

    @Override
    public void readPeriodicInputs() {
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
        public int schedDeltaDesired;
    }
}