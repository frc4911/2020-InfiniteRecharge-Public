package frc.robot.subsystems;

import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.subsystems.Subsystem;
import com.team254.lib.subsystems.SubsystemManager;

import cyberlib.utils.CyberMath;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;

public class Climber extends Subsystem {

    // Hardware
    private final Solenoid mSolBrake; // true == disengaged, false == engaged
    private final AnalogInput mPressureSensor;

    public enum BRAKE {
        DISENGAGE(true),
        ENGAGE(false);

        private final boolean state;

        private BRAKE(boolean state){
            this.state = state;
        }

        public boolean get(){
            return state;
        }
    }

    public enum SystemState {
        HOLDING,
        CLIMBING,
    }

    public enum WantedState {
        HOLD,
        CLIMB,
    }

    private SystemState      mSystemState;
    private WantedState      mWantedState;
    private PeriodicIO       mPeriodicIO;
    private boolean          mStateChanged;
    private BRAKE            mBrakeState;
    private final boolean    mLoggingEnabled = true;                    // used to disable logging for this subsystem only
	private SubsystemManager mSubsystemManager;
    private int              mListIndex;

    private static String sClassName;
    private static int sInstanceCount;
    private static Climber sInstance = null;
    public  static Climber getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new Climber(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Climber(String caller) {
        sClassName        = this.getClass().getSimpleName();
        printUsage(caller);
        mPeriodicIO       = new PeriodicIO();
        mSolBrake         = new Solenoid(0, Ports.BRAKE);
        mPressureSensor   = new AnalogInput(Ports.PRESSURE_SENSOR);
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
    }
    
    private Loop mLoop = new Loop() {
        @Override
        public void onStart(Phase phase) {
            synchronized(Climber.this) {
                mSystemState = SystemState.HOLDING;
                mWantedState = WantedState.HOLD;
                mStateChanged = true;
                System.out.println(sClassName + " state " + mSystemState);
                // this subsystem is "on demand" so goto sleep
                mPeriodicIO.schedDeltaDesired = 0;
                stop(); // put into a known state
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized(Climber.this) {
                SystemState newState;
                switch(mSystemState) {
                    case CLIMBING:
                        newState = handleClimbing();
                        break;
                    case HOLDING:
                    default:
                        newState = handleHolding();
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

    private SystemState handleHolding() {
        if (mStateChanged) {
            mPeriodicIO.brakeOnOffDemand = BRAKE.ENGAGE;
        }

        return defaultStateTransfer();
    }

    private SystemState handleClimbing() {
        if (mStateChanged) {
            mPeriodicIO.brakeOnOffDemand = BRAKE.DISENGAGE;
        }

        return defaultStateTransfer();
    }

    private double convertSensorToPSI(double sensorValue){
        return 250.0 * (sensorValue / 5.0) - 25.0;
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case CLIMB:
                return SystemState.CLIMBING;
            case HOLD:
            default:
                return SystemState.HOLDING;
        }
    }

    public synchronized void setWantedState(WantedState state) {
        if (state != mWantedState){
            mSubsystemManager.scheduleMe(mListIndex, 1, false);
            System.out.println("waking " + sClassName);
        }
        
        mWantedState = state;
    }

    @Override
    public void stop() {
        mSolBrake.set(BRAKE.ENGAGE.get());
        mBrakeState = BRAKE.ENGAGE;
        mPeriodicIO.brakeOnOffDemand = BRAKE.ENGAGE;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        mListIndex = enabledLooper.register(mLoop);
    }

    @Override
    public String getLogHeaders() {
        if (mLoggingEnabled){
            return sClassName+".systemState,"+
                   sClassName+".pressure,"+
                   sClassName+".brakeOnOffDemand,"+
                   sClassName+".schedDeltaDesired,"+
                   sClassName+".schedDeltaActual,"+
                   sClassName+".schedDuration";
        }

        return null;
    }

    private String generateLogValues(boolean telemetry){
        String values;
        mPeriodicIO.pressure = CyberMath.cTrunc(convertSensorToPSI(mPressureSensor.getVoltage()),10);
        if (telemetry){
            values = ""+mSystemState+","+
                        mPeriodicIO.pressure+","+
                        /*mPeriodicIO.brakeOnOffDemand+*/ ","+
                        /*mPeriodicIO.schedDeltaDesired+*/","+
                        /*mPeriodicIO.schedDeltaActual+*/","
                        /*mPeriodicIO.schedDuration*/;
        }
        else{
            mPeriodicIO.schedDuration = Timer.getFPGATimestamp() - mPeriodicIO.lastSchedStart;
            values = ""+mSystemState+","+
                        mPeriodicIO.pressure+","+
                        mPeriodicIO.brakeOnOffDemand+","+
                        mPeriodicIO.schedDeltaDesired+","+
                        mPeriodicIO.schedDeltaActual+","+
                        mPeriodicIO.schedDuration;
            }
    
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
        double now                   = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart   = now;
    }

    @Override
    public void writePeriodicOutputs() {
        if (mBrakeState != mPeriodicIO.brakeOnOffDemand){
            mBrakeState = mPeriodicIO.brakeOnOffDemand;
            mSolBrake.set(mPeriodicIO.brakeOnOffDemand.get());
        }
    }

    @Override
    public int whenRunAgain () {
        if (mStateChanged && mPeriodicIO.schedDeltaDesired==0){
            return 1; // one more loop before going to sleep
        }

        return mPeriodicIO.schedDeltaDesired;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Pressure", mPeriodicIO.pressure);
    }

    public static class PeriodicIO {
        // LOGGING
        public  int    schedDeltaDesired;
        public  double schedDeltaActual;
        public  double schedDuration;
        private double lastSchedStart;

        // INPUTS
        public double pressure;

        //OUTPUTS
        public BRAKE brakeOnOffDemand;
    }
}