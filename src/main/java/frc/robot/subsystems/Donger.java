package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.subsystems.Subsystem;
import com.team254.lib.subsystems.SubsystemManager;

import cyberlib.utils.CheckFaults;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Ports;

public class Donger extends Subsystem {

    // Hardware
    private final TalonSRX mSRXWheels;
    private final Solenoid mSolenoid;

    // Constants
    private final double kCollectSpeed = 1.0;

    public enum SolenoidState {
        EXTEND(true),
        RETRACT(false);

        private final boolean state;

        private SolenoidState(boolean state) {
            this.state = state;
        }

        public boolean get() {
            return state;
        }
    }

    public enum SystemState {
        HOLDING,
        SECURING,
        COLLECTING
    }

    public enum WantedState {
        HOLD,
        SECURE,
        COLLECT
    }

    private SystemState      mSystemState = SystemState.HOLDING;
    private WantedState      mWantedState = WantedState.HOLD;
    private PeriodicIO       mPeriodicIO;
    private boolean          mStateChanged;
    private CheckFaults      mCF = new CheckFaults();
    private SolenoidState    mSolenoidState;
    private final boolean    mLoggingEnabled = true; // used to disable logging for this subsystem only
    private SubsystemManager mSubsystemManager;
    private int              mListIndex;

    private static String sClassName;
    private static int sInstanceCount;
    private static Donger sInstance = null;
    public  static Donger getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new Donger(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Donger(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mPeriodicIO = new PeriodicIO();
        mSRXWheels = TalonSRXFactory.createDefaultTalon(Ports.DONGER);
        mSolenoid = new Solenoid(0, Ports.DONGER_DEPLOY);
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        configMotors();
    }

    private void configMotors() {
        mSRXWheels.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mSRXWheels.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mSRXWheels.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mSRXWheels.setInverted(false);
        mSRXWheels.setSensorPhase(false);
        mSRXWheels.setNeutralMode(NeutralMode.Coast);
    }

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(Phase phase) {
            synchronized(Donger.this) {
                mSystemState  = SystemState.HOLDING;
                mWantedState  = WantedState.HOLD;
                mStateChanged = true;
                System.out.println(sClassName + " state " + mSystemState);
                // this subsystem is "on demand" so goto sleep
                mPeriodicIO.schedDeltaDesired = 0;
                stop();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized(Donger.this) {
                SystemState newState;
                switch(mSystemState) {
                    case SECURING:
                        newState = handleSecuring();
                        break;
                    case COLLECTING:
                        newState = handleCollecting();
                        break;
                    case HOLDING:
                    default:
                        newState = handleHolding();
                }

                if (newState != mSystemState) {
                    System.out.println(sClassName + " state " + mSystemState + " to " + newState+" ("+timestamp+")");
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
            mPeriodicIO.SRXWheelsDemand = 0.0;
            mPeriodicIO.solDemand = SolenoidState.RETRACT;
        }

        return defaultStateTransfer();
    }

    private SystemState handleSecuring() {
        if (mStateChanged) {
            mPeriodicIO.SRXWheelsDemand = 0.0;
            mPeriodicIO.solDemand = SolenoidState.EXTEND;
        }

        return defaultStateTransfer();
    }

    private SystemState handleCollecting() {
        if (mStateChanged) {
            mPeriodicIO.SRXWheelsDemand = kCollectSpeed;
            mPeriodicIO.solDemand = SolenoidState.EXTEND;
        }

        return defaultStateTransfer();
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case SECURE:
                return SystemState.SECURING;
            case COLLECT:
                return SystemState.COLLECTING;
            case HOLD:
            default:
                return SystemState.HOLDING;
        }
    }

    public synchronized void setWantedState(WantedState state) {
        if (state != mWantedState) {
            mSubsystemManager.scheduleMe(mListIndex, 1, false);
            System.out.println("waking " + sClassName);
        }

        mWantedState = state;
    }

    @Override
    public void stop() {
        mSRXWheels.set(ControlMode.PercentOutput, 0.0);
        mPeriodicIO.SRXWheelsDemand = 0;

        mSolenoid.set(SolenoidState.RETRACT.get());
        mSolenoidState = SolenoidState.RETRACT;
        mPeriodicIO.solDemand = SolenoidState.RETRACT;
        // System.out.println(mPeriodicIO.solDemand.toString()+" donger");   

    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        mListIndex = enabledLooper.register(mLoop);
    }

    @Override
    public String getLogHeaders() {
        if (mLoggingEnabled){
            return sClassName+".systemState,"+
                   sClassName+".SRXWheelsDemand,"+
                   sClassName+".solDemand,"+
                   sClassName+".SRXWheelsCurrent,"+
                   sClassName+".SRXWheelsFaults,"+
                   sClassName+".schedDeltaDesired,"+
                   sClassName+".schedDeltaActual,"+
                   sClassName+".schedDuration";
        }
        return null;
    }

    private String generateLogValues(boolean telemetry){
        String values;

        if (telemetry){
            mPeriodicIO.SRXWheelsCurrent = mSRXWheels.getStatorCurrent();
            mPeriodicIO.SRXWheelsFaults = mCF.getFaults(mSRXWheels);

            values = ""+mSystemState+","+
                        mPeriodicIO.SRXWheelsDemand+","+
                        mPeriodicIO.solDemand+","+
                        mPeriodicIO.SRXWheelsCurrent+","+
                        mPeriodicIO.SRXWheelsFaults+","+
                        /*mPeriodicIO.schedDeltaDesired+*/","+
                        /*mPeriodicIO.schedDeltaActual+*/","
                        /*mPeriodicIO.schedDuration*/;
        }
        else {
            mPeriodicIO.schedDuration = Timer.getFPGATimestamp() - mPeriodicIO.lastSchedStart;
            values = ""+mSystemState+","+
                        mPeriodicIO.SRXWheelsDemand+","+
                        mPeriodicIO.solDemand+","+
                        /*mPeriodicIO.SRXWheelsCurrent+*/","+
                        /*mPeriodicIO.SRXWheelsFaults+*/","+
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
        double now = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;
    }

    @Override
    public void writePeriodicOutputs() {
        mSRXWheels.set(ControlMode.PercentOutput, mPeriodicIO.SRXWheelsDemand);

        if (mSolenoidState != mPeriodicIO.solDemand) {
            mSolenoidState = mPeriodicIO.solDemand;
            mSolenoid.set(mPeriodicIO.solDemand.get());
            // System.out.println(mPeriodicIO.solDemand.toString()+" donger");   
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
        SmartDashboard.putString("DongerSRXWheelsFaults", mPeriodicIO.SRXWheelsFaults);
    }

    public static class PeriodicIO {
        // LOGGING
        public SystemState currentState;
        public int         schedDeltaDesired;
        public double      schedDeltaActual;
        public double      schedDuration;
        private double     lastSchedStart;
 
        // INPUTS
        public double      SRXWheelsCurrent;
        public String      SRXWheelsFaults;

        //OUTPUTS
        public double        SRXWheelsDemand;
        public SolenoidState solDemand; // TODO: make sure this is needed for ondemand subsystem
    }
}