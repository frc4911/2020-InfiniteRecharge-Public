package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.subsystems.Subsystem;
import com.team254.lib.subsystems.SubsystemManager;

import cyberlib.utils.CheckFaults;
import cyberlib.utils.CyberMath;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;
import frc.robot.Constants;

public class Collector extends Subsystem {

    // Hardware
    private final TalonSRX mSRXFrontRoller, mSRXSerializer;
    private final Solenoid mSolenoid;

    // Constants
    private final double kCollectSpeed   = 0.50;
    private final double kSerializeSpeed = 0.85;

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
        COLLECTING,
        BACKING
    }

    public enum WantedState {
        HOLD,
        COLLECT,
        BACK
    }

    private SystemState      mSystemState;
    private WantedState      mWantedState;
    private final PeriodicIO mPeriodicIO;
    private boolean          mStateChanged;
    private SolenoidState    mSolenoidState;
    private boolean          mLoggingEnabled = true;
    private CheckFaults      mCF = new CheckFaults();
    private SubsystemManager mSubsystemManager;
    private int              mListIndex;

    private static String sClassName;
    private static int sInstanceCount;
    private static Collector sInstance = null;
    public static Collector getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new Collector(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Collector(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mPeriodicIO = new PeriodicIO();
        mSRXFrontRoller = TalonSRXFactory.createDefaultTalon(Ports.COLLECTOR);
        mSRXSerializer = TalonSRXFactory.createDefaultTalon(Ports.SERIALIZER);
        mSolenoid  = new Solenoid(0, Ports.COLLECTOR_DEPLOY);
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        configMotors();
    }

    private void configMotors() {
        mSRXFrontRoller.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mSRXSerializer.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mSRXFrontRoller.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mSRXSerializer.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mSRXFrontRoller.setInverted(true);
        mSRXSerializer.setInverted(true);

        setNeutralMode(NeutralMode.Coast);
    }

    private void setNeutralMode(NeutralMode mode) {
        mSRXFrontRoller.setNeutralMode(mode);
        mSRXSerializer.setNeutralMode(mode);
    }

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(Phase phase) {
            synchronized(Collector.this) {
                mSystemState   = SystemState.HOLDING;
                mWantedState   = WantedState.HOLD;
                mStateChanged  = true;
                System.out.println(sClassName + " state " + mSystemState);
               // this subsystem is "on demand" so goto sleep
               mPeriodicIO.schedDeltaDesired = 0;
               stop(); // start in known state
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized(Collector.this) {
                SystemState newState;
                switch(mSystemState) {
                    case COLLECTING:
                        newState = handleCollecting();
                        break;
                    case BACKING:
                        newState = handleBacking();
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
            mPeriodicIO.SRXFrontRollerDemand = 0.0;
            mPeriodicIO.SRXSerializerDemand = 0.0;
            mPeriodicIO.solenoidDemand = SolenoidState.RETRACT;
        }
        
        return defaultStateTransfer();
    }

    private SystemState handleCollecting() {
        if (mStateChanged) {
            mPeriodicIO.SRXFrontRollerDemand = kCollectSpeed;
            mPeriodicIO.SRXSerializerDemand = kSerializeSpeed;
            mPeriodicIO.solenoidDemand = SolenoidState.EXTEND;
        }

        return defaultStateTransfer();
    } 

    private SystemState handleBacking() {
        if (mStateChanged) {
            mPeriodicIO.SRXFrontRollerDemand = -kCollectSpeed;
            mPeriodicIO.SRXSerializerDemand = -kSerializeSpeed;
            mPeriodicIO.solenoidDemand = SolenoidState.EXTEND;
        }

        return defaultStateTransfer();
    } 

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case COLLECT:
                return SystemState.COLLECTING;
            case BACK:
                return SystemState.BACKING;
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
        mSRXFrontRoller.set(ControlMode.PercentOutput, 0.0);
        mPeriodicIO.SRXFrontRollerDemand = 0;

        mSRXSerializer.set(ControlMode.PercentOutput, 0.0);
        mPeriodicIO.SRXSerializerDemand = 0;
        
        mSolenoid.set(SolenoidState.RETRACT.get());
        mSolenoidState = SolenoidState.RETRACT;
        mPeriodicIO.solenoidDemand = SolenoidState.RETRACT;    
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        mListIndex = enabledLooper.register(mLoop);
    }

    @Override
    public String getLogHeaders() {
        if (mLoggingEnabled){
            mCF.clearFaults(mSRXFrontRoller);
            mCF.clearFaults(mSRXSerializer);

            return sClassName+".systemState,"+
                   sClassName+".SRXFrontRollerDemand,"+
                   sClassName+".SRXSerializerDemand,"+
                   sClassName+".solenoidDemand,"+
                   sClassName+".SRXFrontRollerCurrent,"+
                   sClassName+".SRXSerializerCurrent,"+
                   sClassName+".SRXFrontRollerFaults,"+
                   sClassName+".SRXSerializerFaults,"+
                   sClassName+".schedDeltaDesired,"+
                   sClassName+".schedDeltaActual,"+
                   sClassName+".schedDuration";
        }
        return null;
    }

    private String generateLogValues(boolean telemetry){
        String values;
        if (telemetry){
            mPeriodicIO.SRXFrontRollerCurrent  = CyberMath.cTrunc(mSRXFrontRoller.getStatorCurrent(),1);
            mPeriodicIO.SRXSerializerCurrent   = CyberMath.cTrunc(mSRXSerializer.getStatorCurrent(),1);
            mPeriodicIO.SRXFrontRollerFaults   = mCF.getFaults(mSRXFrontRoller);
            mPeriodicIO.SRXSerializerFaults    = mCF.getFaults(mSRXSerializer);

            values = ""+mSystemState+","+
                        mPeriodicIO.SRXFrontRollerDemand+","+
                        mPeriodicIO.SRXSerializerDemand+","+
                        mPeriodicIO.solenoidDemand+","+
                        mPeriodicIO.SRXFrontRollerCurrent+","+
                        mPeriodicIO.SRXSerializerCurrent+","+
                        mPeriodicIO.SRXFrontRollerFaults+","+
                        mPeriodicIO.SRXSerializerFaults+","+
                        /*mPeriodicIO.schedDeltaDesired+*/","+
                        /*mPeriodicIO.schedDeltaActual+*/","
                        /*mPeriodicIO.schedDuration*/;
        }
        else{
            mPeriodicIO.schedDuration = Timer.getFPGATimestamp() - mPeriodicIO.lastSchedStart;

            values = ""+mSystemState+","+
                        mPeriodicIO.SRXFrontRollerDemand+","+
                        mPeriodicIO.SRXSerializerDemand+","+
                        mPeriodicIO.solenoidDemand+","+
                        /*mPeriodicIO.SRXFrontRollerCurrent+*/","+
                        /*mPeriodicIO.SRXSerializerCurrent+*/","+
                        /*mPeriodicIO.SRXFrontRollerFaults+*/","+
                        /*mPeriodicIO.SRXSerializerFaults+*/","+
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
        mSRXFrontRoller.set(ControlMode.PercentOutput, mPeriodicIO.SRXFrontRollerDemand);
        mSRXSerializer.set(ControlMode.PercentOutput, mPeriodicIO.SRXSerializerDemand);
        if (mSolenoidState != mPeriodicIO.solenoidDemand) {
            mSolenoidState = mPeriodicIO.solenoidDemand;
            mSolenoid.set(mPeriodicIO.solenoidDemand.get());
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
        SmartDashboard.putString("CollectorSRXFrontRollerFaults", mPeriodicIO.SRXFrontRollerFaults);
        SmartDashboard.putString("CollectorSRxSerializerFaults", mPeriodicIO.SRXSerializerFaults);
    }

    public static class PeriodicIO {
        // LOGGING
        public  double SRXFrontRollerCurrent;
        public  double SRXSerializerCurrent;
        public  String SRXFrontRollerFaults;
        public  String SRXSerializerFaults;
        public  int    schedDeltaDesired;
        public  double schedDeltaActual;
        public  double schedDuration;
        private double lastSchedStart;

        // INPUTS

        //OUTPUTS
        public double SRXFrontRollerDemand;
        public double SRXSerializerDemand;
        public SolenoidState solenoidDemand;
    }
}