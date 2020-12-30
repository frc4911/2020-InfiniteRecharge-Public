package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.subsystems.Subsystem;
import com.team254.lib.subsystems.SubsystemManager;

import cyberlib.utils.CheckFaults;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Ports;

public class Shooter extends Subsystem {

    // Hardware
    private final TalonFX mFXLeft, mFXRight;

    // Constants
    private final double kMinShootDistance = 10.0;
    private final double kMidShootDistance = 15.0;
    private final double kMaxShootDistance = 20.0;
    private final double kMinShootSpeed = 4700.0;
    private final double kMaxShootSpeed = 5200.0;
    private final double kSpeedTolerance = 250.0;

    public enum SystemState {
        HOLDING,
        SHOOTING
    }

    public enum WantedState {
        HOLD,
        SHOOT
    }

    private SystemState      mSystemState = SystemState.HOLDING;
    private WantedState      mWantedState = WantedState.HOLD;
    private PeriodicIO       mPeriodicIO;
    private boolean          mStateChanged;
    private double           mShootRate;
    private double           mHoldSpeed;
    private double           mDistance;
    private CheckFaults      mCF = new CheckFaults();
    private final boolean    mLoggingEnabled = true;   // used to disable logging for this subsystem only
    private SubsystemManager mSubsystemManager;
    private int              mListIndex;

    private static String sClassName;
    private static int sInstanceCount;
    private static Shooter sInstance = null;
    public  static Shooter getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new Shooter(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Shooter(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mPeriodicIO = new PeriodicIO();
        mFXLeft = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_LEFT);
        mFXRight = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_RIGHT);
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        configMotors();
    }

    private void configMotors() {
        mFXLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mFXRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        // only one encoder is needed
        mFXLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.kLongCANTimeoutMs);
    
        mFXLeft.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXRight.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXLeft.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXRight.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXLeft.setInverted(true);
        mFXRight.setInverted(false);

        mFXLeft.setSensorPhase(true);
        mFXRight.setSensorPhase(false);

        mFXLeft.setNeutralMode(NeutralMode.Coast);
        mFXRight.setNeutralMode(NeutralMode.Coast);

        configPID(0.65, 0.0, 0.0, 0.05);
    }

    private void configPID(double kP, double kI, double kD, double kF) {
        mFXLeft.config_kP(0, kP, Constants.kLongCANTimeoutMs);
        mFXLeft.config_kI(0, kI, Constants.kLongCANTimeoutMs);
        mFXLeft.config_kD(0, kD, Constants.kLongCANTimeoutMs);
        mFXLeft.config_kF(0, kF, Constants.kLongCANTimeoutMs);

        mFXRight.config_kP(0, kP, Constants.kLongCANTimeoutMs);
        mFXRight.config_kI(0, kI, Constants.kLongCANTimeoutMs);
        mFXRight.config_kD(0, kD, Constants.kLongCANTimeoutMs);
        mFXRight.config_kF(0, kF, Constants.kLongCANTimeoutMs);
    }

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(Phase phase) {
            synchronized(Shooter.this) {
                mSystemState = SystemState.HOLDING;
                mWantedState = WantedState.HOLD;
                mStateChanged = true;
                System.out.println(sClassName + " state " + mSystemState);
                // this subsystem is "on demand" so goto sleep
                mPeriodicIO.schedDeltaDesired = 0;
                mShootRate = (kMaxShootSpeed - kMinShootSpeed) / (kMaxShootDistance - kMidShootDistance);
                mHoldSpeed = 0;//0.45;
                stop();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized(Shooter.this) {
                SystemState newState;
                switch(mSystemState) {
                    case SHOOTING:
                        newState = handleShooting();
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
        mPeriodicIO.percentDemand = mHoldSpeed;

        return defaultStateTransfer();
    }

    private SystemState handleShooting() {
        mPeriodicIO.velocityPIDDemand = rpmToTicksPer100Ms(getDistanceToVelocityRPM(mDistance));

        return defaultStateTransfer();
    }

    public synchronized boolean readyToShoot() {
        return mSystemState == SystemState.SHOOTING && mPeriodicIO.reachedDesiredSpeed;
    }

    public synchronized void setShootDistance(double distance) {
        if (mSystemState != SystemState.SHOOTING) {
            mWantedState = WantedState.SHOOT;
        }
        if (mSystemState != SystemState.SHOOTING || distance != mDistance) {
            mSubsystemManager.scheduleMe(mListIndex, 1, false);
            System.out.println("waking " + sClassName);
        }

        mDistance = Math.max(kMinShootDistance, Math.min(distance, 35.0));
    }

    public synchronized void setHoldSpeed(double speed) {
        if (speed != mHoldSpeed && mSystemState == SystemState.HOLDING) {
            mSubsystemManager.scheduleMe(mListIndex, 1, false);
            System.out.println("waking " + sClassName);
        }

        mHoldSpeed = speed;
    }

    private double ticksPer100MsToRPM(double speed) {
        return speed / 1365.0 * 1000.0 / 100.0 * 60.0;
    }

    private double rpmToTicksPer100Ms(double speed) {
        return speed * 1365.0 / 1000.0 * 100.0 / 60.0;
    }

    private double getDistanceToVelocityRPM(double distance) {
        return mShootRate * Math.abs(distance - kMidShootDistance) + kMinShootSpeed;
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case SHOOT:
                return SystemState.SHOOTING;
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
        mFXLeft.set(ControlMode.PercentOutput, 0.0);
        mFXRight.set(ControlMode.PercentOutput, 0.0);
        mPeriodicIO.velocityPIDDemand = 0;
        mPeriodicIO.percentDemand = 0;
}
    
    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        mListIndex = enabledLooper.register(mLoop);
    }

    @Override
    public String getLogHeaders() {
        if (mLoggingEnabled){
            mCF.clearFaults(mFXLeft);
            mCF.clearFaults(mFXRight);
            return  sClassName+".systemState,"+
                    sClassName+".distance,"+
                    sClassName+".reachedDesiredSpeed,"+
                    sClassName+".rpm,"+
                    sClassName+".percentDemand,"+
                    sClassName+".velocityPIDDemand,"+
                    sClassName+".SRXLeftCurrent,"+
                    sClassName+".SRXRightCurrent,"+
                    sClassName+".SRXLeftFaults,"+
                    sClassName+".SRXRightFaults,"+
                    sClassName+".schedDeltaDesired,"+
                    sClassName+".schedDeltaActual,"+
                    sClassName+".schedDuration";
        }
        return null;
    }

    private String generateLogValues(boolean telemetry){
        String values;

        if (telemetry){
            mPeriodicIO.SRXLeftCurrent = mFXLeft.getStatorCurrent();
            mPeriodicIO.SRXRightCurrent = mFXRight.getStatorCurrent();
            mPeriodicIO.SRXLeftFaults = mCF.getFaults(mFXLeft);
            mPeriodicIO.SRXRightFaults = mCF.getFaults(mFXRight);

            values = ""+mSystemState+","+
                        mPeriodicIO.distance+","+
                        mPeriodicIO.reachedDesiredSpeed+","+
                        mPeriodicIO.rpm+","+
                        mPeriodicIO.percentDemand+","+
                        mPeriodicIO.velocityPIDDemand+","+
                        mPeriodicIO.SRXLeftCurrent+","+
                        mPeriodicIO.SRXRightCurrent+","+
                        mPeriodicIO.SRXLeftFaults+","+
                        mPeriodicIO.SRXRightFaults+","+
                        /*mPeriodicIO.schedDeltaDesired+*/","+
                        /*mPeriodicIO.schedDeltaActual+*/","
                        /*mPeriodicIO.schedDuration*/;
        }
        else{
            mPeriodicIO.schedDuration = Timer.getFPGATimestamp() - mPeriodicIO.lastSchedStart;
            values = ""+mSystemState+","+
                        mPeriodicIO.distance+","+
                        mPeriodicIO.reachedDesiredSpeed+","+
                        mPeriodicIO.rpm+","+
                        mPeriodicIO.percentDemand+","+
                        mPeriodicIO.velocityPIDDemand+","+
                        /*mPeriodicIO.SRXLeftCurrent+*/","+
                        /*mPeriodicIO.SRXRightCurrent+*/","+
                        /*mPeriodicIO.SRXLeftFaults+*/","+
                        /*mPeriodicIO.SRXRightFaults+*/","+
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

        if (mSystemState == SystemState.SHOOTING) {
            mPeriodicIO.rpm = ticksPer100MsToRPM(mFXLeft.getSelectedSensorVelocity(0));
            mPeriodicIO.reachedDesiredSpeed = mPeriodicIO.rpm >= ticksPer100MsToRPM(mPeriodicIO.velocityPIDDemand) - kSpeedTolerance;
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (mSystemState == SystemState.SHOOTING) {
            mFXLeft.set(ControlMode.Velocity, mPeriodicIO.velocityPIDDemand);
            mFXRight.set(ControlMode.Velocity, mPeriodicIO.velocityPIDDemand);
        } else {
            mFXLeft.set(ControlMode.PercentOutput, mPeriodicIO.percentDemand);
            mFXRight.set(ControlMode.PercentOutput, mPeriodicIO.percentDemand);
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
        SmartDashboard.putNumber("Shooter RPM Set", ticksPer100MsToRPM(mPeriodicIO.velocityPIDDemand));
        SmartDashboard.putNumber("Shooter RPM", mPeriodicIO.rpm);
        SmartDashboard.putBoolean("Shooter Ready", readyToShoot());
        SmartDashboard.putString("ShooterSRXLeftFaults", mPeriodicIO.SRXLeftFaults);
        SmartDashboard.putString("ShooterSRXRightFaults", mPeriodicIO.SRXRightFaults);
    }

    public static class PeriodicIO {
        // LOGGING
        public  double distance;
        public  int    schedDeltaDesired;
        public  double schedDeltaActual;
        public  double schedDuration;
        private double lastSchedStart;

        // INPUTS
        public boolean reachedDesiredSpeed;
        public double  SRXLeftCurrent;
        public double  SRXRightCurrent;
        public String  SRXLeftFaults;
        public String  SRXRightFaults;
        public double  rpm;
        
        //OUTPUTS
        public double percentDemand;
        public double velocityPIDDemand;
    }
}