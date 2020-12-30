package frc.robot.subsystems;

import java.util.Optional;

import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.subsystems.Subsystem;
import com.team254.lib.vision.AimingParameters;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.Limelights.ShootwardsLimelight;

public class Superstructure extends Subsystem {

    private Swerve    mSwerve    = null;
    private Indexer   mIndexer   = null;
    private Collector mCollector = null;
    private Donger    mDonger    = null;
    private Shooter   mShooter   = null;
    private Climber   mClimber   = null;
    private ShootwardsLimelight mShootwardsLimelight = null;
    private RobotState mRobotState = null;
    // private CollectwardsLimelight mCollectwardsLimelight = null;

    public enum SystemState {
        HOLDING,
        COLLECTING,
        SHOOTING,
        CLIMBING,
        CLEARING_BALLS,
        MANUAL_SHOOTING
    }

    public enum WantedState {
        HOLD,
        COLLECT,
        SHOOT,
        CLIMB,
        CLEAR_BALLS,
        MANUAL_SHOOT
    }

    private SystemState   mSystemState = SystemState.HOLDING;
    private WantedState   mWantedState = WantedState.HOLD;
    private boolean       mStateChanged;
    private boolean       mShootSetup;
    private double        mDistance;
    private final boolean mLoggingEnabled = true;  // used to disable logging for this subsystem only
    public PeriodicIO     mPeriodicIO;
    private double        mLastDistanceToGoal;
    private double        mClimberReady = 0;
    private final double  mClimberTimeout = 1.0; // time to disengage brake
    @SuppressWarnings("unused")
    private int           mListIndex;
    private Optional<AimingParameters> mAimingParameters;
    private int           mFastCycle = 10;
    private int           mSlowCycle = 100;

    private static String sClassName;
    private static int sInstanceCount;
    private static Superstructure sInstance = null;
    public  static Superstructure getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new Superstructure(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Superstructure(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mSwerve              = Swerve.getInstance(sClassName);
        mIndexer             = Indexer.getInstance(sClassName);
        mCollector           = Collector.getInstance(sClassName);
        mShooter             = Shooter.getInstance(sClassName);
        mClimber             = Climber.getInstance(sClassName);
        mDonger              = Donger.getInstance(sClassName);
        mShootwardsLimelight = ShootwardsLimelight.getInstance(sClassName);
        mRobotState          = RobotState.getInstance(sClassName);
        // mCollectwardsLimelight = CollectwardsLimelight.getInstance(sClassName);

        mPeriodicIO = new PeriodicIO();
    }

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(Phase phase) {
            synchronized(Superstructure.this) {
                mSystemState = SystemState.HOLDING;
                mWantedState = WantedState.HOLD;
                mStateChanged = true;
                System.out.println(sClassName + " state " + mSystemState);
                switch (phase) {
                    case DISABLED:
                        mPeriodicIO.schedDeltaDesired = 0; // goto sleep
                        break;
                    default:
                        mPeriodicIO.schedDeltaDesired = 100;
                        break;
                }
                stop();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized(Superstructure.this) {
                SystemState newState;
                switch(mSystemState) {
                    case COLLECTING:
                        newState = handleCollecting();
                        break;
                    case SHOOTING:
                        newState = handleShooting();
                        break;
                    case CLIMBING:
                        newState = handleClimbing();
                        break;
                    case CLEARING_BALLS:
                        newState = handleClearingBalls();
                        break;
                    case MANUAL_SHOOTING:
                        newState = handleManualShooting();
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
            mIndexer.setWantedState(Indexer.WantedState.HOLD);
            mCollector.setWantedState(Collector.WantedState.HOLD);
            mShooter.setWantedState(Shooter.WantedState.HOLD);
            mClimber.setWantedState(Climber.WantedState.HOLD);
            if (mIndexer.isFullyLoaded()) {
                mDonger.setWantedState(Donger.WantedState.SECURE);
            } else {
                mDonger.setWantedState(Donger.WantedState.HOLD);
            }

            mShootwardsLimelight.setWantedState(ShootwardsLimelight.WantedState.TARGET);
            mPeriodicIO.schedDeltaDesired = mSlowCycle;
        }

        return defaultStateTransfer();
    }

    private SystemState handleCollecting() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mFastCycle;
        }

        if (!mIndexer.isFullyLoaded()) {
            mCollector.setWantedState(Collector.WantedState.COLLECT);
            mDonger.setWantedState(Donger.WantedState.COLLECT);
            if (mIndexer.isBallEntering()) {
                mIndexer.setWantedState(Indexer.WantedState.LOAD);
            } else {
                mIndexer.setWantedState(Indexer.WantedState.HOLD);
            }
        } else {
            mIndexer.setWantedState(Indexer.WantedState.HOLD);
            mDonger.setWantedState(Donger.WantedState.SECURE);
        }

        return collectingStateTransfer();
    }
    
    private SystemState handleShooting() {
        if (mStateChanged) {
            mSwerve.limeLightAim();
            mShootSetup = true;
            mLastDistanceToGoal = Double.MIN_VALUE;
            mPeriodicIO.schedDeltaDesired = mFastCycle;
        }

        double distanceToGoal = getDistanceToGoal();
        if (mLastDistanceToGoal != distanceToGoal){
            mShooter.setShootDistance(distanceToGoal); // TODO: check if only need to call once
            mLastDistanceToGoal = distanceToGoal;
        }

        if (readyToShootAndOnTarget()) {
            mIndexer.setWantedState(Indexer.WantedState.INDEX);
            if (mIndexer.isBallEntering()) {
                mDonger.setWantedState(Donger.WantedState.COLLECT);
            } else {
                mDonger.setWantedState(Donger.WantedState.HOLD);
            }

            mShootSetup = false;
        } else {
            mIndexer.setWantedState(Indexer.WantedState.HOLD);
            if (mIndexer.isFullyLoaded()) {
                mDonger.setWantedState(Donger.WantedState.SECURE);
            } else {
                mDonger.setWantedState(Donger.WantedState.HOLD);
            }
        }

        return shootingStateTransfer();
    }

    private SystemState handleClimbing() {
        if (mStateChanged){
            mClimber.setWantedState(Climber.WantedState.CLIMB);
            mIndexer.setWantedState(Indexer.WantedState.CLIMB_PREP);
            mClimberReady = Timer.getFPGATimestamp()+mClimberTimeout;
            mPeriodicIO.schedDeltaDesired = mFastCycle;
        }

        if (Timer.getFPGATimestamp()>mClimberReady){
            mIndexer.setWantedState(Indexer.WantedState.CLIMB_MOVE);
        }

        return climberStateTransfer();
    }

    private SystemState handleClearingBalls() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mSlowCycle;
        }

        mCollector.setWantedState(Collector.WantedState.BACK);

        return collectingStateTransfer();
    }

    private SystemState handleManualShooting() {
        if (mStateChanged) {
            mShooter.setShootDistance(mDistance);
            mShootSetup = true;
            mPeriodicIO.schedDeltaDesired = mFastCycle;
        }

        if (mShooter.readyToShoot() || !mShootSetup) {
            mIndexer.setWantedState(Indexer.WantedState.INDEX);
            if (mIndexer.isBallEntering()) {
                mDonger.setWantedState(Donger.WantedState.COLLECT);
            } else {
                mDonger.setWantedState(Donger.WantedState.HOLD);
            }

            mShootSetup = false;
        } else {
            mIndexer.setWantedState(Indexer.WantedState.HOLD);
            mDonger.setWantedState(Donger.WantedState.SECURE);
        }

        return shootingStateTransfer();
    }

    private SystemState collectingStateTransfer() {
        if (mWantedState != WantedState.COLLECT) {
            mCollector.setWantedState(Collector.WantedState.HOLD);
            mIndexer.setWantedState(Indexer.WantedState.HOLD);
            if (mIndexer.isFullyLoaded()) {
                mDonger.setWantedState(Donger.WantedState.SECURE);
            } else {
                mDonger.setWantedState(Donger.WantedState.HOLD);
            }
        }

        return defaultStateTransfer();
    }

    private SystemState shootingStateTransfer() {
        if (mWantedState != WantedState.SHOOT && mWantedState != WantedState.MANUAL_SHOOT) {
            mShooter.setWantedState(Shooter.WantedState.HOLD);
            mIndexer.setWantedState(Indexer.WantedState.HOLD);
            if (mIndexer.isFullyLoaded()) {
                mDonger.setWantedState(Donger.WantedState.SECURE);
            } else {
                mDonger.setWantedState(Donger.WantedState.HOLD);
            }
            
            setShooterHoldSpeed(0.45);
        }

        return defaultStateTransfer();
    }

    private SystemState climberStateTransfer() {
        if (mWantedState != WantedState.CLIMB){
            mClimber.setWantedState(Climber.WantedState.HOLD);
            mIndexer.setWantedState(Indexer.WantedState.HOLD);
        }

        return defaultStateTransfer();
    }

    public synchronized void setClimbOpenLoop(double set) {
        mIndexer.setClimberSpeed(set);
    }

    public synchronized void setManualShootDistance(double distance) {
        mDistance = distance;
        setWantedState(WantedState.MANUAL_SHOOT);
    }

    public synchronized void setShooterHoldSpeed(double speed) {
        mShooter.setHoldSpeed(speed);
    }

    private boolean readyToShootAndOnTarget() {
        return (mShooter.readyToShoot() && mSwerve.isOnTarget()) || !mShootSetup;
    }

    private double getDistanceToGoal() {
        mAimingParameters = mRobotState.getOuterGoalParameters();
        double distance = 0.0;
        if (mAimingParameters.isPresent()) {
            distance = mAimingParameters.get().getRobotToGoal().getTranslation().norm() / 12.0;
        }

        return distance;
    }

    public synchronized WantedState getWantedState() {
        return mWantedState;
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case COLLECT:
                return SystemState.COLLECTING;
            case SHOOT:
                return SystemState.SHOOTING;
            case CLIMB:
                return SystemState.CLIMBING;
            case CLEAR_BALLS:
                return SystemState.CLEARING_BALLS;
            case MANUAL_SHOOT:
                return SystemState.MANUAL_SHOOTING;
            case HOLD:
            default:
                return SystemState.HOLDING;
        }
    }

    public synchronized void setWantedState(WantedState state) {
        if (mWantedState != state){
            mPeriodicIO.schedDeltaDesired = 2;
        }
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
            return sClassName+".systemState,"+
                   sClassName+".shootSetup,"+
                   sClassName+".readyToShootAndOnTarget,"+
                   sClassName+".schedDeltaDesired,"+
                   sClassName+".schedDeltaActual,"+
                   sClassName+".schedDuration";
        }
        return null;
    }

    private String generateLogValues(boolean telemetry){
        String values;
        if (telemetry) {
            values = ""+mSystemState+","+
                        mShootSetup+","+
                        mPeriodicIO.readyToShootAndOnTarget+","+
                        /*mPeriodicIO.schedDeltaDesired+*/","+
                        /*mPeriodicIO.schedDeltaActual+*/","
                        /*mPeriodicIO.schedDuration*/;

        }
        else {
            mPeriodicIO.schedDuration = Timer.getFPGATimestamp() - mPeriodicIO.lastSchedStart;
            values = ""+mSystemState+","+
                        mShootSetup+","+
                        mPeriodicIO.readyToShootAndOnTarget+","+
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
        mPeriodicIO.readyToShootAndOnTarget = readyToShootAndOnTarget();
    }

    @Override
    public void writePeriodicOutputs() {
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
        SmartDashboard.putBoolean("Ready To Shoot and On Target", mPeriodicIO.readyToShootAndOnTarget);
    }

    public static class PeriodicIO {
        // LOGGING
        public int schedDeltaDesired;
        public double schedDeltaActual;
        public double schedDuration;
        private double lastSchedStart;

        // INPUTS
        public boolean readyToShootAndOnTarget;

        //OUTPUTS
    }
}
