package frc.robot.subsystems;

import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.subsystems.Subsystem;
import com.team254.lib.subsystems.SubsystemManager;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ColorSensor.ColorLED;

public class ColorReader extends Subsystem {

    // Hardware
    private final int colorSelectAPort = 7;
    private final int colorSelectBPort = 8;
    private final int colorValuePort = 9;
    private ColorSensor mColorSensor;
    private boolean mRestartScanning;
    private String mScanColor = "";

    public enum SystemState {
        HOLDING,
        READING_RED,
        READING_GREEN,
        READING_BLUE,
        READING_CLEAR,
        READING_ALL,
        SCANNING,
    }

    public enum WantedState {
        HOLD,
        READ_RED,
        READ_GREEN,
        READ_BLUE,
        READ_CLEAR,
        READ_ALL,
        SCAN
    }

    public enum Paper {
        RED,
        GREEN,
        BLUE,
        YELLOW,
        WHITE,
        BLACK
    }

    private SystemState      mSystemState;
    private WantedState      mWantedState;
    private PeriodicIO       mPeriodicIO;
    private boolean          mStateChanged;
    private final boolean    mLoggingEnabled = true;                    // used to disable logging for this subsystem only
	private SubsystemManager mSubsystemManager;
    private int              mListIndex;
    private ColorSensor.ColorLED readAllColor;
    private int              schedDeltaRead = 6;
    private ColorSensor.ColorLED mLastReadColor;

    private static String sClassName;
    private static int sInstanceCount;
    private static ColorReader sInstance = null;
    public  static ColorReader getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new ColorReader(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private ColorReader(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mPeriodicIO = new PeriodicIO();
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        mColorSensor = new ColorSensor(colorSelectAPort,colorSelectBPort,colorValuePort);
    }
    
    private Loop mLoop = new Loop() {
        @Override
        public void onStart(Phase phase) {
            synchronized(ColorReader.this) {
                mSystemState = SystemState.HOLDING;
                mWantedState = WantedState.HOLD;
                mStateChanged = true;
                System.out.println(sClassName + " state " + mSystemState);
                // this subsystem is "on demand" so goto sleep
                mPeriodicIO.schedDeltaDesired = 0;
                stop(); // put into a known state
                mLastReadColor = ColorLED.RED;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized(ColorReader.this) {
                SystemState newState;
                switch(mSystemState) {
                    case READING_RED:
                        newState = handleReadingRed();
                        break;
                    case READING_GREEN:
                        newState = handleReadingGreen();
                        break;
                    case READING_BLUE:
                        newState = handleReadingBlue();
                        break;
                    case READING_CLEAR:
                        newState = handleReadingClear();
                        break;
                    case READING_ALL:
                        newState = handleReadingAll();
                        break;
                    case SCANNING:
                        newState = handleScanning();
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
            mPeriodicIO.schedDeltaDesired = 0; // goto sleep
        }

        return defaultStateTransfer();
    }
    
    private SystemState handleReadingRed() {
        if (mStateChanged) {
            mColorSensor.startFreqReading(ColorSensor.ColorLED.RED);
            mPeriodicIO.schedDeltaDesired = schedDeltaRead;
            mLastReadColor = ColorLED.RED;
        }
        else if (mColorSensor.updateFreqReading()){
            setWantedState(WantedState.HOLD);
        }

        return defaultStateTransfer();
    }

    private SystemState handleReadingGreen() {
        if (mStateChanged) {
            mColorSensor.startFreqReading(ColorSensor.ColorLED.GREEN);
            mPeriodicIO.schedDeltaDesired = schedDeltaRead;
            mLastReadColor = ColorLED.GREEN;
        }
        else if (mColorSensor.updateFreqReading()){
            setWantedState(WantedState.HOLD);
        }

        return defaultStateTransfer();
    }

    private SystemState handleReadingBlue() {
        if (mStateChanged) {
            mColorSensor.startFreqReading(ColorSensor.ColorLED.BLUE);
            mPeriodicIO.schedDeltaDesired = schedDeltaRead;
            mLastReadColor = ColorLED.BLUE;
        }
        else if (mColorSensor.updateFreqReading()){
            setWantedState(WantedState.HOLD);
        }

        return defaultStateTransfer();
    }

    private SystemState handleReadingClear() {
        if (mStateChanged) {
            mColorSensor.startFreqReading(ColorSensor.ColorLED.CLEAR);
            mPeriodicIO.schedDeltaDesired = schedDeltaRead;
            mLastReadColor = ColorLED.CLEAR;
        }
        else if (mColorSensor.updateFreqReading()){
            setWantedState(WantedState.HOLD);
        }

        return defaultStateTransfer();
    }

    private SystemState handleReadingAll() {
        if (mStateChanged) {
            readAllColor = ColorSensor.ColorLED.RED;
            mColorSensor.startFreqReading(readAllColor);
            mPeriodicIO.schedDeltaDesired = schedDeltaRead;
        }

        switch (readAllColor){
            case RED:
                if (mColorSensor.updateFreqReading()){
                    readAllColor = ColorSensor.ColorLED.GREEN;
                    mColorSensor.startFreqReading(readAllColor);
                }
                break;
            case GREEN:
                if (mColorSensor.updateFreqReading()){
                    readAllColor = ColorSensor.ColorLED.BLUE;
                    mColorSensor.startFreqReading(readAllColor);
                }
                break;
            case BLUE:
                if (mColorSensor.updateFreqReading()){
                    readAllColor = ColorSensor.ColorLED.CLEAR;
                    mColorSensor.startFreqReading(readAllColor);
                }
                break;
            case CLEAR:
                if (mColorSensor.updateFreqReading()){
                    setWantedState(WantedState.HOLD);
                }
                break;
            default:
        }

        return defaultStateTransfer();
    }

    private SystemState handleScanning() {
        if (mStateChanged || mRestartScanning) {
            mColorSensor.freqReading(true,mLastReadColor);
            mPeriodicIO.schedDeltaDesired = 3;
            
        }
        else if (mColorSensor.freqReading(false,mLastReadColor)){
            mScanColor = colorLookup(mLastReadColor, mColorSensor.getLastFreqReading(mLastReadColor));
            mColorSensor.startFreqReading(mLastReadColor);
        }

        return defaultStateTransfer();
    }

    private double[][] mColorLookup = {
        // red    grn    blu    yel   whi     bla    paper
        { 78000, 25000, 27000,101000,108000, 16000}, // red sensor
        { 21000, 46000, 64000, 85000,111000, 13000}, // green
        { 33000, 40000,118000, 52000,143000, 17000}, // blue
        {122000,116000,215000,242000,367000, 46000}, // clear
    };
    private String[] mColorName = {"red","green","blue","yellow","white","black"};
    private String colorLookup(ColorSensor.ColorLED sensor, double freq){
        int index = sensor.ordinal();
        int closestIndex=0;
        double distance = Double.POSITIVE_INFINITY;
        double d2 = 0;
        for(int i=0; i<mColorLookup[0].length; i++){
            double dist = Math.abs(mColorLookup[index][i]-freq);
            if (dist < distance){
                closestIndex = i;
                distance = dist;
                d2 = mColorLookup[index][i]-freq;
            }
        }

        return mColorName[closestIndex]+" ("+(Math.round(freq/1000.0)*1000.0)+", "+mColorLookup[index][closestIndex]+", "+Math.round(d2)+")";
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case READ_RED:
                return SystemState.READING_RED;
            case READ_GREEN:
                return SystemState.READING_GREEN;
            case READ_BLUE:
                return SystemState.READING_BLUE;
            case READ_CLEAR:
                return SystemState.READING_CLEAR;
            case READ_ALL:
                return SystemState.READING_ALL;
            case SCAN:
                return SystemState.SCANNING;
            case HOLD:
            default:
                return SystemState.HOLDING;
        }
    }

    public synchronized void setWantedState(WantedState state) {
        if (state != mWantedState){
            mSubsystemManager.scheduleMe(mListIndex, 1, true);
            System.out.println("waking " + sClassName);
        }
        
        mWantedState = state;
    }

    @Override
    public void stop() {
        mRestartScanning = false;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        mListIndex = enabledLooper.register(mLoop);
    }

    @Override
    public String getLogHeaders() {
        if (mLoggingEnabled){
            return sClassName+".systemState,"+
                   sClassName+".schedDeltaDesired,"+
                   sClassName+".schedDeltaActual,"+
                   sClassName+".schedDuration";
        }

        return null;
    }

    private String generateLogValues(boolean telemetry){
        String values;
        if (telemetry){
            values = ""+mSystemState+","+
                        /*mPeriodicIO.schedDeltaDesired+*/","+
                        /*mPeriodicIO.schedDeltaActual+*/","
                        /*mPeriodicIO.schedDuration*/;
        }
        else{
            mPeriodicIO.schedDuration = Timer.getFPGATimestamp() - mPeriodicIO.lastSchedStart;
            values = ""+mSystemState+","+
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
        SmartDashboard.putNumber("RED",   Math.round(mColorSensor.getLastFreqReading(ColorSensor.ColorLED.RED)  /1000.0)*1000.0);
        SmartDashboard.putNumber("GREEN", Math.round(mColorSensor.getLastFreqReading(ColorSensor.ColorLED.GREEN)/1000.0)*1000.0);
        SmartDashboard.putNumber("BLUE",  Math.round(mColorSensor.getLastFreqReading(ColorSensor.ColorLED.BLUE) /1000.0)*1000.0);
        SmartDashboard.putNumber("CLEAR", Math.round(mColorSensor.getLastFreqReading(ColorSensor.ColorLED.CLEAR)/1000.0)*1000.0);
        SmartDashboard.putString("Scan Color", mScanColor);
    }

    public static class PeriodicIO {
        // LOGGING
        public  int    schedDeltaDesired;
        public  double schedDeltaActual;
        public  double schedDuration;
        private double lastSchedStart;

        // INPUTS

        //OUTPUTS
    }

}