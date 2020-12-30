package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.team254.lib.subsystems.Subsystem;

import frc.robot.Constants;

/**
 * Provides an LED controller using a CTR Canifier.
 */
public class LEDCanifier extends Subsystem {
    private CANifier mCanifier;
    private PeriodicOutputs mPeriodicOutputs;
    private boolean mOutputsChanged = true;
    private final boolean mLoggingEnabled = true; // used to disable logging for this subsystem only

    private static String sClassName;
    private static int sInstanceCount;
    private static LEDCanifier sInstance = null;
    public  static LEDCanifier getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new LEDCanifier(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private LEDCanifier(String caller) {
        sClassName        = this.getClass().getSimpleName();
        printUsage(caller);
        mCanifier = new CANifier(Constants.kCanifierId);
        mCanifier.configFactoryDefault(Constants.kLongCANTimeoutMs);
        int defaultFramePeriod = 255;
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_1_General, defaultFramePeriod, Constants.kLongCANTimeoutMs);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, defaultFramePeriod, Constants.kLongCANTimeoutMs);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_3_PwmInputs0, defaultFramePeriod, Constants.kLongCANTimeoutMs);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_4_PwmInputs1, defaultFramePeriod, Constants.kLongCANTimeoutMs);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_5_PwmInputs2, defaultFramePeriod, Constants.kLongCANTimeoutMs);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_6_PwmInputs3, defaultFramePeriod, Constants.kLongCANTimeoutMs);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_8_Misc, defaultFramePeriod, Constants.kLongCANTimeoutMs);
 
        mPeriodicOutputs = new PeriodicOutputs();

        // Force a first update.
        mOutputsChanged = true;
    }

    public int getDeviceId() {
        return mCanifier.getDeviceID();
    }

    public synchronized void setLEDColor(double red, double green, double blue) {
        if (red != mPeriodicOutputs.red || green != mPeriodicOutputs.green || blue != mPeriodicOutputs.blue) {
            mPeriodicOutputs.red = red;
            mPeriodicOutputs.green = green;
            mPeriodicOutputs.blue = blue;
            mOutputsChanged = true;
        }
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
    public synchronized void readPeriodicInputs() {
        // A: Blue
        // B: Green
        // C: Red
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // A: Blue
        // B: Green
        // C: Red
        if (mOutputsChanged) {
            mCanifier.setLEDOutput(mPeriodicOutputs.green, CANifier.LEDChannel.LEDChannelA);
            mCanifier.setLEDOutput(mPeriodicOutputs.red, CANifier.LEDChannel.LEDChannelB);
            mCanifier.setLEDOutput(mPeriodicOutputs.blue, CANifier.LEDChannel.LEDChannelC);
            mOutputsChanged = false;
        }
    }

    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {
        mPeriodicOutputs = new PeriodicOutputs();
        mOutputsChanged = true;
        writePeriodicOutputs();
    }

    @Override
    public void zeroSensors() {
    }

    private static class PeriodicOutputs {
        public double red;
        public double green;
        public double blue;
    }
}
