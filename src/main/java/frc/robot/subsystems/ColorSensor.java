package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;

public class ColorSensor {

    public enum ColorLED {
        RED,
        GREEN,
        BLUE,
        CLEAR
    }

    private enum ReadStage {
        STARTUP,
        READING,
        DONE
    }

    private final DigitalOutput mColorSelectA;
    private final DigitalOutput mColorSelectB;
    private final Counter mColorValue;
    private ColorLED mCurrentColor;

    private double mReadStartTimestamp;
    private double mReadTimeout = .2;

    private double mFreqRed = -1;
    private double mFreqGreen = -1;
    private double mFreqBlue = -1;
    private double mFreqClear = -1;

    private final int mFreqReadsMax = 250;
    private double[] mFreqHistory = new double[mFreqReadsMax];
    private double[] mFreqDurs = new double[mFreqReadsMax];
    private int mFreqReads = 0;
    private ReadStage mStage = ReadStage.DONE;

    public ColorSensor (int colorSelectAPort, int colorSelectBPort, int colorValuePort){
        mColorSelectA = new DigitalOutput(colorSelectAPort);
        mColorSelectB = new DigitalOutput(colorSelectBPort);
        mColorValue   = new Counter(colorValuePort);
    }

    public double getLastFreqReading(ColorLED color){
        switch(color){
            case RED:
                return mFreqRed;
            case GREEN:
                return mFreqGreen;
            case BLUE:
                return mFreqBlue;
            case CLEAR:
                return mFreqClear;
            default:
                break;
        }
        return -1;
    }

    public boolean freqReading(boolean firstTime, ColorLED color){
        if (firstTime){
            mCurrentColor = color;
            turnOnColor(mCurrentColor);
            mReadStartTimestamp = Timer.getFPGATimestamp(); 
            mStage = ReadStage.STARTUP;   
        }
        else {
            double now = Timer.getFPGATimestamp();
            if (mStage == ReadStage.STARTUP) {
                if (now - mReadStartTimestamp > .01){
                    mColorValue.reset();
                    mReadStartTimestamp = now;
                    mStage = ReadStage.READING;
                }
            }
            else{
                double readDur = now - mReadStartTimestamp;
                if (readDur > .025){
                    double freq = mColorValue.get()/readDur;
                    switch (mCurrentColor){
                        case RED:
                            mFreqRed = freq;
                            break;
                        case GREEN:
                            mFreqGreen = freq;
                            break;
                        case BLUE:
                            mFreqBlue = freq;
                            break;
                        case CLEAR:
                            mFreqClear = freq;
                            break;
                        default:
                            break;
                    }
                    return true;
                }
            }
        }
        return false;
    }

    public void startFreqReading(ColorLED color){
        mCurrentColor = color;
        turnOnColor(color);
        mFreqReads = 0;
        mColorValue.reset();
        mReadStartTimestamp = Timer.getFPGATimestamp();
    }

    public boolean updateFreqReading(){

        double readDur = Timer.getFPGATimestamp() - mReadStartTimestamp;
        if (mFreqReads == 0){

        }
        mFreqHistory[mFreqReads] = ((double)mColorValue.get())/readDur;
        mFreqDurs[mFreqReads] = readDur;
        mFreqReads++;

        if(readDur > mReadTimeout || mFreqReads >= mFreqReadsMax){
            double freq = mFreqHistory[mFreqReads-1];
            switch (mCurrentColor){
                case RED:
                    mFreqRed = freq;
                    break;
                case GREEN:
                    mFreqGreen = freq;
                    break;
                case BLUE:
                    mFreqBlue = freq;
                    break;
                case CLEAR:
                    mFreqClear = freq;
                    break;
                default:
                    break;
            }

            // for (int i=0; i<mFreqReads; i++){
            //     if (i==0){
            //         System.out.println(i+": "+mFreqHistory[i]+", "+mFreqDurs[i]+", "+(mFreqHistory[mFreqReads-1]-mFreqHistory[i]));
            //     }
            //     else {
            //         System.out.println(i+": "+(int)mFreqHistory[i]+", "+(double)((int)(((mFreqHistory[mFreqReads-1]-mFreqHistory[i])/mFreqHistory[mFreqReads-1])*10000.0))/100.0+",   "+((double)((int)(mFreqDurs[i]*10000.0)))/10.0+", "+(int)((mFreqDurs[i]-mFreqDurs[i-1])*1000.0));
            //     }
            // }

            mFreqReads = 0;
            return true;
        }


        return false;
    }

    public void resetAllFreqs(){
        mFreqRed = -1;
        mFreqGreen = -1;
        mFreqBlue = -1;
        mFreqClear = -1;
    }

    private void turnOnColor(ColorLED color){
        switch(color){
            case RED:
                mColorSelectA.set(false);
                mColorSelectB.set(false);
                break;
            case GREEN:
                mColorSelectA.set(true);
                mColorSelectB.set(true);
                break;
            case BLUE:
                mColorSelectA.set(false);
                mColorSelectB.set(true);
                break;
            case CLEAR:
                mColorSelectA.set(true);
                mColorSelectB.set(false);
                break;
            default:
        }
    }
}
