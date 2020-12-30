package as7262;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

public class RegisterIO implements IIOProvider {
    private static final int kTimeoutIncrement = 750000; // microseconds
    private static final double DELAY_OVERHEAD_SECONDS = 0.004;

    private I2C port;

    private int update_rate_hz;
    private volatile boolean stop;
    private volatile boolean isInitialized;
    private IIOCompleteNotification notify_sink;
    private double last_update_time;
    private int update_count;

    private AS7262Protocol.ColorData data = new AS7262Protocol.ColorData();
    private AS7262Provider as7262Provider;

    public RegisterIO(I2C i2c_port, int update_rate_hz, IIOCompleteNotification notify_sink) {
        this.port = i2c_port;
        this.update_rate_hz = (byte)Math.max(Math.min(update_rate_hz, 50), 1);
        this.notify_sink = notify_sink;
        as7262Provider = new AS7262Provider(i2c_port);
        isInitialized = false;
    }

    private final double IO_TIMEOUT_SECONDS = 1.0;

    public void stop() {
        stop = true;
    }

    public void run() {
        while (!stop) {
            while (!isInitialized && !stop) {
                System.out.println("Initializing as726xProvider");
                isInitialized = as7262Provider.init();  // io_provider.init();
                if (!isInitialized) {
                    Timer.delay(0.500);
                }
            }

            as7262Provider.setTimeout(kTimeoutIncrement);  // microseconds

            /* Calculate delay to match configured update rate */
            /* Note:  some additional time is removed from the */
            /* 1/update_rate value to ensure samples are not   */
            /* dropped, esp. at higher update rates.           */
            double update_rate = 1.0 / ((double) ((int) (this.update_rate_hz & 0xFF)));
            if (update_rate > DELAY_OVERHEAD_SECONDS) {
                update_rate -= DELAY_OVERHEAD_SECONDS;
            }

            // TODO: Use continuous measurement mode and try to match User parameter?
            //as726xProvider.startContinuous(0);

            /* IO Loop */
            while (isInitialized && !stop) {
                getCurrentData();
                Timer.delay(update_rate);
            }
        }
    }
    
    private void getCurrentData() {
        // Get the readings from as7262 here
        int retries = 3;
        boolean hasData = false;
        float[] calibratedValues = new float[AS7262Registers.AS726x_NUM_CHANNELS + 1];
        int[] rawValues = new int[AS7262Registers.AS726x_NUM_CHANNELS + 1];

//        as7262Provider.startMeasurement();
        while (retries-- > 0) {
            if (as7262Provider.dataReady()) {
                hasData = true;
                as7262Provider.readRawValues(rawValues);
                as7262Provider.readCalibratedValues(calibratedValues);
                break;
            }
        }

        if (hasData) {
            for (int i = 0; i < AS7262Registers.AS726x_NUM_CHANNELS; i++) {
                data.rawValues[i] = rawValues[i];
                data.calibratedValues[i] = calibratedValues[i];
            }
            data.temp_c = as7262Provider.readTemperature();
            this.last_update_time = Timer.getFPGATimestamp();
            notify_sink.setSensorData(data, this.last_update_time);
            update_count++;
        }
    }

    @Override
    public boolean isInitialized()  { return isInitialized; }

    @Override
    public void setGain(byte gain) {
        if (isInitialized) {
            as7262Provider.setGain(gain);
        }
    }

    @Override
    public boolean isConnected() {
        double time_since_last_update = Timer.getFPGATimestamp() - this.last_update_time;
        return time_since_last_update <= IO_TIMEOUT_SECONDS;
    }

    @Override
    public double getUpdateCount() {
        return update_count;
    }

    @Override
    public void enableLogging(boolean enable) {
        as7262Provider.enableLogging(enable);
    }

    @Override
    public void setIntegrationTime(byte time) {
        if (isInitialized) {
            as7262Provider.setIntegrationTime(time);
        }
    }

    @Override
    public void enableDrvLed(boolean enable) {
        if (isInitialized) {
            if (enable) {
                as7262Provider.drvOn();
            } else {
                as7262Provider.drvOff();
            }
        }
    }

    @Override
    public void setDrvCurrentLimit(byte limit) {
        if (isInitialized) {
            as7262Provider.setDrvCurrent(limit);
        }
    }

    @Override
    public void enableIndicateLED(boolean enable) {
        if (isInitialized) {
            as7262Provider.indicatorLED(enable);
        }
    }

    @Override
    public void setIndicateCurrentLimit(byte limit) {
        if (isInitialized) {
            as7262Provider.setIndicatorLEDCurrent(limit);
        }
    }

    @Override
    public void setConversionType(byte type) {
        if (isInitialized) {
            as7262Provider.setMeasurementMode(type);
        }
    }

    @Override
    public void reset() {
        isInitialized = false;
    }
}