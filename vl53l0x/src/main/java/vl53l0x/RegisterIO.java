package vl53l0x;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

public class RegisterIO implements IIOProvider {
    private static final int kTimeoutIncrement = 750000; // milliseconds
    private static final double DELAY_OVERHEAD_SECONDS = 0.004;

    private I2C port;

    private int update_rate_hz;
    private boolean stop;
    private boolean isInitialized;
    private IIOCompleteNotification notify_sink;
    private double last_update_time;
    private int update_count;
    private MovingAverage average;

    private VL53L0XProtocol.LidarData data = new VL53L0XProtocol.LidarData();
    private VL53L0XProvider vl53L0XProvider;

    public RegisterIO(I2C i2c_port, int update_rate_hz, IIOCompleteNotification notify_sink) {
        this.port = i2c_port;
        this.update_rate_hz = (byte)Math.max(Math.min(update_rate_hz, 20), 4);
        this.notify_sink = notify_sink;
        vl53L0XProvider = new VL53L0XProvider(i2c_port);
        isInitialized = false;
        average = new MovingAverage(10);
    }

    private final double IO_TIMEOUT_SECONDS = 1.0;

    public void stop() {
        stop = true;
    }

    public void run() {
        vl53L0XProvider.setTimeout(0);
        while (!isInitialized ) {
            isInitialized = vl53L0XProvider.init(true);  // io_provider.init();
            Timer.delay(0.500);
        }

        vl53L0XProvider.setTimeout(kTimeoutIncrement);  // milliseconds

        /* Calculate delay to match configured update rate */
        /* Note:  some additional time is removed from the */
        /* 1/update_rate value to ensure samples are not   */
        /* dropped, esp. at higher update rates.           */
        double update_rate = 1.0/((double)((int)(this.update_rate_hz & 0xFF)));
        if ( update_rate > DELAY_OVERHEAD_SECONDS) {
        	update_rate -= DELAY_OVERHEAD_SECONDS;
        }

        // TODO: Use continuous measurement mode and try to match User parameneter?
        vl53L0XProvider.startContinuous(0);

        /* IO Loop */
        while (!stop) {
            getCurrentData();
            Timer.delay(update_rate);
        }
    }
    
    private void getCurrentData() {
        // Get the range reading from vl53l0x here
        int retries = 3;
        int range = 0;

        while (retries-- > 0) {
            range = vl53L0XProvider.readRangeSingleMillimeters();
            if (range == 65535) {
                vl53L0XProvider.setTimeout(vl53L0XProvider.getTimeout() + kTimeoutIncrement);
            } else {
                break;
            }
        }
        average.add(range);
        data.range = (int)average.getAverage();
        this.last_update_time = Timer.getFPGATimestamp();
        notify_sink.setLidarData(data, this.last_update_time);
        update_count++;
    }

    @Override
    public boolean isInitialized()  { return isInitialized; }

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
        vl53L0XProvider.enableLogging(enable);
    }
}