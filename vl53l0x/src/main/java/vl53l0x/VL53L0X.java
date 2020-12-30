package vl53l0x;

import edu.wpi.first.wpilibj.I2C;

/**
 * The VL53L0X class provides an interface to Lidar capabilities of the
 * STMicroelectronics VL53L0X Sensor via I2C communications interface on the RoboRIO.
 *
 * <p>The VL53L0X is a laser-ranging sensor, which measures the range to a target
 * object up to 2 m away. It uses time-of-flight measurements of infrared pulses for ranging,
 * allowing it to give accurate results independent of the target’s color and surface
 *
 */

public class VL53L0X implements AutoCloseable {
    public static final int DEFAULT_ADDRESS = 0x29;
    public static final int INVALID_RANGE = 65535;

    private IOCompleteNotification  io_complete_sink;

    private IOExceutor ioExceutor;
    private IIOProvider io;

    /* Configuration/Status */
    private volatile byte update_rate_hz;
    private volatile int range;
    private double last_sensor_timestamp;

    /**
     * Constructs the VL53L0X class using I2C communication, overriding the
     * default update rate with a custom rate which may be from 4 to 25 Hz,
     * representing the number of updates per second sent by the sensor.
     *<p>
     * This constructor should be used if communicating via I2C.
     *<p>
     * Note that increasing the update rate may increase the CPU utilization.
     *<p>
     * @param i2cPortId I2C Port to use
     * @param updateRateInHz Custom Update Rate (Hz)
     */
    public VL53L0X(I2C.Port i2cPortId, int updateRateInHz) {
        commonInit(update_rate_hz);
        io = new RegisterIO(new I2C(i2cPortId, DEFAULT_ADDRESS), updateRateInHz, io_complete_sink);
        ioExceutor.setRegisterIO(io);
        ioExceutor.start();
    }

    /**
     * Returns the sensor reading in in millimeters.  If the sensor can't determine
     * range, it returns a value INVALID_RANGE (65535.)
     * <p>
     * @return Sensor reading in in millimeters
     */
    public synchronized int getRange() {
        return range;
    }

    /**
     * Indicates whether the sensor is currently connected
     * to the host computer.  A connection is considered established
     * whenever communication with the sensor has occurred recently.
     *<p>
     * @return Returns true if a valid update has been recently received
     * from the sensor.
     */
    public boolean isConnected() {
        return io.isConnected();
    }

    /**
     * Indicates whether the sensor initialized successfully.
     *<p>
     * @return Returns <code>true</code> if sensor initialized successfully.
     */
    public boolean isInitialized() {
        return io.isInitialized();
    }

    /**
     * Returns the count of valid updates which have been received from the sensor.
     * This count should increase at the same rate indicated by the configured
     * update rate.
     * @return The number of valid updates received from the sensor.
     */
    public double getUpdateCount() {
        return io.getUpdateCount();
    }

    /**
     * Returns the sensor timestamp corresponding to the last sample
     * retrieved from the sensor.
     * @return The sensor timestamp corresponding to the current sensor data.
     */
    public double getLastSensorTimestamp() {
        return this.last_sensor_timestamp;
    }

    /**
     * Enables or disables logging (via Console I/O) of VL53L0X library internal
     * behaviors, including events such as transient communication errors.
     * @param enable
     */
    public void enableLogging(boolean enable) {
        if ( this.io != null) {
            io.enableLogging(enable);
        }
    }

    private void commonInit( byte update_rate_hz ) {
        this.io_complete_sink = new IOCompleteNotification();
        this.ioExceutor = new IOExceutor();
        this.update_rate_hz = update_rate_hz;
    }

    @Override
    public void close() throws Exception {
        if (ioExceutor != null) {
            ioExceutor.stop();
        }
    }

    /***********************************************************/
    /* CrashTrackingRunnable Implementation                    */
    /***********************************************************/

    private class IOExceutor {
        private Thread mThread = null;

        public void setRegisterIO(IIOProvider io) {
            mThread = new Thread(new CrashTrackingRunnable() {
                @Override
                public void runCrashTracked() {
                    if (io != null) {
                        io.run();
                    }
                }
            });
        }

        public void start() {
            if (mThread != null) {
                mThread.start();
            }
        }

        public void stop() {
            mThread = null;
        }
    }

    /***********************************************************/
    /* IIOCompleteNotification Interface Implementation        */
    /***********************************************************/

    class IOCompleteNotification implements IIOCompleteNotification {

        @Override
        public void setLidarData(VL53L0XProtocol.LidarData data, double sensor_timestamp) {
            VL53L0X.this.range = data.range;
            VL53L0X.this.last_sensor_timestamp = sensor_timestamp;
        }
    }
}