package as7262;

import edu.wpi.first.wpilibj.I2C;

public class AS7262 implements AutoCloseable {

    private IOCompleteNotification io_complete_sink;

    private IOExceutor ioExecutor;
    private IIOProvider io;

    static final int DEFAULT_UPDATE_RATE_HZ = 60;

    /* Configuration/Status */
    private volatile byte update_rate_hz;

    private volatile int temp_c;
    private volatile int[] rawValues = new int[6];
    private volatile float[] calibratedValues = new float[6];
    private double last_sensor_timestamp;


    /**
     * Constructs the AS726x class using I2C communication and the default update rate 1Hz.
     *<p>
     * This constructor should be used if communicating via I2C.
     */
    public AS7262(I2C.Port i2cPortId) { this(i2cPortId, DEFAULT_UPDATE_RATE_HZ); }

    /**
     * Constructs the AS726x class using I2C communication, overriding the
     * default update rate with a custom rate which may be from 1 to 20 Hz,
     * representing the number of updates per second sent by the sensor.
     *<p>
     * This constructor should be used if communicating via I2C.
     *<p>
     * Note that increasing the update rate may increase the CPU utilization.
     *<p>
     * @param i2cPortId I2C Port to use
     * @param updateRateInHz Custom Update Rate (Hz)
     */
    public AS7262(I2C.Port i2cPortId, int updateRateInHz) {
        commonInit(update_rate_hz);
        io = new RegisterIO(new I2C(i2cPortId, AS7262Registers.AS726x_ADDRESS), updateRateInHz, io_complete_sink);
        ioExecutor.setRegisterIO(io);
        ioExecutor.start();
    }

    /**
     * Returns the calibrated color sensor readings.
     * <p>
     * @return calibrated color sensor values.
     */
    public synchronized float[] getCalibratedValues() {
        return calibratedValues;
    }

    /**
     * Returns the raw color sensor readings.
     * <p>
     * @return raw color sensor values.
     */
    public synchronized int[] getRawValues() {
        return rawValues;
    }

    /**
     * Returns the current temperature (in degrees centigrade) reported by
     * the sensor's circuit.
     *<p>
     * This value may be useful in order to perform advanced temperature-
     * correction of raw color values.
     *<p>
     * @return The current temperature (in degrees centigrade).
     */
    public int getTempC()
    {
        return this.temp_c;
    }

    /**
     * Set the integration time for the sensor.
     *<p>
     * @param time the integration time to set in milliseconds. The actual integration time will be time*2.8ms
     */
    public void setIntegrationTime(byte time) {
        io.setIntegrationTime(time);
    }

    /**
     * Set the sensor gain.
     *<p>
     * @param gain the gain to set the sensor to.
     */
    public void setGain(AS7262Registers.ChannelGain gain) {
        io.setGain(gain.getValue());
    }

    /**
     * Turn on/off the driver LED.
     *<p>
     * @param enable True to turn LED on and False to turn it off.
     */
    public void enableDrvLed(boolean enable) {
        io.enableDrvLed(enable);
    }

    /**
     * Set the current limit for the driver LED.
     *<p>
     * @param limit the current limit setting.
     */
    public void setDrvCurrentLimit(AS7262Registers.DriverLEDCurrentLimit limit) {
        io.setDrvCurrentLimit(limit.getValue());
    }

    /**
     * Turn on/off the indicator LED
     *<p>
     * @param  enable True to turn LED on, False to turn it off
     */
    public void enableIndicateLED(boolean enable){
        io.enableIndicateLED(enable);
    }

    /**
     * Set the current limit for the indicator LED.
     *<p>
     * @param limit the current limit setting.
     */
    public void setIndicateCurrentLimit(AS7262Registers.IndicatorLEDCurrentLimit limit){
        io.setIndicateCurrentLimit(limit.getValue());
    }

    /**
     * Set the measurement mode.
     *<p>
     * @param mode the mode to set the sensor to.
     */
    public void setConversionType(AS7262Registers.MeasurementMode mode){
        io.setConversionType(mode.getValue());
    }

    /**
     * Indicates whether the sensor is currently connected
     * to the host computer.  A connection is considered established
     * whenever communication with the sensor has occurred recently.
     *<p>
     * @return Returns <code>true</code> if a valid update has been recently received
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
     * TODO: Do we want continuous mode?
     * This count should increase at the same rate indicated by the configured
     * update rate.
     *<p>
     * @return The number of valid updates received from the sensor.
     */
    public double getUpdateCount() {
        return io.getUpdateCount();
    }

    /**
     * Returns the sensor timestamp corresponding to the last sample
     * retrieved from the sensor.
     *<p>
     * @return The sensor timestamp corresponding to the last sampled sensor data.
     */
    public double getLastSensorTimestamp() {
        return this.last_sensor_timestamp;
    }

    /**
     * Enables or disables logging (via Console I/O) of AS7262 library internal
     * behaviors, including events such as transient communication errors.
     *<p>
     * @param enable
     */
    public void enableLogging(boolean enable) {
        if (this.io != null) {
            io.enableLogging(enable);
        }
    }

    /**
     * Reset the sensor. Sensor may take up to one second to reset.
     */
    public void softReset() {
        if (this.io != null) {
            io.reset();
        }
    }


    private void commonInit( byte update_rate_hz ) {
        this.io_complete_sink = new IOCompleteNotification();
        this.ioExecutor = new IOExceutor();
        this.update_rate_hz = update_rate_hz;
    }

    @Override
    public void close() throws Exception {
        if (ioExecutor != null) {
            ioExecutor.stop();
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
        public void setSensorData(AS7262Protocol.ColorData raw_data_update, double sensor_timestamp) {
            for (int i = 0; i < 6; i++) {
                AS7262.this.rawValues[i] = raw_data_update.rawValues[i];
                AS7262.this.calibratedValues[i] = raw_data_update.calibratedValues[i];
            }
            AS7262.this.temp_c = raw_data_update.temp_c;
            AS7262.this.last_sensor_timestamp = sensor_timestamp;
        }
    }
}
