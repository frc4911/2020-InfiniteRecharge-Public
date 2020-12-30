package as7262;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

import java.nio.ByteBuffer;

import static as7262.AS7262Registers.*;

public class AS7262Provider {

    private I2C mPort;
    private boolean trace = false;
    private double  polling_delay = 0.010; // milliseconds
    private boolean did_timeout;
    private int timeout_start_ms;
    // The value is large enough
    private int io_timeout = 1000000;	// 1 second in microseconds

    private enum BYTE_SIZE {
        SINGLE(1),
        DOUBLE(2),
        QUAD(4);

        public int value;

        BYTE_SIZE(int value) {
            this.value = value;
        }
    }

    /**
     * Creates an AS726xProvider instance.
     * <p/>
     * @param port  an IC2 object to use for I2C communication
     */
    public AS7262Provider(I2C port) {
        mPort = port;
        this.did_timeout = false;
    }

    // Record the current time to check an upcoming timeout against
    private int startTimeout() {
        double now = HALUtil.getFPGATime();
        return (timeout_start_ms = (short) now);
    }

    // Check if timeout is enabled (set to nonzero value) and has expired
    private boolean checkTimeoutExpired() {
        int now = (int) (HALUtil.getFPGATime());
        return (io_timeout > 0 && (now - timeout_start_ms) > io_timeout);
    }

    /**
     * Sets how long to wait with reading or writing using Virtual Registers before aborting.
     * <p>
     * @param timeout in microseconds
     */
    public void setTimeout(int timeout) {
        io_timeout = Math.max(timeout, 0);
    }

    /**
     * Gets how long wait time, in microseconds, is for reading or writing using Virtual Registers before aborting.
     */
    public int getTimeout() {
        return io_timeout;
    }

    public void	enableLogging(boolean enable) {
        trace = enable;
    }

    /**
     * Set up hardware and begin communication with the sensor.
     *<p>
     * @return true on success; otherwise false.
    */
    public boolean init() {
        if (trace) { System.out.println("as7262::init() Entered"); }
//        virtualWriteRegister(AS726X_CONTROL_SETUP, (byte)0x80);
        // Reset the device
        boolean aborted = mPort.write(AS726X_CONTROL_SETUP, 0x80);

        // Wait for 1000ms for it to boot up
        Timer.delay(1.0);

        // try to read the version reg to make sure we can connect
        byte version = virtualReadRegister(AS726X_HW_VERSION);

        if (version != 0x40) {
//            if (trace) { System.out.println("as7262::init() abort - AS726X_HW_VERSION mismatch = " + version); }
            return false;
        }

        enableInterrupt();
        setDrvCurrent(DriverLEDCurrentLimit.LIMIT_100MA);
        drvOff();
        setIntegrationTime((byte)(1000 * polling_delay / AS726x_INTEGRATION_TIME_MULT));
        setGain(ChannelGain.GAIN_64X);
        setMeasurementMode(MeasurementMode.MODE_2);

        if (trace) { System.out.println("as7262::init() Exit"); }
        System.out.println("as7262 initialized");
        return true;
    }

    /**
     * Does a soft reset.
     * <p>
     *  Give sensor at least 1000ms to reset
     */
    public void softReset() {
        byte value = virtualReadRegister(AS726X_CONTROL_SETUP);
        value = (byte)(value | (1 << 7));  // Set RST bit, automatically cleared after reset
        virtualWriteRegister(AS726X_CONTROL_SETUP, value);
    }

    /**
     * Turn on the driver LED.
    */
    public void drvOn() {
        if (trace) { System.out.println("as7262::drvOn() Enter"); }
        byte value = virtualReadRegister(AS726X_LED_CONTROL);
        value = (byte)(value | (0b1 << 3));
        virtualWriteRegister(AS726X_LED_CONTROL, value);
        if (trace) { System.out.println("as7262::drvOn() Exit"); }
    }

    /**
     * Turn off the driver LED.
    */
    public void drvOff() {
        if (trace) { System.out.println("as7262::drvOff() Enter"); }
        byte value = virtualReadRegister(AS726X_LED_CONTROL);
        value = (byte)(value & ~(0b1 << 3));
        virtualWriteRegister(AS726X_LED_CONTROL, value);
        if (trace) { System.out.println("as7262::drvOff() Exit"); }
    }

    /**
     * Set the current limit for the driver LED.
     *<p>
     * @param limit the current limit setting.
    */
    public void setDrvCurrent(DriverLEDCurrentLimit limit) {
        setDrvCurrent(limit.getValue());
    }

    /**
     * Set the current limit for the driver LED.
     *<p>
     * @param current the current limit setting. Should be one of LIMIT_12MA5, LIMIT_25MA, LIMIT_50MA, or LIMIT_100MA = 0b11.
    */
    public void setDrvCurrent(byte current) {
        if (trace) { System.out.println("as7262::setDrvCurrent() Enter"); }
        if (current > 0b11) {
            current = 0b11;
        }

        byte value = virtualReadRegister(AS726X_LED_CONTROL);
        value = (byte)(value & 0b11001111);  // Clear the ICL_DRV bits
        value = (byte)(value | (current << 4));
        virtualWriteRegister(AS726X_LED_CONTROL, value);
        if (trace) { System.out.println("as7262::setDrvCurrent() Exit"); }
    }

    /**
     * Turn on/off the indicator LED
     *<p>
     * @param  on True if you want the LED on, False to turn off
    */
    public void indicatorLED(boolean on) {
        if (trace) { System.out.println("as7262::indicatorLED("+ on + ") Enter"); }
        byte value = virtualReadRegister(AS726X_LED_CONTROL);
        value = (byte) (on ? value | 0b1 : value & ~0b1);
        virtualWriteRegister(AS726X_LED_CONTROL, value);
        if (trace) { System.out.println("as7262::indicatorLED("+ on + ") Exit"); }
    }

    /**
     * Set the current limit for the driver LED.
     *<p>
     * @param current the current limit setting. Should be one of LIMIT_1MA, LIMIT_2MA, LIMIT_4MA, or LIMIT_8MA
    */
    public void setIndicatorLEDCurrent(byte current) {
        if (trace) { System.out.println("as7262::setIndicatorLEDCurrent("+ current + ") Enter"); }
        if (current > 0b11) {
            current = 0b11;
        }

        byte value = virtualReadRegister(AS726X_LED_CONTROL);
        value = (byte)(value & 0b11111001); // Clear the ICL_IND bite
        value = (byte)(value | (current << 1));
        virtualWriteRegister(AS726X_LED_CONTROL, value);
        if (trace) { System.out.println("as7262::setIndicatorLEDCurrent("+ current + ")Exit"); }
    }

    /**
     * Set the conversion mode.
     *<p>
     * @param type the mode to set the sensor to.
     */
    public void setMeasurementMode(MeasurementMode type) {
        setMeasurementMode(type.getValue());
    }

    /**
     * Set the measurement mode.
     *<p>
     * @param mode the mode to set the sensor to. Should be one of MODE_0, MODE_1, MODE_2, ONE_SHOT.
    */
    public void setMeasurementMode(byte mode) {
        if (trace) { System.out.println("as7262::setMeasurementMode("+ mode +") Entered"); }
        if (mode > 0b11) {
            mode = 0b11;
        }

        byte value = virtualReadRegister(AS726X_CONTROL_SETUP);
        value = (byte)(value & 0b11110011); // Clear the BANK bits
        value = (byte)(value | (byte)(mode << 2));
        virtualWriteRegister(AS726X_CONTROL_SETUP, value);

        if (trace) { System.out.println("as7262::setMeasurementMode(), Exit"); }
    }

    /**
     * Set the sensor gain.
     *<p>
     * @param gain the gain to set the sensor to.
     */
    public void setGain(ChannelGain gain) {
        setGain(gain.getValue());
    }

    /**
     * Set the sensor gain.
     *<p>
     * @param gain the gain to set the sensor to. Should be one of GAIN_1X, GAIN_3X7, GAIN_16X, or GAIN_64X = 0b11.
    */
    public void setGain(byte gain) {
        if (trace) { System.out.println("as7262::setGain() Enter"); }
        if (gain > 0b11) {
            gain = 0b11;
        }

        byte value = virtualReadRegister(AS726X_CONTROL_SETUP);
        value = (byte)(value & 0b11001111);  // Clear the GAIN  bits
        value = (byte) (value | (byte)(gain << 4));

        virtualWriteRegister(AS726X_CONTROL_SETUP, value);
        if (trace) { System.out.println("as7262::setGain() Exit"); }
    }

    /**
     * Set the integration time for the sensor.
     *<p>
     * @param time the integration time to set. The actual integration time will be time*2.8ms
    */
    public void setIntegrationTime(byte time) {
        time = (byte) Math.max(1, time);
        if (trace) { System.out.println("as7262::setIntegrationTime("+ time +") Enter"); }
        virtualWriteRegister(AS726X_INT_T, time);
        if (trace) { System.out.println("as7262::setIntegrationTime() Exit"); }
        polling_delay = AS726x_INTEGRATION_TIME_MULT * time / 1000;  // milliseconds
    }

    /**
     * Enable the device interrupt
    */
    public void enableInterrupt() {
        if (trace) { System.out.println("as7262::enableInterrupt() Enter"); }
        byte value = virtualReadRegister(AS726X_CONTROL_SETUP);
        value = (byte) (value | (byte)(0b1 << 6));
        virtualWriteRegister(AS726X_CONTROL_SETUP, value);
        if (trace) { System.out.println("as7262::enableInterrupt() Exit"); }
    }

    /**
     * Disable the device interrupt
    */
    public void disableInterrupt() {
        if (trace) { System.out.println("as7262::disableInterrupt() Enter"); }
        byte value = virtualReadRegister(AS726X_CONTROL_SETUP);
        value = (byte)(value & (byte)~(0b1 << 6));
        virtualWriteRegister(AS726X_CONTROL_SETUP, value);
        if (trace) { System.out.println("as7262::disableInterrupt() Exit"); }
    }

    /**
    *  Begin a measurement. This sets the conversion mode to ONE_SHOT.
    */
    public void startMeasurement() {
        if (trace) { System.out.println("as7262::startMeasurement() Enter"); }
        byte value = virtualReadRegister(AS726X_CONTROL_SETUP);
        value = (byte)(value & (byte)~(0b1));
        virtualWriteRegister(AS726X_CONTROL_SETUP, value);

        setMeasurementMode(MeasurementMode.ONE_SHOT);
        if (trace) { System.out.println("as7262::startMeasurement() Exit"); }
    }

    /**
     * Read an individual raw spectral channel
     *<p>
     * @param channel the channel to read
     * @return the reading as a raw 16-bit integer
     */
    int count = 0;
    private int readChannel(byte channel) {
        if (trace) { System.out.println("as7262::readChannel("+ channel +") Enter"); }

        byte byte0 = virtualReadRegister(channel);
        byte byte1 = virtualReadRegister((byte)(channel+1));

        byte buffer[] = new byte[4];
        buffer[0] = 0;
        buffer[1] = 0;
        buffer[2] = byte0;
        buffer[3] = byte1;
        ByteBuffer readBuffer = ByteBuffer.wrap(buffer);
        int value = readBuffer.getInt();
        if (trace) { System.out.println("as7262::readChannel("+ channel +") Exit value = " + value + ", byte0=" + byte0 + ", byte1=" + byte1); }
        return value;
    }

    /**
     * Check if the sensor is ready to return data
     *<p>
     * @return true if data is ready to be read, false otherwise.
    */
    public boolean dataReady() {
        if (trace) { System.out.println("as7262::dataReady() Enter"); }
        byte value = virtualReadRegister(AS726X_CONTROL_SETUP);
        boolean ready = (value & (1 << 1)) > 0;
        if (trace) { System.out.println("as7262::dataReady() Exit is ready = " +  ready); }
        return ready;
    }

    /**
     * Read the on-board temperature sensor
     *<p>
     * @return the temperature in Centigrade.
    */
    public int readTemperature() {
        if (trace) { System.out.println("as7262::readTemperature() Enter"); }
        int temp = (int)virtualReadRegister(AS726X_DEVICE_TEMP);
        if (trace) { System.out.println("as7262::readTemperature() Exit, temp= " + temp); }
        return temp;
    }

    /**
     * Read raw violet color value (AS7262 only)
     *<p>
     * @return the violet reading as an unsigned 16-bit integer
     */
    public int readViolet() {
        return(readChannel(AS7262_VIOLET));
    }
    /**
     * Read raw blue color value (AS7262 only)
     *<p>
     * @return the blue reading as an unsigned 16-bit integer
     */
     public int readBlue() { return(readChannel(AS7262_BLUE)); }

    /**
     * Read raw green color value (AS7262 only)
     *<p>
     * @return the green reading as an unsigned 16-bit integer
    */
    public int readGreen() { return(readChannel(AS7262_GREEN)); }

    /**
     * Read raw yellow color value (AS7262 only)
     *<p>
     * @return the yellow reading as an unsigned 16-bit integer
    */
    public int readYellow() { return(readChannel(AS7262_YELLOW)); }

    /**
     * Read raw orange color value (AS7262 only)
     *<p>
     * @return the orange reading as an unsigned 16-bit integer
    */
    public int readOrange() { return(readChannel(AS7262_ORANGE)); }

    /**
     * Read raw red color value (AS7262 only)
     *<p>
     * @return the red reading as an unsigned 16-bit integer
    */
    public int readRed() { return(readChannel(AS7262_RED)); }

    /**
     * Read calibrated violet color value (AS7262 only)
     *<p>
     * @return the violet reading as a 32-bit floating point number
    */
    public float readCalibratedViolet() { return(readCalibratedValue(AS7262_VIOLET_CALIBRATED)); }

    /**
     * Read calibrated blue color value (AS7262 only)
     *<p>
     * @return the blue reading as a 32-bit floating point number
    */
    public float readCalibratedBlue() { return(readCalibratedValue(AS7262_BLUE_CALIBRATED)); }

    /**
     * Read calibrated green color value (AS7262 only)
     *<p>
     * @return the green reading as a 32-bit floating point number
    */
    public float readCalibratedGreen() { return(readCalibratedValue(AS7262_GREEN_CALIBRATED)); }

    /**
     * Read calibrated yellow color value (AS7262 only)
     *<p>
     * @return the yellow reading as a 32-bit floating point number
    */
    public float readCalibratedYellow() { return(readCalibratedValue(AS7262_YELLOW_CALIBRATED)); }

    /**
     * Read calibrated orange color value (AS7262 only)
     *<p>
     * @return the orange reading as a 32-bit floating point number
    */
    public float readCalibratedOrange() { return(readCalibratedValue(AS7262_ORANGE_CALIBRATED)); }

    /**
     * Read calibrated red color value (AS7262 only)
     *<p>
     * N@return the red reading as a 32-bit floating point number
    */
    public float readCalibratedRed() { return(readCalibratedValue(AS7262_RED_CALIBRATED)); }

    /*==== END MEASUREMENTS =====*/

    /**
     * Read the raw channels  Defaults to AS726x_NUM_CHANNELS.
     *<p>
     * @param buf the buffer to read the data into
     */
    public void readRawValues(int[] buf) { readRawValues(buf, AS726x_NUM_CHANNELS); }

    /**
     * Read the raw channels
     *<p>
     * @param buf the buffer to read the data into
     * @param num Number of channels to read. Defaults to AS726x_NUM_CHANNELS
     */
    public void readRawValues(int[] buf, int num)
    {
        num = Math.min(num, AS726x_NUM_CHANNELS);
        for (int i = 0; i < num; i++) {
            switch(i) {
                case AS726x_VIOLET:
                    buf[i] = readViolet();
                    break;
                case AS726x_BLUE:
                    buf[i] = readBlue();
                    break;
                case AS726x_GREEN:
                    buf[i] = readGreen();
                    break;
                case AS726x_YELLOW:
                    buf[i] = readYellow();
                    break;
                case AS726x_ORANGE:
                    buf[i] = readOrange();
                    break;
                case AS726x_RED:
                    buf[i] = readRed();
                    break;
                default:
                    break;
            }
        }
    }

    /**
     * Read the calibrated channels.  Defaults to AS726x_NUM_CHANNELS.
     *<p>
     * @param buf the buffer to read the data into
     */
    void readCalibratedValues(float[] buf) { readCalibratedValues(buf, AS726x_NUM_CHANNELS); }

    /**
     * Read the calibrated channels
     * <p>
     * @param buf the buffer to read the data into
     * @param num Number of channels to read. Defaults to AS726x_NUM_CHANNELS
     */
    public void readCalibratedValues(float[] buf, int num ){
        num = Math.min(num, AS726x_NUM_CHANNELS);
        for (int i = 0; i <= num; i++) {
            switch(i) {
                    case AS726x_VIOLET:
                    buf[i] = readCalibratedViolet();
                    break;
                case AS726x_BLUE:
                    buf[i] = readCalibratedBlue();
                    break;
                case AS726x_GREEN:
                    buf[i] = readCalibratedGreen();
                    break;
                case AS726x_YELLOW:
                    buf[i] = readCalibratedYellow();
                    break;
                case AS726x_ORANGE:
                    buf[i] = readCalibratedOrange();
                    break;
                case AS726x_RED:
                    buf[i] = readCalibratedRed();
                    break;
                default:
                    break;
            }
        }
    }

    /**
     * Read an individual calibrated spectral channel
     * <p>
     * @param channel the channel to read
     *<p>
     * @return the reading as a float
    */
    private float readCalibratedValue(byte channel)
    {
        // Read bytes in big endian format
        byte buffer[] = new byte[4];
        buffer[0] = virtualReadRegister(channel);
        buffer[1] = virtualReadRegister((byte)(channel+1));
        buffer[2] = virtualReadRegister((byte)(channel+2));
        buffer[3] = virtualReadRegister((byte)(channel+3));
        ByteBuffer readBuffer = ByteBuffer.wrap(buffer);
        return readBuffer.getFloat();
    }

    private byte virtualReadRegister(byte virtualReg) {
//        if (trace) { System.out.println("as7262::virtualRead("+ virtualReg +"): Enter"); }

        int startTime = startTimeout();
        byte status;
        // Wait for WRITE register to be empty
        while (true) {
            // Read slave I²C status to see if the write buffer is ready.
            status = read8(AS726X_SLAVE_STATUS_REG);

            if (checkTimeoutExpired()) {
                did_timeout = true;
                if (trace) { System.out.println("as7262::virtualReadRegister() timed out. Exiting."); }

                return 0;
            }

            if ((status & AS726X_SLAVE_TX_VALID) == 0) {
                // No inbound TX pending at slave. Okay to write now.
                break;
            }

            Timer.delay(polling_delay);
        }

        // Send the virtual register address (bit 7 should be zero to indicate we are reading a register).
        write8(AS726X_SLAVE_WRITE_REG, virtualReg);

        while (true) {
            // Read the slave I²C status to see if our read data is available.
            status = read8(AS726X_SLAVE_STATUS_REG);
            if ((status & AS726X_SLAVE_RX_VALID) != 0) {
                // Read data is ready.
                break;
            }

            if (checkTimeoutExpired()) {
                did_timeout = true;
                if (trace) { System.out.println("as7262::virtualReadRegister() timed out. Exiting."); }
                return 0;
            }

            Timer.delay(polling_delay);
       }
        // Read the data to complete the operation.
        byte value = read8(AS726X_SLAVE_READ_REG);
        // TODO:  Investigate why first read isn't correct value
        int retries  = 5;
        boolean isSame = false;
        while (!isSame && (retries-- > 0)) {
            byte newValue = read8(AS726X_SLAVE_READ_REG);
            isSame = value == newValue;
            if (!isSame) {
                value = newValue;
            }
        }

        // if (trace) { System.out.println("as7262::virtualRead: Exit"); }
        return value;
    }

    private void virtualWriteRegister(byte virtualReg, byte value) {
        // Wait for WRITE register to be empty
        int startTime = startTimeout();
        byte status;
        while (true) {
            // Read slave I²C status to see if the write buffer is ready.
            status = read8(AS726X_SLAVE_STATUS_REG);
            if ((status & AS726X_SLAVE_TX_VALID) == 0) {
                // No inbound TX pending at slave. Okay to write now.
                break;
            }

            if (checkTimeoutExpired()) {
                did_timeout = true;
                if (trace) { System.out.println("as7262::virtualWriteRegister() timed out.  Exiting."); }
                return;
            }

            Timer.delay(polling_delay);
        }

        // Send the virtual register address (setting bit 7 to indicate a pending write).
        write8(AS726X_SLAVE_WRITE_REG, (byte)(virtualReg | 0x80));

        while (true) {
            // Read the slave I²C status to see if the write buffer is ready.
            status = read8(AS726X_SLAVE_STATUS_REG);
            if ((status & AS726X_SLAVE_TX_VALID) == 0) {
                // No inbound TX pending at slave. Okay to write data now.
                break;
            }

            if (checkTimeoutExpired()) {
                did_timeout = true;
                if (trace) { System.out.println("as7262::virtualWriteRegister() timed out.  Exiting."); }
                return;
            }

            Timer.delay(polling_delay);
        }
        // Send the data to complete the operation.
        write8(AS726X_SLAVE_WRITE_REG, value);
    }

    // Writing one byte of data
    private boolean write8(byte  registerAddress, byte data) {
        boolean aborted  = mPort.write(registerAddress, data);
        if (aborted) {
            if (trace) { System.out.println("as7262::write8("+ registerAddress + ", " + data +") aborted"); }
        }

        return aborted;
    }

    // Read one byte of data
    private synchronized byte read8(int registerAddress) {
        byte[] buffer = new byte[BYTE_SIZE.SINGLE.value];
        boolean aborted = mPort.read(registerAddress, BYTE_SIZE.SINGLE.value, buffer);
        if (aborted) {
            if (trace) { System.out.println("as7262::read8("+ registerAddress + ") aborted"); }
        }

        return buffer[0];
    }
}