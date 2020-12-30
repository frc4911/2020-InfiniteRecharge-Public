package as7262;

public class AS7262Registers {
/**************************************************************************/
//    I2C ADDRESS/BITS
/**************************************************************************/
    public static final int AS726x_ADDRESS= 0x49;  // default I2C address

/**************************************************************************/
//  virtual registers
/**************************************************************************/
    public static final byte AS726X_HW_VERSION     = 0x00;
    public static final byte AS726X_FW_VERSION     = 0x02;
    public static final byte AS726X_CONTROL_SETUP  = 0x04;
    public static final byte AS726X_INT_T          = 0x05;
    public static final byte AS726X_DEVICE_TEMP    = 0x06;
    public static final byte AS726X_LED_CONTROL    = 0x07;

    //for reading sensor data
    public static final byte AS7262_V_HIGH         = 0x08;
    public static final byte AS7262_V_LOW          = 0x09;
    public static final byte AS7262_B_HIGH         = 0x0A;
    public static final byte AS7262_B_LOW          = 0x0B;
    public static final byte AS7262_G_HIGH         = 0x0C;
    public static final byte AS7262_G_LOW          = 0x0D;
    public static final byte AS7262_Y_HIGH         = 0x0E;
    public static final byte AS7262_Y_LOW          = 0x0F;
    public static final byte AS7262_O_HIGH         = 0x10;
    public static final byte AS7262_O_LOW          = 0x11;
    public static final byte AS7262_R_HIGH         = 0x12;
    public static final byte AS7262_R_LOW          = 0x13;

    public static final byte AS7262_V_CAL          = 0x14;
    public static final byte AS7262_B_CAL          = 0x18;
    public static final byte AS7262_G_CAL          = 0x1C;
    public static final byte AS7262_Y_CAL          = 0x20;
    public static final byte AS7262_O_CAL          = 0x24;
    public static final byte AS7262_R_CAL          = 0x28;

/**************************************************************************/
//  hardware registers
/**************************************************************************/
    public static final byte AS726X_SLAVE_STATUS_REG = 0x00;
    public static final byte AS726X_SLAVE_WRITE_REG  = 0x01;
    public static final byte AS726X_SLAVE_READ_REG   = 0x02;
    public static final byte AS726X_SLAVE_TX_VALID   = 0x02;
    public static final byte AS726X_SLAVE_RX_VALID   = 0x01;

/**************************************************************************/
// color registers
/**************************************************************************/
    public static final byte AS7262_VIOLET = 0x08;
    public static final byte AS7262_BLUE   = 0x0A;
    public static final byte AS7262_GREEN  = 0x0C;
    public static final byte AS7262_YELLOW = 0x0E;
    public static final byte AS7262_ORANGE = 0x10;
    public static final byte AS7262_RED    = 0x12;
    public static final byte AS7262_VIOLET_CALIBRATED = 0x14;
    public static final byte AS7262_BLUE_CALIBRATED   = 0x18;
    public static final byte AS7262_GREEN_CALIBRATED  = 0x1C;
    public static final byte AS7262_YELLOW_CALIBRATED = 0x20;
    public static final byte AS7262_ORANGE_CALIBRATED = 0x24;
    public static final byte AS7262_RED_CALIBRATED    = 0x28;

/**************************************************************************/
// Measurement  modes. Default is Mode 2
/**************************************************************************/
public enum MeasurementMode {
        MODE_0(0b00),
        MODE_1(0b01),
        MODE_2(0b10), //default
        ONE_SHOT(0b11);

        private byte value;

        private MeasurementMode(int value) {
            this.value = (byte) value;
        }

        public byte getValue() {
            return value;
        }
    }

/**************************************************************************/
// gain settings. Default is 1x gain
/**************************************************************************/

    public enum ChannelGain {
        GAIN_1X(0b00), // default
        GAIN_3X(0b01),
        GAIN_16X(0b10),
        GAIN_64X(0b11);

        private byte value;

        private ChannelGain(int value) {
            this.value = (byte)value;
        }

        public byte getValue() {
            return value;
        }
    }

/**************************************************************************/
// Indicator LED current limit settings. Default is 1mA
/**************************************************************************/

    public enum IndicatorLEDCurrentLimit {
        LIMIT_1MA(0b00), // default
        LIMIT_2MA(0b01),
        LIMIT_4MA(0b10),
        LIMIT_8MA(0b11);

        private byte value;

        private IndicatorLEDCurrentLimit(int value) {
            this.value = (byte) value;
        }

        public byte getValue() {
            return value;
        }
    }

/**************************************************************************/
//    Driver LED current limit settings. Default is 12.5 mA
/**************************************************************************/

    public enum DriverLEDCurrentLimit {
        LIMIT_12MA5(0b00), // default
        LIMIT_25MA(0b01),
        LIMIT_50MA(0b10),
        LIMIT_100MA(0b11);

        private byte value;

        private DriverLEDCurrentLimit(int value) {
            this.value = (byte) value;
        }

        public byte getValue() {
            return value;
        }
    }

/**************************************************************************/

    public static final double AS726x_INTEGRATION_TIME_MULT = 2.8; // multiplier for integration time

    public static final int AS726x_NUM_CHANNELS = 6; // number of sensor channels

/**************************************************************************/
// Color definitions used by the library
/**************************************************************************/
    public static final int AS726x_VIOLET = 0;
    public static final int AS726x_BLUE   = 1;
    public static final int AS726x_GREEN  = 2;
    public static final int AS726x_YELLOW = 3;
    public static final int AS726x_ORANGE = 4;
    public static final int AS726x_RED    = 5;
}