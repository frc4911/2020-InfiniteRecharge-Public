package vl53l0x;

public class VL53L0XRegisters {
    public static final byte SYSRANGE_START                              = 0x00;

    public static final byte SYSTEM_THRESH_HIGH                          = 0x0C;
    public static final byte SYSTEM_THRESH_LOW                           = 0x0E;

    public static final byte SYSTEM_SEQUENCE_CONFIG                      = 0x01;
    public static final byte SYSTEM_RANGE_CONFIG                         = 0x09;
    public static final byte SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04;

    public static final byte SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A;

    public static final short GPIO_HV_MUX_ACTIVE_HIGH                    = 0x84;

    public static final byte SYSTEM_INTERRUPT_CLEAR                      = 0x0B;

    public static final byte RESULT_INTERRUPT_STATUS                     = 0x13;
    public static final byte RESULT_RANGE_STATUS                         = 0x14;

    public static final short RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN      = 0xBC;
    public static final short RESULT_CORE_RANGING_TOTAL_EVENTS_RTN       = 0xC0;
    public static final short RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF      = 0xD0;
    public static final short RESULT_CORE_RANGING_TOTAL_EVENTS_REF       = 0xD4;
    public static final short RESULT_PEAK_SIGNAL_RATE_REF                = 0xB6;

    public static final byte ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28;

    public static final short I2C_SLAVE_DEVICE_ADDRESS                   = 0x8A;

    public static final byte MSRC_CONFIG_CONTROL                         = 0x60;

    public static final byte PRE_RANGE_CONFIG_MIN_SNR                    = 0x27;
    public static final byte PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56;
    public static final byte PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57;
    public static final byte PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64;

    public static final byte FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67;
    public static final byte FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47;
    public static final byte FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48;
    public static final byte FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44;

    public static final byte PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61;
    public static final byte PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62;

    public static final byte PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50;
    public static final byte PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51;
    public static final byte PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52;

    public static final short SYSTEM_HISTOGRAM_BIN                        = 0x81;
    public static final byte HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33;
    public static final byte HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55;

    public static final byte FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70;
    public static final byte FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71;
    public static final byte FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72;
    public static final byte CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20;

    public static final byte MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46;

    public static final short SOFT_RESET_GO2_SOFT_RESET_N                = 0xBF;
    public static final short IDENTIFICATION_MODEL_ID                    = 0xC0;
    public static final short IDENTIFICATION_REVISION_ID                 = 0xC2;

    public static final short OSC_CALIBRATE_VAL                          = 0xF8;

    public static final byte GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32;
    public static final short GLOBAL_CONFIG_SPAD_ENABLES_REF_0           = 0xB0;
    public static final short GLOBAL_CONFIG_SPAD_ENABLES_REF_1           = 0xB1;
    public static final short GLOBAL_CONFIG_SPAD_ENABLES_REF_2           = 0xB2;
    public static final short GLOBAL_CONFIG_SPAD_ENABLES_REF_3           = 0xB3;
    public static final short GLOBAL_CONFIG_SPAD_ENABLES_REF_4           = 0xB4;
    public static final short GLOBAL_CONFIG_SPAD_ENABLES_REF_5           = 0xB5;

    public static final short GLOBAL_CONFIG_REF_EN_START_SELECT          = 0xB6;
    public static final byte DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E;
    public static final byte DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F;
    public static final short POWER_MANAGEMENT_GO1_POWER_FORCE           = 0x80;

    public static final short VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV          = 0x89;

    public static final byte ALGO_PHASECAL_LIM                           = 0x30;
    public static final byte ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30;
}