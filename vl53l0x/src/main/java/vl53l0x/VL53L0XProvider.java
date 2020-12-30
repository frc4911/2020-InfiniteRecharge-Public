package vl53l0x;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.I2C;

import java.nio.ByteBuffer;

import static vl53l0x.VL53L0XRegisters.*;

/**
 * Borrowed from FRC team 5361. * https://github.com/FRC-Team-Vern/VL53L0X_Example
 * TODO: Improve memory/buffer utilization
 */
public class VL53L0XProvider {

	private I2C mPort;
	private static final int DEFAULT_ADDRESS = 0x29;
	// The value of the address above the default address.
	private int deviceAddress;
	private byte stop_variable;
	private int measurement_timing_budget_us;
	private int timeout_start_ms;
	// The value is large enough
	private int io_timeout = 5000000;	// 10 seconds
	private boolean did_timeout;
	private boolean trace = false;

	private enum BYTE_SIZE {
		SINGLE(1),
		DOUBLE(2);

		public int value;

		BYTE_SIZE(int value) {
			this.value = value;
		}
	}

	public enum vcselPeriodType {
		VcselPeriodPreRange,
		VcselPeriodFinalRange
	}

	private class SequenceStepEnables {
		byte tcc, msrc, dss, pre_range, final_range;
	}

	private class SequenceStepTimeouts {
		short pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

		short msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
		int msrc_dss_tcc_us,    pre_range_us,    final_range_us;
	};

	private class BooleanCarrier {
		boolean value = false;
		BooleanCarrier(boolean inValue) {
			this.value = inValue;
		}
	}

	public VL53L0XProvider(I2C port) {
		mPort = port;
		this.did_timeout = false;
	}

	public void	enableLogging(boolean enable) {  trace = enable; }

	/**
	 * Initialize sensor using sequence based on VL53L0X_DataInit(),
	 * VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
	 *
	 * <p>This function does not perform reference SPAD calibration
	 * (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
	 * is performed by ST on the bare modules; it seems like that should work well
	 * enough unless a cover glass is added.
	 *
	 * <p>If io_2v8 is true, the sensor is configured for 2V8 mode.
	 *
	 * @return <code>true</code> if succeed; otherwise <code>false</code>
	 */
	public final boolean init(boolean io_2v8) {
		if (trace) { System.out.println("vl53l0x::init() Entered"); }
		// sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
		if (io_2v8) {
			mPort.write(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
					read(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV).get() | 0x01); // set bit 0
		}

		// "Set I2C standard mode"
		mPort.write(0x88, 0x00);

		mPort.write(0x80, 0x01);
		mPort.write(0xFF, 0x01);
		mPort.write(0x00, 0x00);
		stop_variable = read(0x91).get();
		mPort.write(0x00, 0x01);
		mPort.write(0xFF, 0x00);
		mPort.write(0x80, 0x00);

		// disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
		mPort.write(MSRC_CONFIG_CONTROL, read(MSRC_CONFIG_CONTROL).get() | 0x12);

		// set final range signal rate limit to 0.25 MCPS (million counts per second)
		setSignalRateLimit(0.25f);

		mPort.write(SYSTEM_SEQUENCE_CONFIG, 0xFF);

		byte[] spad_count = new byte[1];
		BooleanCarrier spad_type_is_aperture = new BooleanCarrier(false);
		if (!getSpadInfo(spad_count, spad_type_is_aperture)) {
			if (trace) { System.out.println("vl53l0x::init() Exit getSpadInfo() is false"); }
			return false;
		}

		// The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
		// the API, but the same data seems to be more easily readable from
		// GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
//		ByteBuffer ref_spad_map = ByteBuffer.allocateDirect(6);
		byte[] ref_spad_map = new byte[6];
		mPort.read(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6, ref_spad_map);

		mPort.write(0xFF, 0x01);
		mPort.write(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
		mPort.write(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
		mPort.write(0xFF, 0x00);
		mPort.write(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

		byte first_spad_to_enable = (byte) (spad_type_is_aperture.value ? 12 : 0); // 12 is the first aperture spad
		byte spads_enabled = 0;

		byte[] ref_spad_map_array = new byte[6];
		System.arraycopy(ref_spad_map,0, ref_spad_map_array, 0, 6);
//		(ref_spad_map.clone()ref_spad_map_array);
		for (byte i = 0; i < 48; ++i) {
			if (i < first_spad_to_enable || spads_enabled == spad_count[0]) {
				// This bit is lower than the first one that should be enabled, or
				// (reference_spad_count) bits have already been enabled, so zero this bit
				ref_spad_map_array[i / 8] &= ~(1 << (i % 8));
			}
			else if (((ref_spad_map_array[i / 8] >> (i % 8)) & 0x1) == 0x01) {
				spads_enabled++;
			}
		}

		writeBulk(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map_array, 6);

		// DefaultTuningSettings
		mPort.write(0xFF, 0x01);
		mPort.write(0x00, 0x00);

		mPort.write(0xFF, 0x00);
		mPort.write(0x09, 0x00);
		mPort.write(0x10, 0x00);
		mPort.write(0x11, 0x00);

		mPort.write(0x24, 0x01);
		mPort.write(0x25, 0xFF);
		mPort.write(0x75, 0x00);

		mPort.write(0xFF, 0x01);
		mPort.write(0x4E, 0x2C);
		mPort.write(0x48, 0x00);
		mPort.write(0x30, 0x20);

		mPort.write(0xFF, 0x00);
		mPort.write(0x30, 0x09);
		mPort.write(0x54, 0x00);
		mPort.write(0x31, 0x04);
		mPort.write(0x32, 0x03);
		mPort.write(0x40, 0x83);
		mPort.write(0x46, 0x25);
		mPort.write(0x60, 0x00);
		mPort.write(0x27, 0x00);
		mPort.write(0x50, 0x06);
		mPort.write(0x51, 0x00);
		mPort.write(0x52, 0x96);
		mPort.write(0x56, 0x08);
		mPort.write(0x57, 0x30);
		mPort.write(0x61, 0x00);
		mPort.write(0x62, 0x00);
		mPort.write(0x64, 0x00);
		mPort.write(0x65, 0x00);
		mPort.write(0x66, 0xA0);

		mPort.write(0xFF, 0x01);
		mPort.write(0x22, 0x32);
		mPort.write(0x47, 0x14);
		mPort.write(0x49, 0xFF);
		mPort.write(0x4A, 0x00);

		mPort.write(0xFF, 0x00);
		mPort.write(0x7A, 0x0A);
		mPort.write(0x7B, 0x00);
		mPort.write(0x78, 0x21);

		mPort.write(0xFF, 0x01);
		mPort.write(0x23, 0x34);
		mPort.write(0x42, 0x00);
		mPort.write(0x44, 0xFF);
		mPort.write(0x45, 0x26);
		mPort.write(0x46, 0x05);
		mPort.write(0x40, 0x40);
		mPort.write(0x0E, 0x06);
		mPort.write(0x20, 0x1A);
		mPort.write(0x43, 0x40);

		mPort.write(0xFF, 0x00);
		mPort.write(0x34, 0x03);
		mPort.write(0x35, 0x44);

		mPort.write(0xFF, 0x01);
		mPort.write(0x31, 0x04);
		mPort.write(0x4B, 0x09);
		mPort.write(0x4C, 0x05);
		mPort.write(0x4D, 0x04);

		mPort.write(0xFF, 0x00);
		mPort.write(0x44, 0x00);
		mPort.write(0x45, 0x20);
		mPort.write(0x47, 0x08);
		mPort.write(0x48, 0x28);
		mPort.write(0x67, 0x00);
		mPort.write(0x70, 0x04);
		mPort.write(0x71, 0x01);
		mPort.write(0x72, 0xFE);
		mPort.write(0x76, 0x00);
		mPort.write(0x77, 0x00);

		mPort.write(0xFF, 0x01);
		mPort.write(0x0D, 0x01);

		mPort.write(0xFF, 0x00);
		mPort.write(0x80, 0x01);
		mPort.write(0x01, 0xF8);

		mPort.write(0xFF, 0x01);
		mPort.write(0x8E, 0x01);
		mPort.write(0x00, 0x01);
		mPort.write(0xFF, 0x00);
		mPort.write(0x80, 0x00);


		mPort.write(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
		mPort.write(GPIO_HV_MUX_ACTIVE_HIGH, read(GPIO_HV_MUX_ACTIVE_HIGH).get() & ~0x10); // active low

		mPort.write(SYSTEM_INTERRUPT_CLEAR, 0x01);

		// -- VL53L0X_SetGpioConfig() end

		measurement_timing_budget_us = getMeasurementTimingBudget();

		// "Disable MSRC and TCC by default"
		// MSRC = Minimum Signal Rate Check
		// TCC = Target CentreCheck
		// -- VL53L0X_SetSequenceStepEnable() begin

		mPort.write(SYSTEM_SEQUENCE_CONFIG, 0xE8);

		// -- VL53L0X_SetSequenceStepEnable() end

		// "Recalculate timing budget"
		setMeasurementTimingBudget(measurement_timing_budget_us);

		// VL53L0X_StaticInit() end

		// VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

		// -- VL53L0X_perform_vhv_calibration() begin

		mPort.write(SYSTEM_SEQUENCE_CONFIG, 0x01);
		if (!performSingleRefCalibration((byte) 0x40)) {
			if (trace) { System.out.println("vl53l0x::init() Exited (performSingleRefCalibration 1 failed"); }
			return false;
		}

		// -- VL53L0X_perform_vhv_calibration() end

		// -- VL53L0X_perform_phase_calibration() begin

		mPort.write(SYSTEM_SEQUENCE_CONFIG, 0x02);
		if (!performSingleRefCalibration((byte) 0x00)) {
			if (trace) { System.out.println("vl53l0x::init() Exited (performSingleRefCalibration 2 failed"); }
			return false;
		}

		// -- VL53L0X_perform_phase_calibration() end

		// "restore the previous Sequence Config"
		mPort.write(SYSTEM_SEQUENCE_CONFIG, 0xE8);

		// VL53L0X_PerformRefCalibration() end

		// Set this last section for long range
//        setLongRange();

		if (trace) { System.out.println("vl53l0x::init() Exited"); }
		return true;
	}

	public void setTimeout(int timeout) {
		if (timeout < 0 ) {
			timeout = 0;
		}

		io_timeout = timeout;
	}

	public int getTimeout() {
		return io_timeout;
	}


	private void setLongRange() {
		setSignalRateLimit(0.1f);
		// increase laser pulse periods (defaults are 14 and 10 PCLKs)
		setVcselPulsePeriod(vcselPeriodType.VcselPeriodPreRange, (byte)18);
		setVcselPulsePeriod(vcselPeriodType.VcselPeriodFinalRange, (byte)14);
	}

	/**
	 * Performs a single-shot range measurement and returns the reading
	 * in millimeters
	 * <p> Based on VL53L0X_PerformSingleRangingMeasurement()
	 *
	 * @return range reading in millimeters
	 */
	public int readRangeSingleMillimeters() {
		mPort.write(0x80, 0x01);
		mPort.write(0xFF, 0x01);
		mPort.write(0x00, 0x00);
		mPort.write(0x91, stop_variable);
		mPort.write(0x00, 0x01);
		mPort.write(0xFF, 0x00);
		mPort.write(0x80, 0x00);

		mPort.write(SYSRANGE_START, 0x01);

		// "Wait until start bit has been cleared"
		int startTime = startTimeout();
		while ((read(SYSRANGE_START).get() & 0x01) == 0x01) {
			if (checkTimeoutExpired()){
				did_timeout = true;
				if (trace) { System.out.println("vl53l0x::readRangeSingleMillimeters() exited due to timed out"); }
				return 65535;
			}
		}

		if (trace) { System.out.println("readRangeSingleMillimeters took " + (startTimeout() - startTime)); }
		return readRangeContinuousMillimeters();
	}

	/**
	 * Returns a range reading in millimeters when continuous mode is active
	 * (readRangeSingleMillimeters() also calls this function after starting a
	 *  single-shot range measurement)
	 *
	 * @return range reading in millimeters
	 */
	public int readRangeContinuousMillimeters() {
		int startTime = startTimeout();
		while ((read(RESULT_INTERRUPT_STATUS).get() & 0x07) == 0) {
			if (checkTimeoutExpired()) {
				did_timeout = true;
				if (trace) { System.out.println("vl53l0x::readRangeContinuousMillimeters() exited due to timed out"); }
				return 65535;
			}
		}

		if (trace) { System.out.println("readRangeContinuousMillimeters took " + (startTimeout() - startTime)); }

		// assumptions: Linearity Corrective Gain is 1000 (default);
		// fractional ranging is not enabled
		// ByteBuffer byte_buffer_range = read16(RESULT_RANGE_STATUS.value + 10);

		short range = read16(RESULT_RANGE_STATUS + 10).getShort();
		mPort.write(SYSTEM_INTERRUPT_CLEAR, 0x01);
		//	byte_buffer_range.clear();
		return range;
	}

	private int getAddressFromDevice() {
		ByteBuffer deviceAddress = ByteBuffer.allocateDirect(BYTE_SIZE.SINGLE.value);
		mPort.read(I2C_SLAVE_DEVICE_ADDRESS, BYTE_SIZE.SINGLE.value, deviceAddress);
		return deviceAddress.get();
	}

	//ByteBuffer registerWithDataToSendBuffer = ByteBuffer.allocateDirect(128);`

	// Writing two bytes of data back-to-back is a special case of writeBulk
	private synchronized boolean write16(int registerAddress, short data) {
		//ByteBuffer registerWithDataToSendBuffer = ByteBuffer.allocateDirect(3);
		byte[] buffer = new byte[3];
		ByteBuffer registerWithDataToSendBuffer = ByteBuffer.wrap(buffer);
		registerWithDataToSendBuffer.put((byte) registerAddress);
		registerWithDataToSendBuffer.putShort(1, data);
		return mPort.writeBulk(registerWithDataToSendBuffer, 3);
	}

	private synchronized boolean write32(int registerAddress, int data) {
		//ByteBuffer registerWithDataToSendBuffer = ByteBuffer.allocateDirect(5);
		byte[] buffer = new byte[5];
		ByteBuffer registerWithDataToSendBuffer = ByteBuffer.wrap(buffer);
		registerWithDataToSendBuffer.put((byte) registerAddress);
		registerWithDataToSendBuffer.putInt(1, data);
		return mPort.writeBulk(registerWithDataToSendBuffer, 5);
	}

	private synchronized boolean writeBulk(int registerAddress, byte[] data, int size) {
		byte[] buffer = new byte[size + 1];
		ByteBuffer registerWithDataToSendBuffer = ByteBuffer.wrap(buffer);
		registerWithDataToSendBuffer.put((byte) registerAddress);
		for (int i=0; i < size; ++i) {
			registerWithDataToSendBuffer.put(i+1, data[i]);
		}
		return mPort.writeBulk(registerWithDataToSendBuffer, size + 1);
	}

	private int readInt(int registerAddress)  {
		byte[] buffer = new byte[2* BYTE_SIZE.DOUBLE.value];
		mPort.read(registerAddress, 2* BYTE_SIZE.SINGLE.value, buffer);
//		int value = (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
		ByteBuffer buffer1 = ByteBuffer.wrap(buffer);
		return buffer1.getInt();
	}

	private ByteBuffer read(int registerAddress) {
//		ByteBuffer bufferResults = ByteBuffer.allocateDirect(BYTE_SIZE.SINGLE.value);
		byte[] buffer = new byte[BYTE_SIZE.SINGLE.value];
		ByteBuffer bufferResults = ByteBuffer.wrap(buffer);
		mPort.read(registerAddress, BYTE_SIZE.SINGLE.value, buffer);
		return bufferResults;
	}

	// Reading two bytes of data back-to-back is a special, 2-byte case of read
	private ByteBuffer read16(int registerAddress) {
//		ByteBuffer bufferResults = ByteBuffer.allocateDirect(BYTE_SIZE.DOUBLE.value);
		byte[] buffer = new byte[BYTE_SIZE.DOUBLE.value];
		ByteBuffer bufferResults = ByteBuffer.wrap(buffer);
		mPort.read(registerAddress, BYTE_SIZE.DOUBLE.value, buffer);
		return bufferResults;
	}

	/**
	 * Set the return signal rate limit check value in units of MCPS (mega counts
	 * per second). "This represents the amplitude of the signal reflected from the
	 * target and detected by the device"; setting this limit presumably determines
	 * the minimum measurement necessary for the sensor to report a valid reading.
	 * Setting a lower limit increases the potential range of the sensor but also
	 * seems to increase the likelihood of getting an inaccurate reading because of
	 * unwanted reflections from objects other than the intended target.
	 * Defaults to 0.25 MCPS as initialized by the ST API and this library.
	 *
	 * @param limit_Mcps limit in MCPS (mega counts per second)
	 * @return true if succeeded; otherwise false
	 */
	private boolean setSignalRateLimit(float limit_Mcps) {
		if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }
		return !write16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (short) (limit_Mcps * (1<<7)));
	}

	/**
	 * Get reference SPAD (single photon avalanche diode) count and type
	 *
	 * based on VL53L0X_get_info_from_device(),
	 * but only gets reference SPAD count and type
	 *
	 * @param count byte array to fill in
	 * @param type_is_aperture a (@link BooleanCarrier) object used hold boolean register value
	 * @return true if succeeded; otherwise false
	 */
	private boolean getSpadInfo(byte[] count, BooleanCarrier type_is_aperture) {
		if (trace) { System.out.println("vl53l0x::getSpadInfo() Entered"); }
		byte tmp_byte = 0x00; // ByteBuffer.allocateDirect(BYTE_SIZE.SINGLE.value);

		mPort.write(0x80, 0x01);
		mPort.write(0xFF, 0x01);
		mPort.write(0x00, 0x00);

		mPort.write(0xFF, 0x06);
		mPort.write(0x83, read(0x83).get() | 0x04);
		mPort.write(0xFF, 0x07);
		mPort.write(0x81, 0x01);

		mPort.write(0x80, 0x01);

		mPort.write(0x94, 0x6b);
		mPort.write(0x83, 0x00);
		startTimeout();
		while (read(0x83).get() == 0x00) {
			if (checkTimeoutExpired()) {
				if (trace) { System.out.println("vl53l0x::getSpadInfo() Exit (timed out)"); }
				return false;
			}
		}
		mPort.write(0x83, 0x01);
		tmp_byte = read(0x92).get();

		count[0] = (byte) (tmp_byte & 0x7f);
//	  count.put(0, count_byte);
		boolean type_is_aperture_boolean = (((tmp_byte) & 0x01) == 0x01);
		type_is_aperture.value = type_is_aperture_boolean;

		mPort.write(0x81, 0x00);
		mPort.write(0xFF, 0x06);
		mPort.write(0x83, read( 0x83  & ~0x04).get());
		mPort.write(0xFF, 0x01);
		mPort.write(0x00, 0x01);

		mPort.write(0xFF, 0x00);
		mPort.write(0x80, 0x00);

		if (trace) { System.out.println("vl53l0x::getSpadInfo() Exit"); }
		return true;
	}

	/**
	 * Get the measurement timing budget in microseconds
	 *
	 * <p>Based on VL53L0X_get_measurement_timing_budget_micro_seconds()
	 *
	 * @return measurement timing budget in microseconds
	 */
	private int getMeasurementTimingBudget() {
		if (trace) { System.out.println("vl53l0x::getMeasurementTimingBudget() Enter"); }

		SequenceStepEnables enables = new SequenceStepEnables();
		SequenceStepTimeouts timeouts = new SequenceStepTimeouts();

		final short StartOverhead     = 1910; // note that this is different than the value in set_
		final short EndOverhead        = 960;
		final short MsrcOverhead       = 660;
		final short TccOverhead        = 590;
		final short DssOverhead        = 690;
		final short PreRangeOverhead   = 660;
		final short FinalRangeOverhead = 550;

		// "Start and end overhead times always present"
		int budget_us = StartOverhead + EndOverhead;

		getSequenceStepEnables(enables);
		getSequenceStepTimeouts(enables, timeouts);

		if (enables.tcc == 0x01) {
			budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
		}

		if (enables.dss == 0x01)  {
			budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
		}
		else if (enables.msrc == 0x01) {
			budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
		}

		if (enables.pre_range == 0x01) {
			budget_us += (timeouts.pre_range_us + PreRangeOverhead);
		}

		if (enables.final_range == 0x01) {
			budget_us += (timeouts.final_range_us + FinalRangeOverhead);
		}

//		measurement_timing_budget_us = budget_us; // store for internal reuse

		if (trace) { System.out.println("vl53l0x::getMeasurementTimingBudget() Exit"); }
		return budget_us;
	}

	/**
	 * Set the measurement timing budget in microseconds, which is the time allowed
	 * for one measurement; the ST API and this library take care of splitting the
	 * timing budget among the sub-steps in the ranging sequence. A longer timing
	 * budget allows for more accurate measurements. Increasing the budget by a
	 * factor of N decreases the range measurement standard deviation by a factor of
	 * sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
	 *
	 * <p>Based on VL53L0X_set_measurement_timing_budget_micro_seconds()
	 *
	 * @param budget_us measurement timing budget in microseconds
	 * @return true if succeeded; otherwise false
	 */
	private boolean setMeasurementTimingBudget(int budget_us) {
		if (trace) { System.out.println("vl53l0x::setMeasurementTimingBudget() Enter"); }

		SequenceStepEnables enables = new SequenceStepEnables();
		SequenceStepTimeouts timeouts = new SequenceStepTimeouts();

		final short StartOverhead      = 1320; // note that this is different than the value in get_
		final short EndOverhead        = 960;
		final short MsrcOverhead       = 660;
		final short TccOverhead        = 590;
		final short DssOverhead        = 690;
		final short PreRangeOverhead   = 660;
		final short FinalRangeOverhead = 550;

		final int MinTimingBudget = 20000;

		if (budget_us < MinTimingBudget) {
			if (trace) { System.out.println("vl53l0x::setMeasurementTimingBudget() Exit (budget_us[" + budget_us + "] < MinTimingBudget[" + MinTimingBudget + "])"); }
//			return false;
			budget_us = 33000;
		}

		int used_budget_us = StartOverhead + EndOverhead;

		getSequenceStepEnables(enables);
		getSequenceStepTimeouts(enables, timeouts);

		if (enables.tcc == 0x01) {
			used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
		}

		if (enables.dss == 0x01) {
			used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
		}
		else if (enables.msrc == 0x01) {
			used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
		}

		if (enables.pre_range == 0x01) {
			used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
		}

		if (enables.final_range == 0x01) {
			used_budget_us += FinalRangeOverhead;

			// "Note that the final range timeout is determined by the timing
			// budget and the sum of all other timeouts within the sequence.
			// If there is no room for the final range timeout, then an error
			// will be set. Otherwise the remaining time will be applied to
			// the final range."

			if (used_budget_us > budget_us) {
				// "Requested timeout too big."
				if (trace) { System.out.println("vl53l0x::setMeasurementTimingBudget() Exit (used_budget_us > budget_us)"); }
				return false;
			}

			int final_range_timeout_us = budget_us - used_budget_us;

			// set_sequence_step_timeout() begin
			// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

			// "For the final range timeout, the pre-range timeout
			//  must be added. To do this both final and pre-range
			//  timeouts must be expressed in macro periods MClks
			//  because they have different vcsel periods."
			short final_range_timeout_mclks = (short) timeoutMicrosecondsToMclks(final_range_timeout_us, (byte) timeouts.final_range_vcsel_period_pclks);

			if (enables.pre_range == 0x01) {
				final_range_timeout_mclks += timeouts.pre_range_mclks;
			}

			write16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_range_timeout_mclks));

			// set_sequence_step_timeout() end

//			measurement_timing_budget_us = budget_us; // store for internal reuse
		}

		if (trace) { System.out.println("vl53l0x::setMeasurementTimingBudget() Exit"); }
		return true;
	}

	/**
	 * Get sequence step enables
	 *
	 * <p>Based on VL53L0X_GetSequenceStepEnables()
	 * @param enables a SequenceStepEnables instance to be filled in
	 */
	private void getSequenceStepEnables(SequenceStepEnables enables) {
		if (trace) { System.out.println("vl53l0x::getSequenceStepEnables() Enter"); }

		byte sequence_config = read(SYSTEM_SEQUENCE_CONFIG).get();

		enables.tcc          = (byte) ((sequence_config >> 4) & 0x1);
		enables.dss          = (byte) ((sequence_config >> 3) & 0x1);
		enables.msrc         = (byte) ((sequence_config >> 2) & 0x1);
		enables.pre_range    = (byte) ((sequence_config >> 6) & 0x1);
		enables.final_range  = (byte) ((sequence_config >> 7) & 0x1);

		if (trace) { System.out.println("vl53l0x::getSequenceStepEnables() Exit"); }
	}

	/**
	 * Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
	 *
	 * <p>Based on VL53L0X_calc_timeout_mclks()
	 *
	 * @param timeout_period_us timeout period in microseconds
	 * @param vcsel_period_pclks VCSEL period in PCLKs
	 * @return timeout in MCLKs
	 */
	private int timeoutMicrosecondsToMclks(int timeout_period_us, byte vcsel_period_pclks)
	{
		int macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
		return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
	}

	/**
	 * Get sequence step timeouts
	 *
	 * <p>Based on VL53L0X_get_sequence_step_timeout(),
	 * <p>But gets all timeouts instead of just the requested one, and also stores
	 * intermediate values
	 *
	 * @param enables SequenceStepEnables instnace
	 * @param timeouts SequenceStepTimeouts instance
	 */

	ByteBuffer result = ByteBuffer.allocateDirect(2);
	private void getSequenceStepTimeouts(SequenceStepEnables enables, SequenceStepTimeouts timeouts) {
		if (trace) { System.out.println("vl53l0x::getSequenceStepTimeouts() Entered"); }

		timeouts.pre_range_vcsel_period_pclks = getVcselPulsePeriod(vcselPeriodType.VcselPeriodPreRange);

		timeouts.msrc_dss_tcc_mclks = (short) (read(MSRC_CONFIG_TIMEOUT_MACROP).get() + 1);
		timeouts.msrc_dss_tcc_us = timeoutMclksToMicroseconds(timeouts.msrc_dss_tcc_mclks, timeouts.pre_range_vcsel_period_pclks);

		//ByteBuffer result = ByteBuffer.allocateDirect(2);
		result.clear();
		mPort.read(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, 2, result);
		timeouts.pre_range_mclks = decodeTimeout(result.get());
		timeouts.pre_range_us = timeoutMclksToMicroseconds(timeouts.pre_range_mclks, timeouts.pre_range_vcsel_period_pclks);

		timeouts.final_range_vcsel_period_pclks = getVcselPulsePeriod(vcselPeriodType.VcselPeriodFinalRange);
		result.clear();
		mPort.read(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, 2, result);
		timeouts.final_range_mclks = decodeTimeout(result.get());

		if (enables.pre_range == 0x01) {
			timeouts.final_range_mclks -= timeouts.pre_range_mclks;
		}

		timeouts.final_range_us = timeoutMclksToMicroseconds(timeouts.final_range_mclks, timeouts.final_range_vcsel_period_pclks);
		if (trace) { System.out.println("vl53l0x::getSequenceStepTimeouts() Exit"); }
	}

	/**
	 * Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
	 *
	 * <p>Based on VL53L0X_calc_timeout_us()
	 *
	 * @param timeout_period_mclks timeout period in MCLKs
	 * @param vcsel_period_pclks vcsel period in PCLKs
	 * @return timeout in microseconds
	 */
	private int timeoutMclksToMicroseconds(short timeout_period_mclks, short vcsel_period_pclks) {
		int macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
		return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
	}

	private int calcMacroPeriod(int vcsel_period_pclks) {
		return ((((int)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000);
	}

	/**
	 * Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
	 * from register value.
	 *
	 * Based on VL53L0X_decode_vcsel_period()
	 *
	 * @param reg_val value from a register
	 * @return VCSEL period in PCLKs
	 */
	private int decodeVcselPeriod(int reg_val) {
		return (((reg_val) + 1) << 1);
	}

	/**
	 * Encode VCSEL (vertical cavity surface emitting laser) pulse period register
	 * value from period in PCLKs
	 *
	 * <p>Based on VL53L0X_encode_vcsel_period()
	 *
	 * @param period_pclks period in PCLKs
	 * @return encoded period
	 */
	private byte encodeVcselPeriod(byte period_pclks) {
		return (byte)(((period_pclks) >> 1) - 1);
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
	 * Get the VCSEL pulse period in PCLKs for the given period type.
	 * based on VL53L0X_get_vcsel_pulse_period()
	 *
	 * @param type VCSEL period type
	 * @return pulse period in PCLKs
	 */
	private byte getVcselPulsePeriod(vcselPeriodType type) {
		if (type == vcselPeriodType.VcselPeriodPreRange) {
			return (byte) decodeVcselPeriod(read(PRE_RANGE_CONFIG_VCSEL_PERIOD).get());
		} else if (type == vcselPeriodType.VcselPeriodFinalRange) {
			return (byte) decodeVcselPeriod(read(FINAL_RANGE_CONFIG_VCSEL_PERIOD).get());
		} else {
			return (byte) 255;
		}
	}

	/**
	 * Start continuous ranging measurements. If period_ms (optional) is 0 or not
	 * given, continuous back-to-back mode is used (the sensor takes measurements as
	 * often as possible); otherwise, continuous timed mode is used, with the given
	 * inter-measurement period in milliseconds determining how often the sensor
	 * takes a measurement.
	 *
	 * <p>Based on VL53L0X_StartMeasurement()
	 *
	 * @param period_ms measurement period in milliseconds.
	 */
	public void startContinuous(int period_ms) {
		mPort.write(0x80, 0x01);
		mPort.write(0xFF, 0x01);
		mPort.write(0x00, 0x00);
		mPort.write(0x91, stop_variable);
		mPort.write(0x00, 0x01);
		mPort.write(0xFF, 0x00);
		mPort.write(0x80, 0x00);

		if (period_ms != 0) {
			// continuous timed mode
			// VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

			short osc_calibrate_val = read16(OSC_CALIBRATE_VAL).getShort();

			if (osc_calibrate_val != 0) {
				period_ms *= osc_calibrate_val;
			}

			write32(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
			// VL53L0X_SetInterMeasurementPeriodMilliSeconds() end
			mPort.write(SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
		} else {
			// continuous back-to-back mode
			mPort.write(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
		}
	}

	/**
	 * Stop continuous measurements
	 *
	 * <p>Based on VL53L0X_StopMeasurement()
	 */
	public void stopContinuous(){
		mPort.write(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

		mPort.write(0xFF, 0x01);
		mPort.write(0x00, 0x00);
		mPort.write(0x91, 0x00);
		mPort.write(0x00, 0x01);
		mPort.write(0xFF, 0x00);
	}

	/**
	 * Encode sequence step timeout register value from timeout in MCLKs
	 * <p>
	 * @param timeout_mclks timout in MCLKs
	 * @return encoded timeout value
	 */
	private short encodeTimeout(short timeout_mclks)
	{
		// format: "(LSByte * 2^MSByte) + 1"
		int ls_byte = 0;
		short ms_byte = 0;

		if (timeout_mclks > 0) {
			ls_byte = timeout_mclks - 1;

			while ((ls_byte & 0xFFFFFF00) > 0) {
				ls_byte >>= 1;
				ms_byte++;
			}

			return (short) ((ms_byte << 8) | (ls_byte & 0xFF));
		}
		else {
			return 0;
		}
	}

	/**
	 * Decode sequence step timeout in MCLKs from register value
	 *
	 * <p>Based on VL53L0X_decode_timeout()
	 * <p>
	 * @param reg_val 16-bit register
	 * @return decoded timeout value in MCLKs
	 */
	private short decodeTimeout(short reg_val)
	{
		// format: "(LSByte * 2^MSByte) + 1"
		return (short)((short)((reg_val & 0x00FF) <<
				(short)((reg_val & 0xFF00) >> 8)) + 1);
	}


	/**
	 * Performs and single reference calibration
	 *
	 * <p>Based on VL53L0X_perform_single_ref_calibration()
	 * <p>
	 * @param vhv_init_byte
	 * @return <code>true</code> if succeed; otherwise <code>false</code>
	 */
	private boolean performSingleRefCalibration(byte vhv_init_byte) {
		if (trace) { System.out.println("vl53l0x::performSingleRefCalibration() Enter"); }
		mPort.write(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

		startTimeout();
		if (trace) { System.out.println("vl53l0x::performSingleRefCalibration() (timeout = " + getTimeout()+ ")"); }
		while ((read(RESULT_INTERRUPT_STATUS).get() & 0x07) == 0) {
			if (checkTimeoutExpired()) {
				if (trace) { System.out.println("vl53l0x::performSingleRefCalibration() Exit (timeout expired)"); }
				return false;
			}
		}

		mPort.write(SYSTEM_INTERRUPT_CLEAR, 0x01);
		mPort.write(SYSRANGE_START, 0x00);

		if (trace) { System.out.println("vl53l0x::performSingleRefCalibration() Exit"); }
		return true;
	}

	/**
	 * Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
	 * given period type (pre-range or final range) to the given value in PCLKs.
	 * Longer periods seem to increase the potential range of the sensor.
	 * Valid values are (even numbers only):
	 * pre:  12 to 18 (initialized default: 14)
	 * final: 8 to 14 (initialized default: 10)
	 * <p>Based on VL53L0X_set_vcsel_pulse_period()
	 * <p>
	 * @param type VSCEL type
	 * @param period_pclks period in PCLKs
	 * @return <code>true</code> if succeed; otherwise <code>false</code>
	 */
	private boolean setVcselPulsePeriod(vcselPeriodType type, byte period_pclks) {
		if (trace) { System.out.println("vl53l0x::setVcselPulsePeriod() Enter"); }
		byte vcsel_period_reg = encodeVcselPeriod(period_pclks);

		SequenceStepEnables enables = new SequenceStepEnables();
		SequenceStepTimeouts timeouts = new SequenceStepTimeouts();

		getSequenceStepEnables(enables);
		getSequenceStepTimeouts(enables, timeouts);

		// "Apply specific settings for the requested clock period"
		// "Re-calculate and apply timeouts, in macro periods"

		// "When the VCSEL period for the pre or final range is changed,
		// the corresponding timeout must be read from the device using
		// the current VCSEL period, then the new VCSEL period can be
		// applied. The timeout then must be written back to the device
		// using the new VCSEL period.
		//
		// For the MSRC timeout, the same applies - this timeout being
		// dependant on the pre-range vcsel period."

		if (type == vcselPeriodType.VcselPeriodPreRange) {
			// "Set phase check limits"
			switch (period_pclks) {
				case 12:
					mPort.write(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
					break;

				case 14:
					mPort.write(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
					break;

				case 16:
					mPort.write(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
					break;

				case 18:
					mPort.write(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
					break;

				default:
					// invalid period
					return false;
			}

			mPort.write(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

			// apply new VCSEL period
			mPort.write(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

			// update timeouts

			// set_sequence_step_timeout() begin
			// (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

			int new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

			write16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout((short)new_pre_range_timeout_mclks));

			// set_sequence_step_timeout() end

			// set_sequence_step_timeout() begin
			// (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

			int new_msrc_timeout_mclks =timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

			mPort.write(MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

			// set_sequence_step_timeout() end
		} else if (type == vcselPeriodType.VcselPeriodFinalRange) {
			switch (period_pclks) {
				case 8:
					mPort.write(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
					mPort.write(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
					mPort.write(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
					mPort.write(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
					mPort.write(0xFF, 0x01);
					mPort.write(ALGO_PHASECAL_LIM, 0x30);
					mPort.write(0xFF, 0x00);
					break;

				case 10:
					mPort.write(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
					mPort.write(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
					mPort.write(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
					mPort.write(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
					mPort.write(0xFF, 0x01);
					mPort.write(ALGO_PHASECAL_LIM, 0x20);
					mPort.write(0xFF, 0x00);
					break;

				case 12:
					mPort.write(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
					mPort.write(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
					mPort.write(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
					mPort.write(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
					mPort.write(0xFF, 0x01);
					mPort.write(ALGO_PHASECAL_LIM, 0x20);
					mPort.write(0xFF, 0x00);
					break;

				case 14:
					mPort.write(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
					mPort.write(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
					mPort.write(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
					mPort.write(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
					mPort.write(0xFF, 0x01);
					mPort.write(ALGO_PHASECAL_LIM, 0x20);
					mPort.write(0xFF, 0x00);
					break;

				default:
					// invalid period
					if (trace) { System.out.println("vl53l0x::setVcselPulsePeriod() Exit (invalid period)"); };
					return false;
			}

			// apply new VCSEL period
			mPort.write(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

			// update timeouts

			// set_sequence_step_timeout() begin
			// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

			// "For the final range timeout, the pre-range timeout
			//  must be added. To do this both final and pre-range
			//  timeouts must be expressed in macro periods MClks
			//  because they have different vcsel periods."

			int new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

			if (enables.pre_range == 0x01) {
				new_final_range_timeout_mclks += timeouts.pre_range_mclks;
			}

			write16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout((short)new_final_range_timeout_mclks));

			// set_sequence_step_timeout end
		} else {
			// invalid type
			if (trace) { System.out.println("vl53l0x::setVcselPulsePeriod() Exit (invalid type)"); }
			return false;
		}

		// "Finally, the timing budget must be re-applied"

		setMeasurementTimingBudget(measurement_timing_budget_us);

		// "Perform the phase calibration. This is needed after changing on vcsel period."
		// VL53L0X_perform_phase_calibration() begin

//		ByteBuffer sequence_config = read(SYSTEM_SEQUENCE_CONFIG);
		byte[] buffer = new byte[1];
		mPort.read(SYSTEM_SEQUENCE_CONFIG, 1, buffer);
		byte sequence_config = buffer[0];

		mPort.write(SYSTEM_SEQUENCE_CONFIG, 0x02);
		performSingleRefCalibration((byte)0x0);
		mPort.write(SYSTEM_SEQUENCE_CONFIG, sequence_config);

		// VL53L0X_perform_phase_calibration() end
		if (trace) { System.out.println("vl53l0x::setVcselPulsePeriod() Exit"); }
		return true;
	}
}