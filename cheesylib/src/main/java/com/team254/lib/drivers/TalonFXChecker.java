package com.team254.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.lib.subsystems.Subsystem;

import java.util.ArrayList;

public class TalonFXChecker extends MotorChecker<TalonFX> {
    private static class StoredTalonFXConfiguration {
        public ControlMode mMode;
        public double mSetValue;
    }

    protected ArrayList<TalonFXChecker.StoredTalonFXConfiguration> mStoredConfigurations = new ArrayList<>();

    public static boolean checkMotors(Subsystem subsystem,
                                      ArrayList<MotorConfig<TalonFX>> motorsToCheck,
                                      CheckerConfig checkerConfig) {
        TalonFXChecker checker = new TalonFXChecker();
        return checker.checkMotorsImpl(subsystem, motorsToCheck, checkerConfig);
    }

    @Override
    protected void storeConfiguration() {
        // record previous configuration for all talons
        for (MotorConfig<TalonFX> config : mMotorsToCheck) {
            LazyTalonFX talon = (LazyTalonFX) config.mMotor;

            TalonFXChecker.StoredTalonFXConfiguration configuration = new TalonFXChecker.StoredTalonFXConfiguration();
            configuration.mMode = talon.getControlMode();
            configuration.mSetValue = talon.getLastSet();

            mStoredConfigurations.add(configuration);
        }
    }

    @Override
    protected void restoreConfiguration() {
        for (int i = 0; i < mMotorsToCheck.size(); ++i) {
            mMotorsToCheck.get(i).mMotor.set(mStoredConfigurations.get(i).mMode,
                    mStoredConfigurations.get(i).mSetValue);
        }
    }

    @Override
    protected void setMotorOutput(TalonFX motor, double output) {
        motor.set(ControlMode.PercentOutput, output);
    }

    /**
     * TalonFX is a brushless motor.
     * <p/>
     * @param motor a TalonFX instance
     * @return the supply current
     */
    @Override
    protected double getMotorCurrent(TalonFX motor) {
        return motor.getSupplyCurrent();
    }
}