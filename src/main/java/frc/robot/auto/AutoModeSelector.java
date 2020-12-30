package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.modes.DoNothingMode;
import frc.robot.auto.modes.RendezvousMode;
import frc.robot.auto.modes.ShootMode;
import frc.robot.auto.modes.StealMode;
import frc.robot.auto.modes.TenBallMode;
import frc.robot.auto.modes.TestMode;
import frc.robot.auto.modes.TrenchMode;

public class AutoModeSelector {

    public enum AutoModeChoice {
        TEN_BALL,
        STEAL,
        RENDEZVOUS,
        TRENCH,
        SHOOT,
        TESTMODE,
        NONE
    }

    private SendableChooser<AutoModeChoice> mAutoModeChooser;

    public AutoModeSelector() {
        mAutoModeChooser = new SendableChooser<AutoModeChoice>();
        mAutoModeChooser.addOption("Ten Ball Mode", AutoModeChoice.TEN_BALL);
        mAutoModeChooser.addOption("Steal Mode", AutoModeChoice.STEAL);
        mAutoModeChooser.addOption("Rendezvous Mode", AutoModeChoice.RENDEZVOUS);
        mAutoModeChooser.addOption("Trench Mode", AutoModeChoice.TRENCH);
        mAutoModeChooser.addOption("Shoot Mode", AutoModeChoice.SHOOT);
        mAutoModeChooser.addOption("Test Mode", AutoModeChoice.TESTMODE);
        mAutoModeChooser.setDefaultOption("None", AutoModeChoice.NONE);
        SmartDashboard.putData("Auto Mode", mAutoModeChooser);
    }

    public AutoModeBase getSelectedAutoMode() {
        AutoModeChoice choice = mAutoModeChooser.getSelected();
        SmartDashboard.putString("Selected Auto Mode", choice.toString());
        switch (choice) {
            case TEN_BALL:
                return new TenBallMode();
            case STEAL:
                return new StealMode();
            case RENDEZVOUS:
                return new RendezvousMode();
            case TRENCH:
                return new TrenchMode();
            case SHOOT:
                return new ShootMode();
            case TESTMODE:
                return new TestMode();
            default:
                return new DoNothingMode();
        }
    }

}