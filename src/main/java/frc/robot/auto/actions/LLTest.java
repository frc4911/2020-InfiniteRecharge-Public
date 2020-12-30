package frc.robot.auto.actions;

import java.util.Optional;

import com.team254.lib.autos.actions.Action;
import com.team254.lib.vision.AimingParameters;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.Limelights.ShootwardsLimelight;
import frc.robot.subsystems.Limelights.Limelight.WantedState;


public class LLTest implements Action {
    private ShootwardsLimelight shootwardsLimelight = ShootwardsLimelight.getInstance("LLTest");
    private RobotState RS = RobotState.getInstance("LLTest");
    private Optional<AimingParameters> AP;
    // private List<AimingParameters> PC;
    // private List<AimingParameters> powerCells;

    @Override
    public void start() {
        System.out.println("limelight test");
        //LLManager.setShootwardsActive(true);
        shootwardsLimelight.setWantedState(WantedState.TARGET);
    }

    @Override
    public void update() {
        AP = RS.getOuterGoalParameters();
        // PC = RS.getPowerCell();

        if (AP.isPresent()) {
            SmartDashboard.putString("Robot to outer", AP.get().getRobotToGoal().toString());
            SmartDashboard.putNumber("stability outer", AP.get().getStability());
        } else {
            SmartDashboard.putNumber("stability outer", 0);
        }
        
        // for (int i = 0; i < PC.size(); i++) {
        //     SmartDashboard.putString("powercell #" + i, PC.get(i).getRobotToGoal().getTranslation().toString());
        // }
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
	public void done() {
		// TODO Auto-generated method stub
		
    }
}
    