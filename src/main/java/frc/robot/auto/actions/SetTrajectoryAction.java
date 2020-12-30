package frc.robot.auto.actions;

import com.team254.lib.autos.actions.RunOnceAction;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import frc.robot.subsystems.Swerve;

public class SetTrajectoryAction extends RunOnceAction {
	Trajectory<TimedState<Pose2dWithCurvature>> trajectory;
    double goalHeading;
    double rotationScalar;
	Swerve swerve;
	
	public SetTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double goalHeading, double rotationScalar){
		this.trajectory = trajectory;
        this.goalHeading = goalHeading;
        this.rotationScalar = rotationScalar;
		swerve = Swerve.getInstance("SetTrajectoryAction");
	}
	
	@Override
	public synchronized void runOnce(){
		swerve.setTrajectory(trajectory, goalHeading, rotationScalar);
	}
}