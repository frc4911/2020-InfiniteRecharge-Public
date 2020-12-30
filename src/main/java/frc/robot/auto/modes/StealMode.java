package frc.robot.auto.modes;

import com.team254.lib.autos.actions.WaitAction;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.CollectAction;
import frc.robot.auto.actions.SetTrajectoryAction;
import frc.robot.auto.actions.ShootAction;
import frc.robot.auto.actions.WaitToFinishPathAction;

public class StealMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SetTrajectoryAction(trajectories.startToStealPath.get(true), 0.0, 1.0));
        runAction(new CollectAction(true));
        runAction(new WaitToFinishPathAction());
        runAction(new WaitAction(1.0));
        runAction(new SetTrajectoryAction(trajectories.stealToGoalPath.get(true), 15.0, 1.0)); // 0.0
        runAction(new WaitAction(2.0));
        runAction(new CollectAction(false));
        runAction(new WaitToFinishPathAction());
        runAction(new ShootAction(3.0));

        // Still Testing
        runAction(new SetTrajectoryAction(trajectories.goalToRendezvousPath.get(true), 15.0, 1.0));
        runAction(new CollectAction(true));
        runAction(new WaitToFinishPathAction());
        runAction(new SetTrajectoryAction(trajectories.threeBallsPath.get(true), 15.0, 1.0));
        runAction(new WaitToFinishPathAction());
        // runAction(new SetTrajectoryAction(trajectories.threeBallsToGoalPath.get(true), 15.0, 1.0));
        // runAction(new WaitToFinishPathAction());
        runAction(new CollectAction(false));
        runAction(new ShootAction(3.0));
    }

}