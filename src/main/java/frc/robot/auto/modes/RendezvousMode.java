package frc.robot.auto.modes;

import com.team254.lib.autos.actions.WaitAction;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.CollectAction;
import frc.robot.auto.actions.SetTrajectoryAction;
import frc.robot.auto.actions.ShootAction;
import frc.robot.auto.actions.WaitToFinishPathAction;

public class RendezvousMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        // runAction(new ShootAction(5.0));
        // runAction(new SetTrajectoryAction(trajectories.startToThreePowerCells.get(true), -30.0, 0.5));
        // runAction(new WaitToFinishPathAction());
        // runAction(new WaitAction(1.0));
        // runAction(new SetTrajectoryAction(trajectories.collectThreePowerCells.get(true), -30.0, 0.5));
        // runAction(new WaitToFinishPathAction());
        // runAction(new WaitAction(1.0));
        // runAction(new SetTrajectoryAction(trajectories.moveToTwoPowerCells.get(true), -30.0, 0.5));
        // runAction(new WaitToFinishPathAction());
        // runAction(new SetTrajectoryAction(trajectories.moveToTwoPowerCells2.get(true), 60.0, 0.5));

        runAction(new ShootAction(3.0));
        runAction(new SetTrajectoryAction(trajectories.startToThreePowerCells.get(true), -30.0, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new CollectAction(true));
        runAction(new SetTrajectoryAction(trajectories.startToThreePowerCells2.get(true), -30.0, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new WaitAction(1.0));
        runAction(new CollectAction(false));
        // runAction(new SetTrajectoryAction(trajectories.collectThreePowerCells.get(true), -30.0, 0.5));
        
    }

}