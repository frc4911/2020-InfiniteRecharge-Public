package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.CollectAction;
import frc.robot.auto.actions.SetTrajectoryAction;
import frc.robot.auto.actions.ShootAction;
import frc.robot.auto.actions.WaitToFinishPathAction;

public class TrenchMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ShootAction(2.5));
        runAction(new SetTrajectoryAction(trajectories.startToTrenchPath.get(true), 0.0, 1.0));
        runAction(new CollectAction(true));
        runAction(new WaitToFinishPathAction());
        runAction(new SetTrajectoryAction(trajectories.trenchToStartPath.get(true), 0.0, 1.0));
        runAction(new CollectAction(false));
        runAction(new WaitToFinishPathAction());
        runAction(new ShootAction(2.5));
    }

}