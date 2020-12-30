package frc.robot.auto.modes;

import com.team254.lib.autos.actions.WaitAction;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.ManualShootAction;
import frc.robot.auto.actions.SetTrajectoryAction;
import frc.robot.auto.actions.WaitToFinishPathAction;

public class ShootMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitAction(2.0));
        // runAction(new ShootAction(3.0));
        runAction(new ManualShootAction(3.0));
        runAction(new SetTrajectoryAction(trajectories.backAwayFromLinePath.get(true), 0.0, 1.0));
        runAction(new WaitToFinishPathAction());
    }

}