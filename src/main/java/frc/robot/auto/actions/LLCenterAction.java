package frc.robot.auto.actions;

import com.team254.lib.autos.actions.RunOnceAction;

import frc.robot.loops.LimelightProcessor;
import frc.robot.subsystems.Swerve;

public class LLCenterAction extends RunOnceAction {

    @Override
    public void runOnce() {
        LimelightProcessor.getInstance().ledOn(true);
        Swerve.getInstance("LLCenterAction").limeLightAim();
    }
    
}