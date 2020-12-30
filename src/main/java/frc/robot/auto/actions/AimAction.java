package frc.robot.auto.actions;

import com.team254.lib.autos.actions.Action;

import frc.robot.subsystems.Swerve;

public class AimAction implements Action {
	
    private Swerve mSwerve;
	private String sClassName;
	
	public AimAction() {
		sClassName = this.getClass().getSimpleName();
		mSwerve = Swerve.getInstance(sClassName);
	}
	
	@Override
	public boolean isFinished() {
		return true;
	}
	
	@Override
	public void start() {
        mSwerve.limeLightAim();
	}
	
	@Override
	public void update() {

	}
	
	@Override
	public void done() {

	}
	
}
