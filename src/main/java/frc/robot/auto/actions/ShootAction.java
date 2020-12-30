package frc.robot.auto.actions;

import com.team254.lib.autos.actions.Action;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Superstructure;

public class ShootAction implements Action {
	
	private Superstructure mSuperstructure = Superstructure.getInstance("ShootAction");
	private double target = 0.0;

	public ShootAction(double duration) {
		target = Timer.getFPGATimestamp() + duration;
	}
	
	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() >= target;
	}
	
	@Override
	public void start() {
		mSuperstructure.setWantedState(Superstructure.WantedState.SHOOT);
	}
	
	@Override
	public void update() {

	}
	
	@Override
	public void done() {
		mSuperstructure.setWantedState(Superstructure.WantedState.HOLD);
	}
	
}
