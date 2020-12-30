package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Ports;

public class Pigeon {

	private static Pigeon instance = null;
	
	private PigeonIMU pigeon;
    
	private Pigeon(){
		try{
			pigeon = new PigeonIMU(Ports.PIGEON);
		}catch(Exception e){
			System.out.println(e);
		}
	}

	public static Pigeon getInstance(){
		if(instance == null){
			instance = new Pigeon();
		}
		return instance;
	}
	
	public boolean isGood(){
		return (pigeon.getState() == PigeonState.Ready) ? true : false;
	}
	
	public Rotation2d getYaw(){
		double [] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
		SmartDashboard.putNumber("Pigeon Heading", -pigeon.getFusedHeading(fusionStatus));
		return Rotation2d.fromDegrees(-pigeon.getFusedHeading(fusionStatus)/*-ypr[0]*/);
	}

	public double getPitch(){
		double [] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr[1];
	}

	public double getRoll(){
		double [] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr[2];
	}

	public double[] getYPR(){
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr;
	}
	
	public void setAngle(double angle){
		pigeon.setFusedHeading(-angle * 64.0, Constants.kLongCANTimeoutMs);
		pigeon.setYaw(-angle, Constants.kLongCANTimeoutMs);
		System.out.println("Pigeon angle set to: " + angle);
	}
	
	public void outputToSmartDashboard(){
		SmartDashboard.putString("Pigeon Good", pigeon.getState().toString());
	}

}
