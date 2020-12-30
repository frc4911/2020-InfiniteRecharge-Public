package frc.robot.loops;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team254.lib.loops.Loop;
import com.team254.lib.vision.TargetInfo;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class LimelightProcessor implements Loop {

    static LimelightProcessor instance = new LimelightProcessor();
	private NetworkTable mTable;
	private NetworkTableEntry ledMode;

	public List<NetworkTableEntry> outerGoal;
	// public List<Double> innerGoal;
	// public NetworkTableEntry cornerX, cornerY;

	// private RobotState robotState = RobotState.getInstance(); TODO: bring back when needed
	private boolean updatesAllowed = true;


	public void enableUpdates(boolean enable){
		updatesAllowed = enable;
	}
	

	public void ledOn(boolean on){
		if(ledMode.getDouble(1) != 0 && on)
			ledMode.setNumber(0);
		else if(ledMode.getDouble(0) != 1 && !on)
			ledMode.setNumber(1);
	}


	public static LimelightProcessor getInstance(){
		return instance;
	}
	
	public LimelightProcessor(){

	}
	
	@Override 
	public void onStart(Phase phase){
		System.out.println("LimeLight loop started");
		mTable = NetworkTableInstance.getDefault().getTable("limelight-shoot");
		ledMode = mTable.getEntry("ledMode");

		outerGoal = Arrays.asList(mTable.getEntry("tx"), mTable.getEntry("ty"), mTable.getEntry("tv"));

	}
	
	@Override 
	public void onLoop(double timestamp) {
		List<TargetInfo> targets = new ArrayList<TargetInfo>();
		if (updatesAllowed && outerGoalVisible()) {
			// add main target info
			targets.add(new TargetInfo(Math.tan(Math.toRadians(outerGoal.get(0).getDouble(0))), Math.tan(Math.toRadians(outerGoal.get(1).getDouble(0)))));
		}
		//robotState.addVisionUpdate(timestamp, targets);
		
	}


	public boolean outerGoalVisible() {
		return (outerGoal.get(2).getDouble(0) == 1.0);
	}



	
	@Override
	public void onStop(double timestamp){
		
	}

	

	public TargetInfo getTargetInfo(double nx, double ny){
		double vpw = 2.0 * Math.tan(Math.toRadians(29.8));// 27.0 29.8
		double vph = 2.0 * Math.tan(Math.toRadians(24.85));//24.85  20.5
		double x = vpw / 2.0 * nx;
		double y = vph / 2.0 * ny;
		double ax = Math.atan2(x, 1.0);
		double ay = Math.atan2(y, 1.0);

		return new TargetInfo(Math.tan(ax), Math.tan(ay));
	}





}