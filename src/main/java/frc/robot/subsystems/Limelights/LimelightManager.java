// package frc.robot.subsystems.Limelights;

// import java.util.List;
// import com.team254.lib.loops.ILooper;
// import com.team254.lib.loops.Loop;
// import com.team254.lib.subsystems.Subsystem;
// import com.team254.lib.vision.TargetInfo;

// import frc.robot.RobotState;

// /**
//  * Class that manages using multiple Limelight 2's, one at a time
//  * 
//  * @see Limelight
//  */
// public class LimelightManager extends Subsystem {
    // private static LimelightManager sInstance = null;
    // private ShootwardsLimelight mShootwardsLimelight;
    // private CollectwardsLimelight mCollectwardsLimelight;
    // private List<Limelight> mAllLimelights;
    // private int emptyObservationsCount;
    // private int totalObservationsCount;
    // private int firstNonNullCount;

    // private LimelightManager() {
    //     mShootwardsLimelight = new ShootwardsLimelight();
    //     mCollectwardsLimelight = new CollectwardsLimelight();
    //     mAllLimelights = List.of(mShootwardsLimelight, mCollectwardsLimelight);
    // }

    // public static LimelightManager getInstance() {
    //     if (sInstance == null) {
    //         sInstance = new LimelightManager();
    //     }
    //     return sInstance;
    // }

    // @Override
    // public synchronized void zeroSensors() {
    //     mAllLimelights.forEach(limelight -> limelight.zeroSensors());
    //     RobotState.getInstance().resetVision();
    // }


    // @Override
    // public void registerEnabledLoops(ILooper mEnabledLooper) {
    //     Loop mLoop = new Loop() {
    //         @Override
    //         public void onStart(double timestamp) {
    //             System.out.println("Starting LimelightManager");
    //         }

    //         @Override
    //         public void onLoop(double timestamp) {
    //             synchronized (LimelightManager.this) {
    //                 for(Limelight limelight : mAllLimelights)
    //                     if (limelight.isActive()) {
    //                         List<TargetInfo> observations = limelight.getTarget();
    //                         // logging
    //                         totalObservationsCount++;
    //                         if (!observations.isEmpty() && firstNonNullCount == 0) {
    //                             firstNonNullCount = totalObservationsCount;
    //                         }
                        
    //                         RobotState.getInstance().addVisionUpdate(
    //                             timestamp - limelight.getLatency(),
    //                             limelight.getTarget(),
    //                             limelight.getTargetTypes(),
    //                             limelight);
    //                     }
    //             }
    //         }

    //         @Override
    //         public void onStop(double timestamp) {
    //             stop();
    //         }
    //     };

    //     mEnabledLooper.register(mLoop);
    // }

    // @Override
    // public String getLogHeaders() {
    //     StringBuilder headers = new StringBuilder();
    //     for (int i=0; i<mAllLimelights.size(); i++){
    //         if (headers.length()>0){
    //             headers.append(",");
    //         }
    //         headers.append(mAllLimelights.get(i).getLogHeaders());
    //     }
    //     return headers.toString();
    // }

    // @Override
    // public String getLogValues() {
    //     StringBuilder values = new StringBuilder();
    //     for (int i=0; i<mAllLimelights.size(); i++){
    //         if (values.length()>0){
    //             values.append(",");
    //         }
    //         values.append(mAllLimelights.get(i).getLogValues());
    //     }
    //     return values.toString();
    // }

    // @Override
    // public synchronized void readPeriodicInputs(boolean doOptionalWork) {
    //     mAllLimelights.forEach(limelight -> limelight.readPeriodicInputs(doOptionalWork));
    // }

    // @Override
    // public synchronized void writePeriodicOutputs(boolean doOptionalWork) {
    //     mAllLimelights.forEach(limelight -> limelight.writePeriodicOutputs(doOptionalWork));
    // }

    // @Override
    // public synchronized void stop() {
    //     mAllLimelights.forEach(limelight -> limelight.stop());
    // }

    // @Override
    // public boolean checkSystem() {
    //     return true;
    // }

    // @Override
    // public void outputTelemetry(boolean doOptionalWork) {
    //     mAllLimelights.forEach(limelight -> limelight.outputTelemetry(doOptionalWork));
    // }

    // public void setShootwardsActive(boolean activate) {
    //     // setActive(0, active); TODO: REWRITE CLEARLY FOR COLLECTOR
    //     mShootwardsLimelight.setActive(activate);
    //     if (activate) {
    //         // logging
    //         emptyObservationsCount = 0;
    //         totalObservationsCount = 0;
    //         firstNonNullCount = 0;
    //         // mActiveLimelights = 1;
    //     } else {
    //         System.out.println("emptyObservationsCount: " + emptyObservationsCount);
    //         System.out.println("totalObservationsCount: " + totalObservationsCount);
    //         System.out.println("firstNonNullCount: " + firstNonNullCount);
    //         // mActiveLimelights = 0;
    //     }
    // }

    // public void setCollectwardsActive(boolean activate) {
    //     mCollectwardsLimelight.setActive(activate);
    // }

    // public Limelight getShootwardsLimelight() {
    //     return mShootwardsLimelight;
    // }

    // public Limelight getCollectwardsLimelight() {
    //     return mCollectwardsLimelight;
    // }

    // public synchronized void triggerOutputs() {
    //     mAllLimelights.forEach(limelight -> limelight.triggerOutputs());
    // }

    // public synchronized void setAllLeds(Limelight.LedMode mode) {
    //     mAllLimelights.forEach(limelight -> limelight.setLed(mode));
    // }


//}
