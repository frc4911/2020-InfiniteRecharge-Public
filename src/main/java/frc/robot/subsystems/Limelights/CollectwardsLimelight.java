package frc.robot.subsystems.Limelights;

import java.util.ArrayList;
import com.team254.lib.vision.TargetInfo;
import frc.robot.Constants;

public class CollectwardsLimelight extends Limelight {
    
    private static String sClassName;
    private static int sInstanceCount;
    private static CollectwardsLimelight sInstance = null;
    public static CollectwardsLimelight getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new CollectwardsLimelight(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private CollectwardsLimelight(String caller) {
        super(Constants.kCollectwardsLimelightConstants);
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        useRawContours(true);
    }

    // returns raw contours as target infos
    @Override
    protected void getRawTargetInfos(ArrayList<TargetInfo> targets) {
        add(targets, TargetType.RAW_CONTOURS);
    }
}
