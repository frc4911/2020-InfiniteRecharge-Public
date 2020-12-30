package frc.robot.subsystems.Limelights;

import java.util.ArrayList;
import java.util.Comparator;
// import java.util.Optional;

import com.team254.lib.geometry.Translation2d;
// import com.team254.lib.vision.AimingParameters;
import com.team254.lib.vision.TargetInfo;

import frc.robot.Constants;
// import frc.robot.RobotState;

public class ShootwardsLimelight extends Limelight {    
    // private final boolean mUsingZoom = false;
    private static final Comparator<Translation2d> xSort = Comparator.comparingDouble(Translation2d::x);

    // private Optional<AimingParameters> mAimingParameters = null; // for zooming (zoom at certain distance)

    private static String sClassName;
    private static int sInstanceCount;
    private static ShootwardsLimelight sInstance = null;
    public  static ShootwardsLimelight getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new ShootwardsLimelight(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private ShootwardsLimelight(String caller) {
        super(Constants.kShootwardsLimelightConstants);
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        useRawCorners(true);
        setPipeline(0);
    }

    // if available, returns main contour as a target (always first)
    // if avaiable returns corners as targets (left is second and right is third)
    @Override
    protected void getRawTargetInfos(ArrayList<TargetInfo> targets) {
        if (!seesTarget()) {
            setZoom(1);
            return;
        }

        add(targets, TargetType.MAIN_CONTOUR);
        add(targets, getTopCorners());

        // if corners where added manage zoom
        // if(super.mTargets.size() == 3 && mUsingZoom) {
        //     manageZoom();
        // }
    }

    // private void manageZoom() {        
    //     double distance = getDistance();
    //     if (super.getCachedZoom() == 1) {
    //         // keep seperate if statements
    //         if(cachedPointsInRect(Constants.kZoomInProportion) && (distance > Constants.kZoomInDistance)) {
    //             setZoom(2);
    //         }
    //     // super.getCachedZoom() == 2
    //     } else if (!cachedPointsInRect(Constants.kZoomOutProportion) || distance < Constants.kZoomOutDistance) {
    //         setZoom(1);
    //     }
    // }

    // -1 if no target found, else distance in ft
    // private double getDistance() {
    //     mAimingParameters = RobotState.getInstance().getOuterGoalParameters();
    //     double distance = 0;
    //     if (mAimingParameters.isPresent()) {
    //         distance = mAimingParameters.get().getRobotToGoal().getTranslation().norm() / 12.0;
    //     }
    //     return distance;
    // }

    // get the top corners in the order left then right
    private ArrayList<Translation2d> getTopCorners() {
        ArrayList<Translation2d> topCorners = new ArrayList<>();
        ArrayList<Translation2d> rawCorners = get(TargetType.RAW_CORNERS);

        // half hexagon shape should have 4 or 3 corners
        // three corners is acceptable, because skewness makes it difficult to observe all 4 corners (for robustness)
        if (!(rawCorners.size() == 3 || rawCorners.size() == 4)) {
            return topCorners;
        }

        // since hexagon shape =>  "\__/", 
        // left most and rightmost corners are always the top corners
        rawCorners.sort(xSort);
        // leftmost corner
        topCorners.add(new Translation2d( 
            rawCorners.get(0).x(), 
            rawCorners.get(0).y()));
        // rightmost corner
        topCorners.add(new Translation2d(
            rawCorners.get(rawCorners.size() - 1).x(), 
            rawCorners.get(rawCorners.size() - 1).y()));
        return topCorners;
        
    }

    private void setZoom(int zoom) {
        super.setPipeline(zoom - 1);
    }
}
