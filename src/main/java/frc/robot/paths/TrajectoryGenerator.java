package frc.robot.paths;

import frc.robot.Constants;
import frc.robot.planners.DriveMotionPlanner;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryUtil;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import edu.wpi.first.wpilibj.Timer;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 120.0;
    private static final double kMaxAccel = 60.0; //120.0;
    private static final double kMaxDecel = 72.0; //72.0;
    private static final double kMaxVoltage = 9.0;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if(mTrajectorySet == null) {
        	double startTime = Timer.getFPGATimestamp();
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation in: " + (Timer.getFPGATimestamp() - startTime) + " seconds");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_decel,
            double max_voltage,
            double default_vel,
            int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_decel, max_voltage, 
            default_vel, slowdown_chunks);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_decel,
            double max_voltage,
            double default_vel,
            int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_decel, max_voltage, 
            default_vel, slowdown_chunks);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the right.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON LEFT! (mirrored about +x axis for RIGHT)
    static final Pose2d autoStartingPose = new Pose2d(Constants.kRobotLeftStartingPose.getTranslation().translateBy(new Translation2d(/*-0.5*/0.0, 0.0)), Rotation2d.fromDegrees(-90.0));

    static final Pose2d closeHatchScoringPose = Constants.closeHatchPosition.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength - 3.5, -2.0)));
    public static final Pose2d farHatchScoringPose = Constants.farHatchPosition.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength - 5.0, 8.0)));
    static final Pose2d humanLoaderPose = Constants.humanLoaderPosition.transformBy(Pose2d.fromTranslation(new Translation2d(Constants.kRobotHalfLength - 4.0, 2.0)));
    static final Pose2d ballIntakePose = new Pose2d(Constants.autoBallPosition.transformBy(Pose2d.fromTranslation(new Translation2d(Constants.kRobotHalfLength + 9.0, 0.0))).getTranslation(), Rotation2d.fromDegrees(0.0));
    static final Pose2d portScoringPose = Constants.rocketPortPosition.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength - 6.0, 0.0)));

    static final Pose2d farShipScoringPose = Constants.farShipPosition.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength - 4.0, 0.0)));
    static final Pose2d midShipScoringPose = Constants.midShipPosition.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength - 4.0, 0.0)));
    static final Pose2d closeShipScoringPose = Constants.closeShipPosition.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength - 4.0, 6.0)));

    // Test Poses Alex
    private static final Pose2d startingPose = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0));
    private static final Pose2d midPose = new Pose2d(new Translation2d(60.0, 0.0), Rotation2d.fromDegrees(0.0));
    private static final Pose2d mid2Pose = new Pose2d(new Translation2d(60.0, 100.0), Rotation2d.fromDegrees(90.0));
    private static final Pose2d endingPose = new Pose2d(new Translation2d(115.0, 100.0), Rotation2d.fromDegrees(0.0));


    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> left) {
                this.left = left;
                this.right = TrajectoryUtil.mirrorTimed(left, left.defaultVelocity());
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }

        // Rendezvous Paths
        public final MirroredTrajectory startToThreePowerCells;
        public final MirroredTrajectory startToThreePowerCells2;
        public final MirroredTrajectory collectThreePowerCells;      
        public final MirroredTrajectory moveToTwoPowerCells;
        public final MirroredTrajectory moveToTwoPowerCells2;            
        public final MirroredTrajectory collectTwoPowerCells; 
        
        // Steal Paths
        public final MirroredTrajectory startToStealPath;
        public final MirroredTrajectory stealToGoalPath;
        public final MirroredTrajectory goalToRendezvousPath;
        public final MirroredTrajectory threeBallsPath;
        public final MirroredTrajectory threeBallsToGoalPath;

        // Trench Paths
        public final MirroredTrajectory startToTrenchPath;
        public final MirroredTrajectory trenchToStartPath;

        // Test Paths Alex
        public final MirroredTrajectory backAwayFromLinePath;

        public final MirroredTrajectory startToEndPath;
        public final MirroredTrajectory testPath;
        public final MirroredTrajectory testPath2;
        public final MirroredTrajectory testPath3;
        public final MirroredTrajectory testPath4;

        public final MirroredTrajectory testPathBrian;

        private TrajectorySet() {
            // Rendezvous Paths
            startToThreePowerCells = new MirroredTrajectory(getStartToThreePowerCells());
            startToThreePowerCells2 = new MirroredTrajectory(getStartToThreePowerCells2());
            // System.out.println(startToThreePowerCells.left.toString());
            collectThreePowerCells = new MirroredTrajectory(getCollectThreePowerCells());
            // System.out.println(collectThreePowerCells.left.toString());
            moveToTwoPowerCells = new MirroredTrajectory(getMoveToTwoPowerCells());
            // System.out.println(moveToTwoPowerCells.left.toString());
            moveToTwoPowerCells2 = new MirroredTrajectory(getMoveToTwoPowerCells2());
            // System.out.println(moveToTwoPowerCells2.left.toString());
            collectTwoPowerCells = new MirroredTrajectory(getCollectTwoPowerCells());
            // System.out.println(collectTwoPowerCells.left.toString());

            // Trench Paths
            startToTrenchPath = new MirroredTrajectory(getStartToTrench());
            trenchToStartPath = new MirroredTrajectory(getTrenchToStart());

            // Steal Paths
            startToStealPath = new MirroredTrajectory(getStartToSteal());
            stealToGoalPath = new MirroredTrajectory(getStealToGoal());
            goalToRendezvousPath = new MirroredTrajectory(getGoalToRendezvous());
            threeBallsPath = new MirroredTrajectory(getThreeBalls());
            threeBallsToGoalPath = new MirroredTrajectory(getThreeBallsToGoal());

            // Test Paths Alex
            backAwayFromLinePath = new MirroredTrajectory(getBackAwayFromLinePath());

            startToEndPath = new MirroredTrajectory(getStartToEndPath());
            // System.out.println(startToEndPath.left.toString());
            testPath = new MirroredTrajectory(getTestPath());
            // System.out.println(testPath.left.toString());
            testPath2 = new MirroredTrajectory(getTestPath2());
            // System.out.println(testPath2.left.toString());
            testPath3 = new MirroredTrajectory(getTestPath3());
            // System.out.println(testPath3.left.toString());
            testPath4 = new MirroredTrajectory(getTestPath4());
            // System.out.println(testPath4.left.toString());

            testPathBrian = new MirroredTrajectory(getBrianPath());
            // System.out.println("Brian's path");
            // System.out.println(testPathBrian.left.toString());
            // System.out.println("Brian's path end");

        }

        /*************************************Rendezvous Paths*****************************************/

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToThreePowerCells() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(-133.5)));
            waypoints.add(new Pose2d(new Translation2d(-100.0, -95.0), Rotation2d.fromDegrees(-133.5)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/20.0, 20.0, 60.0, kMaxVoltage, 20.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToThreePowerCells2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-100.0, -95.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(-105.0, -95.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(-93.0, -71.0), Rotation2d.fromDegrees(63.4)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/20.0, 20.0, 60.0, kMaxVoltage, 20.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getCollectThreePowerCells() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-95.0, -95.0), Rotation2d.fromDegrees(60.0)));
            waypoints.add(new Pose2d(new Translation2d(-83.0, -71.0), Rotation2d.fromDegrees(60.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/20.0, 20.0, 60.0, kMaxVoltage, 20.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getMoveToTwoPowerCells() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-83.0, -71.0), Rotation2d.fromDegrees(-30.0)));
            waypoints.add(new Pose2d(new Translation2d(-70.0, -81.0), Rotation2d.fromDegrees(-30.0)));
            waypoints.add(new Pose2d(new Translation2d(-33.0, -15.0), Rotation2d.fromDegrees(60.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/20.0, 20.0, 60.0, kMaxVoltage, 20.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getMoveToTwoPowerCells2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-33.0, -15.0), Rotation2d.fromDegrees(150.0)));
            waypoints.add(new Pose2d(new Translation2d(-118.0, 15.0), Rotation2d.fromDegrees(150.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/20.0, 20.0, 60.0, kMaxVoltage, 20.0, 1);
        }        

        private Trajectory<TimedState<Pose2dWithCurvature>> getCollectTwoPowerCells() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-95.0, -95.0), Rotation2d.fromDegrees(60.0)));
            waypoints.add(new Pose2d(new Translation2d(-83.0, -71.0), Rotation2d.fromDegrees(60.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/20.0, 20.0, 60.0, kMaxVoltage, 20.0, 1);
        }

        /*************************************Trench Paths*****************************************/

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToTrench() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(135.0)));
            waypoints.add(new Pose2d(new Translation2d(-65.0, 65.0), Rotation2d.fromDegrees(135.0)));
            waypoints.add(new Pose2d(new Translation2d(-195.0, 65.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/20.0, 20.0, 60.0, kMaxVoltage, 20.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchToStart() {
            List<Pose2d> waypoints = new ArrayList<>();
            // waypoints.add(new Pose2d(new Translation2d(-185.0, 61.0), Rotation2d.fromDegrees(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(-40.0, 40.0), Rotation2d.fromDegrees(-45.0)));
            // waypoints.add(new Pose2d(new Translation2d(-20.0, 20.0), Rotation2d.fromDegrees(-45.0)));
            waypoints.add(new Pose2d(new Translation2d(-195.0, 65.0), Rotation2d.fromDegrees(-18.4)));
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(-18.4)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/20.0, 20.0, 60.0, kMaxVoltage, 20.0, 1);
        }

        /*************************************Steal Paths*****************************************/

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToSteal() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(-122.0, 0.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/120.0, 60.0, 60.0, kMaxVoltage, 120.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStealToGoal() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-122.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(-100.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(-10.0, 160.0), Rotation2d.fromDegrees(60.6)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/120.0 * 0.75, 60.0, 60.0, kMaxVoltage, 120.0 * 0.75, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getGoalToRendezvous() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-10.0, 160.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(-80.0, 160.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/120.0 * 0.5, 60.0, 60.0, kMaxVoltage, 120.0 * 0.5, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThreeBalls() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-80.0, 160.0), Rotation2d.fromDegrees(120.0)));
            waypoints.add(new Pose2d(new Translation2d(-108.0, 112.0), Rotation2d.fromDegrees(120.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/120.0 * 0.5, 60.0, 60.0, kMaxVoltage, 120.0 * 0.5, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThreeBallsToGoal() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-48.0, 112.0), Rotation2d.fromDegrees(32.0)));
            waypoints.add(new Pose2d(new Translation2d(-10.0, 160.0), Rotation2d.fromDegrees(32.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/120.0 * 0.5, 60.0, 60.0, kMaxVoltage, 120.0 * 0.5, 1);
        }

        /*************************************Test Paths*****************************************/

        private Trajectory<TimedState<Pose2dWithCurvature>> getBackAwayFromLinePath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(-60.0, 0.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/120.0, 60.0, 60.0, kMaxVoltage, 120.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getBrianPath(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d((10*12), 0.0), Rotation2d.fromDegrees(90.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), 30, kMaxAccel, kMaxDecel, kMaxVoltage, 30.0, 1);
        }
        
        // private Trajectory<TimedState<Pose2dWithCurvature>> getStraightPath(){
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(Constants.kRobotLeftStartingPose);
        //     waypoints.add(Constants.kRobotLeftStartingPose.transformBy(Pose2d.fromTranslation(new Translation2d(72.0, 0.0))));

        //     return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToEndPath(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(startingPose);
            waypoints.add(midPose);
            waypoints.add(mid2Pose);
            waypoints.add(endingPose);

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(60.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(60.0, -95.0), Rotation2d.fromDegrees(270.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(-60.0, 0.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/20.0, 20.0, 60.0, kMaxVoltage, 20.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath3() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(135.0)));
            waypoints.add(new Pose2d(new Translation2d(-40.0, 40.0), Rotation2d.fromDegrees(135.0)));
            waypoints.add(new Pose2d(new Translation2d(-185.0, 61.0), Rotation2d.fromDegrees(180.0)));
            
            // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/20.0, 20.0, 60.0, kMaxVoltage, 20.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath4() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-185.0, 61.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(-40.0, 40.0), Rotation2d.fromDegrees(-45.0)));
            waypoints.add(new Pose2d(new Translation2d(-20.0, 20.0), Rotation2d.fromDegrees(-45.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }
    }
    
}
