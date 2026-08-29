// package frc.robot.Subsystems.Vision;

// import frc.robot.Subsystems.Vision.VisionMeasurement;
// import org.wpilib.math.geometry.Pose2d;
// import org.wpilib.math.geometry.Pose3d;
// import org.wpilib.networktables.NetworkTable;
// import org.wpilib.driverstation.DriverStation;
// import org.wpilib.smartdashboard.SmartDashboard;

// import java.util.Optional;

// import org.photonvision.PhotonPoseEstimator.PoseStrategy;

// import com.google.flatbuffers.Constants;

// import org.wpilib.command2.SubsystemBase;
// import frc.robot.LimelightHelpers;
// import frc.robot.RobotUtils;
// import frc.robot.Constants.FieldConstants;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.Constants.FieldConstants.HubMeasurements;

// public class LimelightIO extends SubsystemBase{
//     public boolean blueAlliance;
//    private Pose2d llPose;
//    private double timestamp;
//    private final String limelightName = ShooterConstants.LIMELIGHT_NAME;
//    // private final NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
//    private final double[] defaultArray = {0};
//    private LimelightHelpers.PoseEstimate poseEstimator;
//    private Pose2d allianceHub;
//    // private final LimelightHelpers llHelpers;
//    // nt instance above if needed
  
//    public LimelightIO(boolean isBlue){
//     blueAlliance = isBlue;

//     poseEstimator = getPoseEstimate();
//     poseEstimator.isMegaTag2 = true;

//     if (blueAlliance) {
//         allianceHub = HubMeasurements.BLUEHUB_POSE;
//     } else {allianceHub = HubMeasurements.REDHUB_POSE;}
//    }

//    public Optional<VisionMeasurement> returnLLVisionMeasurement(){
//        if(poseEstimator == null || poseEstimator.tagCount == 0) return Optional.empty();
//        llPose = poseEstimator.pose;
//        timestamp = poseEstimator.timestampSeconds;
//        return Optional.of(new VisionMeasurement(llPose, timestamp));
//    }

//    private LimelightHelpers.PoseEstimate getPoseEstimate() {
//     return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
//    }

//    public int getLLTagCount() {
//        LimelightHelpers.PoseEstimate est = getPoseEstimate();
//        if (est == null) return 0;
//        return est.tagCount;
//    }

//    // do these work? =====
//    public double getLLRobotRelativeDistance(){
//         LimelightHelpers.PoseEstimate est = getPoseEstimate();
//         if (est == null || est.tagCount == 0) return 0;
//         Pose2d pose = est.pose;
//        // field relative
//         return pose.getTranslation().getNorm();
//    }

//    public double getLLAbsoluteDistance(){
//         // table.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(defaultArray); IF NOT MT2
//         Pose2d llAbsoluteTag = poseEstimator.pose;
//         return llAbsoluteTag.getTranslation().getNorm();
//    }
//    // ======
   
//    public double getBotDistanceFromHubCenter(Pose2d odom){
//     double distance = RobotUtils.hypot((odom.getX() - allianceHub.getX()), 
//         odom.getY() - allianceHub.getY());
//     return RobotUtils.metersToInches(distance);
//    }

//    public double getAngleDegreeOffsetFromHubCenter(double yawDeg, Pose2d odom) {
//     yawDeg += (blueAlliance ? 0 : 180);
//     double targetPoint = Math.atan2(allianceHub.getY() - odom.getY(), 
//         allianceHub.getX() - odom.getX()) * (180/Math.PI);
//     double difference = yawDeg - targetPoint;
//     return difference;
//    }

//    @Override
//    public void periodic() {
//     poseEstimator = getPoseEstimate();
//    }
// }

