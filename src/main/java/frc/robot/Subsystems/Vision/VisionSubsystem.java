// package frc.robot.Subsystems.Vision;

// import org.wpilib.math.geometry.Pose2d;
// import org.wpilib.math.geometry.Pose3d;
// import org.wpilib.smartdashboard.SmartDashboard;
// import org.wpilib.command2.SubsystemBase;
// import frc.robot.Subsystems.Vision.LimelightIO;
// import frc.robot.Subsystems.Vision.VisionMeasurement;
// import java.util.Optional;

// public class VisionSubsystem extends SubsystemBase {

//    private final LimelightIO limelight;

//    public VisionSubsystem(LimelightIO limelightIO) {
//        limelight = limelightIO;
//        // pv later --- alex wtf does this mean
//    }

//    public Optional<VisionMeasurement> getVisionMeasurement() {
//        // grab Limelight MegaTag2 pose
//        Optional<VisionMeasurement> llMeasurement = limelight.returnLLVisionMeasurement();
//        if (llMeasurement.isPresent()) {
//            return llMeasurement;
//        }
//        return Optional.empty();
//    }

//    public double getVisionAbsoluteDistance(){
//        // temp ll only
//        return limelight.getLLAbsoluteDistance();
//    }

// // HELPERS

//    public int getTagCount(){
//        return limelight.getLLTagCount();
//    }

//    // ? ==
//    public double getTagDistance(){
//        return limelight.getLLAbsoluteDistance();
//    }
//    // ==

//    public double getBotToHubDistance(Pose2d odom) {
//     return limelight.getBotDistanceFromHubCenter(odom);
//    }

//    public double getAngleDiffBotToHub(double yaw, Pose2d odom) {
//     return limelight.getAngleDegreeOffsetFromHubCenter(yaw, odom);
//    }

//    @Override
//    public void periodic() {
//     SmartDashboard.putNumber("LLTagCount", getTagCount());
//     //SmartDashboard.putNumber("botDistanceFromHub", getBotToHubDistance());
//    }

//    @Override
//    public void simulationPeriodic() {

//    }
// }


