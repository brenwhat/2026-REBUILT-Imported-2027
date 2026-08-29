// package frc.robot.Subsystems.Vision;

// import org.wpilib.math.geometry.Pose2d;
// import org.wpilib.networktables.NetworkTableInstance;
// import org.wpilib.smartdashboard.SmartDashboard;
// import org.wpilib.command2.SubsystemBase;
// import frc.robot.Constants.FieldConstants;
// import frc.robot.LimelightHelpers;
// import frc.robot.RobotUtils;

// public class VisionTESTING extends SubsystemBase {
//     /*
//     public Vision() {
        
//     }
//     */

//     public double getHubTagTarget() {
//     //System.out.println("f");
//     if (LimelightHelpers.getTV("limelight")) {
//       int tid = (int)(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0));
//       SmartDashboard.putNumber("tid", tid);
//      // Pose3d tagPose = LimelightHelpers.getTar
//       double tx = LimelightHelpers.getTX("limelight") * -0.05;
//       //System.out.println("TX: " + tx);
//       SmartDashboard.putNumber("tx", tx);
//       return tx;
//     }
//     return 0;
//   }

//   public double getDistanceFromCenterHub(boolean redAlliance) {
//     if (LimelightHelpers.getTV("limelight")) {
//       Pose2d botPose;
//       if (redAlliance) {
//         botPose = (LimelightHelpers.getBotPose2d_wpiRed("limelight"));
//       } else {
//         botPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
//       }   
//       SmartDashboard.putNumber("botPoseX", botPose.getX());
//       SmartDashboard.putNumber("botPoseY", botPose.getY());
//       double distanceMeters = RobotUtils.hypot(FieldConstants.HubMeasurements.ALLIANCEHUB_POSE.getX() - botPose.getX(), FieldConstants.HubMeasurements.ALLIANCEHUB_POSE.getY() - botPose.getY());
//       SmartDashboard.putNumber("Test Distance from Hub", RobotUtils.metersToInches(distanceMeters));
//       return RobotUtils.metersToInches(distanceMeters);
//     }
//     return 50; // if we dont see an apriltag, im making 50 inches our default because i think thats where were usually shooting
//   }
// }
