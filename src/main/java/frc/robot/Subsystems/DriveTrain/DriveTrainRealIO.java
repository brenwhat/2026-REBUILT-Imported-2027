// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DriveTrain;


import com.studica.frc.AHRS;  //TODO not sure how to do this

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

import java.util.Optional;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchState;
import org.wpilib.smartdashboard.SmartDashboard;

public class DriveTrainRealIO extends DriveTrain {

  private AHRS gyro;

  public DriveTrainRealIO(){
    gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    refreshGyro();
  }

  public void refreshGyro() {
    if (gyro !=  null) {
      boolean blueAlliance;
      Optional<Alliance> alliance = MatchState.getAlliance();
          if (alliance.isPresent()) {
            blueAlliance = !(alliance.get() == Alliance.RED);
          } else {blueAlliance = false;}

        gyro.setAngleAdjustment(
      blueAlliance ? 0 : 180);
    }
  }

  public SwerveModule initializeModule(int drive_port, int steer_port, int sensor_port){
    return new SwerveModuleRealIO(drive_port, steer_port, sensor_port);
  }

  public Rotation2d getGyroAngle() {
    if (gyro != null) {
      SmartDashboard.putNumber("lalala", Rotation2d.fromDegrees(gyro.getAngle()).getRotations());
      return Rotation2d.fromDegrees(gyro.getAngle());} //gyro reports CW positive, negate to return CCW positive
    return new Rotation2d();
  }

  public double getGyroRate() {
    if (gyro != null) {
      return gyro.getRate();
    } else {return 0;}
  }

  public void resetGyroAngle() {
    if (gyro == null) return;
    gyro.zeroYaw();
    //if (pose_estimator != null) pose_estimator.resetPose(new Pose2d(odom_pose.getX(), odom_pose.getY(), new Rotation2d()));
  }

  @Override
  public void periodic() {
    //the child periodic() method overrides the parent class periodic(), which has to be explicitly called
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
}
