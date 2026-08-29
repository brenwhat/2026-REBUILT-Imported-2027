// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DriveTrain;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.framework.RobotBase;

public abstract class SwerveModule {

  public SwerveModuleVelocity target_state;
  // private boolean isBraked = false;

  public SwerveModule() {
    target_state = new SwerveModuleVelocity();
  }

  public void setState(SwerveModuleVelocity state) {
    target_state = state;
  }

  abstract void setModuleMode(boolean brake);
  abstract void setBrakeAngle(double angle);

  public void update() {
    if (!RobotBase.isDisabled()) { //just in case, idk
      setSpeed(target_state.velocity);
      setAngle(target_state.angle);
    }
  }

  abstract double getSpeed();
  abstract double getPosition();
  abstract Rotation2d getAngle();
  abstract SwerveModuleVelocity getModuleState();
  abstract SwerveModulePosition getModulePosition();
  /**
   * 
   * @param speed the speed to go at, in meters per second.
   */
  abstract void setSpeed(double speed);
  abstract void setAngle(Rotation2d angle);
}
