// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.TransferQueue;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;

import org.wpilib.math.util.Units;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchState;
import org.wpilib.framework.TimedRobot;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.HubTimer;

public class Robot extends TimedRobot {
  private Command autonomous_command;

  private final RobotContainer m_robot_container;
  private final HubTimer m_hubTimer;
  public boolean blueAlliance;

  public Robot() {
    m_robot_container = new RobotContainer();
    m_hubTimer = new HubTimer();
    FollowPathCommand.warmupCommand().schedule();
    SignalLogger.enableAutoLogging(false);
    m_robot_container.m_drive.refreshAlliance();
  }

  @Override
  public void robotPeriodic() {
    //LimelightHelpers.SetIMU("limelight", 1);
    m_robot_container.updateShooterDistance();
    SmartDashboard.putNumber("Match Time", MatchState.getMatchTime());
    SmartDashboard.putBoolean("Hub Active", m_hubTimer.isHubActive());
    SmartDashboard.putNumber("Shift Time", m_hubTimer.getRemainingHubShift());
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}


  @Override
  public void disabledPeriodic() {
    m_robot_container.m_drive.refreshAlliance();
    autonomous_command = m_robot_container.getAutonomousCommand();
  }

  @Override
  public void disabledExit() {
    m_robot_container.m_drive.refreshAlliance();
  }

  @Override
  public void autonomousInit() {
    //m_robot_container.m_drive.resetGyroAngle();
    //m_robot_container.m_drive.refreshAlliance();
    m_robot_container.m_drive.autoVisionMeasurement = false;
    autonomous_command = m_robot_container.getAutonomousCommand();
    if (autonomous_command != null) {
      System.out.println("AUTO INITIALIZED");
      //m_robot_container.m_drive.resetGyroAngle();
      autonomous_command.schedule();
    }

  }

  @Override
  public void autonomousPeriodic() {
    m_robot_container.autoAlignSwerve();
  }

  @Override
  public void autonomousExit() {
    m_robot_container.m_shooter.enabled = false;
      m_robot_container.m_agitator.stopAgitation();
      m_robot_container.m_agitator.stopGate();
      //m_robot_container.alignHub.cancel();
      // m_robot_container.StopSwerve.schedule();
  }

  @Override
  public void teleopInit() {
    m_robot_container.m_drive.refreshAlliance();
    if (autonomous_command != null) {
      autonomous_command.cancel();
      m_robot_container.m_shooter.enabled = false;
      m_robot_container.m_agitator.stopAgitation();
      m_robot_container.m_agitator.stopGate();
      // m_robot_container.AlignHub.cancel();
      // m_robot_container.StopSwerve.schedule();
    }
  }

  @Override
  public void teleopPeriodic() {
     m_robot_container.updateSwerve();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
