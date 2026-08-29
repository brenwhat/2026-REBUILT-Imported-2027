// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DriveTrain;

import static org.wpilib.units.Units.Degrees;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.Angle;
import org.wpilib.smartdashboard.SmartDashboard;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.DriveMotorConfigs;
import frc.robot.Constants.Swerve.SteerMotorConfigs;


public class SwerveModuleRealIO extends SwerveModule{

  public TalonFX steer_motor, drive_motor;
  public int steerPortVar;
  private boolean braked = false;
  private double brakeAngle = 90;

  public CANcoder steer_sensor;

  public SwerveModuleRealIO(int drive_port, int steer_port, int sensor_port){
    drive_motor = new TalonFX(drive_port,"CANivore");  // TODO add canbus numbers
    steer_motor = new TalonFX(steer_port,"CANivore");
    steer_sensor = new CANcoder(sensor_port,"CANivore");
    steerPortVar = steer_port;
    
    TalonFXConfiguration drive_configs = new TalonFXConfiguration();

    drive_configs.Slot0 = new Slot0Configs()
      .withKP(DriveMotorConfigs.kP)
      .withKI(DriveMotorConfigs.kI)
      .withKD(DriveMotorConfigs.kD)
      .withKS(DriveMotorConfigs.kS)
      .withKV(DriveMotorConfigs.kV)
      .withKA(DriveMotorConfigs.kA)
      .withKG(DriveMotorConfigs.kG);

    drive_configs.Feedback.SensorToMechanismRatio = 5.01;

    drive_configs.Audio.AllowMusicDurDisable = true;

    drive_motor.getConfigurator().apply(drive_configs);

    TalonFXConfiguration steer_configs = new TalonFXConfiguration();

    steer_configs.Slot0 = new Slot0Configs()
      .withKP(SteerMotorConfigs.kP)
      .withKI(SteerMotorConfigs.kI)
      .withKD(SteerMotorConfigs.kD)
      .withKS(SteerMotorConfigs.kS)
      .withKV(SteerMotorConfigs.kV)
      .withKA(SteerMotorConfigs.kA)
      .withKG(SteerMotorConfigs.kG);

    steer_configs.Feedback.FeedbackRemoteSensorID = sensor_port;
    steer_configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    //steer_configs.Feedback.RotorToSensorRatio = 22.5;
    // steer_configs.Feedback.SensorToMechanismRatio = 1; // defaults to 1
    steer_configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    steer_configs.Voltage.PeakForwardVoltage = 3.5;
    steer_configs.Voltage.PeakReverseVoltage = -3.5;

    steer_configs.ClosedLoopGeneral.ContinuousWrap = true;

    steer_configs.Audio.AllowMusicDurDisable = true;

    steer_motor.getConfigurator().apply(steer_configs);
  }

  public double getSpeed(){
    //getRotorVelocity() returns StatusSignal<AngularVelocity> with base unit rps
    return drive_motor.getRotorVelocity().getValueAsDouble() * Swerve.WHEEL_DIAMETER / Swerve.GEAR_RATIO;
  }

  public double getPosition(){
    //getRotorPosition() returns StatusSignal<Angle> with base unit rotations
    return drive_motor.getRotorPosition().getValueAsDouble() * Swerve.WHEEL_DIAMETER / Swerve.GEAR_RATIO;
  }

  public Rotation2d getAngle(){
    //getPosition() returns StatusSignal<Angle> with base unit rotations
    double a = steer_sensor.getAbsolutePosition().getValueAsDouble();
    a = MathUtil.inputModulus(a, -0.5, 0.5);
    SmartDashboard.putNumber("out_angle", a);
    return Rotation2d.fromRotations(a);
  }

  public SwerveModuleVelocity getModuleState(){
    return new SwerveModuleVelocity(getSpeed(), getAngle());
  }

  public SwerveModulePosition getModulePosition(){
    SmartDashboard.putNumber("rotor_position", getPosition());
    return new SwerveModulePosition(getPosition(), getAngle());
  }

  public void setBrakeAngle(double angle) {
    brakeAngle = angle;
  }

  public void setModuleMode(boolean brake) {
    braked = brake;
    // if (brake) {
    //   drive_motor.setNeutralMode(NeutralModeValue.Brake);
    //   steer_motor.setNeutralMode(NeutralModeValue.Brake);
    // } else {
    //   drive_motor.setNeutralMode(NeutralModeValue.Coast);
    //   steer_motor.setNeutralMode(NeutralModeValue.Coast);
    // }
  } 

  /**
   * @param speed the speed to go at, in meters per second.
   */
  public void setSpeed(double speed){
    if (braked) {speed = 0;}
    SmartDashboard.putNumber("in_speed", speed / Swerve.WHEEL_DIAMETER);
    drive_motor.setControl(new VelocityVoltage(speed / Swerve.WHEEL_DIAMETER));

  }

  public void setAngle(Rotation2d angle){
    if (braked) {angle = Rotation2d.fromDegrees(brakeAngle);}
    SmartDashboard.putNumber("in_angle", angle.getRotations());
    SmartDashboard.putNumber("in_angle_degrees", angle.getDegrees());
    // this works fine
    // steer_motor.setVoltage(3);

    // System.out.println("steer port var" + steerPortVar);
    //if (steerPortVar == 3 || steerPortVar == 7) {
      // var request = new PositionVoltage(0).withSlot(0);
      // var angleInDegrees = Degrees.of(80);
      steer_motor.setControl(new PositionVoltage(angle.getRotations()));
    //}
    // steer_motor.setControl(0);
    // steer_motor.setControl(new PositionVoltage(0))

    String DashboardKey = "steerAbsPos" + steerPortVar;
    SmartDashboard.putString(DashboardKey, steer_sensor.getAbsolutePosition().toString());

    // String DashboardKey2 = "steerPos" + steerPortVar;
    // SmartDashboard.putString(DashboardKey2, steer_motor.getPosition().toString());

    String DashboardKey3 = "steerVel" + steerPortVar;
    SmartDashboard.putString(DashboardKey3, steer_motor.getDutyCycle().toString());

  }

}
