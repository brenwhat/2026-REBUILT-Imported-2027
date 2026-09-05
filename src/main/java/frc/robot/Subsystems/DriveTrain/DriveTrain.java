// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DriveTrain; //Accidentally changed the folder name to be uppercase this year, oh well :P

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.PathPlannerLogging;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.estimator.PoseEstimator;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.util.sendable.Sendable;
import org.wpilib.util.sendable.SendableBuilder;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchState;
import org.wpilib.hardware.accelerometer.ADXL345_I2C.AllAxes;
import org.wpilib.driverstation.Alliance;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.FieldObject2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.FunctionalCommand;
import org.wpilib.command2.StartEndCommand;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FieldConstants.HubMeasurements;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.RobotUtils;
//import frc.robot.Subsystems.Vision.LimelightIO;
//import frc.robot.Subsystems.Vision.VisionSubsystem;

public abstract class DriveTrain extends SubsystemBase {

  /** The drive train's SwerveModule objects. */
  public SwerveModule[] modules;
  /** @hidden */
  public SwerveDriveKinematics kinematics;
  /** The desired module states. */
  public SwerveModuleVelocity[] module_states;

  /** Module positions for SwerveDrivePoseEstimator. */
  public SwerveModulePosition[] module_positions;
  /** Pose estimator. */
  public SwerveDrivePoseEstimator pose_estimator;
  /** Odometry-based 2d pose. */
  public Pose2d odom_pose;
  //public LimelightIO limelightInst;
  /** State publisher for AdvantageScope. */
  protected StructArrayPublisher<SwerveModuleVelocity> adv_real_states_pub, adv_target_states_pub;
  /** State publisher for AdvantageScope. */
  protected StructPublisher<Rotation2d> adv_gyro_pub;
  private Translation2d allianceHub;
  private double orientationYaw;
  public boolean blueAlliance;
  public boolean autoVisionMeasurement = false;

  public Orchestra orchestra;

  private Field2d field = new Field2d();
  private final FieldObject2d visionPoseEstimate = field.getObject("Vision Pose");
  private final FieldObject2d odomPoseEstimate = field.getObject("Odometry Pose");
  private final FieldObject2d pathplannerPath = field.getObject("PathPlanner Path");
  private final FieldObject2d pathplannerTarget = field.getObject("PathPlanner Target");
  private final FieldObject2d pathplannerPose = field.getObject("PathPlanner Pose");


  public SwerveModuleVelocity[] tempStates;
  private PIDController alignmentPID = new PIDController(0.1, 0, 0.004);
  //public VisionSubsystem visionSub;

  public DriveTrain() {
    kinematics = new SwerveDriveKinematics(
        new Translation2d(Constants.Swerve.OFFSET, Constants.Swerve.OFFSET), // front left
        new Translation2d(Constants.Swerve.OFFSET, -Constants.Swerve.OFFSET), // front right
        new Translation2d(-Constants.Swerve.OFFSET, Constants.Swerve.OFFSET), // back left
        new Translation2d(-Constants.Swerve.OFFSET, -Constants.Swerve.OFFSET) // back right
    );

    orchestra = new Orchestra();
    orchestra.loadMusic("Music/output.chrp");
    SmartDashboard.putData("Field",field);

    tempStates = new SwerveModuleVelocity[]{
        new SwerveModuleVelocity(),
        new SwerveModuleVelocity(),
        new SwerveModuleVelocity(),
        new SwerveModuleVelocity(),
    };
  
    modules = new SwerveModule[4];
    modules[0] = initializeModule(Constants.Port.DRIVE_MOTOR_FRONT_LEFT, Constants.Port.STEER_MOTOR_FRONT_LEFT,
        Constants.Port.FRONT_LEFT_CODER);
    modules[1] = initializeModule(Constants.Port.DRIVE_MOTOR_FRONT_RIGHT, Constants.Port.STEER_MOTOR_FRONT_RIGHT,
        Constants.Port.FRONT_RIGHT_CODER);
    modules[2] = initializeModule(Constants.Port.DRIVE_MOTOR_BACK_LEFT, Constants.Port.STEER_MOTOR_BACK_LEFT,
        Constants.Port.BACK_LEFT_CODER);
    modules[3] = initializeModule(Constants.Port.DRIVE_MOTOR_BACK_RIGHT, Constants.Port.STEER_MOTOR_BACK_RIGHT,
        Constants.Port.BACK_RIGHT_CODER);

    module_states = new SwerveModuleVelocity[] {
        new SwerveModuleVelocity(),
        new SwerveModuleVelocity(),
        new SwerveModuleVelocity(),
        new SwerveModuleVelocity()
    };

    module_positions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
    };
    //visionSub = new VisionSubsystem(limelightInst);
    odom_pose = new Pose2d();
    resetGyroAngle();
    pose_estimator = new SwerveDrivePoseEstimator(
      kinematics, 
      getGyroAngle(), 
      module_positions, 
      odom_pose,
      VecBuilder.fill(0.1,0.1,0), // odometry trust
      VecBuilder.fill(0.5,0.5,9999999)); // vision trust;

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (IOException | ParseException e) {
      e.printStackTrace();
      return;
    }
    AutoBuilder.configure(
        pose_estimator::getEstimatedPosition,
        (pose) -> pose_estimator.resetPose(pose),
        () -> kinematics.toChassisVelocities(module_states),
        (speeds, feedforwards) -> setSwerveDrive(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(Constants.Auto.translation_kP, Constants.Auto.translation_kI,
                Constants.Auto.translation_kD),
            new PIDConstants(Constants.Auto.rotation_kP, Constants.Auto.rotation_kI, Constants.Auto.rotation_kD)),
        config,
        () -> {
          Optional<Alliance> alliance = MatchState.getAlliance();  // this method should exist
          if (alliance.isPresent()) {
            boolean red = alliance.get() == Alliance.RED;
            if (red) {
              //var transformedState = PathPlannerTrajectory.transformStateForAlliance(PathPlannerPath.getInitialState(), DriverStation.getAlliance());
              //pose_estimator.resetPose(new Pose2d(transformedState.poseMeters.getTranslation(), transformedState.holonomicRotation));
            }
            return red;
          } 
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

    pose_estimator.resetPose(
      new Pose2d(
        pose_estimator.getEstimatedPosition().getX(),
        pose_estimator.getEstimatedPosition().getY() + 4,
        pose_estimator.getEstimatedPosition().getRotation()
      )
    );
    refreshAlliance();
    setupDashboard();
    SmartDashboard.putString("Music", " ");
  }

  /**
   * Creates either a SwerveModuleRealIO or SwerveModuleSimIO object.
   * 
   * @param drive_port  port number of the drive motor
   * @param steer_port  port number of the steer motor
   * @param sensor_port port number of the module's CANcoder
   * @return The constructed SwerveModule object
   */

  protected abstract SwerveModule initializeModule(int drive_port, int steer_port, int sensor_port);

  /**
   * Calculates and sends inputs to swerve modules given field-relative speeds.
   * Calls setSwerveDrive(ChassisVelocities chassis_speeds)
   * 
   * @param x_metersPerSecond  X-axis speed in m/s. Forward is positive.
   * @param y_metersPerSecond  Y-axis speed in m/s. Right is positive.
   * @param a_radiansPerSecond Angular speed in rad/s. CCW is positive.
   */
  public void setModuleMode(boolean brake) {
    for (int i = 0; i <= 3; i++) {
      modules[i].setModuleMode(brake);
      modules[i].setBrakeAngle(i == 0 || i == 3 ? 45 : -45);
    }
  }

  public void setSwerveDrive(double x_metersPerSecond, double y_metersPerSecond, double a_radiansPerSecond) {
    // converts speeds from field's frame of reference to robot's frame of reference
    ChassisVelocities chassis_speeds = new ChassisVelocities(
        x_metersPerSecond,
        y_metersPerSecond,
        a_radiansPerSecond
    ).toRobotRelative(getGyroAngle());

    setSwerveDrive(chassis_speeds);
  }

  public void updTempStates(double newAngle){
    for(int i = 0; i < 4; i++){
      tempStates[i].velocity = 0;
      tempStates[i].angle = Rotation2d.fromDegrees(newAngle);
    };
  };

  /**
   * Calculates and sends inputs to swerve modules given robot-relative speeds.
   * 
   * @param chassis_speeds The desired robot-relative chassis speeds.
   */
  public void setAngle(double angle){
    // updTempStates(angle);
    SmartDashboard.putNumber("input angle", angle);
    // setModules(tempStates);
    for(int i = 0; i<4; i++){
      modules[i].setAngle(Rotation2d.fromDegrees(angle));
    }
    SmartDashboard.putNumber("in_angle from func", modules[0].getAngle().getDegrees());
  }

  public void setSwerveDrive(ChassisVelocities chassis_speeds) {
    // fix weird change over time shenanigans

    SmartDashboard.putNumber("in_x", chassis_speeds.vx);
    SmartDashboard.putNumber("in_y", chassis_speeds.vy);

    chassis_speeds.omega *= -1;

    chassis_speeds = discretize_chassis_speeds(chassis_speeds);

    SmartDashboard.putNumber("in_a", chassis_speeds.omega);

    module_states = kinematics.toSwerveModuleVelocities(chassis_speeds);

    // change target wheel directions if the wheel has to rotate more than 90*
    for (int i = 0; i < module_states.length; i++) {
      module_states[i].optimize(modules[i].getAngle());
    }

    // normalize wheel speeds of any are greater than max speed
    SwerveDriveKinematics.desaturateWheelVelocities(module_states, Constants.Swerve.MAX_SPEED);
    
    setModules(module_states);

    for (int i = 0; i < modules.length; i++) {
      module_positions[i] = modules[i].getModulePosition();
    }
  }

  /**
   * Sends calculated inputs to swerve modules.
   * 
   * @param module_states The desired module states.
   */

  public void setModules(SwerveModuleVelocity[] module_states) {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(module_states[i]); //utb moduleStates
    }
  }

  // Thanks to Team 4738 for modified discretization code
  /**
   * Accounts for drift while simultaneously translating and rotating by
   * discretizing.
   * 
   * @param speeds Desired chassis speeds.
   * @return Adjusted chassis speeds.
   */
  public ChassisVelocities discretize_chassis_speeds(ChassisVelocities speeds) {
    double dt = Constants.Robot.LOOP_TIME_SECONDS;
    // makes a Pose2d for the target delta over one time loop
    var desired_delta_pose = new Pose2d(
        speeds.vx * dt,
        speeds.vy * dt,
        new Rotation2d(speeds.omega * dt * 1) // tunable
    );
    // makes a Twist2d object that maps new pose to delta pose
    Transform2d twist = new Pose2d().minus(desired_delta_pose);  // note for later to check this

    return new ChassisVelocities((twist.getX() / dt), (twist.getY() / dt), (speeds.omega));
  }

  /**
   * Returns the measured swerve module positions for odometry.
   * 
   * @return The measured swerve module positions.
   */

  public SwerveModulePosition[] getModulePositions() {
    var positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getModulePosition();
    }
    return positions;
  }

  /**
   * Returns the measured swerve module states for telemetry.
   * 
   * @return The measured swerve module states.
   */

  public SwerveModuleVelocity[] getModuleStates() {
    var states = new SwerveModuleVelocity[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getModuleState();
    }
    return states;
  }

  /**
   * Gets either the measured yaw from the AHRS or the calculated angle from the
   * simulation.
   * Forward is 0, CCW is positive.
   * 
   * @return The robot yaw.
   */

  public abstract Rotation2d getGyroAngle();
  public abstract double getGyroRate();

  public abstract void resetGyroAngle();
  public abstract void refreshGyro();

  /**
   * One-time method to instantiate NT publishers for AdvantageScope and Elastic.
   */

  private void setupDashboard() {

    // instantiate network publishers for advantagescope
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable adv_swerve = inst.getTable("adv_swerve");
    adv_real_states_pub = adv_swerve.getStructArrayTopic("States", SwerveModuleVelocity.struct).publish();
    adv_target_states_pub = adv_swerve.getStructArrayTopic("Target States", SwerveModuleVelocity.struct).publish();
    adv_gyro_pub = adv_swerve.getStructTopic("Gyro", Rotation2d.struct).publish();

    // create swerve drive publishers for elastic dashboard (one time setup,
    // auto-call lambdas)
    SmartDashboard.putData("Swerve Target States", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> module_states[0].angle.getDegrees(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> module_states[0].velocity, null);

        builder.addDoubleProperty("Front Right Angle", () -> module_states[1].angle.getDegrees(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> module_states[1].velocity, null);

        builder.addDoubleProperty("Back Left Angle", () -> module_states[2].angle.getDegrees(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> module_states[2].velocity, null);

        builder.addDoubleProperty("Back Right Angle", () -> module_states[3].angle.getDegrees(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> module_states[3].velocity, null);

        builder.addDoubleProperty("Robot Angle", () -> getGyroAngle().getDegrees(), null);
      }
    });
    // same here
    SmartDashboard.putData("Swerve Real States", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> modules[0].getAngle().getDegrees(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> modules[0].getSpeed(), null);

        builder.addDoubleProperty("Front Right Angle", () -> modules[1].getAngle().getDegrees(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> modules[1].getSpeed(), null);

        builder.addDoubleProperty("Back Left Angle", () -> modules[2].getAngle().getDegrees(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> modules[2].getSpeed(), null);

        builder.addDoubleProperty("Back Right Angle", () -> modules[3].getAngle().getDegrees(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> modules[3].getSpeed(), null);

        builder.addDoubleProperty("Robot Angle", () -> getGyroAngle().getDegrees(), null);
      }
    });

  }

  /**
   * Publishes telemetry readings to AdvantageScope.
   */

  private void publishAdv() {
    adv_real_states_pub.set(getModuleStates());
    adv_target_states_pub.set(module_states);
    adv_gyro_pub.set(getGyroAngle());
  }

  public double getDistanceFromHub() {
    double distance = RobotUtils.hypot((pose_estimator.getEstimatedPosition().getX() - allianceHub.getX()), 
    pose_estimator.getEstimatedPosition().getY() - allianceHub.getY());
    return RobotUtils.metersToInches(distance);
  }

  public double getAngleDegreeOffsetFromHubCenter() {
    // double targetPoint = Math.toDegrees(Math.atan2(allianceHub.getY() - odom_pose.getY(), 
    //     allianceHub.getX() - odom_pose.getX()));
    Rotation2d targetAngle = allianceHub.minus(pose_estimator.getEstimatedPosition().getTranslation()).getAngle();
    double difference = targetAngle.minus(Rotation2d.fromDegrees(orientationYaw)).getDegrees();
    return (Math.abs(difference) < 1 ? 0 : difference);
  }

  public double getTurnToHub() {
    double degreeDifference = getAngleDegreeOffsetFromHubCenter();
    double rotationSpeed = alignmentPID.calculate(0, degreeDifference);
    return rotationSpeed;
  }

  public boolean refreshAlliance() {
    Optional<Alliance> alliance = MatchState.getAlliance();
    if (alliance.isPresent()) {
      blueAlliance = !(alliance.get() == Alliance.RED);
    } // else {blueAlliance = false;}
    if (blueAlliance) {
      allianceHub = HubMeasurements.BLUEHUB_POSE;
    } else {allianceHub = HubMeasurements.REDHUB_POSE;}
    refreshGyro();
    SmartDashboard.putBoolean("blueAlliance", blueAlliance);
    return blueAlliance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Aligned", getAngleDegreeOffsetFromHubCenter() == 0);
    SmartDashboard.putNumber("gyro", getGyroAngle().getDegrees());
    SmartDashboard.putNumber("pose_angle", odom_pose.getRotation().getDegrees());
    SmartDashboard.putBoolean("auto Vision", autoVisionMeasurement);
    for (SwerveModule module : modules) {
      module.update();
    }
    //femboy
    orientationYaw = getGyroAngle().getDegrees();
    //orientationYaw = odom_pose.getRotation().getDegrees();
    SmartDashboard.putNumber("poseEstimate_angle", orientationYaw);
    odom_pose = pose_estimator.update(Rotation2d.fromDegrees(orientationYaw), getModulePositions());
    //+ (blueAlliance == true ? 0 : 180); //maybe?
    //there is a chance we do not need to add 180 degrees for red alliance
    //by using the pose_estimator estimated positon, it may already know correctly withoud adjustment
    SmartDashboard.putNumber("orientationYaw", orientationYaw);
    SmartDashboard.putNumber("orientationYaw Radians", Math.toRadians(orientationYaw));
    LimelightHelpers.SetRobotOrientation(ShooterConstants.LIMELIGHT_NAME, orientationYaw, 0, 0, 0, 0, 0);
    double visionTrust = 0.5;
    
    boolean useMT2 = true;
    LimelightHelpers.PoseEstimate limelightMeasurement = 
      useMT2 ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ShooterConstants.LIMELIGHT_NAME)
      : LimelightHelpers.getBotPoseEstimate_wpiBlue(ShooterConstants.LIMELIGHT_NAME);

    //if (!(DriverStation.isAutonomous() && (autoVisionMeasurement == false))) {
    if ((Math.abs(getGyroRate()) < 360) && (limelightMeasurement.tagCount > 0)) {
      visionTrust += (0.5 * limelightMeasurement.avgTagDist);
      //if were not moving faster than 360 degrees/sec and we see tags 
      pose_estimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionTrust, visionTrust, 9999999));
      if(limelightMeasurement.pose != null){
        pose_estimator.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds
        );
      }
    //}
  }
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      pathplannerPath.setPoses(pose);
    });
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      pathplannerPath.setPoses(poses);
    });
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      pathplannerTarget.setPose(pose);
    });
    field.setRobotPose(pose_estimator.getEstimatedPosition());
    visionPoseEstimate.setPose(limelightMeasurement.pose);
    odomPoseEstimate.setPose(odom_pose);

    SmartDashboard.putNumber("poseX Inches", RobotUtils.metersToInches(pose_estimator.getEstimatedPosition().getX()));
    SmartDashboard.putNumber("poseY Inches", RobotUtils.metersToInches(pose_estimator.getEstimatedPosition().getY()));
    SmartDashboard.putNumber("botHub-angleDiff", getAngleDegreeOffsetFromHubCenter());
    SmartDashboard.putNumber("botHub-distInches", getDistanceFromHub());
    publishAdv();
  }


  public Command musicCommand(String filename, int tracks) {
    return new FunctionalCommand(
      () -> {
        int t = 0;
        for (SwerveModule module : modules) {
            orchestra.addInstrument(((SwerveModuleRealIO)module).drive_motor, t++ % tracks); //increment track number by 1; reset when max tracks reached
            orchestra.addInstrument(((SwerveModuleRealIO)module).steer_motor, t++ % tracks);
        }
        orchestra.loadMusic("Music/" + filename + ".chrp");
        orchestra.play();
        SmartDashboard.putString("Music", filename);},
      () -> {},
      canceled -> {
        orchestra.stop();
        SmartDashboard.putString("Music", " ");},
      () -> !orchestra.isPlaying()
    ).ignoringDisable(true);
  }

  public Command musicCommand(String filename) {
    return musicCommand(filename, 8);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
