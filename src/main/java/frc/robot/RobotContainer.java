// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.Seconds;

import java.io.IOException;
import java.io.SequenceInputStream;

import org.json.simple.parser.ParseException;
import org.opencv.core.Mat;

//import frc.robot.Constants.OperatorConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;

import frc.robot.Subsystems.Agitator.Agitator;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.DriveTrain.DriveTrainRealIO;
import frc.robot.Subsystems.DriveTrain.DriveTrainSimIO;
// import frc.robot.SubsystemManager;
import frc.robot.commands.Autos;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.kinematics.ChassisSpeeds;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.driverstation.DriverStation;
//import frc.robot.commands.ExampleCommand;
//import frc.robot.commands.IntakeCommand.ToggleIntake;
import org.wpilib.driverstation.Joystick;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.FunctionalCommand;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.RepeatCommand;
import org.wpilib.command2.RunCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.StartEndCommand;
import org.wpilib.command2.WaitCommand;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.command2.button.JoystickButton;
import org.wpilib.command2.button.Trigger;
import org.wpilib.command2.button.JoystickButton;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.Intake.direction;
import frc.robot.Subsystems.Intake.Intake.intakeDirection;
import frc.robot.Subsystems.Intake.Intake.intakeMode;
import frc.robot.Subsystems.Shooter.Shooter;
//import frc.robot.Subsystems.Vision.LimelightIO;
//import frc.robot.Subsystems.Vision.VisionSubsystem;
//import frc.robot.Subsystems.Vision.Vision;
//import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ControllerButtonConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.events.PointTowardsZoneTrigger;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private boolean testingShooter = false;
  private boolean facingHub = false;
  private boolean shooting = false;
  private boolean lockSwerve = false;
  private boolean staticAuto = false;
  
  //private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Integer> autoChooser2 = new SendableChooser<>();
  public record JoystickInputs(double drive_x, double drive_y, double drive_a) {}
  //check is need joystick inputs or not
  private Joystick main_stick = new Joystick(Constants.IO.MAIN_PORT);
  private Joystick second_stick = new Joystick(Constants.IO.COPILOT_PORT);
  private Joystick test_stick = new Joystick(3);
  //

  public final DriveTrain m_drive = Robot.isReal() ? new DriveTrainRealIO() : new DriveTrainSimIO();
  public final Intake m_intake = new Intake();
  //public final AgitatorTest m_agitatorTest = new AgitatorTest();
  public final Shooter m_shooter = new Shooter();
  // public final LimelightIO m_limelightio = new LimelightIO(m_drive.blueAlliance);
  // public final VisionSubsystem m_vision = new VisionSubsystem(m_limelightio);
  public final Agitator m_agitator = new Agitator();

  private boolean runningSequence1 = false;
  private boolean runningSequence2 = false;
  private boolean autoAlignHub = false;

  private final String redRightBumpAutoName = "RED Bump Intake Right";
  private final String blueRightBumpAutoName = "Bump Intake Right";
  private final String redLeftBumpAutoName = "RED Bump Intake Left";
  private final String blueLeftBumpAutoName = "Bump Intake Left";

  PathPlannerAuto redRightBumpAuto = new PathPlannerAuto(redRightBumpAutoName);
  PathPlannerAuto blueRightBumpAuto = new PathPlannerAuto(blueRightBumpAutoName);
  PathPlannerAuto redLeftBumpAuto = new PathPlannerAuto(redLeftBumpAutoName);
  PathPlannerAuto blueLeftBumpAuto = new PathPlannerAuto(blueLeftBumpAutoName);
  
  PathPlannerAuto[] autos = {
    redRightBumpAuto,
    blueRightBumpAuto,
    redLeftBumpAuto,
    blueLeftBumpAuto
  };

  //The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drive.resetGyroAngle();

    // SendableChooser<String> leftRightAuto = new SendableChooser<>();
    // leftRightAuto.addOption("Left", null);
    // leftRightAuto.addOption("Right", null);

    //autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser2.setDefaultOption("Red Right Bump", 0);
    autoChooser2.addOption("Blue Right Bump", 1);
    autoChooser2.addOption("Red Left Bump", 2);
    autoChooser2.addOption("Blue Left Bump", 3);
    autoChooser2.addOption("Center Shoot", 4);

    //autoChooser2.addOption("Start Hub", "RED Bump Intake Right");

    //SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Auto Selection", autoChooser2);
    // SmartDashboard.putData("Auto Side", leftRightAuto);

    InstantCommand disableVisionMeasurement = new InstantCommand(() -> {
      m_drive.autoVisionMeasurement = false;
    });

    InstantCommand enableVisionMeasurement = new InstantCommand(() -> {
      m_drive.autoVisionMeasurement = true;
    });

    // RunCommand alignHub = new RunCommand(() -> {
    //   if (DriverStation.isAutonomous()) {
    //     double angle_radiansPerSecond;
    //     angle_radiansPerSecond = m_drive.getTurnToHub(); //* (m_limelightio.blueAlliance == true ? 1 : -1);

    //     m_drive.setSwerveDrive(
    //     0,0,
    //     angle_radiansPerSecond
    //     );

    //     SmartDashboard.putNumber("auto angle", angle_radiansPerSecond);
    //   }
    // });

    NamedCommands.registerCommand("Enable Vision", enableVisionMeasurement);
    NamedCommands.registerCommand("Disable Vision", disableVisionMeasurement);

    SequentialCommandGroup intakeInOut = new SequentialCommandGroup(
      new InstantCommand(() -> {
        m_intake.extendArm();
        m_intake.setModes(direction.EXTENDING, intakeMode.AUTOMATIC);
      }),
      new WaitCommand(0.25),
      new InstantCommand(() -> {
        m_intake.retractArm();
        m_intake.setModes(direction.RETRACTING, intakeMode.AUTOMATIC);
      }),
      new WaitCommand(0.25)
    );

    NamedCommands.registerCommand("Intake In Out", intakeInOut);

    // NamedCommands.registerCommand("Intake 10x", new SequentialCommandGroup(
    //   intakeInOut,
    //   intakeInOut,
    //   intakeInOut,
    //   intakeInOut,
    //   intakeInOut,
    //   intakeInOut,
    //   intakeInOut,
    //   intakeInOut,
    //   intakeInOut,
    //   intakeInOut
    // ));

    NamedCommands.registerCommand("Align Hub", new InstantCommand(() -> {autoAlignHub = true;}));
    NamedCommands.registerCommand("Cancel Align", new InstantCommand(() -> {autoAlignHub = false;}));

    NamedCommands.registerCommand("Hub Shooter", new InstantCommand(() -> {
      m_shooter.setModes(true, false, true, true);
    }));

    NamedCommands.registerCommand("Run Shooter",new InstantCommand(() -> {
      m_shooter.setModes(true, false, true, false);
    }
    ));
    NamedCommands.registerCommand("Stop Shooter",new InstantCommand(() -> {
      m_shooter.enabled = false;
      }
    ));

    NamedCommands.registerCommand("Run Agitator",new InstantCommand(() -> {
      m_agitator.runAgitation(1);
      m_agitator.runGate(1);}));
    NamedCommands.registerCommand("Stop Agitator",new InstantCommand(() -> {
      m_agitator.stopGate();
      m_agitator.stopAgitation();}));

    //setting up pathplanner commands

    new EventTrigger("Extend Intake").onTrue(new InstantCommand(() -> {
      m_intake.setModes(direction.EXTENDING, intakeMode.AUTOMATIC);
      m_intake.extendArm();
    }
    ));
    new EventTrigger("Retract Intake").onTrue(new InstantCommand(() -> {
      m_intake.setModes(direction.RETRACTING, intakeMode.AUTOMATIC);
      m_intake.retractArm();
    }
    ));

    // Configure the trigger bindings
    configureBindings();
  }

  public void autoAlignSwerve() {
    if (DriverStation.isAutonomous() && autoAlignHub) {
        double angle_radiansPerSecond;
        angle_radiansPerSecond = m_drive.getTurnToHub(); //* (m_limelightio.blueAlliance == true ? 1 : -1);

        m_drive.setSwerveDrive(
        0,0,
        angle_radiansPerSecond
        );

        SmartDashboard.putNumber("auto angle", angle_radiansPerSecond);
      }
  }

  public void updateSwerve() {
    if (!DriverStation.isAutonomous() && !lockSwerve){
    double driveMult = 1.25; //change this constant to change the drive speed.
    double rotationMult = 1.5; //change this constant to change the turn speed.
    
    double mult = m_shooter.enabled ? 0.2 : driveMult;
    //double degreeDifference = getHubDegreeDiff();
    //facingHub = (degreeDifference == 0);
    SmartDashboard.putBoolean("tv",LimelightHelpers.getTV(ShooterConstants.LIMELIGHT_NAME));

    double x_metersPerSecond = (Math.abs(main_stick.getRawAxis(1)) < 0.1) ? 0 : 2.7 * -main_stick.getRawAxis(1);
    SmartDashboard.putNumber("x_mps", x_metersPerSecond);

    double y_metersPerSecond = (Math.abs(main_stick.getRawAxis(0)) < 0.1) ? 0 : 2.7 * -main_stick.getRawAxis(0);

    double angle_radiansPerSecond;

    // if pressing button 6 then we align to the hub
    if ((main_stick.getRawAxis(2) >= 0.2)) {
      angle_radiansPerSecond = m_drive.getTurnToHub(); //* (m_limelightio.blueAlliance == true ? 1 : -1);
      SmartDashboard.putBoolean("align attemp", true);
    } else {  
      angle_radiansPerSecond = (Math.abs(main_stick.getRawAxis(4)) < 0.2) ? 0 : -3 * Math.signum(main_stick.getRawAxis(4))
      * Math.pow(main_stick.getRawAxis(4), 2) * rotationMult;
      SmartDashboard.putBoolean("align attemp", false);
    }
    //negative turn values go right, positive go left
    
    //SmartDashboard.putNumber("axis_0", leftStickLeftRight);
    //SmartDashboard.putNumber("angle", angle_radiansPerSecond);

    int forwards = (m_drive.blueAlliance ? 1 : -1);
    m_drive.setSwerveDrive(
      x_metersPerSecond * mult * forwards, 
      y_metersPerSecond * mult * forwards, 
      angle_radiansPerSecond
      );
    }
  }

  public void updateShooterDistance() {
    m_shooter.distanceFromTargetInches = m_drive.getDistanceFromHub();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * org.wpilib.command2.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link org.wpilib.command2.button.CommandPS4Controller
   * PS4} controllers or {@link org.wpilib.command2.button.CommandJoystick Flight
   * joysticks}
   */

  private void configureBindings() {
    //MAIN STICK -------------------------
    new JoystickButton(main_stick, 8).onTrue(
      new InstantCommand(() -> {
        m_drive.refreshAlliance();
        m_drive.resetGyroAngle();
      })
    );

    //stop intake
    new JoystickButton(main_stick, 3).onTrue(
      new InstantCommand(() -> {
        m_intake.setIntakeDirection(intakeDirection.OFF);
        m_intake.setModes(direction.IGNORE, intakeMode.MANUAL);
      })
    );

    //extend hopper
    new JoystickButton(main_stick, 5).onTrue(
      new InstantCommand(() -> {
        m_intake.extendArm();
        m_intake.setModes(direction.EXTENDING, intakeMode.AUTOMATIC);
        m_intake.setIntakeDirection(intakeDirection.IN);
      })
    );

    //retract hopper
    new JoystickButton(main_stick, 4).onTrue(
      new InstantCommand(() -> {
        m_intake.retractArm();
        m_intake.setModes(direction.RETRACTING, intakeMode.AUTOMATIC);
        m_intake.setIntakeDirection(intakeDirection.OUT);
      })
    );

    //lock wheels
    new Trigger(() -> main_stick.getRawButton(6)).whileTrue(
      new RunCommand(() -> {
        m_drive.setModuleMode(true);
        lockSwerve = true;
      }).finallyDo(
        () -> {
          m_drive.setModuleMode(false);
          lockSwerve = false;
        })
      );

    //SECOND STICK -------------------------
    //sequences fml

    //creating our sequence commands
    InstantCommand extendHopperCommand = new InstantCommand(() -> {
              if (runningSequence1) {
                  m_intake.extendArm();
                  m_intake.setModes(direction.EXTENDING, intakeMode.AUTOMATIC);
                }
            });
    InstantCommand retractHopperCommand = new InstantCommand(() -> {
              if (runningSequence1) {
                  m_intake.retractArm();
                  m_intake.setModes(direction.RETRACTING, intakeMode.AUTOMATIC);
                }
            });

    double waitInterval = 0.25;
    SequentialCommandGroup waitIntakeOuttakeCommand = new SequentialCommandGroup(extendHopperCommand, new WaitCommand(waitInterval), retractHopperCommand, new WaitCommand(waitInterval));
    RepeatCommand repeatIntakeOuttakeCommand = new RepeatCommand(waitIntakeOuttakeCommand);

    //after waiting for .4 seconds, basically do the second half of sequence 1
    var spamIntakeSequence = new WaitCommand(0.3).finallyDo(() -> {
          if (runningSequence1) {
            m_agitator.runAgitation(1);
            m_agitator.runGate(1);
            //turn on gate and agitator
            repeatIntakeOuttakeCommand.schedule();
          }
        });
    
    //same explanation as above basically
    var slowRetractSequence = new WaitCommand(0.4).finallyDo(() -> {
          if (runningSequence2) {
            m_agitator.runAgitation(1);
            m_agitator.runGate(1);
            //turn on gate and agitator
            m_intake.setHopperSpeed(0.3);
            m_intake.setModes(direction.RETRACTING, intakeMode.AUTOMATIC);
            m_intake.retractArm();
          }
        });
      //--------------------------------------

    //sequence 1
    //run shooter and such + intake goes in and out
    new Trigger(() -> (second_stick.getRawButton(6) || second_stick.getRawAxis(3) >= 0.2)).whileTrue(
      new StartEndCommand(() -> {
        //only run if were not running the other sequence and were not already running this sequence
        if (!runningSequence2 && !runningSequence1) {
        runningSequence1 = true;
        //start shooter first
        boolean autoDist = (second_stick.getRawButton(6));
        m_shooter.setModes(true, false, autoDist, false);
          // 90 is where we usually are, so a good number to rev up to
        //give shooter time to rev up
        m_intake.setModes(direction.IGNORE, intakeMode.MANUAL);
        m_intake.setIntakeDirection(intakeDirection.OFF);
        //turn intake off
        //have intake go in and out with shooter
        spamIntakeSequence.schedule();
      } else {
        //if were already running a sequence then just return
        return;
      }
    },
      () -> {
        if (runningSequence1) {
          //turn everything back off
          runningSequence1 = false;
          m_agitator.stopAgitation();
          m_agitator.stopGate();
          m_shooter.enabled = false;
          m_intake.setModes(direction.IGNORE, intakeMode.AUTOMATIC);
          repeatIntakeOuttakeCommand.cancel();
        }
      }));

      new Trigger(() -> second_stick.getRawButton(6) || second_stick.getRawAxis(3) >= 0.2).whileTrue(
        new InstantCommand(() -> {
          if (runningSequence1 || runningSequence2) {
            if (second_stick.getRawAxis(3) >= 0.2) {
            m_shooter.autoDistance = false;
          } else {m_shooter.autoDistance = true;}
          }
        })
      );

        //stop everything once we stop holding down the butto
    //sequence 2
    new Trigger(() -> second_stick.getRawAxis(2) >= 0.2).whileTrue(
      new StartEndCommand(() -> {
        //only run if were not running the other sequence and were not already running this sequence
        if (!runningSequence2 && !runningSequence1) {
        //start shooter first
        runningSequence2 = true;
        m_shooter.setModes(true,false,true,false);
        //give shooter time to rev up
        //slowly retract intake while shooting and such
        slowRetractSequence.schedule();
      } else {return;} //return if were already running a sequence
      },
      () -> {
        //reset back to normal
        runningSequence2 = false;
        m_intake.setHopperSpeed(1);
        m_agitator.stopAgitation();
        m_agitator.stopGate();
        shooting = m_shooter.enabled = false;
        slowRetractSequence.cancel();
      }
    ));

    //extend/retract hopper
    //new control
    new Trigger(() -> second_stick.getPOV() == 0).onTrue(
      new InstantCommand(() -> {
        m_intake.extendArm();
        m_intake.setModes(direction.EXTENDING, intakeMode.MANUAL);
        m_intake.setIntakeDirection(intakeDirection.OFF);
    }));

    new Trigger(() -> second_stick.getPOV() == 180).onTrue(
      new InstantCommand(() -> {
        m_intake.retractArm();
        m_intake.setModes(direction.RETRACTING, intakeMode.AUTOMATIC);
    }));

    //old control
    // new JoystickButton(second_stick, 1).onTrue(
    //   new InstantCommand(() -> {
    //     if (m_intake.currentDirection == direction.RETRACTING) {
    //     m_intake.extendArm();
    //     m_intake.setModes(direction.EXTENDING, intakeMode.AUTOMATIC);
    //     m_intake.setIntakeDirection(intakeDirection.IN);
    //     } else {
    //     m_intake.retractArm();
    //     m_intake.setModes(direction.RETRACTING, intakeMode.AUTOMATIC);
    //     }
    //   })
    // );

    //running intake
    //new control
    // new JoystickButton(second_stick, 3).onTrue(
    //   new InstantCommand(() -> {
    //     intakeDirection iDirection = intakeDirection.IN;
    //     if (second_stick.getRawButton(5)) {iDirection = intakeDirection.OUT;}
    //     if (m_intake.currentIntakeDirection != intakeDirection.OFF && m_intake.currentIntakeDirection == iDirection) {
    //       iDirection = intakeDirection.OFF;
    //     }
    //     m_intake.setModes(direction.IGNORE, intakeMode.MANUAL);
    //     m_intake.setIntakeDirection(iDirection);
    //   })
    // );

    //old control
    new Trigger(() -> second_stick.getRawButton(3)).whileTrue(
      new RunCommand(() -> {
        boolean forward = !second_stick.getRawButton(5);
        m_intake.setModes(direction.IGNORE, intakeMode.MANUAL);
        if (forward) {
          m_intake.setIntakeDirection(intakeDirection.IN);
        } else {m_intake.setIntakeDirection(intakeDirection.OUT);}
      }).finallyDo(() -> {
        m_intake.setIntakeDirection(intakeDirection.OFF);
      })
    );

    // agitator + gate forward/reverse
    //new control
    // new JoystickButton(second_stick, 2).onTrue(
    //   new InstantCommand(() -> {
    //     int mult = 1;
    //     if (second_stick.getRawButton(5)) {mult = -1;}
    //     if (m_agitator.lastMult == mult) {mult = 0;}
    //     m_agitator.runAgitation(mult);
    //     m_agitator.runGate(mult);
    //   }) 
    // );

    //old control
    new Trigger(() -> (second_stick.getRawButton(2))).whileTrue(
      new RunCommand(() -> {
        int mult = second_stick.getRawButton(5) ? -1 : 1;
        m_agitator.runAgitation(mult);
        m_agitator.runGate(mult);
      }
      ).finallyDo(() -> {
        m_agitator.stopGate();
        m_agitator.stopAgitation();
      })
    );
   
    // shooter bindings

    //old control
    new Trigger(() -> (second_stick.getRawButton(4)) || second_stick.getRawButton(1)).whileTrue(
        new RunCommand(() -> {
          boolean reversed = second_stick.getRawButton(5);
          boolean againstHub = second_stick.getRawButton(1);
          m_shooter.setModes(true, reversed, true, againstHub);
        })
          .finallyDo(() -> {
            m_shooter.enabled = false;
          })
      );

      // new Trigger(() -> second_stick.getRawButton(3)).whileTrue(
      //   new RunCommand(() -> m_shooter.basketballin())
      //     .finallyDo(m_shooter::stopMotors)
      // );
      //remnant from when we shot into the hoops lolololol

    // shooter test bindings
    new Trigger(() -> ((test_stick.getRawButton(4) || test_stick.getRawButton(1)) && 
      (test_stick.getRawButton(5) || test_stick.getRawButton(6)))).onTrue(
      new InstantCommand(() -> {
        double base = 0.1 * (test_stick.getRawButton(5) ? -1 : 1);
        double mult = test_stick.getRawAxis(3) >= 0.2 ? 10 : 1;
        if (test_stick.getRawButton(4)) {
        m_shooter.changeTopRollerRPS(base * mult);
        } else {
          m_shooter.changeBottomRollerRPS(base * mult);
        }
      })
    );

    new Trigger(() -> test_stick.getRawAxis(2)>= 0.2).whileTrue(
      new RunCommand(() -> {
        m_shooter.setModes(true, false, false, false);
        m_agitator.runAgitation(1);
        m_agitator.runGate(1);
      }).finallyDo(
        () -> {
          m_shooter.enabled = false;
          m_agitator.stopAgitation();
          m_agitator.stopGate();
        })
    );
  }

  public Command getAutonomousCommand() {
    //m_drive.refreshAlliance();
    m_drive.resetGyroAngle();
    int autoNum = autoChooser2.getSelected();

    if (autoNum == 4) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        m_shooter.setModes(true, false, true, false);
      }),
      new WaitCommand(0.5),
      new InstantCommand(() -> {
        m_agitator.runAgitation(1);
        m_agitator.runGate(1);
      })
    );
    } else {
      PathPlannerAuto m_auto = autos[autoNum];
      return m_auto;
    }
  }
}
