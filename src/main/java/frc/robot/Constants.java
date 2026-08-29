// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int reverseControls = 1;
  }

  public static class Robot {
    public static final double LOOP_TIME_SECONDS = 0.02;
  }

  public static class ControllerButtonConstants {
    public static final int Y = 4;
    public static final int B = 2;
    public static final int A = 1;
    public static final int X = 3;
    public static final int RB = 6;
    public static final int LB = 5;
  }

public static class IntakeConstants{

  public static final double EXTENDER_GEAR_RATIO = 40;
  public static final double POSITION_CONVERSION = (Math.PI * 0.04572)/EXTENDER_GEAR_RATIO;
  public static final int EXTENDER_CURRENT_LIMIT = 30;
  public static final double EXTENSION_MAX = -26; // 4.5 prongs left at -48
  public static final double EXTENSION_MIN = -7; // ignore lololol
  public static final double EXTENDER_VOLTAGE = 8; //8
  ;

  public static final double INTAKE_VOLTAGE = -10; // -9

  public static final int AGITATOR_CURRENT_LIMIT = 30;
  public static final int AGITATOR_RPM = 4500;
}

public static class ShooterConstants{
  public static final double GATE_GEAR_RATIO = 5;
  public static final double GATE_RPM = -5100;
  public static final int GATE_CURRENT_LIMIT = 40;
  public static final String LIMELIGHT_NAME = "limelight";
  public static final double STATIC_DISTANCE_INCHES = 90;

  public static class BottomMotorConfigs {
    public static final double kP = 0.2;
    public static final double kI = 0;
    public static final double kD = 0.002;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kG = 0;
  }

  public static class TopMotorConfigs {
    public static final double kP = .7;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kG = 0;
  }
}

  public static final class Port {
    // CAN IDs 
    public static final int STEER_MOTOR_FRONT_LEFT = 1;
    public static final int STEER_MOTOR_FRONT_RIGHT = 3;
    public static final int STEER_MOTOR_BACK_LEFT = 4;
    public static final int STEER_MOTOR_BACK_RIGHT = 2;

    public static final int DRIVE_MOTOR_FRONT_LEFT = 8;
    public static final int DRIVE_MOTOR_FRONT_RIGHT = 7;
    public static final int DRIVE_MOTOR_BACK_LEFT = 10;
    public static final int DRIVE_MOTOR_BACK_RIGHT = 6;

    public static final int FRONT_LEFT_CODER = 7;
    public static final int FRONT_RIGHT_CODER = 3;
    public static final int BACK_LEFT_CODER = 1;
    public static final int BACK_RIGHT_CODER = 5;

    // for da intake
    public static final int INTAKE_EXTENDER_MOTOR = 27; // idk we get to it when we get to it
    public static final int INTAKE_MOTOR = 30;
    public static final int AGITATOR_MOTOR = 26;

    // for da shooter
    public static final int SHOOTER_GATE_MOTOR = 16;
    public static final int SHOOTER_TOP_ROLLER_MOTOR = 13;
    public static final int NEAR_SHOOTER_BOTTOM_ROLLER_MOTOR = 5;
    public static final int FAR_SHOOTER_BOTTOM_ROLLER_MOTOR = 18;
  }

  public static class IO {
    public static final int MAIN_PORT = 0;
    public static final int COPILOT_PORT = 1; // reset to 1
    public static final int LEFT_BOARD_PORT = 2;
    public static final int RIGHT_BOARD_PORT = 3;

    public static final class Board {
      public static final class Left {
        /* 
        these are the constants i stole from motif
        im only leaving them here for future reference when we need to map buttons
        because i have the memory of a sad goldfish

        public static final int SHOOT = 1;
        public static final int AIM = 2;

        public static final int AMP = 3;

        public static final int SHOOT_OVERRIDE = 5;
        public static final int REV = 6;

        public static final int LEFT_CLIMB = 0;
        */
      }
      public static final class Right {
        /*
        public static final int UP_DOWN_INTAKE = 1;
        public static final int OVERRIDE = 4;
        public static final int OUTTAKE = 9;
        public static final int INTAKE = 8;

        public static final int INC_OR_DEC_TAR = 3;
        public static final int MOVE_TAR = 6;

        public static final int RIGHT_CLIMB = 0;
        */
      }
    }
  }

  public static class Swerve {
    public static final double WHEEL_DIAMETER = 3.78 * 0.0254 * Math.PI;
    public static final double GEAR_RATIO = 6.75;
    public static final double OFFSET = 12.23 * 0.0254;

    public static final double MAX_SPEED = 5;
    public static final double DRIVE_SPEED = 0.9;
    public static final double TURN_SPEED = 1;

    public static class DriveMotorConfigs {
      public static final double kP = .1; //0.1
      public static final double kI = 0;
      public static final double kD = 0.1;//0.1;
      public static final double kS = 0.13;//0.13;
      public static final double kV = 0.7;//0.7; 
      public static final double kA = 0;
      public static final double kG = 0;
    }

    public static class SteerMotorConfigs {
      public static final double kP = 15;//0.5;
      public static final double kI = 0;
      public static final double kD = 0.75;
      public static final double kS = 0; //doesnt work?
      public static final double kV = 0;
      public static final double kA = 0;
      public static final double kG = 0;
    }
  }

  public static class Auto {
    public static final double translation_kP = 0.7d;
    public static final double translation_kI = 0d;
    public static final double translation_kD = 0d;
    
    public static final double rotation_kP = 1.5d;
    public static final double rotation_kI = 0d;
    public static final double rotation_kD = 0d;
  }

  public static class FieldConstants {

    public static class HubMeasurements {
      //inches
      public static final Translation2d BLUEHUB_POSE = new Translation2d(Units.inchesToMeters(182.11),Units.inchesToMeters(158.84));
      public static final Translation2d REDHUB_POSE = new Translation2d(Units.inchesToMeters(651.22 - 182.11),Units.inchesToMeters(158.84));
      public static final double HUB_WIDTH = 47;
    }
  }
}
