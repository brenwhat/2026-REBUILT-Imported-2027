package frc.robot.Subsystems.Shooter;

import javax.xml.crypto.dsig.keyinfo.RetrievalMethod;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.motor.Talon;

import frc.robot.RobotUtils;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Port;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.BottomMotorConfigs;
import frc.robot.Constants.ShooterConstants.TopMotorConfigs;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {
    private boolean testingShooter = false;
    private double testBRRPS = -25;
    private double testTRRPS = 104;

    private double inchesFromHub = 120; // only use for testing shooter
    private double bottomRollerFF = -0; // i think this is the one we should keep constant
    private double bottomRollerTargetRPS = -25; //-22.5
    private double topRollerTargetRPS = 85;

    // TODO: get canbus number for the talons
    private CANBus canBus0 = new CANBus();
    private TalonFX farBottomRollerMotor = new TalonFX(Port.FAR_SHOOTER_BOTTOM_ROLLER_MOTOR, canBus0);
    private TalonFX nearBottomRollerMotor = new TalonFX(Port.NEAR_SHOOTER_BOTTOM_ROLLER_MOTOR, canBus0);
    private TalonFX topRollerMotor = new TalonFX(Port.SHOOTER_TOP_ROLLER_MOTOR, canBus0);

    public boolean enabled = false;
    public boolean autoDistance = true;
    public boolean reversed = false;
    public boolean againstHub = false;
    public double staticDistanceInches = ShooterConstants.STATIC_DISTANCE_INCHES;

    private VelocityVoltage bottomMotor_request = new VelocityVoltage(0).withSlot(0);
    private VelocityVoltage topMotor_request = new VelocityVoltage(0).withSlot(0);

    public double distanceFromTargetInches = 0;

    public Shooter() {
        TalonFXConfiguration topMotor_configs = new TalonFXConfiguration();
        TalonFXConfiguration bottomMotor_configs = new TalonFXConfiguration();

        topMotor_configs.Slot0 = new Slot0Configs()
        .withKP(TopMotorConfigs.kP)
        .withKI(TopMotorConfigs.kI)
        .withKD(TopMotorConfigs.kD)
        .withKS(TopMotorConfigs.kS)
        .withKV(TopMotorConfigs.kV)
        .withKA(TopMotorConfigs.kA)
        .withKG(TopMotorConfigs.kG);

        bottomMotor_configs.Slot0 = new Slot0Configs()
        .withKP(BottomMotorConfigs.kP)
        .withKI(BottomMotorConfigs.kI)
        .withKD(BottomMotorConfigs.kD)
        .withKS(BottomMotorConfigs.kS)
        .withKV(BottomMotorConfigs.kV)
        .withKA(BottomMotorConfigs.kA)
        .withKG(BottomMotorConfigs.kG);
        
        farBottomRollerMotor.getConfigurator().apply(bottomMotor_configs);
        nearBottomRollerMotor.getConfigurator().apply(bottomMotor_configs);
        topRollerMotor.getConfigurator().apply(topMotor_configs);

        farBottomRollerMotor.setControl(new Follower(Port.NEAR_SHOOTER_BOTTOM_ROLLER_MOTOR, MotorAlignmentValue.Aligned));

        SmartDashboard.putNumber("B Bottom Roller Target RPS", testBRRPS);
        SmartDashboard.putNumber("B Top Roller Target RPS", testTRRPS);
    }

    public double getTopMotorRPSFromDistanceInches(double distanceFromTargetInches) {
        //double distanceOffset = (-1 * (distanceFromTargetInches/10)); //offset the distance
        //distanceFromTargetInches += distanceOffset;
        //distance is from center bottom of bot to ground center of hub
        // double topRollerRPS = 0.00079997*Math.pow(distanceFromTargetInches,2) + 0.198584*distanceFromTargetInches+29.26717;//test function
        //distanceFromTargetInches -= 41; //subtract half of bot length and half of hub length
        distanceFromTargetInches += 5;
        double topRollerRPS = 0.00116398*(Math.pow(distanceFromTargetInches, 2)) + 0.272546*distanceFromTargetInches + 15.25056;
        return topRollerRPS;
        // item 0 is for the bottom voltage, item 1 is for the top voltage
    }

    public double getVoltageFromRPS(double rps) {
        return 0.2203 + 0.1107*rps; // only for top roller
    }

    //im keeping this just because im fond of it
    // public boolean basketballin() {
    //     double topRollerRPS = 70;
    //      VelocityVoltage bottomMotor_request = new VelocityVoltage(0).withSlot(0);
    //     VelocityVoltage topMotor_request = new VelocityVoltage(0).withSlot(0);

    //     nearBottomRollerMotor.setControl(bottomMotor_request.withVelocity(-30).withFeedForward(-3.2));
    //     topRollerMotor.setControl(topMotor_request.withVelocity(topRollerRPS).withFeedForward(getVoltageFromRPS(topRollerRPS)));
    //     return true;
    // }

    public void setModes(boolean enabled, boolean reversed, boolean autoDistance, boolean againstHub) {
        this.enabled = enabled;
        this.reversed = reversed;
        this.autoDistance = autoDistance;
        this.againstHub = againstHub;
    }

    // new functions for testing with rotations per second
    public void changeBottomRollerRPS(double change) {
        testBRRPS += change;
        SmartDashboard.putNumber("B Bottom Roller Target RPS", testBRRPS);
    }

    public void changeTopRollerRPS(double change) {
        testTRRPS += change;
        SmartDashboard.putNumber("B Top Roller Target RPS", testTRRPS);
    }
    
    public void changeDistanceInches(double change) {
        inchesFromHub += change;
        SmartDashboard.putNumber("Inches From Hub", inchesFromHub);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hub Distance Shooter", distanceFromTargetInches);
        
        SmartDashboard.putNumber("Bottom Roller Actual RPS", nearBottomRollerMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Top Roller Actual RPS", topRollerMotor.getVelocity().getValueAsDouble());

        if (enabled) {
            double topRollerRPS = getTopMotorRPSFromDistanceInches(distanceFromTargetInches);
            double bottomRollerRPS = bottomRollerTargetRPS;

            //topRollerRPS = 20;
            SmartDashboard.putNumber("Bottom Roller Target RPS", bottomRollerRPS);
            SmartDashboard.putNumber("Top Roller Target RPS", topRollerRPS);
            bottomRollerRPS = -25;
            bottomRollerFF = -3.5;
            
            if (!autoDistance) {
                topRollerRPS = getTopMotorRPSFromDistanceInches(staticDistanceInches);
            }
            if (againstHub) {
                topRollerRPS = 13;
                bottomRollerRPS = -30;
                bottomRollerFF = -3.1;
            }
            if (reversed) {
                topRollerRPS = -20;
                bottomRollerRPS *= -1;
                bottomRollerFF *= -1;
            }

            // bottomRollerRPS *= 0.5;
            // bottomRollerFF *= 0.5; //juggle

            if (!testingShooter) {
                nearBottomRollerMotor.setControl(bottomMotor_request.withVelocity(-23).withFeedForward(bottomRollerFF));
                topRollerMotor.setControl(topMotor_request.withVelocity(topRollerRPS).withFeedForward(getVoltageFromRPS(topRollerRPS)));
            } else {
                nearBottomRollerMotor.setControl(bottomMotor_request.withVelocity(testBRRPS).withFeedForward(bottomRollerFF));
                topRollerMotor.setControl(topMotor_request.withVelocity(testTRRPS).withFeedForward(getVoltageFromRPS(testTRRPS)));
            }
        } else {
            nearBottomRollerMotor.stopMotor();
            topRollerMotor.stopMotor();
        }
    }
}
