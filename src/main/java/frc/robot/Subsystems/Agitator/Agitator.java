package frc.robot.Subsystems.Agitator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Port;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.BottomMotorConfigs;

public class Agitator extends SubsystemBase {
    private final SparkMax agitatorMotor;
    private final RelativeEncoder agitatorEncoder;
    private final SparkMaxConfig agitatorMotorConfig;
    private final AlternateEncoderConfig agitatorEncoderConfig;

    private final SparkMax gateMotor;
    private final RelativeEncoder gateEncoder;
    private final SparkMaxConfig gateMotorConfig;
    private final AlternateEncoderConfig gateEncoderConfig;

    public double lastMult;

    public Agitator() {
        // TODO motors need canbus number
        agitatorMotor = new SparkMax(Port.AGITATOR_MOTOR, MotorType.kBrushless);
        agitatorMotorConfig = new SparkMaxConfig();
        agitatorEncoderConfig = new AlternateEncoderConfig();  
        agitatorMotorConfig.idleMode(SparkMaxConfig.IdleMode.kCoast)
            .smartCurrentLimit(IntakeConstants.AGITATOR_CURRENT_LIMIT);
        agitatorEncoderConfig.positionConversionFactor(3); //agitator gear ratio
        agitatorEncoderConfig.apply(agitatorEncoderConfig);
        agitatorEncoder = agitatorMotor.getEncoder();
        
        gateMotor = new SparkMax(Port.SHOOTER_GATE_MOTOR, MotorType.kBrushless);
        gateMotorConfig = new SparkMaxConfig();
        gateEncoderConfig = new AlternateEncoderConfig();  
        gateMotorConfig.idleMode(SparkMaxConfig.IdleMode.kCoast)
            .smartCurrentLimit(ShooterConstants.GATE_CURRENT_LIMIT);
        gateEncoderConfig.positionConversionFactor(3); //gate gear ratio
        gateEncoderConfig.apply(gateEncoderConfig);
        gateEncoder = gateMotor.getEncoder();

        gateMotorConfig.closedLoop.pidf(0.0006, 0, 0, 6).outputRange(-1, 1);
        agitatorMotorConfig.closedLoop.pidf(0.0006, 0, 0, 6).outputRange(-1, 1);
        //gateEncoder.setPosition(0);
        gateMotor.configure(gateMotorConfig, ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        agitatorMotor.configure(agitatorMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    public void runAgitation(int mult) {
        agitatorMotor.setVoltage(11 * mult); // 8
        // agitatorMotor.getClosedLoopController().
        // setReference(IntakeConstants.AGITATOR_RPM*mult, ControlType.kVelocity);
    }

    public void stopAgitation() {
        agitatorMotor.stopMotor();
    }

    public void runGate(double mult) {
        gateMotor.setVoltage(-12 * mult);
        // gateMotor.getClosedLoopController().
        // setReference(ShooterConstants.GATE_RPM*mult, ControlType.kVelocity);
        lastMult = mult;
    }

    public void stopGate() {
        gateMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // optional telemetry:
        super.periodic();
        SmartDashboard.putNumber("Agitator Current", agitatorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Gate Current", gateMotor.getOutputCurrent());
        SmartDashboard.putNumber("Agitator Velocity", agitatorEncoder.getVelocity());
        SmartDashboard.putNumber("Gate Velocity", gateEncoder.getVelocity());
    }
}
