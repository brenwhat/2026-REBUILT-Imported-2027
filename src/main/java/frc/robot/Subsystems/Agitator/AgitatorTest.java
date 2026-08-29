package frc.robot.Subsystems.Agitator;

import com.ctre.phoenix6.hardware.TalonFX;

import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Port;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AgitatorTest extends SubsystemBase {
    private SparkMax agitatorMotor;
    private final RelativeEncoder encoder;
    private final SparkMaxConfig agitatorMotorConfig;
    private final AlternateEncoderConfig agitatorEncoderConfig;

    public AgitatorTest() {
        agitatorMotor = new SparkMax(Port.SHOOTER_GATE_MOTOR, MotorType.kBrushless);
        agitatorMotorConfig = new SparkMaxConfig();
        agitatorEncoderConfig = new AlternateEncoderConfig();  
        agitatorMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(10000);//IntakeConstants.EXTENDER_CURRENT_LIMIT);
        agitatorEncoderConfig.positionConversionFactor(IntakeConstants.EXTENDER_GEAR_RATIO);
        agitatorMotorConfig.apply(agitatorEncoderConfig);
        encoder = agitatorMotor.getEncoder();
        encoder.setPosition(0);
        agitatorMotor.configure(agitatorMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }
        
    private TalonFX bottomRollerMotor = new TalonFX(Port.NEAR_SHOOTER_BOTTOM_ROLLER_MOTOR);
    private TalonFX topRollerMotor = new TalonFX(Port.SHOOTER_TOP_ROLLER_MOTOR);

    private double voltageAgitator = 0.0;
    private double voltageBottom = 0.0;
    private double voltageTop = 0.0;

    private enum currentMotorVoltage {
        AGITATOR,
        BOTTOM,
        TOP
    }

    private currentMotorVoltage thisVoltage = currentMotorVoltage.AGITATOR;

    public void switchCase() {
        switch (thisVoltage) {
            case AGITATOR:
                thisVoltage = currentMotorVoltage.BOTTOM;
                break;
            case BOTTOM:
                thisVoltage = currentMotorVoltage.TOP;
                break;
            case TOP:
                thisVoltage = currentMotorVoltage.AGITATOR;
                break; 
        }
        SmartDashboard.putString("currentVoltage", thisVoltage.toString());
    }

    public void changeVoltage(Double posNeg) {
        switch(thisVoltage) {
            case AGITATOR:
                voltageAgitator += (0.1 * posNeg);
                break;
            case BOTTOM:
                voltageBottom += (0.1 * posNeg);
                break;
            case TOP:
                voltageTop += (0.1 * posNeg);
                break;
        } 
        SmartDashboard.putNumber("voltageAgitator", voltageAgitator);
        SmartDashboard.putNumber("voltageBottom", voltageBottom);
        SmartDashboard.putNumber("voltageTop", voltageTop);
    }

    public void incrementVoltage() {
        changeVoltage(2.5);
    }
    public void decrementVoltage() {
        changeVoltage(-2.5);
    }

    public void runMotors() {
        agitatorMotor.setVoltage(voltageAgitator);
        bottomRollerMotor.setVoltage(voltageBottom);
        topRollerMotor.setVoltage(voltageTop);
        SmartDashboard.putNumber("currentAgitator", agitatorMotor.getOutputCurrent());
        //SmartDashboard.putNumber("currentBottom", bottomRollerMotor.getSupplyCurrent());
        //SmartDashboard.putNumber("currentTop", voltageTop);
    }

    public void stopMotors() {
        agitatorMotor.setVoltage(0);
        bottomRollerMotor.setVoltage(0);
        topRollerMotor.setVoltage(0);
    }
}
