package frc.robot.Subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.SubsystemBase;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Port;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import org.wpilib.hardware.discrete.DigitalInput;

public class Intake extends SubsystemBase {

    //public static final Intake instance = new Intake();

    private final SparkMax extenderMotor;
    private final RelativeEncoder encoder;
    private final SparkMaxConfig extenderMotorConfig;
    private final AlternateEncoderConfig extenderEncoderConfig;

    //TODO: get canbus number for talons
    private TalonFX intakeMotor = new TalonFX(Port.INTAKE_MOTOR, new CANBus());

    private DigitalInput armLimit = new DigitalInput(0);
    private boolean limitReached = !armLimit.get();
    private boolean positionSet = false;

    public enum direction {
        EXTENDING,
        RETRACTING,
        IGNORE
    }

    public enum intakeMode {
        AUTOMATIC,
        MANUAL
    }

    public enum intakeDirection {
        IN,
        OUT,
        OFF
    }

    public direction currentDirection = direction.EXTENDING;
    private intakeMode currentMode = intakeMode.AUTOMATIC;
    public intakeDirection currentIntakeDirection = intakeDirection.IN;
    private double hopperMult = 1;

    public Intake() {
        extenderMotor = new SparkMax(0, Port.INTAKE_EXTENDER_MOTOR, MotorType.kBrushless);
        extenderMotorConfig = new SparkMaxConfig();
        extenderEncoderConfig = new AlternateEncoderConfig();  
        // extenderMotor.restoreFactoryDefaults();
        // extenderMotorConfig = new SparkBaseConfig();
        extenderMotorConfig.idleMode(SparkMaxConfig.IdleMode.kCoast)
            .smartCurrentLimit(IntakeConstants.EXTENDER_CURRENT_LIMIT);
        extenderEncoderConfig.positionConversionFactor(IntakeConstants.EXTENDER_GEAR_RATIO);
        extenderMotorConfig.apply(extenderEncoderConfig);
        encoder = extenderMotor.getEncoder();
        encoder.setPosition(0);
        extenderMotor.configure(extenderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // intake functions
    public void setIntakeDirection(intakeDirection direction) {
        currentIntakeDirection = direction;
    }

    public void runIntake(int mult) {
        intakeMotor.setVoltage(IntakeConstants.INTAKE_VOLTAGE * mult);
    }

    /** Extend intake until max */
    public void extendArm() {
        if (!isFullyExtended() || !positionSet) {
            currentDirection = direction.EXTENDING;
            extenderMotor.setVoltage(-IntakeConstants.EXTENDER_VOLTAGE * hopperMult);
        } else {
            extenderMotor.setVoltage(0);
        }
    }

    /** Retract intake until fully retracted */
    public void retractArm() {
        if (!isFullyRetracted() || !positionSet) {
            currentDirection = direction.RETRACTING;
            extenderMotor.setVoltage(IntakeConstants.EXTENDER_VOLTAGE * hopperMult);
        } else {
            extenderMotor.setVoltage(0);
        }
    }

    public void setModes(direction setDirection, intakeMode setMode) {
        currentDirection = (setDirection != direction.IGNORE) ? setDirection : currentDirection;
        currentMode = setMode;
    }

    public void setHopperSpeed(double mult) {
        hopperMult = mult;
    }

    // getting intake arm positions
    public boolean isFullyRetracted() {
        return positionSet ? encoder.getPosition().get() >= IntakeConstants.EXTENSION_MIN : false;
    }

    public boolean isFullyExtended() {
        return positionSet ? (encoder.getPosition().get() <= IntakeConstants.EXTENSION_MAX) : false;
    }

    /** Returns current extension in encoder units */

    @Override
    public void periodic() {
        // optional telemetry:
        super.periodic();
        limitReached = !armLimit.get();
        SmartDashboard.putBoolean("Intake Arm Max Reached", limitReached);
        //SmartDashboard.putNumber("Intake position", encoder.getPosition());
        
        //reset position once we INITIALLY hit the limit switch
        if (limitReached && !positionSet) {
            positionSet = true;
            encoder.setPosition(IntakeConstants.EXTENSION_MAX);
        }

        //stop arm if we know what our position is and were outside of limits
        if (positionSet && 
        ((encoder.getPosition().get() <= IntakeConstants.EXTENSION_MAX && currentDirection == direction.EXTENDING)|| 
        (encoder.getPosition().get() >= IntakeConstants.EXTENSION_MIN && currentDirection == direction.RETRACTING))) {
            extenderMotor.setVoltage(0);
        }

        //run intake when we extend and stop it when we retract if its automatic
        //run whatever the codriver is doing if its manual
        if (currentMode == intakeMode.AUTOMATIC) {
            if ((encoder.getPosition().get() <= -23) && positionSet) {
                runIntake(currentIntakeDirection == intakeDirection.OUT ? -1 : 1);
            } else {intakeMotor.stopMotor();}
        } else {
            int mult;
            switch (currentIntakeDirection) {
                case IN:
                mult = 1;
                break;
                case OUT:
                mult = -1;
                break;
                default:
                mult = 0;
                break;
            }
            runIntake(mult);
        }

        SmartDashboard.putBoolean("Intake Motor Running", intakeMotor.getMotorVoltage().getValueAsDouble() != 0);
    }
}


