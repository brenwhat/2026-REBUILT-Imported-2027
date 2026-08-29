package frc.robot.commands.IntakeCommand;

import org.wpilib.command2.InstantCommand;
import frc.robot.Subsystems.Intake.Intake;

public class ToggleIntake extends InstantCommand {

    private final Intake intake;

    public ToggleIntake(Intake intake) {
        System.out.println("jasdjfjasdfj");
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        //intake.toggle();
    }
}