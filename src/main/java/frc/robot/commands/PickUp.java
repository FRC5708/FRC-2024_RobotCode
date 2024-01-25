package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class PickUp extends Command {

    private IntakeSubsystem intake;
    private BooleanSupplier button;

    public PickUp(IntakeSubsystem intake, BooleanSupplier button) {
        this.intake = intake;
        this.button = button;
        addRequirements(this.intake);
    }

    @Override
    public void execute() {
        intake.runIntake(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        intake.runIntake(0);
        
    }

    @Override
    public boolean isFinished() {
        return !button.getAsBoolean();
    }
}

