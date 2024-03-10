package frc.robot.commands;

import frc.robot.subsystems.ThingsSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RunClimber extends Command{
  /** Creates a new ClimberTeleop. */
  private ThingsSubsystem m_things;
  private double m_power;

  public RunClimber(ThingsSubsystem things, double power) {
    m_things = things;
    m_power = power;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially schedulede
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    m_things.runClimberMotor(m_power);
  }

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  public boolean isFinished() {
    return false;
  }
}
