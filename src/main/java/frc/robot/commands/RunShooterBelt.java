// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ThingsSubsystem;

public class RunShooterBelt extends Command {
  /** Creates a new RunShooterBelt. */
  private ThingsSubsystem m_things;
  private double m_shooterPower;
  private double m_beltPower;

  public RunShooterBelt(ThingsSubsystem things, double shooterPower, double beltPower) {
    m_things = things;
    m_shooterPower = shooterPower;
    m_beltPower = beltPower;
    addRequirements(m_things);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_things.runShooterMotors(m_shooterPower);
    m_things.runBeltMotors(m_beltPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
