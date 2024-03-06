// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ThingsSubsystem;

public class RunEverything extends Command {
  private ThingsSubsystem m_things;
  private double m_shooterPower;
  private double m_beltPower;
  private double m_climberPower;

  public RunEverything(ThingsSubsystem things, double shooterPower, double beltPower, double climberPower) {
    m_things = things;
    m_shooterPower = shooterPower;
    m_beltPower = beltPower;
    m_climberPower = climberPower;
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
    m_things.runClimberMotor(m_climberPower);
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
