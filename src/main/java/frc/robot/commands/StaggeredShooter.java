// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ThingsSubsystem;

public class StaggeredShooter extends Command {
  /** Creates a new StaggeredShooter. */
  private ThingsSubsystem m_things;
  private double m_power;
  private double m_startTime;
  public StaggeredShooter(ThingsSubsystem things, double power) {
    m_things = things;
    m_power = power;
    addRequirements(m_things);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_things.runShooterMotor(1, m_power);
    if(System.currentTimeMillis() - m_startTime > 1500) {
      m_things.runShooterMotor(2,m_power);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - m_startTime > 1000;
  }
}
