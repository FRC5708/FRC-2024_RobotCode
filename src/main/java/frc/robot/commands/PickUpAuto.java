// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ThingsSubsystem;

public class PickUpAuto extends Command {
  /** Creates a new PickUpAuto. */
    private ThingsSubsystem m_things;
    private double startTime;
  public PickUpAuto(ThingsSubsystem things) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_things = things;
    addRequirements(m_things);
    startTime = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_things.runIntake(0.5);
    m_things.beltMotor.set(0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() > startTime + 1000;
  }
}
