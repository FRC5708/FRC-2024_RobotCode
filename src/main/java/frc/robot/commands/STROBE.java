// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BlinkySubsystem;

public class STROBE extends Command {
  /** Creates a new STROBE. */
  private BlinkySubsystem m_blinky;
  public STROBE(BlinkySubsystem blinky) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_blinky = blinky;
    addRequirements(m_blinky);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double startTime = System.currentTimeMillis();
    boolean temp = false;
    if((System.currentTimeMillis() - startTime) % 500 == 0) {
      m_blinky.setLights(temp);
      temp = !temp;
    }

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
