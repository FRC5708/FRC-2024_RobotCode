// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ThingsSubsystem;

public class DefaultThings extends Command {
  /** Creates a new DefaultThings. */
  private ThingsSubsystem m_things;
  private double m_shooterPower;
  private double m_beltPower;
  private double m_climberPower;

  public DefaultThings(ThingsSubsystem things, BooleanSupplier shooter, double beltPower, double climberPower) {
    m_things = things;
    if(shooter.getAsBoolean() == true) {
      m_shooterPower = -1;
    }
    else {
      m_shooterPower = 0;
    }
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
    SmartDashboard.putNumber("fpodn piudfn",m_shooterPower);
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
