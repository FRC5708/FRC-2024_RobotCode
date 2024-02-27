// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import java.util.function.BooleanSupplier;
import frc.robot.Constants;
import frc.robot.commands.RunBelt;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunShooterBelt;
import frc.robot.subsystems.ThingsSubsystem;

public class RobotContainer {
  boolean setupSwerve = true;
  CommandXboxController m_driverController = new CommandXboxController(0);
  DriveSubsystem m_drive;
  ThingsSubsystem m_things;
  IntakeSubsystem m_intake;

  public RobotContainer() {

    m_drive = new DriveSubsystem();
    m_things = new ThingsSubsystem();
    m_intake = new IntakeSubsystem();
    NamedCommands.registerCommand("Start intake", new RunIntake(m_intake, 0.3));
    NamedCommands.registerCommand("Stop intake", new RunIntake(m_intake, 0));
    NamedCommands.registerCommand("Start belt", new RunBelt(m_things, -0.6));
    NamedCommands.registerCommand("Stop belt", new RunBelt(m_things, 0));
    NamedCommands.registerCommand("Spin up", new RunShooter(m_things, -1));
    NamedCommands.registerCommand("Spin down", new RunShooter(m_things,0));

    m_drive.setDefaultCommand(
        m_drive.driveCommand(m_driverController::getLeftX, m_driverController::getLeftY, m_driverController::getRightX));

    m_intake.setDefaultCommand(new RunIntake(m_intake,0));
    m_things.setDefaultCommand(new RunShooterBelt(m_things, 0));

    configureBindings();
  }

  private void configureBindings() {
      m_driverController.start().onTrue(m_drive.zeroGyro());
      //Reverse Intake
      m_driverController.y().whileTrue(new RunIntake(m_intake, -0.3));
      //Intake
      m_driverController.leftTrigger().whileTrue(new RunIntake(m_intake, 0.3));
      m_driverController.leftTrigger().whileTrue(new RunBelt(m_things, -0.6));
      //Shoots
      m_driverController.rightTrigger().whileTrue(new RunShooter(m_things, -1));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Shoot move shoot");
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_drive;
  }
}
