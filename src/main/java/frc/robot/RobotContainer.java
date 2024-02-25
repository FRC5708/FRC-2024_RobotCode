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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import java.util.function.BooleanSupplier;
import frc.robot.Constants;
import frc.robot.commands.PickUpAuto;
import frc.robot.commands.ShootAuto;
import frc.robot.subsystems.ThingsSubsystem;

public class RobotContainer {
  boolean setupSwerve = true;
  CommandXboxController m_driverController = new CommandXboxController(0); // CHANGE PORT
  DriveSubsystem m_drive;
  ThingsSubsystem m_things;

  public RobotContainer() {

    m_drive = new DriveSubsystem();
    m_things = new ThingsSubsystem();
    NamedCommands.registerCommand("Shoot", new ShootAuto(m_things));
    NamedCommands.registerCommand("Pick up", new PickUpAuto(m_things));

    m_drive.setDefaultCommand(
        m_drive.driveCommand(m_driverController::getLeftX, m_driverController::getLeftY, m_driverController::getRightX));

    configureBindings();
  }

  private void configureBindings() {
      m_driverController.start().onTrue(m_drive.zeroGyro());
      m_driverController.a().whileTrue(m_things.runBelt(1));
      m_driverController.b().whileTrue(m_things.runShooter(1));
      m_driverController.b().whileTrue(m_things.runShooter(1));
      m_driverController.x().whileTrue(m_things.runIntake(1));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Shoot move shoot");
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_drive;
  }
}
