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
import frc.robot.Constants.*;
import frc.robot.commands.RunBelt;

public class RobotContainer {
  boolean setupSwerve = true;
  CommandXboxController m_driverController = new CommandXboxController(0); // CHANGE PORT
  DriveSubsystem m_drive;

  public RobotContainer() {

    m_drive = new DriveSubsystem();
    NamedCommands.registerCommand("Run Belt", new RunBelt());

    m_drive.setDefaultCommand(
        m_drive.driveCommand(m_driverController::getLeftX, m_driverController::getLeftY, m_driverController::getRightX));

    configureBindings();
  }

  private void configureBindings() {
      m_driverController.start().onTrue(m_drive.zeroGyro());
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("the economy");
  }
}
