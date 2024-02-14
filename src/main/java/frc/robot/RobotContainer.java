// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TestSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

public class RobotContainer {
  boolean setupSwerve = true;
  XboxController m_driverController = new XboxController(0); // CHANGE PORT
  DriveSubsystem m_drive;
  TestSubsystem test;

  public RobotContainer() {
    if (setupSwerve) {
      m_drive = new DriveSubsystem();
      /*drive.setDefaultCommand(new DefaultSwerve(drive, 
      () -> controller.getLeftX(), 
      () -> controller.getLeftY(), 
      () -> controller.getRightX(), 
      () -> controller.getAButtonPressed(),
      () -> drive.getHeading()));
      drive.setDefaultCommand(new TestDrive(drive, () -> controller.getLeftX(), 
      () -> controller.getLeftY(), 
      () -> controller.getRightX()));*/

      m_drive.setDefaultCommand(
        new RunCommand(
            () -> m_drive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                false, false),
            m_drive));
    }
    else {
      test = new TestSubsystem();
    }
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
