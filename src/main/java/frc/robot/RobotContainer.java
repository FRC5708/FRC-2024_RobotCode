// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.commands.DefaultSwerve;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

public class RobotContainer {
  
  XboxController controller = new XboxController(0); // CHANGE PORT
  SwerveSubsystem drive = new SwerveSubsystem();

  public RobotContainer() {
    drive.setDefaultCommand(new DefaultSwerve(drive, 
    () -> controller.getLeftX(), 
    () -> controller.getLeftY(), 
    () -> controller.getRightX(), 
    () -> controller.getAButtonPressed(),
    () -> drive.getHeading()));

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
