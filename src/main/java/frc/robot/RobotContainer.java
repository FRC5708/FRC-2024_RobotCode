// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.BlinkySubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import java.util.function.BooleanSupplier;
import frc.robot.Constants;
import frc.robot.commands.RunBelt;
import frc.robot.commands.RunClimber;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunShooterBelt;
import frc.robot.commands.STROBE;
import frc.robot.commands.StopBelt;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopShooter;
import frc.robot.subsystems.ThingsSubsystem;

public class RobotContainer {
  boolean setupSwerve = true;
  CommandXboxController m_driverController = new CommandXboxController(0);
  DriveSubsystem m_drive;
  ThingsSubsystem m_things;
  IntakeSubsystem m_intake;
  BlinkySubsystem m_blinky;

  public RobotContainer() {

    m_drive = new DriveSubsystem();
    m_things = new ThingsSubsystem();
    m_intake = new IntakeSubsystem();
    m_blinky = new BlinkySubsystem();
    NamedCommands.registerCommand("Start intake", new RunIntake(m_intake, 0.25));
    NamedCommands.registerCommand("Stop intake", new StopIntake(m_intake));
    NamedCommands.registerCommand("Start belt", new RunBelt(m_things, -0.7));
    NamedCommands.registerCommand("Stop belt", new StopBelt(m_things));
    NamedCommands.registerCommand("Spin up", new RunShooter(m_things, -1));
    NamedCommands.registerCommand("Spin down", new StopShooter(m_things));
    NamedCommands.registerCommand("Shooter & belt", new RunShooterBelt(m_things,-1,-0.7));

    m_drive.setDefaultCommand(
        m_drive.driveCommand(m_driverController::getLeftX, m_driverController::getLeftY, m_driverController::getRightX));

    m_intake.setDefaultCommand(new RunIntake(m_intake,0));
    m_things.setDefaultCommand(new RunShooterBelt(m_things, 0, 0));
    m_blinky.setDefaultCommand(m_blinky.setLightsCommand(true));

    configureBindings();
  }

  private void configureBindings() {
      m_driverController.start().onTrue(m_drive.zeroGyro());
<<<<<<< Updated upstream
      /*Reverse Intake
      m_driverController.y().whileTrue(new RunIntake(m_intake, -0.25
      ));
=======
      //Reverse Intake
      m_driverController.y().whileTrue(new RunIntake(m_intake, -0.25));
      //Reverse belt
      m_driverController.x().whileTrue(new RunBelt(m_things, 0.6));
>>>>>>> Stashed changes
      //Intake
      m_driverController.leftTrigger().whileTrue(new RunIntake(m_intake, 0.25));
      m_driverController.leftTrigger().whileTrue(new RunBelt(m_things, -0.7));
      //Shoots into speaker
      m_driverController.rightTrigger().whileTrue(new RunShooterBelt(m_things, -1,-1));
      m_driverController.a().whileTrue(new RunShooter(m_things, -1));
      //Shoots into amp
      m_driverController.b().whileTrue(new RunShooterBelt(m_things, -.15,-0.35));
      //Climb Up
      m_driverController.rightBumper().whileTrue(new RunClimber(m_things, .25));
      //Climb Down
      m_driverController.leftBumper().whileTrue(new RunClimber(m_things, -.25));
<<<<<<< Updated upstream
      */
=======
      //Optical assault
      //m_driverController.a().whileTrue(new STROBE(m_blinky));
>>>>>>> Stashed changes
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Shoot move shoot");
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_drive;
  }
}
