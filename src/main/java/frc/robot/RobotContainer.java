// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.BlinkySubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import java.util.function.BooleanSupplier;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultThings;
import frc.robot.commands.RunBelt;
import frc.robot.commands.RunClimber;
import frc.robot.commands.RunEverything;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunShooterBelt;
import frc.robot.commands.STROBE;
import frc.robot.commands.StaggeredShooter;
import frc.robot.commands.StopBelt;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopShooter;
import frc.robot.subsystems.ThingsSubsystem;

public class RobotContainer {
  boolean setupSwerve = true;
  CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  CommandXboxController m_babyController = new CommandXboxController(OperatorConstants.kBabyControllerPort);
  DriveSubsystem m_drive;
  ThingsSubsystem m_things;
  IntakeSubsystem m_intake;
  BlinkySubsystem m_blinky;
  SendableChooser<Command> autoChooser;

  public RobotContainer() {

    m_drive = new DriveSubsystem();
    m_things = new ThingsSubsystem();
    m_intake = new IntakeSubsystem();
    m_blinky = new BlinkySubsystem();
    NamedCommands.registerCommand("Start intake", new RunIntake(m_intake, 0.2));
    NamedCommands.registerCommand("Stop intake", new StopIntake(m_intake));
    NamedCommands.registerCommand("Start belt", new RunBelt(m_things, -0.8));
    NamedCommands.registerCommand("Stop belt", new StopBelt(m_things));
    NamedCommands.registerCommand("Spin up", new StaggeredShooter(m_things, -1));
    NamedCommands.registerCommand("Spin down", new StopShooter(m_things));
    NamedCommands.registerCommand("Shooter & belt", new RunShooterBelt(m_things,-1,-0.7));

    autoChooser = AutoBuilder.buildAutoChooser("2 note front");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    m_drive.setDefaultCommand(
        m_drive.driveCommand(m_driverController::getLeftX, m_driverController::getLeftY, m_driverController::getRightX));

    m_intake.setDefaultCommand(new RunIntake(m_intake,0));
    m_things.setDefaultCommand(new DefaultThings(m_things, () -> m_driverController.rightTrigger().getAsBoolean(), 0,0));
    //m_things.setDefaultCommand(new RunEverything(m_things, 0, 0, 0));
    m_blinky.setDefaultCommand(m_blinky.setLightsCommand(false));
      

    configureBindings();
  }

  private void configureBindings() {
      //Sets zero
      m_driverController.start().onTrue(m_drive.zeroGyro());
      //Intake
      m_driverController.leftTrigger().whileTrue(new RunIntake(m_intake, 0.2));// was .25
      m_driverController.leftTrigger().whileTrue(new RunBelt(m_things, -.8));//
      //Shoots into speaker
      m_driverController.rightTrigger().onTrue(new StaggeredShooter(m_things, -1));
      m_driverController.rightTrigger().debounce(1).whileTrue(new RunShooter(m_things,-1));
      //Shoots into amp
      m_driverController.rightBumper().whileTrue(new RunShooter(m_things, -.15));
      //Intake from human player
      m_driverController.a().whileTrue(new RunShooter(m_things, .6));
      //Reverse belt & intake
      m_driverController.a().whileTrue(new RunIntake(m_intake, 0.2));
      m_driverController.a().whileTrue(new RunBelt(m_things, -0.8));


      //Second Controller Reverse Intake
      m_babyController.y().whileTrue(new RunIntake(m_intake, -0.25));
      m_babyController.y().whileTrue(new RunBelt(m_things, 1));
      //Second Controler Run Belt
      m_babyController.a().whileTrue(new RunBelt(m_things, -1));
      //Optical assault
      //m_driverController.y().whileTrue(m_blinky.setLightsCommand(true));
  }

  public Command getAutonomousCommand() {
    //return new PathPlannerAuto("Shoot move shoot");
    return autoChooser.getSelected();
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_drive;
  }
}
