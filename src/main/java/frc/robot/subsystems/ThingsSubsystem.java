// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AboveChassisConstants;

public class ThingsSubsystem extends SubsystemBase {
  /** Creates a new ThingsSubsystem. */
  public CANSparkMax shooterMotor1 = new CANSparkMax(AboveChassisConstants.shooterMotorID1, MotorType.kBrushless);
  public CANSparkMax shooterMotor2 = new CANSparkMax(AboveChassisConstants.shooterMotorID2, MotorType.kBrushless);
  public CANSparkMax intakeMotor = new CANSparkMax(AboveChassisConstants.intakeMotorID, MotorType.kBrushless);
  public CANSparkMax beltMotor = new CANSparkMax(AboveChassisConstants.beltMotorID,MotorType.kBrushless);

  public ThingsSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command runIntake(double power) {
    return run(() -> intakeMotor.set(power));
  }

  public Command runShooter(double power) {
    return run(() -> runMotors(power));
  }

  public void runMotors(double power) {
    shooterMotor1.set(power);
    shooterMotor2.set(power);
  }

  public Command runBelt(double power) {
    return run(() -> beltMotor.set(power));
  }

}
