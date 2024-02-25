// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AboveChassisConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public CANSparkMax intakeMotor1 = new CANSparkMax(AboveChassisConstants.intakeMotorID1, MotorType.kBrushless);
  public CANSparkMax intakeMotor2 = new CANSparkMax(AboveChassisConstants.intakeMotorID2, MotorType.kBrushless);

  public IntakeSubsystem() {
  
  }

  public void runIntakeMotors(double power) {
    intakeMotor1.set(power);
    intakeMotor2.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
