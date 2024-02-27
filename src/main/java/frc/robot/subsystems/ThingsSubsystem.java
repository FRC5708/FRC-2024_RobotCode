// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AboveChassisConstants;

public class ThingsSubsystem extends SubsystemBase {
  /** Creates a new ThingsSubsystem. */
  public CANSparkMax shooterMotor1 = new CANSparkMax(AboveChassisConstants.shooterMotorID1,MotorType.kBrushless);
  public CANSparkMax shooterMotor2 = new CANSparkMax(AboveChassisConstants.shooterMotorID2,MotorType.kBrushless);
  public CANSparkMax beltMotor1 = new CANSparkMax(AboveChassisConstants.beltMotorID1,MotorType.kBrushless);
  public CANSparkMax beltMotor2 = new CANSparkMax(AboveChassisConstants.beltMotorID2,MotorType.kBrushless);
  public CANSparkMax climbingMotor = new CANSparkMax(AboveChassisConstants.climberID1, MotorType.kBrushless);

  public ThingsSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runShooterMotors(double power) {
    shooterMotor1.set(power);
    shooterMotor2.set(power);
  }

  public void runBeltMotor(double power) {
    beltMotor1.set(power);
    beltMotor2.set(power);
  }

  public void runClimberMotor(double power) {
    climbingMotor.set(power);

  }
}
