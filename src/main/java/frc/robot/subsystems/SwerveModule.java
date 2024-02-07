// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkLowLevel;
import com.ctre.phoenix6.hardware.CANcoder;


public class SwerveModule extends SubsystemBase{
  private static final double wheelRadius = SwerveConstants.wheelRadius;
  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = 15; 
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final CANcoder m_absoluteEncoder;

  private final double m_absoluteEncoderOffset;

  private final SparkPIDController m_drivePIDController;

  private final SparkPIDController m_turningPIDController;

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param absoluteEncoderID what's it look like.
   * @param absoluteEncoderOffset 
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int absoluteEncoderID,
      double absoluteEncoderOffset) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, CANSparkLowLevel.MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, CANSparkLowLevel.MotorType.kBrushless);

    m_absoluteEncoderOffset = absoluteEncoderOffset;
    m_absoluteEncoder = new CANcoder(absoluteEncoderID);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getEncoder();

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.driveGearRatio * Math.PI * 2 * wheelRadius);
    m_driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.driveGearRatio * Math.PI * 2 * wheelRadius / 60);  
    
    m_turningEncoder.setPositionConversionFactor(Constants.SwerveConstants.turningGearRatio * Math.PI * 2);
    m_turningEncoder.setVelocityConversionFactor(Constants.SwerveConstants.turningGearRatio * Math.PI * 2 / 60);

    m_drivePIDController = m_driveMotor.getPIDController();
    m_turningPIDController = m_turningMotor.getPIDController();
    m_drivePIDController.setFeedbackDevice(m_driveEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(0);
    m_turningPIDController.setPositionPIDWrappingMaxInput(2*Math.PI);

    m_drivePIDController.setP(SwerveConstants.driveP);
    m_drivePIDController.setI(SwerveConstants.driveI);
    m_drivePIDController.setD(SwerveConstants.driveD);
    m_drivePIDController.setOutputRange(-1,1);

    m_turningPIDController.setP(SwerveConstants.turningP);
    m_turningPIDController.setI(SwerveConstants.turningI);
    m_turningPIDController.setD(SwerveConstants.turningD);
    m_turningPIDController.setOutputRange(-1,1);

    m_driveEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getPosition()));

    m_drivePIDController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(state.angle.getRadians(), CANSparkMax.ControlType.kPosition);

  }

  public double getAbsoluteEncoderRad() {
    double angle;

    angle = 1 - m_absoluteEncoder.getAbsolutePosition().getValue();

    // Convert into radians
    angle *= 2.0 * Math.PI;
    angle -= m_absoluteEncoderOffset;

    return angle;
  }

  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(getAbsoluteEncoderRad());
  }
}
