// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
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


public class FirstAttemptSwerveModule extends SubsystemBase{
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

  private final PIDController m_drivePIDController = new PIDController(SwerveConstants.driveP, SwerveConstants.driveI, SwerveConstants.driveD);

  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          SwerveConstants.turningP,
          SwerveConstants.turningI,
          SwerveConstants.turningD,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

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
  public FirstAttemptSwerveModule(
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

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
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
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getPosition()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getPosition(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
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