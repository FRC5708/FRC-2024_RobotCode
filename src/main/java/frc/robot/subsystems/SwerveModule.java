// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.StatusSignal;

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
  public final CANSparkMax m_drivingSparkMax;
  public final CANSparkMax m_turningSparkMax;

  public final RelativeEncoder m_drivingEncoder;
  public final CANcoder m_turningEncoder;
  private StatusSignal<Double> m_turningEncoderSignal;
  private double m_turningEncoderPosition = 0;

  public final SparkPIDController m_drivingPIDController;
  public final PIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a CANCoder
   * Encoder.
   */
  public SwerveModule(int drivingCANId, int turningCANId,int encoderCANId , double chassisAngularOffset) {
    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningEncoder = new CANcoder(encoderCANId);
    m_turningEncoderSignal = m_turningEncoder.getAbsolutePosition();
    m_turningPIDController = new PIDController(
      SwerveConstants.turningP,
      SwerveConstants.turningI,
      SwerveConstants.turningD
    );

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(Constants.SwerveConstants.driveGearRatio * Math.PI * 2 * SwerveConstants.wheelRadius);
    m_drivingEncoder.setVelocityConversionFactor(Constants.SwerveConstants.driveGearRatio * Math.PI * 2 * SwerveConstants.wheelRadius / 60);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(SwerveConstants.driveP);
    m_drivingPIDController.setI(SwerveConstants.driveI);
    m_drivingPIDController.setD(SwerveConstants.driveD);
    m_drivingPIDController.setFF((1/94.6));
    m_drivingPIDController.setOutputRange(-1,1);

    m_drivingSparkMax.setIdleMode(IdleMode.kBrake);
    m_turningSparkMax.setIdleMode(IdleMode.kBrake);
    m_drivingSparkMax.setSmartCurrentLimit(50);
    m_turningSparkMax.setSmartCurrentLimit(20);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(getTurningEncoderPositionNow());
    m_drivingEncoder.setPosition(0);
  }

  // turning encoder position must return radians, but this depends on
  // CANCoder configuration. either set it in flash memory or reconfigure
  // it at startup like the spark maxes are reconfigured.
  public double getTurningEncoderPositionNow() {
    // The CANCoder can't be polled on every use of the turning encoder.
    // It would be best to fetch the current encoder signal at the start of
    // each periodic tick and then use that value for the rest of the tick.
    m_turningEncoderSignal.refresh();
    m_turningEncoderPosition = m_turningEncoderSignal.getValueAsDouble();
    return m_turningEncoderPosition;
  }

  public double getTurningEncoderPosition() {
    return m_turningEncoderPosition;
  }

  /**
   * Returns the current velocity and direction of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
      m_drivingEncoder.getVelocity(),
      new Rotation2d(getTurningEncoderPosition() - m_chassisAngularOffset)
    );
  }

  /**
   * Returns the current position and direction of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(getTurningEncoderPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // If the turning encoder isn't polled at the start of the periodic tick,
    // it must be polled now otherwise the control loop won't be accurate.
    getTurningEncoderPositionNow();

    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(getTurningEncoderPosition()));

    // Command driving SPARK MAX towards its respective setpoints.
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(
      getTurningEncoderPosition(),
      optimizedDesiredState.angle.getRadians()
    );
    m_turningSparkMax.setVoltage(turnOutput);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
