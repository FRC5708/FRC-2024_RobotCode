package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestSubsystem extends SubsystemBase {
    private CANSparkMax m_frontRightDriveMotor;
    private CANSparkMax m_frontRightTurningMotor;
    private CANSparkMax m_frontLeftDriveMotor;
    private CANSparkMax m_frontLeftTurningMotor;
    private CANSparkMax m_backLeftDriveMotor;
    private CANSparkMax m_backLeftTurningMotor;
    private CANSparkMax m_backRightDriveMotor;
    private CANSparkMax m_backRightTurningMotor;

    private CANcoder goof1;
    private CANcoder goof2;
    private CANcoder goof3;
    private CANcoder goof4;

    private StatusSignal<Double> m_frontRightAbsoluteEncoder;
    private StatusSignal<Double> m_frontLeftAbsoluteEncoder;
    private StatusSignal<Double> m_backRightAbsoluteEncoder;
    private StatusSignal<Double> m_backLeftAbsoluteEncoder;


    private final RelativeEncoder m_frontRightDriveEncoder;
    private final RelativeEncoder m_frontRightTurningEncoder;
    private final RelativeEncoder m_frontLeftDriveEncoder;
    private final RelativeEncoder m_frontLeftTurningEncoder;
    private final RelativeEncoder m_backLeftDriveEncoder;
    private final RelativeEncoder m_backLeftTurningEncoder;
    private final RelativeEncoder m_backRightDriveEncoder;
    private final RelativeEncoder m_backRightTurningEncoder;


  public TestSubsystem() {
    m_frontLeftDriveMotor = new CANSparkMax(SwerveConstants.frontLeftDriveMotor, CANSparkLowLevel.MotorType.kBrushless);
    m_frontLeftTurningMotor = new CANSparkMax(SwerveConstants.frontLeftTurningMotor, CANSparkLowLevel.MotorType.kBrushless);
    m_backLeftDriveMotor = new CANSparkMax(SwerveConstants.backLeftDriveMotor, CANSparkLowLevel.MotorType.kBrushless);
    m_backLeftTurningMotor = new CANSparkMax(SwerveConstants.backLeftTurningMotor, CANSparkLowLevel.MotorType.kBrushless);
    m_frontRightDriveMotor = new CANSparkMax(SwerveConstants.frontRightDriveMotor, CANSparkLowLevel.MotorType.kBrushless);
    m_frontRightTurningMotor = new CANSparkMax(SwerveConstants.frontRightTurningMotor, CANSparkLowLevel.MotorType.kBrushless);
    m_backRightDriveMotor = new CANSparkMax(SwerveConstants.backRightDriveMotor, CANSparkLowLevel.MotorType.kBrushless);
    m_backRightTurningMotor = new CANSparkMax(SwerveConstants.backRightTurningMotor, CANSparkLowLevel.MotorType.kBrushless);

    m_frontRightDriveEncoder = m_frontRightDriveMotor.getEncoder();
    m_frontRightTurningEncoder = m_frontRightTurningMotor.getEncoder();
    m_frontLeftDriveEncoder = m_frontLeftDriveMotor.getEncoder();
    m_frontLeftTurningEncoder = m_frontLeftTurningMotor.getEncoder();
    m_backRightDriveEncoder = m_backRightDriveMotor.getEncoder();
    m_backRightTurningEncoder = m_backRightTurningMotor.getEncoder();
    m_backLeftDriveEncoder = m_backLeftDriveMotor.getEncoder();
    m_backLeftTurningEncoder = m_backLeftTurningMotor.getEncoder();

    goof1 = new CANcoder(SwerveConstants.frontLeftAbsoluteEncoder);
    goof2 = new CANcoder(SwerveConstants.frontRightAbsoluteEncoder);
    goof3 = new CANcoder(SwerveConstants.backRightAbsoluteEncoder);
    goof4 = new CANcoder(SwerveConstants.backLeftAbsoluteEncoder);

    m_frontLeftAbsoluteEncoder = goof1.getAbsolutePosition();
    m_frontRightAbsoluteEncoder = goof2.getAbsolutePosition();
    m_backRightAbsoluteEncoder = goof3.getAbsolutePosition();
    m_backLeftAbsoluteEncoder = goof4.getAbsolutePosition();

  }

  public void report() {
    m_backLeftAbsoluteEncoder.refresh();
    m_backRightAbsoluteEncoder.refresh();
    m_frontLeftAbsoluteEncoder.refresh();
    m_frontRightAbsoluteEncoder.refresh();
    SmartDashboard.putNumber("Front Left Drive Encoder: ", m_frontLeftDriveEncoder.getPosition());
    SmartDashboard.putNumber("Front Left Turning Encoder: ", m_frontLeftTurningEncoder.getPosition());
    SmartDashboard.putNumber("Front Left Absolute Encoder: ", m_frontLeftAbsoluteEncoder.getValue());
    SmartDashboard.putNumber("Back Left Drive Encoder: ", m_backLeftDriveEncoder.getPosition());
    SmartDashboard.putNumber("Back Left Turning Encoder: ", m_backLeftTurningEncoder.getPosition());
    SmartDashboard.putNumber("Back Left Absolute Encoder: ", m_backLeftAbsoluteEncoder.getValue());
    SmartDashboard.putNumber("Back Right Drive Encoder: ", m_backRightDriveEncoder.getPosition());
    SmartDashboard.putNumber("Back Right Turning Encoder: ", m_backRightTurningEncoder.getPosition());
    SmartDashboard.putNumber("Back Right Absolute Encoder: ", m_backRightAbsoluteEncoder.getValue());
    SmartDashboard.putNumber("Front Right Drive Encoder: ", m_frontRightDriveEncoder.getPosition());
    SmartDashboard.putNumber("Front Right Turning Encoder: ", m_frontRightTurningEncoder.getPosition());
    SmartDashboard.putNumber("Front Right Absolute Encoder: ", m_frontRightAbsoluteEncoder.getValue());
  }

}
