package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.DriveConstants;
//import frc.robot.util.Constants.IOConstants;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultSwerve extends Command {

  // Create variables
  private final SwerveSubsystem swerveSubsystem;
  private final DoubleSupplier xSpeedFunc, ySpeedFunc, turningSpeedFunc, headingFunc;
  private final BooleanSupplier fieldOriented;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private double initialHeading;
  private PIDController angleController; 
 // private double added;
 // private int counter;

  // Command constructor
  public DefaultSwerve(SwerveSubsystem swerveSubsystem,
  DoubleSupplier xSpeedFunc, DoubleSupplier ySpeedFunc, DoubleSupplier turningSpeedFunc,
  BooleanSupplier fieldOriented, DoubleSupplier headingFunc){

    // Assign values passed from constructor
    this.swerveSubsystem = swerveSubsystem;
    this.xSpeedFunc = xSpeedFunc;
    this.ySpeedFunc = ySpeedFunc;
    this.turningSpeedFunc = turningSpeedFunc;
    this.fieldOriented = fieldOriented;
    this.headingFunc = headingFunc;

    // Should be zero but eh
    this.initialHeading = headingFunc.getAsDouble();

    // Slew rate limiter
    this.xLimiter = new SlewRateLimiter(DriveConstants.teleOpMaxAccel);
    this.yLimiter = new SlewRateLimiter(DriveConstants.teleOpMaxAccel);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.teleOpMaxAngularAccel);

    // Set default PID values for anglecontroller
    angleController = new PIDController(DriveConstants.angleP, DriveConstants.angleI, DriveConstants.angleD);

    addRequirements(swerveSubsystem);

  }

  @Override
  public void initialize() {
    initialHeading = headingFunc.getAsDouble();
  }

  @Override
  public void execute(){

    // Get joystick values
    double xSpeed = xSpeedFunc.getAsDouble();
    double ySpeed = ySpeedFunc.getAsDouble();
    double turningSpeed = turningSpeedFunc.getAsDouble();

    // Slew rate & max speed
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.maxSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.maxSpeed;

    // Test heading control, throws away previous turning values
    initialHeading += turningSpeed;

    // Get current navx heading
    double newHeading = headingFunc.getAsDouble();

    turningSpeed = angleController.calculate(newHeading, initialHeading) * 100;

    // Limit turning rate
    if (turningSpeed > DriveConstants.maxAngularSpeed){
      turningSpeed = DriveConstants.maxAngularSpeed;
    }
    else if (Math.abs(turningSpeed) > DriveConstants.maxAngularSpeed){
      turningSpeed = -DriveConstants.maxAngularSpeed;
    }
    
    SmartDashboard.putNumber("Heading", headingFunc.getAsDouble());

    ChassisSpeeds chassisSpeeds;
    if(fieldOriented.getAsBoolean()) {
        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, turningSpeed, Rotation2d.fromDegrees(swerveSubsystem.getRobotDegrees()));;
    }
    else {
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, Rotation2d.fromDegrees(swerveSubsystem.getRobotDegrees()));;
    }
    // Create module states using array
    SwerveModuleState[] moduleStates = swerveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds);
 
    // Set the module state
    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Stop all module motor movement when command ends
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.setModuleStates(
        swerveSubsystem.getKinematics().toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                0,0,0,new Rotation2d())
        )
    );
}

  @Override
  public boolean isFinished(){return false;}

}

