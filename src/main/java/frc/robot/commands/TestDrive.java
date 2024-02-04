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

public class TestDrive extends Command {

  // Create variables
  private final SwerveSubsystem swerveSubsystem;
  private final DoubleSupplier xSpeedFunc, ySpeedFunc, turningSpeedFunc;

  // Command constructor
  public TestDrive(SwerveSubsystem swerveSubsystem,
  DoubleSupplier xSpeedFunc, DoubleSupplier ySpeedFunc, DoubleSupplier turningSpeedFunc) {
    // Assign values passed from constructor
    this.swerveSubsystem = swerveSubsystem;
    this.xSpeedFunc = xSpeedFunc;
    this.ySpeedFunc = ySpeedFunc;
    this.turningSpeedFunc = turningSpeedFunc;

    addRequirements(swerveSubsystem);

  }

  @Override
  public void initialize() {}

  @Override
  public void execute(){

    // Get joystick values
    double xSpeed = xSpeedFunc.getAsDouble();
    double ySpeed = ySpeedFunc.getAsDouble();
    double turningSpeed = turningSpeedFunc.getAsDouble();

    // Limit turning rate
    if (turningSpeed > DriveConstants.maxAngularSpeed){
      turningSpeed = DriveConstants.maxAngularSpeed;
    }
    else if (Math.abs(turningSpeed) > DriveConstants.maxAngularSpeed){
      turningSpeed = -DriveConstants.maxAngularSpeed;
    }
    
    
    ChassisSpeeds chassisSpeeds;
    
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    // Create module states using array
    SwerveModuleState[] moduleStates = swerveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds);
 
    // Set the module state
    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Stop all module motor movement when command ends
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,0);
    swerveSubsystem.setModuleStates(
        swerveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds));
}

  @Override
  public boolean isFinished(){return false;}

}

