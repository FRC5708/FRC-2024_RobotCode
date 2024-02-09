package frc.robot.commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveGoTo extends Command {

  
  private boolean finished;
  DriveSubsystem driveSubsystem;

  private PIDController vXController, vYController;
  private PIDController thetaController;

  private double vX;
  private double vY;
  private double vT;

  private double xSetpoint;
  private double ySetpoint;
  private double tSetpoint;

  private boolean reset;
  private double tolerance = 1.0;

  private Pose2d startingPose;
  private Pose2d resetPose;

  // Command constructor
  public SwerveGoTo(DriveSubsystem driveSubsystem, double xSetpoint, double ySetpoint, 
  double tSetpoint, boolean reset, Pose2d resetPose){

  this.reset = reset;
  this.resetPose = resetPose;

    addRequirements(driveSubsystem);

    tolerance = 0.05;

    this.driveSubsystem = driveSubsystem;

    this.tSetpoint = tSetpoint;
    this.xSetpoint = xSetpoint * 2;
    this.ySetpoint = ySetpoint * 2;

    vXController = new PIDController(1.8, 0.005, 0);
    vYController = new PIDController(1.4, 0.005, 0);

    thetaController = new PIDController(0.1, 0.004, 0.02);
    thetaController.enableContinuousInput(0, 360);

    vX = -0;
    vY = -0;
    vT = -0;
  }

  @Override
  public void initialize() {
    if(reset){
      driveSubsystem.resetOdometry(resetPose);
    }

    finished = false;

    startingPose = driveSubsystem.getPose();

  }

  @Override
  public void execute(){

    if(driveSubsystem.getPose().getY()  > (ySetpoint/2) - tolerance){
      if(driveSubsystem.getPose().getY()  < (ySetpoint/2) + tolerance){

        if(driveSubsystem.getPose().getX()  > (xSetpoint/2) - tolerance){
          if(driveSubsystem.getPose().getX()  < (xSetpoint/2) + tolerance){
            finished = true;
          }
        }

      }
    }

      vT = -thetaController.calculate(driveSubsystem.getRobotDegrees(), tSetpoint);

      vT = Math.abs(vT) > 0.05 ? vT : 0.0;

      double xError = ( driveSubsystem.getPose().getX() + (driveSubsystem.getPose().getX() - startingPose.getX()));
      double yError = ( driveSubsystem.getPose().getY() + (driveSubsystem.getPose().getY() - startingPose.getY()));
       
      vX = vXController.calculate(xError, xSetpoint); // X-Axis PID
      vY = vYController.calculate(yError, ySetpoint); // Y-Axis PID

      // Create chassis speeds
      ChassisSpeeds chassisSpeeds;
  
      // Apply chassis speeds with desired velocities
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, vT, new Rotation2d(driveSubsystem.getHeading()*Math.PI/180));

      // Create states array
      SwerveModuleState[] moduleStates = driveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds);
    
      // Move swerve modules
      driveSubsystem.setModuleStates(moduleStates);

  }

  // Stop all module motor movement when command ends
  @Override
  public void end(boolean interrupted){
    driveSubsystem.setModuleStates(
        driveSubsystem.getKinematics().toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                0,0,0,new Rotation2d())
        )
    );
  }

  @Override
  public boolean isFinished(){return finished;}

  
}