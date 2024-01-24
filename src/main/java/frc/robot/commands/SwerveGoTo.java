package frc.robot.commands;
import frc.robot.subsystems.SwerveSubsystem;
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
  SwerveSubsystem swerveSubsystem;

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
  public SwerveGoTo(SwerveSubsystem swerveSubsystem, double xSetpoint, double ySetpoint, 
  double tSetpoint, boolean reset, Pose2d resetPose){

  this.reset = reset;
  this.resetPose = resetPose;

    addRequirements(swerveSubsystem);

    tolerance = 0.05;

    this.swerveSubsystem = swerveSubsystem;

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
      swerveSubsystem.resetOdometry(resetPose);
    }

    finished = false;

    startingPose = swerveSubsystem.getPose();

  }

  @Override
  public void execute(){

    if(swerveSubsystem.getPose().getY()  > (ySetpoint/2) - tolerance){
      if(swerveSubsystem.getPose().getY()  < (ySetpoint/2) + tolerance){

        if(swerveSubsystem.getPose().getX()  > (xSetpoint/2) - tolerance){
          if(swerveSubsystem.getPose().getX()  < (xSetpoint/2) + tolerance){
            finished = true;
          }
        }

      }
    }

      vT = -thetaController.calculate(swerveSubsystem.getRobotDegrees(), tSetpoint);

      vT = Math.abs(vT) > 0.05 ? vT : 0.0;

      double xError = ( swerveSubsystem.getPose().getX() + (swerveSubsystem.getPose().getX() - startingPose.getX()));
      double yError = ( swerveSubsystem.getPose().getY() + (swerveSubsystem.getPose().getY() - startingPose.getY()));
       
      vX = vXController.calculate(xError, xSetpoint); // X-Axis PID
      vY = vYController.calculate(yError, ySetpoint); // Y-Axis PID

      // Create chassis speeds
      ChassisSpeeds chassisSpeeds;
  
      // Apply chassis speeds with desired velocities
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, vT, new Rotation2d(swerveSubsystem.getHeading()*Math.PI/180));

      // Create states array
      SwerveModuleState[] moduleStates = swerveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds);
    
      // Move swerve modules
      swerveSubsystem.setModuleStates(moduleStates);

  }

  // Stop all module motor movement when command ends
  @Override
  public void end(boolean interrupted){
    swerveSubsystem.setModuleStates(
        swerveSubsystem.getKinematics().toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                0,0,0,new Rotation2d())
        )
    );
  }

  @Override
  public boolean isFinished(){return finished;}

  
}