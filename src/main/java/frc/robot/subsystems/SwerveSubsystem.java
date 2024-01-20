package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj.SPI;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(SwerveConstants.frontLeftPosition,
                                                                                SwerveConstants.frontRightPosition,
                                                                                SwerveConstants.backLeftPosition,
                                                                                SwerveConstants.backRightPosition);

    // Create 4 swerve modules with attributes from constants
    private final SwerveModule frontLeft = new SwerveModule(
            SwerveConstants.frontLeftDriveMotor,
            SwerveConstants.frontLeftTurningMotor,
            SwerveConstants.frontLeftDriveEncoderA,
            SwerveConstants.frontLeftDriveEncoderB,
            SwerveConstants.frontLeftTurningEncoderA,
            SwerveConstants.frontLeftTurningEncoderB
            );

    private final SwerveModule frontRight = new SwerveModule(
            SwerveConstants.frontRightDriveMotor,
            SwerveConstants.frontRightTurningMotor,
            SwerveConstants.frontLeftDriveEncoderA,
            SwerveConstants.frontLeftDriveEncoderB,
            SwerveConstants.frontLeftTurningEncoderA,
            SwerveConstants.frontLeftTurningEncoderB);

    private final SwerveModule backLeft = new SwerveModule(
            SwerveConstants.backLeftDriveMotor,
            SwerveConstants.backLeftTurningMotor,
            SwerveConstants.backLeftDriveEncoderA,
            SwerveConstants.backLeftDriveEncoderB,
            SwerveConstants.backLeftTurningEncoderA,
            SwerveConstants.backRightTurningEncoderB);

    private final SwerveModule backRight = new SwerveModule(
            SwerveConstants.backRightDriveMotor,
            SwerveConstants.backRightTurningMotor,
            SwerveConstants.backRightDriveEncoderA,
            SwerveConstants.backRightDriveEncoderB,
            SwerveConstants.backRightTurningEncoderA,
            SwerveConstants.backRightTurningEncoderB); 

  // Create the navX using roboRIO expansion port
  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  
  // Returns positions of the swerve modules for odometry
  public SwerveModulePosition[] getModulePositions(){

    return( new SwerveModulePosition[]{
      frontLeft.getPosition(), 
      frontRight.getPosition(), 
      backLeft.getPosition(),
      backRight.getPosition()});

  }
  private SwerveDriveOdometry odometer;

  // Swerve subsystem constructor
  public SwerveSubsystem() {

    // Reset robot encoders on startup
    resetAllEncoders();

    // Zero navX heading on new thread when robot starts
    new Thread(() -> {
        try {
            Thread.sleep(1000);
            //gyro.calibrate();
            zeroHeading();
        } catch (Exception e) {
        }
    }).start();

    // Set robot odometry object to current robot heading and swerve module positions
    odometer = new SwerveDriveOdometry(driveKinematics, 
    getOdometryAngle(), getModulePositions());
    // new Rotation2d(gyro.getYaw() * -1 / 180 * Math.PI), getModulePositions()

  }

  // Reset gyro heading 
  public void zeroHeading() {
    gyro.reset();
  }

  // Reset gyro yaw
  public void resetYaw(){
    gyro.zeroYaw();
  }

  /*public void calibrateGyro(){
    gyro.calibrate();
  }*/

  // Return gyro heading, make sure to read navx docs on this
  public double getHeading(){
    return gyro.getAngle();
  }

  // Return the robot odometry in pose meters
  public Pose2d getOdometryMeters(){
    return(odometer.getPoseMeters());
  }

  // Return heading in Rotation2d format
  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  // Move the swerve modules to the desired SwerveModuleState
  public void setModuleStates(SwerveModuleState[] desiredStates) {

    // Make sure robot rotation is all ways possible by changing other module roation speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxSpeed);
    
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
}

  // Return robot position caculated by odometer
  public Pose2d getPose(){
    return odometer.getPoseMeters();
  }

  // Reset odometer to new Pose2d location
  public void resetOdometry(Pose2d pose){
   odometer.resetPosition(getOdometryAngle(), getModulePositions(), pose);
  }

  // Reset odometer to new Pose2d location but with roation
  public void resetOdometry(Pose2d pose, Rotation2d rot){
    odometer.resetPosition(rot, getModulePositions(), pose);
  }

  // Return an angle from -180 to 180 for robot odometry
  public Rotation2d getOdometryAngle(){
    return(Rotation2d.fromDegrees(gyro.getYaw()));
  }

  // Reset all swerve module encoders
  public void resetAllEncoders(){
      frontLeft.resetEncoders();
      frontRight.resetEncoders();
      backLeft.resetEncoders();
      backRight.resetEncoders();
  }
}