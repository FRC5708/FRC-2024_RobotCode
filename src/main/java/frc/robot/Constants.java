package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    //NWU, m/s, radians
    public class AboveChassisConstants {
        public static int intakeMotorID = 0;
        public static int beltMotorID = 0;
        public static int shooterMotorID1 = 0;
        public static int shooterMotorID2 = 0;
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final double driveDeadband = 0.05;
      }
    
      public static class ModuleConstants {
        // The max free speed of the module
        public static final double maxSpeed = 4.5;

        public static double trackWidth = 0.5588;
        public static double wheelBase = 0.4826;

        public static Translation2d frontLeftPosition = new Translation2d(trackWidth/2,wheelBase/2);
        public static Translation2d frontRightPosition = new Translation2d(-trackWidth/2,wheelBase/2);
        public static Translation2d backLeftPosition = new Translation2d(trackWidth/2,-wheelBase/2);
        public static Translation2d backRightPosition = new Translation2d(-trackWidth/2,-wheelBase/2);

        public static SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(frontLeftPosition,
                                                                                frontRightPosition,
                                                                                backLeftPosition,
                                                                                backRightPosition);

      }
}