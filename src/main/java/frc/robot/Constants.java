package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    //NWU, m/s, radians
    public class AboveChassisConstants {
        public static int intakeMotorID1 = 9;
        public static int intakeMotorID2 = 10;
        public static int beltMotorID1 = 11;
        public static int beltMotorID2 = 12;
        public static int shooterMotorID1 = 13;
        //14 is bottom, 13 is top
        public static int shooterMotorID2 = 14;
        public static int climberID1 = 15;
        public static int pduID = 20;
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kBabyControllerPort = 1;
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