package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    //NWU, m/s, radians
    public class DriveConstants {
        //goofy numbers with no experimental basis
        public static double maxSpeed = 3;
        public static double maxAngularSpeed = 2;

        public static double teleOpMaxAccel = 7;
        public static double teleOpMaxAngularAccel = 3;

        public static double angleP = 0;
        public static double angleI = 0;
        public static double angleD = 0;
    }

    public class SwerveConstants {
;
        //DON'T KNOW THESE
        public static double turningP = 0;
        public static double turningI = 0;
        public static double turningD = 0;
        public static double driveP = 0;
        public static double driveI = 0;
        public static double driveD = 0;

        public static double trackWidth = 0.5588;
        public static double wheelBase = 0.4826;

        public static Translation2d frontLeftPosition = new Translation2d(trackWidth/2,wheelBase/2);
        public static Translation2d frontRightPosition = new Translation2d(-trackWidth/2,wheelBase/2);
        public static Translation2d backLeftPosition = new Translation2d(trackWidth/2,-wheelBase/2);
        public static Translation2d backRightPosition = new Translation2d(-trackWidth/2,-wheelBase/2);

        public static int frontLeftDriveMotor = 0;
        public static int frontLeftTurningMotor = 0;
        public static int frontLeftDriveEncoderA = 0;
        public static int frontLeftDriveEncoderB = 0;
        public static int frontLeftTurningEncoderA = 0;
        public static int frontLeftTurningEncoderB = 0;

        public static int frontRightDriveMotor = 0;
        public static int frontRightTurningMotor = 0;
        public static int frontRightDriveEncoderA = 0;
        public static int frontRightDriveEncoderB = 0;
        public static int frontRightTurningEncoderA = 0;
        public static int frontRightTurningEncoderB = 0;

        public static int backRightDriveMotor = 0;
        public static int backRightTurningMotor = 0;
        public static int backRightDriveEncoderA = 0;
        public static int backRightDriveEncoderB = 0;
        public static int backRightTurningEncoderA = 0;
        public static int backRightTurningEncoderB = 0;

        public static int backLeftDriveMotor = 0;
        public static int backLeftTurningMotor = 0;
        public static int backLeftDriveEncoderA = 0;
        public static int backLeftDriveEncoderB = 0;
        public static int backLeftTurningEncoderA = 0;
        public static int backLeftTurningEncoderB = 0;
    }
}