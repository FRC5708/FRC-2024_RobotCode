package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    //NWU, m/s, radians
    public class AboveChassisConstants {
        public static int intakeMotorID = 0;
        public static int beltMotorID = 0;
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final double driveDeadband = 0.05;
      }
    
      public static class ModuleConstants {
        // The max free speed of the module
        public static final double maxSpeed = 4.5;
      }
}