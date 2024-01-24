package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class ShootMoveShoot extends SequentialCommandGroup {
    private SwerveSubsystem drive;

    public ShootMoveShoot() {
        addCommands(new SwerveGoTo(drive, 0.5,0.3,Math.PI,true,new Pose2d()));
    //not finished-make a shooter!
    }
}
