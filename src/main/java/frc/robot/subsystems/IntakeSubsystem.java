package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AboveChassisConstants;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax intakeMotor = new CANSparkMax(AboveChassisConstants.intakeMotorID, MotorType.kBrushless);
    private CANSparkMax beltMotor = new CANSparkMax(AboveChassisConstants.beltMotorID,MotorType.kBrushless);

    public IntakeSubsystem() {}
    //powers  from -1 to 1
    public void runIntake(double power) {
        intakeMotor.set(power);
    }

    public void runBelt(double power) {
        beltMotor.set(power);
    }
}
