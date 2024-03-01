// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BlinkySubsystem extends SubsystemBase {
  /** Creates a new BlinkySubsystem. */

  PowerDistribution m_PDU;
  public BlinkySubsystem() {
    m_PDU = new PowerDistribution(Constants.AboveChassisConstants.pduID, ModuleType.kRev);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLights(boolean loobean) {
    m_PDU.setSwitchableChannel(loobean);
  }

  public Command setLightsCommand(boolean loobean) {
    return run(() -> setLights(loobean));
  }
}
