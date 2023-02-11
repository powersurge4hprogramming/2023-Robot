// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StoppyBarConstants;

public class StoppyBarSubsystem extends SubsystemBase {
  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      StoppyBarConstants.kFowardSolenoidPort, StoppyBarConstants.kBackwardSolenoidPort);

  /** Creates a new StoppyBarSubsystem. */
  public StoppyBarSubsystem() {
    setName("StoppyBarSubsystem");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setStop(boolean on) {
    if (on) {
      m_doubleSolenoid.set(Value.kForward);
    } else {
      m_doubleSolenoid.set(Value.kReverse);
    }
  }
}
