// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StoppyBarConstants;
import frc.robot.structs.LEDManager;

public class StoppyBarSubsystem extends SubsystemBase {
  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      StoppyBarConstants.kFowardSolenoidPort, StoppyBarConstants.kBackwardSolenoidPort);

  private boolean m_stoppyOn = false;

  /** Creates a new StoppyBarSubsystem. */
  public StoppyBarSubsystem() {
    m_doubleSolenoid.set(Value.kReverse);
    setName("StoppyBarSubsystem");
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("STOPPY ON", m_stoppyOn);
  }

  private void updateLEDs() {
    LEDManager.setStoppyBarLEDs(m_stoppyOn);
  }

  /** sets stoppy bar on/off, runs once */
  public CommandBase setStop(boolean on) {
    return this.runOnce(() -> {
      m_stoppyOn = on;
      updateLEDs();
      if (on) {
        m_doubleSolenoid.set(Value.kForward);
      } else {
        m_doubleSolenoid.set(Value.kReverse);
      }
    }).withName("SetStoppyBar");
  }
}
