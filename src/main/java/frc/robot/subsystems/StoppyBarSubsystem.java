// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.StoppyBarConstants.*;

public class StoppyBarSubsystem extends SubsystemBase {
  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, kFowardSolenoidPort,
      kBackwardSolenoidPort);

  /** Creates a new StoppyBarSubsystem. */
  public StoppyBarSubsystem() {
    m_doubleSolenoid.set(Value.kReverse);
    setName("StoppyBarSubsystem");
  }

  private void setStop(boolean on) {
    if (on) {
      m_doubleSolenoid.set(Value.kForward);
    } else {
      m_doubleSolenoid.set(Value.kReverse);
    }
  }

  /**
   * Sets stoppy bar on/off and updates LEDs
   * 
   * @param on whether the stoppy bar should be set on
   * @return a command which updates the stoppy bar and LEDs, runs once
   */
  public CommandBase setStopCommand(boolean on) {
    return this.runOnce(() -> setStop(on)).withName("SetStoppyBar");
  }

  public boolean stoppyOn() {
    return m_doubleSolenoid.get() == Value.kForward;
  }

  @Override
  public void periodic() {
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("");
    builder.addBooleanProperty("On", this::stoppyOn, null);
  }
}
