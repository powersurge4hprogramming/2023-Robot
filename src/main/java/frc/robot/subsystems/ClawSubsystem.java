// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.QuartetConstants.ClawConstants.*;
import frc.robot.structs.LEDManager;

public class ClawSubsystem extends SubsystemBase {

  // forward 30, reverse, 60
  private final DoubleSolenoid m_doubleSolenoidUpstream = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      kClawUpstreamFwd, kClawUpstreamBkwd);

  // forward grab, reverse release
  private final DoubleSolenoid m_doubleSolenoidDownstream = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      kClawDownstreamFwd, kClawDownstreamBkwd);

  private PickupMode m_pickupMode = PickupMode.None;

  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    new DoubleSolenoidSim(PneumaticsModuleType.REVPH, kClawUpstreamFwd, kClawUpstreamBkwd);
    new DoubleSolenoidSim(PneumaticsModuleType.REVPH, kClawDownstreamFwd, kClawDownstreamBkwd);

    m_doubleSolenoidUpstream.set(DoubleSolenoid.Value.kOff);
    m_doubleSolenoidUpstream.set(DoubleSolenoid.Value.kReverse);

    setName("ClawSubsystem");
  }

  @Override
  public void periodic() {
  }

  /**
   * Updates the LEDs based on the {@link PickupMode} the robot is currently in.
   */
  private void updateLEDs() {
    LEDManager.setPickupLEDs(m_pickupMode);
  }

  /**
   * Changes the pickup mode and update LEDs
   * 
   * @param mode the new pickup mode
   * @return a command which changes the pickup mode, runs once
   */
  public CommandBase setPickupModeCommand(PickupMode mode) {
    return this.runOnce(() -> {
      m_pickupMode = mode;
      updateLEDs();
    }).withName("SetPickupMode");
  }

  /**
   * Saves the pickup mode then grabs the object
   * 
   * @param mode the new pickup mode
   * @return a command which changes the pickup mode and then grabs the object,
   *         runs once
   */
  public CommandBase grabCommand(PickupMode mode) {
    return this.runOnce(() -> {
      m_pickupMode = mode;
      updateLEDs();
      grabCommand();
    }).withName("GrabModeSet");
  }

  /**
   * Grabs the object based on the saved pickup mode
   * 
   * @return a command which grabs the object, runs once
   */
  public CommandBase grabCommand() {
    return this.runOnce(() -> {
      switch (m_pickupMode) {
        case Cone:
          m_doubleSolenoidUpstream.set(DoubleSolenoid.Value.kReverse);
          m_doubleSolenoidDownstream.set(DoubleSolenoid.Value.kForward);
          break;
        case Cube:
          m_doubleSolenoidUpstream.set(DoubleSolenoid.Value.kForward);
          m_doubleSolenoidDownstream.set(DoubleSolenoid.Value.kForward);
          break;
        case None:
        default:
          m_doubleSolenoidUpstream.set(DoubleSolenoid.Value.kOff);
          m_doubleSolenoidDownstream.set(DoubleSolenoid.Value.kOff);
          break;
      }
    }).withName("GrabUseMode");
  }

  /**
   * Releases the claw
   * 
   * @return a command which releases the claw, runs once
   */
  public CommandBase releaseCommand() {
    return this.runOnce(() -> {
      m_doubleSolenoidUpstream.set(DoubleSolenoid.Value.kOff);
      m_doubleSolenoidDownstream.set(DoubleSolenoid.Value.kReverse);
    }).withName("Release");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("");
    builder.addStringProperty("Mode", () -> m_pickupMode.toString(), null);
    builder.addBooleanProperty("Grabbed", () -> m_doubleSolenoidDownstream.get() == Value.kForward, null);
  }
}
