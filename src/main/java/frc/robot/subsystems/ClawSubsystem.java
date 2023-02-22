// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.QuartetConstants.ClawConstants;
import frc.robot.structs.LEDManager;

public class ClawSubsystem extends SubsystemBase {
  /** Enum for determining the mode of robot pickup, for pressures. */
  public static enum PickupMode {
    Cone,
    Cube,
    None
  }

  // forward 30, reverse, 60
  private final DoubleSolenoid m_doubleSolenoidUpstream = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      ClawConstants.kDoubleSolenoidClawUpstream.getFirst(), ClawConstants.kDoubleSolenoidClawUpstream.getSecond());

  // forward grab, reverse release
  private final DoubleSolenoid m_doubleSolenoidDownstream = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      ClawConstants.kDoubleSolenoidClawDownstream.getFirst(), ClawConstants.kDoubleSolenoidClawDownstream.getSecond());

  private final CANSparkMax m_swivelMotor = new CANSparkMax(ClawConstants.kSwivelMotor, MotorType.kBrushed);

  private PickupMode m_pickupMode = PickupMode.None;

  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    m_swivelMotor.setIdleMode(IdleMode.kBrake);

    new DoubleSolenoidSim(PneumaticsModuleType.REVPH,
        ClawConstants.kDoubleSolenoidClawUpstream.getFirst(), ClawConstants.kDoubleSolenoidClawUpstream.getSecond());
    new DoubleSolenoidSim(PneumaticsModuleType.REVPH,
        ClawConstants.kDoubleSolenoidClawDownstream.getFirst(),
        ClawConstants.kDoubleSolenoidClawDownstream.getSecond());

    m_doubleSolenoidUpstream.set(DoubleSolenoid.Value.kOff);
    m_doubleSolenoidUpstream.set(DoubleSolenoid.Value.kReverse);

    setName("ClawSubsystem");
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("CLAW MODE", m_pickupMode.name());
  }

  /**
   * Runs the swivel to a specified speed.
   * 
   * @param speed the duty cycle speed to set the motor (-1 to 1)
   */
  private void runSwivel(double speed) {
    m_swivelMotor.set(speed);
  }

  /**
   * Updates the LEDs based on the {@link PickupMode} the robot is currently in.
   */
  private void updateLEDs() {
    LEDManager.setPickupLEDs(m_pickupMode);
  }

  /**
   * Runs the swivel
   * 
   * @param speed the duty cycle speed to set the motor (-1 to 1)
   * @return a command which runs the swivel until interrupted
   */
  public CommandBase runSwivelCommand(double speed) {
    return this.startEnd(() -> runSwivel(speed), () -> runSwivel(0.0)).withName("RunSwivel");
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
}
