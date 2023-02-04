// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.QuartetConstants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
  /** Enum for determining the mode of robot pickup, for pressures and LEDs */
  public static enum PickupMode {
    Cone,
    Cube,
    Error,
    None
  }

  private final Solenoid m_yellowLED = new Solenoid(PneumaticsModuleType.REVPH, ClawConstants.kYellowLEDPort);
  private final Solenoid m_purpleLED = new Solenoid(PneumaticsModuleType.REVPH, ClawConstants.kPurpleLEDPort);

  // TODO 30 psi foward 60 psi reverse
  private final DoubleSolenoid m_doubleSolenoidUpstream = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      ClawConstants.kDoubleSolenoidClawUpstream.getFirst(), ClawConstants.kDoubleSolenoidClawUpstream.getSecond());

  // TODO grab forward release reverse
  private final DoubleSolenoid m_doubleSolenoidDownstream = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      ClawConstants.kDoubleSolenoidClawDownstream.getFirst(), ClawConstants.kDoubleSolenoidClawDownstream.getSecond());

  // TODO switch to brake mode by making bc/cal RED
  private final PWMTalonSRX m_swivelMotor = new PWMTalonSRX(ClawConstants.kSwivelMotor);

  private PickupMode m_pickupMode = PickupMode.None;

  private final Timer m_strobeTimer = new Timer();

  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {

  }

  @Override
  public void periodic() {
    SmartDashboard.putString("CLAW MODE", m_pickupMode.name());

    if (m_pickupMode == PickupMode.Error) {
      if (m_strobeTimer.get() >= 0.27 && m_strobeTimer.get() <= 0.54) {
        m_yellowLED.set(false);
        m_purpleLED.set(true);
      } else {
        m_yellowLED.set(true);
        m_purpleLED.set(false);
        m_strobeTimer.reset();
      }
    }
  }

  /** run the swivel claw */
  public void runSwivel(double speed) {
    m_swivelMotor.set(speed);
  }

  public void setPickupMode(PickupMode mode) {
    m_pickupMode = mode;
    updateLEDs();
  }

  /** Save pickup mode then grab */
  public void grab(PickupMode mode) {
    m_pickupMode = mode;
    updateLEDs();
    grab();
  }

  /** Grab using saved pickup mode */
  public void grab() {
    switch (m_pickupMode) {
      case Cone:
        m_doubleSolenoidUpstream.set(DoubleSolenoid.Value.kReverse);
        m_doubleSolenoidDownstream.set(DoubleSolenoid.Value.kForward);
        break;
      case Cube:
        m_doubleSolenoidUpstream.set(DoubleSolenoid.Value.kForward);
        m_doubleSolenoidDownstream.set(DoubleSolenoid.Value.kForward);
        break;
      case Error:
      case None:
      default:
        m_doubleSolenoidUpstream.set(DoubleSolenoid.Value.kOff);
        m_doubleSolenoidDownstream.set(DoubleSolenoid.Value.kOff);
    }
  }

  public void release() {
    m_doubleSolenoidUpstream.set(DoubleSolenoid.Value.kOff); // TODO what the heck
    m_doubleSolenoidUpstream.set(DoubleSolenoid.Value.kReverse);
  }

  private void updateLEDs() {
    m_strobeTimer.reset();
    m_strobeTimer.stop();
    switch (m_pickupMode) {
      case Cone:
        m_yellowLED.set(true);
        m_purpleLED.set(false);
        break;
      case Cube:
        m_yellowLED.set(false);
        m_purpleLED.set(true);
        break;
      case Error:
        m_yellowLED.set(true);
        m_purpleLED.set(true);
        m_strobeTimer.start();
        break;
      case None:
      default:
        m_yellowLED.set(false);
        m_purpleLED.set(false);
        break;
    }
  }
}
