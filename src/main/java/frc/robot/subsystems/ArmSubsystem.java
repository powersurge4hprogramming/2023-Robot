// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.QuartetConstants.LocationType;

import static frc.robot.Constants.QuartetConstants.ArmConstants.*;

public class ArmSubsystem extends SubsystemBase {

  private final DigitalInput m_limitSwitch = new DigitalInput(kLimitSwitchPort);

  private final DoubleSolenoid m_lockSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, kLockSolenoidFwd,
      kLockSolenoidBkwd);
  private final CANSparkMax m_motor = new CANSparkMax(kMotorPort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  private final SparkMaxPIDController m_pidController = m_motor.getPIDController();

  private double m_setpoint = LocationType.Starting.armInches;

  /** Creates a new ArmSubsystem, position units are inches. */
  public ArmSubsystem() {
    m_motor.restoreFactoryDefaults();

    m_encoder.setPositionConversionFactor(kDistancePerRevInches);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(true);

    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    //m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ((float) kMinPosInches));
    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ((float) kMaxPosInches));

    m_motor.setSmartCurrentLimit(60, 30);

    m_pidController.setOutputRange(kMin, kMax);
    m_pidController.setP(kP);
    m_pidController.setD(kD);
    m_pidController.setFF(kF);

    new DoubleSolenoidSim(PneumaticsModuleType.REVPH, kLockSolenoidFwd, kLockSolenoidFwd);
    new DIOSim(m_limitSwitch);

    setName("ArmSubsystem");
  }

  /**
   * Gets the position
   * 
   * @return the position of the encoder in degrees
   */
  public double getLength() {
    return m_encoder.getPosition();
  }

  /**
   * Gets the velocity
   * 
   * @return the velocity in rpm
   */
  private double getVelocity() {
    return m_encoder.getVelocity();
  }

  /**
   * Runs motor to a specified position.
   * 
   * @param position the position (in subclass units) to set the motor to
   */
  private void setPosition(double position) {
    if (m_setpoint != position) {
      m_setpoint = position;
      m_pidController.setReference(position, ControlType.kPosition);
    }
  }

  private boolean atSetpoint() {
    return (Math.abs(m_setpoint - getLength()) <= kPositionTolerance)
        && (Math.abs(getVelocity()) <= kVelocityTolerance);
  }

  /** Stops motor from running, will interrupt any control mode. */
  public void disableMotor() {
    m_motor.setIdleMode(IdleMode.kCoast);
    m_motor.stopMotor();
  }

  @Override
  public void periodic() {
    if (m_limitSwitch.get() == true) {
      m_encoder.setPosition(0);
      if (m_motor.get() < 0) {
        m_motor.set(0.0);
      }
    }
  }

  private CommandBase moveToLength(double length) {
    return this.runOnce(() -> setPosition(length)).andThen(new WaitUntilCommand(this::atSetpoint))
        .handleInterrupt(() -> setPosition(m_encoder.getPosition()))
        .withName("ArmToLength" + length);
  }

  public CommandBase moveToLocation(LocationType location) {
    return moveToLength(location.armInches);
  }

  public CommandBase incrementPosition(double increment) {
    return moveToLength(m_setpoint + increment);
  }

  public CommandBase lockPosition() {
    return moveToLength(m_setpoint);
  }

  public void resetEncoders() {
    m_encoder.setPosition(LocationType.Starting.armInches);
  }

  /**
   * Sets the lock mode on the arm.
   * 
   * @param locked whether the arm should be locked
   */
  public void setArmLock(boolean locked) {
    if (locked) {
      m_lockSolenoid.set(Value.kForward);
    } else {
      m_lockSolenoid.set(Value.kReverse);
    }
  }

  /**
   * Toggles the arm's locking
   * 
   * @return a command which toggles the arm lock and the finishes
   */
  public CommandBase toggleArmLock() {
    return this.runOnce(() -> {
      m_lockSolenoid.toggle();
    }).withName("ToggleArmLock");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("");
    builder.addDoubleProperty("Length", m_encoder::getPosition, null);
    builder.addDoubleProperty("Setpoint", () -> m_setpoint, null);
    builder.addBooleanProperty("Reached", this::atSetpoint, null);
    builder.addBooleanProperty("Down", () -> m_limitSwitch.get(), null);
    builder.addBooleanProperty("Rev Limited", () -> m_motor.getFault(FaultID.kSoftLimitRev), null);
    builder.addBooleanProperty("Fwd Limited", () -> m_motor.getFault(FaultID.kSoftLimitFwd), null);
  }

}
