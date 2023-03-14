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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.QuartetConstants.LocationType;

import static frc.robot.Constants.QuartetConstants.ArmConstants.*;

import java.util.function.DoubleSupplier;

public class ArmSubsystem extends SubsystemBase {

  private final DoubleSolenoid m_lockSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, kLockSolenoidFwd,
      kLockSolenoidBkwd);
  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkMaxPIDController m_pidController;
  private final DoubleSupplier m_shoulderAngle;

  private double m_setpoint = LocationType.Starting.armInches;

  /** Creates a new ArmSubsystem, position units are inches. */
  public ArmSubsystem(DoubleSupplier shoulderAngle) {
    m_shoulderAngle = shoulderAngle;

    m_motor = new CANSparkMax(kMotorPort, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
    m_pidController = m_motor.getPIDController();

    m_motor.restoreFactoryDefaults();

    m_encoder.setPositionConversionFactor(kDistancePerRevInches);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(true);

    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ((float) kMinPosInches));
    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ((float) kMaxPosInches));

    m_motor.setSmartCurrentLimit(60, 30);

    m_pidController.setOutputRange(kMin, kMax);
    m_pidController.setP(kP);
    m_pidController.setD(kD);
    m_pidController.setFF(kF);

    new DoubleSolenoidSim(PneumaticsModuleType.REVPH, kLockSolenoidFwd, kLockSolenoidFwd);

    setName("ArmSubsystem");
  }

  /**
   * Runs motor to a specified position.
   * 
   * @param position the position (in subclass units) to set the motor to
   */
  private void setPosition(double position) {
    m_setpoint = position;
    m_pidController.setReference(position, ControlType.kPosition);
  }

  /** Stops motor from running, will interrupt any control mode. */
  public void disableMotor() {
    m_motor.setIdleMode(IdleMode.kCoast);
    m_motor.stopMotor();
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

  private boolean atSetpoint() {
    return (Math.abs(m_setpoint - getLength()) <= kPositionTolerance)
        && (Math.abs(getVelocity()) <= kVelocityTolerance);
  }

  private double maxSetpoint() {
    double horizMax = (41 / (Math.cos(Math.toRadians(m_shoulderAngle.getAsDouble())))) - 7; // 41 = max ext. (real 48),
                                                                                            // 7=arm ext. when enc 0
    double vertMax = (41 / (Math.cos(Math.toRadians(90 - m_shoulderAngle.getAsDouble())))) - 7;

    return Math.min(horizMax, vertMax);
  }

  @Override
  public void periodic() {
    if (m_setpoint > maxSetpoint()) {
      m_pidController.setReference(maxSetpoint(), ControlType.kPosition);
    } else if (m_setpoint < maxSetpoint()) {
      m_pidController.setReference(m_setpoint, ControlType.kPosition);
    }
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

  public CommandBase moveToLocation(LocationType location) {
    return this.runOnce(() -> setPosition(location.armInches)).andThen(new WaitUntilCommand(this::atSetpoint))
        .withName("ArmToLocation" + location.toString());
  }

  public CommandBase incrementPosition(double increment) {
    return this.runOnce(() -> setPosition(getLength() + increment)).andThen(new WaitUntilCommand(this::atSetpoint))
        .withName("ArmIncrement" + increment);
  }

  public CommandBase lockPosition() {
    return incrementPosition(0);
  }

  public void resetEncoders() {
    m_encoder.setPosition(LocationType.Starting.armInches);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("");
    builder.addDoubleProperty("Angle", m_encoder::getPosition, null);
    builder.addDoubleProperty("Setpoint", () -> m_setpoint, null);
    builder.addDoubleProperty("Max Setpoint", this::maxSetpoint, null);
    builder.addBooleanProperty("Reached", this::atSetpoint, null);
    builder.addBooleanProperty("Rev Limited", () -> m_motor.getFault(FaultID.kSoftLimitRev), null);
    builder.addBooleanProperty("Fwd Limited", () -> m_motor.getFault(FaultID.kSoftLimitFwd), null);
  }

}
