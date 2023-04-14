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
    m_encoder.setPosition(m_setpoint);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(true);

    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    // m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ((float)
    // kMinPosInches));
    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ((float) kMaxPosInches));

    m_motor.setSmartCurrentLimit(20, 25);

    m_pidController.setOutputRange(kMin, kMax);
    m_pidController.setP(kP);
    m_pidController.setD(kD);
    m_pidController.setFF(kF);

    new DoubleSolenoidSim(PneumaticsModuleType.REVPH, kLockSolenoidFwd, kLockSolenoidFwd);
    new DIOSim(m_limitSwitch);

    setName("ArmSubsystem");
  }

  private void setSpeed(double speed) {
    m_motor.set(speed);
  }

  /**
   * Sets the speed of the arm
   * 
   * @param speed the speed from -1 to 1 of the arm motor
   * @return a command which sets the arm to the speed and then zeroes and locks
   *         the arm on interrupt
   */
  public CommandBase setSpeedCmd(double speed) {
    return this.startEnd(() -> setSpeed(speed), () -> setSpeed(0.0)).finallyDo((end) -> lockPosition())
        .withName("ArmRunSpeed");
  }

  /**
   * Runs arm to a specified position.
   * 
   * @param position the length to set the motor to
   */
  private void setPosition(double position) {
    if (m_setpoint != position) {
      m_setpoint = position;
      m_pidController.setReference(position, ControlType.kPosition);
    }
  }

  private CommandBase moveToLength(double length) {
    return this.runOnce(() -> setPosition(length)).andThen(new WaitUntilCommand(this::atSetpoint))
        .handleInterrupt(this::lockPosition)
        .withName("ArmToLength" + length);
  }

  public CommandBase moveToLocation(LocationType location) {
    return moveToLength(location.armInches);
  }

  private void incrementPosition(double increment) {
    m_setpoint = m_setpoint + increment;
    m_pidController.setReference(m_setpoint, ControlType.kPosition);
  }

  /**
   * Increments arm a specified amount
   * 
   * @param increment the length to increment the arm
   * @return a command which increments the arm setpoint without awaiting the
   *         setpoint
   */
  public CommandBase incrementLength(double increment) {
    return this.runOnce(() -> incrementPosition(increment)).withName("ArmIncrement" + increment);
  }

  private void lockPosition() {
    m_setpoint = m_encoder.getPosition();
    m_pidController.setReference(m_setpoint, ControlType.kPosition);
  }

  /**
   * Locks the arm
   * 
   * @return a command which sets the arm to the current setpoint
   */
  public CommandBase lockLength() {
    return this.runOnce(() -> lockPosition()).withName("ArmPositionLock");
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

  public boolean atSetpoint() {
    return (Math.abs(m_setpoint - getLength()) <= kPositionTolerance);
    // && (Math.abs(getVelocity()) <= kVelocityTolerance);
  }

  private void disableMotor() {
    m_motor.setIdleMode(IdleMode.kCoast);
    m_motor.stopMotor();
  }

  /**
   * @return an IRREVERSIBLE command which stops the arm from running and sends it
   *         to coast,
   *         will interrupt any control
   *         mode.
   */
  public CommandBase disableMotorCommand() {
    return this.runOnce(() -> disableMotor()).withName("DisableArmMotor");
  }

  private void resetEncoder() {
    m_encoder.setPosition(LocationType.Starting.armInches);
  }

  public CommandBase resetEncoderCommand() {
    return this.runOnce(() -> resetEncoder()).withName("ResetArmEncoder");
  }

  /**
   * Sets the lock mode on the arm.
   * 
   * @param locked whether the arm should be locked
   */
  private void setArmLock(boolean locked) {
    if (locked) {
      m_lockSolenoid.set(Value.kForward);
    } else {
      m_lockSolenoid.set(Value.kReverse);
    }
  }

  /**
   * Sets the lock mode on the arm.
   * 
   * @param locked whether the arm should be locked
   * @return a command which sets the arm lock and finishes
   */
  public CommandBase setArmLockCommand(boolean locked) {
    return this.runOnce(() -> setArmLock(locked)).withName("SetArmLock");
  }

  @Override
  public void periodic() {
    if ((m_limitSwitch.get() == true) && (m_motor.getAppliedOutput() <= 0.0 || m_setpoint <= 0.0)) {
      m_encoder.setPosition(0);
      setPosition(0.0);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("");
    builder.addDoubleProperty("Length", m_encoder::getPosition, null);
    builder.addDoubleProperty("Setpoint", () -> m_setpoint, null);
    builder.addBooleanProperty("Reached", this::atSetpoint, null);
    builder.addBooleanProperty("Soft Limited",
        () -> m_motor.getFault(FaultID.kSoftLimitRev) || m_motor.getFault(FaultID.kSoftLimitFwd), null);
    builder.addBooleanProperty("Down", () -> m_limitSwitch.get(), null);
    builder.addBooleanProperty("Arm Locked", () -> m_lockSolenoid.get() == Value.kForward, null);
    builder.addDoubleProperty("Duty Cycle", () -> m_motor.getAppliedOutput(), null);

  }

}
