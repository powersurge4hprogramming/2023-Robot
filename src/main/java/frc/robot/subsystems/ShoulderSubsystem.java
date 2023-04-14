// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.QuartetConstants.LocationType;

import static frc.robot.Constants.QuartetConstants.ShoulderConstants.*;

public class ShoulderSubsystem extends SubsystemBase {

  private final CANSparkMax m_motor = new CANSparkMax(kMotorPort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  private final SparkMaxPIDController m_pidController = m_motor.getPIDController();

  private double m_setpoint = LocationType.Starting.shoulderDegrees;

  /** Creates a new ShoulderSubsystem, position units are degrees. */
  public ShoulderSubsystem() {
    m_motor.restoreFactoryDefaults();

    m_encoder.setPositionConversionFactor(kDegreesPerRev);
    m_encoder.setPosition(m_setpoint);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(false);

    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) kMinDegrees);
    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) kMaxDegrees);

    m_motor.setSmartCurrentLimit(20, 25);

    m_pidController.setOutputRange(kMin, kMax);
    m_pidController.setP(kP);
    m_pidController.setD(kD);
    m_pidController.setFF(kF);

    setName("ShoulderSubsystem");
  }

  private void setSpeed(double speed) {
    m_motor.set(speed);
  }

  /**
   * Sets the speed of the shoulder
   * 
   * @param speed the speed from -1 to 1 of the shoulder motor
   * @return a command which sets the shoulder to the speed and then zeroes and
   *         locks
   *         the shoulder on interrupt
   */
  public CommandBase setSpeedCmd(double speed) {
    return this.startEnd(() -> setSpeed(speed), () -> setSpeed(0.0)).finallyDo((end) -> lockPosition())
        .withName("ShoulderRunSpeed");
  }

  /**
   * Runs motor to a specified position.
   * 
   * @param angle the angle to set the motor to
   */
  private void setPosition(double angle) {
    m_setpoint = angle;
    m_pidController.setReference(angle, ControlType.kPosition);
  }

  private CommandBase moveToAngle(double angle) {
    return this.runOnce(() -> setPosition(angle)).andThen(new WaitUntilCommand(this::atSetpoint))
        .handleInterrupt(this::lockPosition)
        .withName("ShoulderToAngle" + angle);
  }

  public CommandBase moveToLocation(LocationType location) {
    return moveToAngle(location.shoulderDegrees);
  }

  private void incrementPosition(double increment) {
    m_setpoint = m_setpoint + increment;
    m_pidController.setReference(m_setpoint, ControlType.kPosition);
  }

  /**
   * Increments shoulder a specified amount
   * 
   * @param increment the angle to increment the shoulder
   * @return a command which increments the shoulder setpoint without awaiting the
   *         setpoint
   */
  public CommandBase incrementAngle(double increment) {
    return this.runOnce(() -> incrementPosition(increment)).withName("ShoulderIncrement" + increment);
  }

  private void lockPosition() {
    m_setpoint = m_encoder.getPosition();
    m_pidController.setReference(m_setpoint, ControlType.kPosition);
  }

  /**
   * Locks the shoulder
   * 
   * @return a command which sets the shoulder to the current setpoint
   */
  public CommandBase lockAngle() {
    return this.runOnce(() -> lockPosition()).withName("ShoulderAngleLock");
  }

  /**
   * Gets the position
   * 
   * @return the position of the encoder in degrees
   */
  public double getAngle() {
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
    return (Math.abs(m_setpoint - getAngle()) <= kPositionTolerance)
        && (Math.abs(getVelocity()) <= kVelocityTolerance);
  }

  private void disableMotor() {
    m_motor.setIdleMode(IdleMode.kCoast);
    m_motor.stopMotor();
  }

  /**
   * @return an IRREVERSIBLE command which stops the shoulder from running and
   *         sends it
   *         to coast,
   *         will interrupt any control
   *         mode.
   */
  public CommandBase disableMotorCommand() {
    return this.runOnce(() -> disableMotor()).withName("DisableShoulderMotor");
  }

  private void resetEncoder() {
    m_encoder.setPosition(LocationType.Starting.shoulderDegrees);
  }

  public CommandBase resetEncoderCommand() {
    return this.runOnce(() -> resetEncoder()).withName("ResetShoulderEncoder");
  }

  @Override
  public void periodic() {
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("");
    builder.addDoubleProperty("Angle", m_encoder::getPosition, null);
    builder.addDoubleProperty("Setpoint", () -> m_setpoint, null);
    builder.addBooleanProperty("Reached", this::atSetpoint, null);
    builder.addBooleanProperty("Soft Limited",
        () -> m_motor.getFault(FaultID.kSoftLimitRev) || m_motor.getFault(FaultID.kSoftLimitFwd), null);
  }
}
