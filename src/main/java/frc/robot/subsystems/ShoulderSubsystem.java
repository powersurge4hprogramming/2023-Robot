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

  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkMaxPIDController m_pidController;

  private double m_setpoint;

  /** Creates a new ShoulderSubsystem, position units are degrees. */
  public ShoulderSubsystem() {
    m_motor = new CANSparkMax(kMotorPort, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
    m_pidController = m_motor.getPIDController();

    m_motor.restoreFactoryDefaults();

    m_encoder.setPositionConversionFactor(kDegreesPerRev);
    m_encoder.setPosition(LocationType.Starting.shoulderDegrees);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(false);

    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) kMinDegrees);
    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) kMaxDegrees);

    m_motor.setSmartCurrentLimit(30, 25);

    m_pidController.setOutputRange(kMin, kMax);
    m_pidController.setP(kP);
    m_pidController.setD(kD);
    m_pidController.setFF(kF);

    setName("ShoulderSubsystem");
  }

  /**
   * Runs motor to a specified position.
   * 
   * @param angle the position (in subclass units) to set the motor to
   */
  private void setPosition(double angle) {
    m_setpoint = angle;
    m_pidController.setReference(angle, ControlType.kPosition);
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

  @Override
  public void periodic() {
  }

  public CommandBase moveToLocation(LocationType location) {
    return this.runOnce(() -> setPosition(location.shoulderDegrees)).andThen(new WaitUntilCommand(this::atSetpoint))
        .withName("ShoulderToLocation" + location.toString());
  }

  public CommandBase incrementPosition(double increment) {
    return this.runOnce(() -> setPosition(getLength() + increment)).andThen(new WaitUntilCommand(this::atSetpoint))
        .withName("ShoulderIncrement" + increment);
  }

  public CommandBase lockPosition() {
    return incrementPosition(0);
  }

  public void resetEncoders() {
    m_encoder.setPosition(LocationType.Starting.shoulderDegrees);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("");
    builder.addDoubleProperty("Angle", m_encoder::getPosition, null);
    builder.addDoubleProperty("Setpoint", () -> m_setpoint, null);
    builder.addBooleanProperty("Reached", this::atSetpoint, null);
    builder.addBooleanProperty("Rev Limited", () -> m_motor.getFault(FaultID.kSoftLimitRev), null);
    builder.addBooleanProperty("Fwd Limited", () -> m_motor.getFault(FaultID.kSoftLimitFwd), null);
  }
}
