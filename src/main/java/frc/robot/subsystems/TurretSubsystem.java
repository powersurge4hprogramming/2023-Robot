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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static frc.robot.Constants.QuartetConstants.TurretConstants.*;

public class TurretSubsystem extends SubsystemBase {

  private final CANSparkMax m_motor = new CANSparkMax(kMotorPort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  private final SparkMaxPIDController m_pidController = m_motor.getPIDController();

  private double m_setpoint = kStartingDegrees;

  /** Creates a new TurretSubsystem, position units are degrees. */
  public TurretSubsystem() {
    m_motor.restoreFactoryDefaults();

    m_motor.setInverted(true);
    m_encoder.setPositionConversionFactor(kDegreesPerRev);
    m_encoder.setPosition(kStartingDegrees);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(false);

    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -((float) kMaxRotations * 360));
    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ((float) kMaxRotations * 360));

    m_motor.setSmartCurrentLimit(20, 25);

    m_pidController.setOutputRange(kMin, kMax);
    m_pidController.setP(kP);
    m_pidController.setD(kD);
    m_pidController.setFF(kF);
    m_pidController.setPositionPIDWrappingEnabled(true);
    m_pidController.setPositionPIDWrappingMinInput(-180);
    m_pidController.setPositionPIDWrappingMaxInput(180);

    setName("TurretSubsystem");
  }

  private void setSpeed(double speed) {
    m_motor.set(speed);
  }

  /**
   * Sets the speed of the turret
   * 
   * @param speed the speed from -1 to 1 of the turret motor
   * @return a command which sets the turret to the speed and then zeroes and
   *         locks
   *         the turret on interrupt
   */
  public CommandBase setSpeedCmd(double speed) {
    return this.startEnd(() -> setSpeed(speed), () -> setSpeed(0.0)).finallyDo((end) -> lockPosition())
        .withName("TurretRunSpeed");
  }

  /**
   * Runs motor to a specified position.
   * 
   * @param angle the position (in degrees) to set the motor to
   */
  public void setPosition(double angle) {
    if (m_setpoint != angle) {
      m_setpoint = angle;
      m_pidController.setReference(m_setpoint, ControlType.kPosition);
    }
  }

  /**
   * Runs turret to a specified position.
   * 
   * @param angle the position (in degrees) to set the turret motor to
   */
  public CommandBase moveToAngle(double angle) {
    return this.runOnce(() -> setPosition(angle)).andThen(new WaitUntilCommand(this::atSetpoint))
        .handleInterrupt(this::lockPosition)
        .withName("TurretToAngle" + angle);
  }

  /**
   * Resets the turret completely
   * 
   * @return a command which resets the turret to its starting position
   *         irregardless of position PID wrapping
   */
  public CommandBase absoluteReset() {
    return moveToAngle(kStartingDegrees)
        .beforeStarting(() -> m_pidController.setPositionPIDWrappingEnabled(false), new Subsystem[0])
        .finallyDo((boolean interrupted) -> m_pidController.setPositionPIDWrappingEnabled(true))
        .withName("TurretAbsoluteReset");
  }

  private void incrementPosition(double increment) {
    m_setpoint = m_setpoint + increment;
    m_pidController.setReference(m_setpoint, ControlType.kPosition);
  }

  /**
   * Increments turret a specified amount
   * 
   * @param increment the angle to increment the turret
   * @return a command which increments the turret setpoint without awaiting the
   *         setpoint
   */
  public CommandBase incrementAngle(double increment) {
    return this.runOnce(() -> incrementPosition(increment)).withName("TurretIncrement" + increment);
  }

  private void lockPosition() {
    m_setpoint = m_encoder.getPosition();
    m_pidController.setReference(m_setpoint, ControlType.kPosition);
  }

  public CommandBase lockAngle() {
    return this.runOnce(() -> lockPosition()).withName("TurretAngleLock");
  }

  /**
   * Gets the position
   * 
   * @return the position of the encoder in inches or degrees
   */
  public double getDegrees() {
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
    /*
     * return (Math.abs(m_setpoint - getLength()) <= kPositionTolerance)
     * && (Math.abs(getVelocity()) <= kVelocityTolerance);
     */

    if (Math.abs(getVelocity()) >= kVelocityTolerance) {
      return false;
    }

    // from wpi PIDController
    double positionError = MathUtil.inputModulus(m_setpoint - getDegrees(), -180, 180);
    return (positionError <= kPositionTolerance);
  }

  private void disableMotor() {
    m_motor.setIdleMode(IdleMode.kCoast);
    m_motor.stopMotor();
  }

  /**
   * @return an IRREVERSIBLE command which stops the turret from running and sends
   *         it
   *         to coast,
   *         will interrupt any control
   *         mode.
   */
  public CommandBase disableMotorCommand() {
    return this.runOnce(() -> disableMotor()).withName("DisableTurretrMotor");
  }

  private void resetEncoder() {
    m_encoder.setPosition(kStartingDegrees);
  }

  /**
   *
   * @return a command which resets the encoder to kStartingDegrees
   */
  public CommandBase resetEncoderCommand() {
    return this.runOnce(() -> resetEncoder()).withName("ResetTurretEncoder");
  }

  @Override
  public void periodic() {
  }

  /**
   * Gets the rotations of the turret.
   * 
   * @return the positive or negative decimal number of the rotations the turret
   *         has moved
   */
  private double getRotations() {
    return m_encoder.getPosition() / 360.0;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("");
    builder.addDoubleProperty("Rotations", this::getRotations, null);
    builder.addDoubleProperty("Angle", this::getDegrees, null);
    builder.addDoubleProperty("Setpoint", () -> m_setpoint, null);
    builder.addBooleanProperty("Reached", this::atSetpoint, null);
    builder.addBooleanProperty("Soft Limited",
        () -> m_motor.getFault(FaultID.kSoftLimitRev) || m_motor.getFault(FaultID.kSoftLimitFwd), null);
  }

}
