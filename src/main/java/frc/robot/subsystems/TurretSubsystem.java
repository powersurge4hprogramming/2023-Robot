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

  private double m_setpoint;

  /** Creates a new TurretSubsystem, position units are degrees. */
  public TurretSubsystem() {
    m_motor.restoreFactoryDefaults();

    m_motor.setInverted(true);
    m_encoder.setPositionConversionFactor(kDegreesPerRev);
    m_encoder.setPosition(kStartingDegrees);
    m_motor.setIdleMode(IdleMode.kCoast);

    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -((float) kMaxRotations * 360));
    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ((float) kMaxRotations * 360));

    m_pidController.setOutputRange(kMin, kMax);
    m_pidController.setP(kP);
    m_pidController.setD(kD);
    m_pidController.setFF(kF);
    m_pidController.setPositionPIDWrappingEnabled(true);
    m_pidController.setPositionPIDWrappingMinInput(-180);
    m_pidController.setPositionPIDWrappingMaxInput(180);

    m_motor.setSmartCurrentLimit(20, 25);

    setName("TurretSubsystem");
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

  /**
   * Gets the rotations of the turret.
   * 
   * @return the positive or negative decimal number of the rotations the turret
   *         has moved
   */
  private double getRotations() {
    return m_encoder.getPosition() / 360.0;
  }

  /**
   * Runs motor to a specified position.
   * 
   * @param angle the position (in degrees) to set the motor to
   */
  public void setPosition(double angle) {
    if (m_setpoint != angle) {
      m_setpoint = angle;
      m_pidController.setReference(angle, ControlType.kPosition);
    }
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

  /** Stops motor from running, will interrupt any control mode. */
  public void disableMotor() {
    m_motor.setIdleMode(IdleMode.kCoast);
    m_motor.stopMotor();
  }

  @Override
  public void periodic() {
  }

  /**
   * Runs motor to a specified position.
   * 
   * @param angle the position (in degrees) to set the motor to
   */
  public CommandBase moveToAngle(double angle) {
    return this.runOnce(() -> {
      setPosition(angle);
    }).handleInterrupt(() -> setPosition(m_encoder.getPosition()))
        .andThen(new WaitUntilCommand(this::atSetpoint).withName("TurretToAngle" + angle));
  }

  public CommandBase incrementPosition(double increment) {
    return moveToAngle(m_setpoint + increment);
  }

  public CommandBase lockPosition() {
    return moveToAngle(getDegrees());
  }

  public CommandBase absoluteReset() {
    return moveToAngle(0).beforeStarting(() -> m_pidController.setPositionPIDWrappingEnabled(false), new Subsystem[0])
        .finallyDo((boolean interrupted) -> m_pidController.setPositionPIDWrappingEnabled(true))
        .withName("TurretAbsoluteReset");
  }

  public void resetEncoders() {
    m_encoder.setPosition(kStartingDegrees);
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
