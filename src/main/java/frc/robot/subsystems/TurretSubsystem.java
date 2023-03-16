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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import static frc.robot.Constants.QuartetConstants.TurretConstants.*;

public class TurretSubsystem extends SubsystemBase {

  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkMaxPIDController m_pidController;

  private double m_setpoint;

  /** Creates a new TurretSubsystem, position units are degrees. */
  public TurretSubsystem() {
    m_motor = new CANSparkMax(kMotorPort, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
    m_pidController = m_motor.getPIDController();
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
   * Runs motor to a specified speed, not for usage in the PID.
   * 
   * @speed the duty cycle speed (-1 to 1) to set the motor
   */
  private void setSpeed(double speed) {
    // System.out.println("SetDuty" + getName());
    m_motor.set(speed);
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

  /**
   * Runs motor to a specified position.
   * 
   * @param angle the position (in degrees) to set the motor to
   */
  public CommandBase moveToAngle(double angle) {
    return this.runOnce(() -> {
      setPosition(angle);
    }).handleInterrupt(() -> setPosition(m_encoder.getPosition()))
        .andThen(new WaitUntilCommand(this::atSetpoint).withName("TurretToAngle"));
  }

  /** Stops motor from running, will interrupt any control mode. */
  public void disableMotor() {
    m_motor.setIdleMode(IdleMode.kCoast);
    m_motor.stopMotor();
  }

  /**
   * Runs the motor at a specified speed.
   * 
   * @param speed the duty cycle speed (-1 to 1) to set the motor
   * @return a command which runs the motor until interrupted
   */
  public CommandBase setSpeedCommand(double speed) {
    return this.startEnd(() -> setSpeed(speed), () -> disableMotor()).withName("RunSpeed" + getName());
  }

  /**
   * Gets the position
   * 
   * @return the position of the encoder in inches or degrees
   */
  private double getLength() {
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
    return (Math.abs(m_setpoint - getLength()) <= kPositionTolerance)
        && (Math.abs(getVelocity()) <= kVelocityTolerance);
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

  public void resetEncoders() {
    m_encoder.setPosition(kStartingDegrees);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("");
    builder.addDoubleProperty("Rotations", this::getRotations, null);
    builder.addDoubleProperty("Setpoint", () -> m_setpoint, null);
    builder.addBooleanProperty("Setpoint", this::atSetpoint, null);
    builder.addBooleanProperty("Rev Limited", () -> m_motor.getFault(FaultID.kSoftLimitRev), null);
    builder.addBooleanProperty("Fwd Limited", () -> m_motor.getFault(FaultID.kSoftLimitFwd), null);
  }

}
