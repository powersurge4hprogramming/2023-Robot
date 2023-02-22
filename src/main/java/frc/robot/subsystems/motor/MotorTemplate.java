// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class MotorTemplate extends SubsystemBase {

  protected final CANSparkMax m_motor;
  protected final RelativeEncoder m_encoder;
  protected final SparkMaxPIDController m_pidController;

  /** Creates a new MotorTemplate. */
  public MotorTemplate(int motorPort) {
    m_motor = new CANSparkMax(motorPort, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
    m_pidController = m_motor.getPIDController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** runs motor to speed -1 to 1, not for PID */
  private void setSpeed(double speed) {
    m_motor.set(speed);
  }

  /** runs motor to position */
  public void setPosition(double position) {
    m_pidController.setReference(position, ControlType.kPosition);
  }

  /** stops motor from running */
  public void stopMotor() {
    m_motor.stopMotor();
  }

  /** runs motor, runs until canceled */
  public CommandBase setSpeedCommand(double speed) {
    return this.startEnd(() -> setSpeed(speed), () -> setSpeed(0.0)).withName("RunSpeed" + getName());
  }

  /** position (in or degrees) */
  public double getLength() {
    return m_encoder.getPosition();
  }

  /** velocity (rpm) */
  public double getVelocity() {
    return m_encoder.getVelocity();
  }
}
