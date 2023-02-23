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

  /**
   * Creates a new motor template based off an assumption the motor is brushless
   * and attached to a {@link CANSparkMax}
   * 
   * @param motorPort the CAN ID of the motor
   */
  public MotorTemplate(int motorPort) {
    m_motor = new CANSparkMax(motorPort, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
    m_pidController = m_motor.getPIDController();
  }

  /**
   * Runs motor to a specified speed, not for usage in the PID.
   * 
   * @speed the duty cycle speed (-1 to 1) to set the motor
   */
  private void setSpeed(double speed) {
    m_motor.set(speed);
  }

  /**
   * Runs motor to a specified position.
   * 
   * @param position the position (in subclass units) to set the motor to
   */
  public void setPosition(double position) {
    m_pidController.setReference(position, ControlType.kPosition);
  }

  /** Stops motor from running, will interrupt any control mode. */
  public void stopMotor() {
    m_motor.stopMotor();
  }

  /**
   * Runs the motor at a specified speed.
   * 
   * @param speed the duty cycle speed (-1 to 1) to set the motor
   * @return a command which runs the motor until interrupted
   */
  public CommandBase setSpeedCommand(double speed) {
    return this.startEnd(() -> setSpeed(speed), () -> stopMotor()).withName("RunSpeed" + getName());
  }

  /**
   * Gets the position
   * 
   * @return the position of the encoder in inches or degrees
   */
  public double getLength() {
    return m_encoder.getPosition();
  }

  /**
   * Gets the velocity
   * 
   * @return the velocity in rpm
   */
  public double getVelocity() {
    return m_encoder.getVelocity();
  }
}
