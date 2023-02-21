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
import frc.robot.Constants.QuartetConstants.ArmConstants;

public abstract class MotorTemplate extends SubsystemBase {

  protected final CANSparkMax m_motor;
  protected final RelativeEncoder m_encoder;
  protected final SparkMaxPIDController m_pidController;

  /** Creates a new ActuatorSubsystem. */
  public MotorTemplate(double motorPort) {
    m_motor = new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
    m_pidController = m_motor.getPIDController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** runs arm, not for PID */
  private void setSpeed(double speed) {
    m_motor.set(speed);
  }

  /** runs arm to position in inches */
  public void setPosition(double position) {
    m_pidController.setReference(position, ControlType.kPosition);
  }

  public void stopMotor() {
    m_motor.stopMotor();
  }

  /** runs arm, runs until canceled */
  public CommandBase setSpeedCommand(double speed) {
    return this.startEnd(() -> setSpeed(speed), () -> setSpeed(0.0)).withName("RunSpeed");
  }

  /** position (in) */
  public double getLength() {
    return m_encoder.getPosition();
  }

  /** velocity (rpm) */
  public double getVelocity() {
    return m_encoder.getVelocity();
  }
}
