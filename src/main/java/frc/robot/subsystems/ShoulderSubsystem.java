// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.QuartetConstants.ShoulderConstants;

public class ShoulderSubsystem extends SubsystemBase {

  private final CANSparkMax m_motor = new CANSparkMax(ShoulderConstants.kMotorPort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  private final SparkMaxPIDController m_pidController = m_motor.getPIDController();

  /** Creates a new ShoulderSubsystem. */
  public ShoulderSubsystem() {
    m_motor.restoreFactoryDefaults();

    m_encoder.setPositionConversionFactor(ShoulderConstants.kDegreesPerRev);
    m_encoder.setPosition(60); // TODO figure out starting degrees
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(false); // TODO figure out inversion, up is positive

    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ShoulderConstants.kMinDegrees);
    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ShoulderConstants.kMaxDegrees);

    m_pidController.setOutputRange(-0.15, 0.15);
    m_pidController.setP(0.01);

    setName("ShoulderSubsystem");
  }

  @Override
  public void periodic() {
    double deg = m_encoder.getPosition();
    if (deg >= ShoulderConstants.kMaxDegrees && (Math.signum(m_motor.get()) == 1)) {
      m_motor.set(0.0);
    } else if (deg <= ShoulderConstants.kMinDegrees && (Math.signum(m_motor.get()) == -1)) {
      m_motor.set(0.0);
    }
    SmartDashboard.putNumber("Shoulder Angle", deg);
  }

  /** runs shoulder, not for PID */
  private void runShoulder(double speed) {
    m_motor.set(speed);
  }

  /** runs shoulder to position in degrees */
  public void runShoulderPosition(double position) {
    m_pidController.setReference(position, ControlType.kPosition);
  }

  public void stopShoulder() {
    m_motor.stopMotor();
  }

  /** runs arm, runs until canceled */
  public CommandBase runShoulderCommand(double speed) {
    return this.startEnd(() -> runShoulder(speed), () -> runShoulder(0.0)).withName("RunShoulder");
  }

  /** angle (degrees) */
  public double getAngle() {
    return m_encoder.getPosition();
  }

  /** velocity (rpm) */
  public double getVelocity() {
    return m_encoder.getVelocity();
  }
}
