// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.QuartetConstants.ShoulderConstants.*;

public class ShoulderSubsystem extends MotorTemplate {

  /** Creates a new ShoulderSubsystem, position units are degrees. */
  public ShoulderSubsystem() {
    super(kMotorPort);

    m_motor.restoreFactoryDefaults();

    m_encoder.setPositionConversionFactor(kDegreesPerRev);
    m_encoder.setPosition(kStartingDegrees); // TODO figure out starting degrees
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(true);

    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, kMinDegrees);
    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, kMaxDegrees);

    m_motor.setSmartCurrentLimit(20, 25);

    m_pidController.setOutputRange(kMin, kMax);
    m_pidController.setP(kP);
    m_pidController.setD(kD);

    setName("ShoulderSubsystem");
  }

  @Override
  public void periodic() {
    /*
     * double deg = m_encoder.getPosition();
     * if (deg >= ShoulderConstants.kMaxDegrees && (Math.signum(m_motor.get()) ==
     * 1)) {
     * m_motor.set(0.0);
     * } else if (deg <= ShoulderConstants.kMinDegrees &&
     * (Math.signum(m_motor.get()) == -1)) {
     * m_motor.set(0.0);
     * }
     */
    SmartDashboard.putNumber("Shoulder Angle", m_encoder.getPosition());
  }
}
