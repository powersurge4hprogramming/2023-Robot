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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shoulder Angle", m_encoder.getPosition());
  }
}
