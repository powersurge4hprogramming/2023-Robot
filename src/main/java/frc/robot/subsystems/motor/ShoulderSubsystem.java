// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.QuartetConstants.ShoulderConstants;

public class ShoulderSubsystem extends MotorTemplate {

  /** Creates a new ShoulderSubsystem, position units are degrees. */
  public ShoulderSubsystem() {
    super(ShoulderConstants.kMotorPort);

    m_motor.restoreFactoryDefaults();

    m_encoder.setPositionConversionFactor(ShoulderConstants.kDegreesPerRev);
    m_encoder.setPosition(60); // TODO figure out starting degrees
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(false); // TODO figure out inversion, up is positive

    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ShoulderConstants.kMinDegrees);
    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ShoulderConstants.kMaxDegrees);

    m_motor.setSmartCurrentLimit(20, 25);

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
}
