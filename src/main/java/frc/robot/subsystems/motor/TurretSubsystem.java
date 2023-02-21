// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.QuartetConstants.TurretConstants;

public class TurretSubsystem extends MotorTemplate {

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    super(TurretConstants.kMotorPort);

    m_motor.restoreFactoryDefaults();

    m_motor.setInverted(true);
    m_encoder.setPositionConversionFactor(TurretConstants.kDegreesPerRev);
    m_encoder.setPosition(180); // TODO confirm starting opposite robot front
    m_motor.setIdleMode(IdleMode.kCoast);

    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -((float) TurretConstants.kMaxRotations * 360));
    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ((float) TurretConstants.kMaxRotations * 360));

    m_pidController.setOutputRange(-0.15, 0.15);
    m_pidController.setP(0.01);
    m_pidController.setD(0.01);
    m_pidController.setPositionPIDWrappingEnabled(true);
    m_pidController.setPositionPIDWrappingMinInput(-180);
    m_pidController.setPositionPIDWrappingMaxInput(180);

    setName("TurretSubsystem");
  }

  @Override
  public void periodic() {
    double rot = getRotations();
    if (Math.abs(rot) >= TurretConstants.kMaxRotations && (Math.signum(m_motor.get()) == Math.signum(rot))) {
      m_motor.set(0.0);
    }
    SmartDashboard.putNumber("Turret Rot", rot);
  }

  private double getRotations() {
    return m_encoder.getPosition() / 360.0;
  }
}
