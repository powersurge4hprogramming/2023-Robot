// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.QuartetConstants.TurretConstants.*;

public class TurretSubsystem extends MotorTemplate {

  /** Creates a new TurretSubsystem, position units are degrees. */
  public TurretSubsystem() {
    super(kMotorPort);

    m_motor.restoreFactoryDefaults();

    m_motor.setInverted(true);
    m_encoder.setPositionConversionFactor(kDegreesPerRev);
    m_encoder.setPosition(kStartingDegrees); // TODO confirm starting opposite robot front
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Rot", getRotations());
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
}
