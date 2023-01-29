// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.QuartetConstants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

  private final CANSparkMax m_motor = new CANSparkMax(TurretConstants.kMotorPort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    m_encoder.setPositionConversionFactor(TurretConstants.kDegreesPerRev);
    m_motor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
      double rot = getRotations();
      if (rot >= TurretConstants.kMaxRotations && (Math.signum(m_motor.get()) == Math.signum(rot))) {
        m_motor.set(0.0);
      }
      SmartDashboard.putNumber("Turret Rotations", rot);
  }

  public void run(double speed) {
    m_motor.set(speed);
  }

  private double getRotations() {
    return m_encoder.getPosition() / 360.0;
  }
}
