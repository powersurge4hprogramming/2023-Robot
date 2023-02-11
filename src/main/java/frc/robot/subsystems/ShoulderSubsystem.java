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
import frc.robot.Constants.QuartetConstants.ShoulderConstants;

public class ShoulderSubsystem extends SubsystemBase {

  private final CANSparkMax m_motor = new CANSparkMax(ShoulderConstants.kMotorPort, MotorType.kBrushless);
  
  private final RelativeEncoder m_encoder = m_motor.getEncoder();

  /** Creates a new ShoulderSubsystem. */
  public ShoulderSubsystem() {
    m_encoder.setPositionConversionFactor(ShoulderConstants.kDegreesPerRev);
    m_motor.setIdleMode(IdleMode.kBrake);
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

  public void run(double speed) {
    m_motor.set(speed);
  }

  public double getAngle() {
    return m_encoder.getPosition();
  }
}
