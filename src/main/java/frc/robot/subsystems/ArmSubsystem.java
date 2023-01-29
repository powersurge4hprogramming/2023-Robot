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
import frc.robot.Constants.QuartetConstants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private final CANSparkMax m_motor = new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_encoder.setPositionConversionFactor(ArmConstants.kDistancePerRevInches);
    m_motor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    double pos = m_encoder.getPosition();
    if (pos >= ArmConstants.kMaxPosInches && (Math.signum(m_motor.get()) == 1)) {
      m_motor.set(0.0);
    } else if (pos <= ArmConstants.kMinPosInches && (Math.signum(m_motor.get()) == -1)) {
      m_motor.set(0.0);
    }
    SmartDashboard.putNumber("Arm Length", pos);
  }
}
