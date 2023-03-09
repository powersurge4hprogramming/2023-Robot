// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.QuartetConstants.ArmConstants.*;

public class ArmSubsystem extends MotorTemplate {

  private final DoubleSolenoid m_lockSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, kLockSolenoidFwd,
      kLockSolenoidBkwd);

  /** Creates a new ArmSubsystem, position units are inches. */
  public ArmSubsystem() {
    super(kMotorPort);

    new DoubleSolenoidSim(PneumaticsModuleType.REVPH, kLockSolenoidFwd, kLockSolenoidFwd);

    m_motor.restoreFactoryDefaults();

    m_encoder.setPositionConversionFactor(kDistancePerRevInches);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(true);

    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ((float) kMinPosInches));
    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ((float) kMaxPosInches));

    m_motor.setSmartCurrentLimit(60, 30);

    m_pidController.setOutputRange(kMin, kMax);
    m_pidController.setP(kP);
    m_pidController.setD(kD);
    m_pidController.setFF(kF);

    setName("ArmSubsystem");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Length", m_encoder.getPosition());
  }

  /**
   * Sets the lock mode on the arm.
   * 
   * @param locked whether the arm should be locked
   */
  public void setArmLock(boolean locked) {
    if (locked) {
      m_lockSolenoid.set(Value.kForward);
    } else {
      m_lockSolenoid.set(Value.kReverse);
    }
  }

  /**
   * Toggles the arm's locking
   * 
   * @return a command which toggles the arm lock and the finishes
   */
  public CommandBase toggleArmLock() {
    return this.runOnce(() -> {
      m_lockSolenoid.toggle();
    }).withName("ToggleArmLock");
  }
}
