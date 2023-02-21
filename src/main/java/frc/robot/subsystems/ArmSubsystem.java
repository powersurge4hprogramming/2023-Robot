// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.QuartetConstants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private final CANSparkMax m_motor = new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  private final SparkMaxPIDController m_pidController = m_motor.getPIDController();

  private final DoubleSolenoid m_lockSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      ArmConstants.kLockSolenoidFwd, ArmConstants.kLockSolenoidFwd);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    new DoubleSolenoidSim(PneumaticsModuleType.REVPH, ArmConstants.kLockSolenoidFwd, ArmConstants.kLockSolenoidFwd);

    m_motor.restoreFactoryDefaults();

    m_encoder.setPositionConversionFactor(ArmConstants.kDistancePerRevInches);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(true);

    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ((float) ArmConstants.kMinPosInches));
    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ((float) ArmConstants.kMaxPosInches));

    m_pidController.setOutputRange(-0.6, 0.4);
    m_pidController.setP(0.05);

    setName("ArmSubsystem");
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

  /** runs arm, not for PID */
  private void runArm(double speed) {
    m_motor.set(speed);
  }

  /** runs arm to position in inches */
  public void runArmPosition(double position) {
    m_pidController.setReference(position, ControlType.kPosition);
  }

  public void stopArm() {
    m_motor.stopMotor();
  }

  /** runs arm, runs until canceled */
  public CommandBase runArmCommand(double speed) {
    return this.startEnd(() -> runArm(speed), () -> runArm(0.0)).withName("RunArm");
  }

  /** position (in) */
  public double getLength() {
    return m_encoder.getPosition();
  }

  /** velocity (rpm) */
  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  public void setArmLock(boolean locked) {
    if (locked) {
      m_lockSolenoid.set(Value.kForward);
    } else {
      m_lockSolenoid.set(Value.kReverse);
    }
  }

  public CommandBase toggleArmLock() {
    return this.runOnce(() -> {
      m_lockSolenoid.toggle();
    }).withName("ToggleArmLock");
  }
}
