// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pid;

import static frc.robot.Constants.QuartetConstants.TurretConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.motor.TurretSubsystem;

public class TurretSetAngle extends CommandBase {
  private final DoubleSupplier m_setpointSupplier;
  private final TurretSubsystem m_turretSubsystem;
  private double m_actualSetpoint = -1;

  /**
   * Constructs a new {@link PIDPositionSet} for the {@link TurretSubsystem}
   * that
   * will move the robot to the specified length, then finish.
   * 
   * @param setpoint        (degrees) The setpoint for the turret
   * @param turretSubsystem the {@link TurretSubsystem} for the position to be set
   */
  public TurretSetAngle(double setpoint, TurretSubsystem turretSubsystem) {
    m_setpointSupplier = () -> setpoint;
    m_turretSubsystem = turretSubsystem;
  }

  /**
   * Constructs a new {@link PIDPositionSet} for the {@link TurretSubsystem}
   * that
   * will move the robot to the specified length, then finish.
   * 
   * @param setpoint        (degrees) The setpoint for the turret
   * @param turretSubsystem the {@link TurretSubsystem} for the position to be set
   */
  public TurretSetAngle(DoubleSupplier setpoint, TurretSubsystem turretSubsystem) {
    m_setpointSupplier = setpoint;
    m_turretSubsystem = turretSubsystem;
  }

  @Override
  public void initialize() {
    m_actualSetpoint = m_setpointSupplier.getAsDouble();
    m_turretSubsystem.setPosition(m_actualSetpoint);
  }

  @Override
  public void execute() {
    if (m_setpointSupplier.getAsDouble() != m_actualSetpoint) {
      m_actualSetpoint = m_setpointSupplier.getAsDouble();
      System.out.println(m_actualSetpoint);
      m_turretSubsystem.setPosition(m_actualSetpoint);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_actualSetpoint - m_turretSubsystem.getLength()) <= kPositionTolerance)
        && (Math.abs(m_turretSubsystem.getVelocity()) <= kVelocityTolerance);
  }

  @Override
  public void end(boolean interrupted) {
    m_turretSubsystem.stopMotor();
  }
}
