// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretDynamicAngle extends CommandBase {
  private final TurretSubsystem m_turret;
  private final DoubleSupplier m_turretController;

  private double m_setpoint;

  /** Creates a new TurretDynamicAngle. */
  public TurretDynamicAngle(DoubleSupplier turretController, TurretSubsystem turret) {
    m_turretController = turretController;
    m_turret = turret;
    m_setpoint = turretController.getAsDouble();

    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_setpoint = m_turretController.getAsDouble();
    m_turret.setPosition(m_setpoint);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_setpoint != m_turretController.getAsDouble()) {
      m_setpoint = m_turretController.getAsDouble();
      m_turret.setPosition(m_setpoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.setPosition(m_turret.getDegrees());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
