// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretDynamicAngle extends CommandBase {
  private final TurretSubsystem m_turret;
  private final IntSupplier m_turretController;
  private final DoubleSupplier m_gyro;

  private double m_setpoint;

  /** Creates a new TurretDynamicAngle. */
  public TurretDynamicAngle(IntSupplier turretController, DoubleSupplier gyroPosition, TurretSubsystem turret) {
    m_turretController = turretController;
    m_turret = turret;
    m_gyro = gyroPosition;
    m_setpoint = turretController.getAsInt();

    addRequirements(m_turret);
  }

  private boolean validateAngle() {
    double turretSupply = m_turretController.getAsInt();
    return (turretSupply == 0 || turretSupply == 90 || turretSupply == 180 ||
        turretSupply == 270);
  }

  private double turretModded() {
    return MathUtil.inputModulus(m_turret.getDegrees(), -180, 180);
  }

  private double gyroModded() {
    return MathUtil.inputModulus(m_gyro.getAsDouble(), -180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*
     * if (validateAngle()) {
     * m_setpoint = m_turretController.getAsInt();
     * m_turret.setPosition(m_setpoint);
     * }
     */

    if (validateAngle()) {

      double turretAdd = ((double) m_turretController.getAsInt()) - (gyroModded() + turretModded());
      m_setpoint = turretModded() + turretAdd;
      m_turret.setPosition(m_setpoint);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * if (m_setpoint != m_turretController.getAsInt() && validateAngle()) {
     * m_setpoint = m_turretController.getAsInt();
     * m_turret.setPosition(m_setpoint);
     * }
     */

    if (validateAngle()) {
      double turretAdd = ((double) m_turretController.getAsInt()) - (gyroModded() + turretModded());
      m_setpoint = turretModded() + turretAdd;
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
