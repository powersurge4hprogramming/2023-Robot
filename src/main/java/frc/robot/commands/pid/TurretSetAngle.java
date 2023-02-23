// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pid;

import static frc.robot.Constants.QuartetConstants.TurretConstants.*;
import frc.robot.subsystems.motor.TurretSubsystem;

public class TurretSetAngle extends PIDPositionSet {

  /**
   * Constructs a new {@link PIDPositionSet} for the {@link TurretSubsystem}
   * that
   * will move the robot to the specified length, then finish.
   * 
   * @param setpoint  (degrees) The setpoint for the turret
   * @param subsystem the {@link TurretSubsystem} for the position to be set
   */
  public TurretSetAngle(double setpoint, TurretSubsystem subsystem) {
    super(setpoint, subsystem);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_setpoint - m_subsystem.getLength()) <= kPositionTolerance)
        && (Math.abs(m_subsystem.getVelocity()) <= kVelocityTolerance);
  }
}
