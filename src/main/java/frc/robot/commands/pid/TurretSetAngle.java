// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pid;

import frc.robot.subsystems.motor.MotorTemplate;

public class TurretSetAngle extends PIDPositionSet {

  /** sets turret to angle (degrees) then finishes */
  public TurretSetAngle(double setpoint, MotorTemplate subsystem) {
    super(setpoint, subsystem);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_setpoint - m_subsystem.getLength()) <= 1) && (Math.abs(m_subsystem.getVelocity()) <= 1);
  }
}
