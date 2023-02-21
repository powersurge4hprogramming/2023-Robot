// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pid;

import frc.robot.subsystems.motor.MotorTemplate;

public class ArmSetLength extends PIDPositionSet {

  /** sets arm to length (in) then finishes */
  public ArmSetLength(double setpoint, MotorTemplate subsystem) {
    super(setpoint, subsystem);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_setpoint - m_subsystem.getLength()) <= 0.25) && (Math.abs(m_subsystem.getVelocity()) <= 1);
  }
}
