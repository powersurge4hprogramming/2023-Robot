// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pid;

import static frc.robot.Constants.QuartetConstants.ShoulderConstants.*;
import frc.robot.subsystems.motor.MotorTemplate;

public class ShoulderSetAngle extends PIDPositionSet {

  /**
   * Constructs a new {@link PIDPositionSet} for the {@link ShoulderSubsystem}
   * that
   * will move the robot to the specified length, then finish
   * 
   * @param setpoint  (degrees) The setpoint for the shoulder
   * @param subsystem the {@link ShoulderSubsystem} for the position to be set
   */
  public ShoulderSetAngle(double setpoint, MotorTemplate subsystem) {
    super(setpoint, subsystem);
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(m_setpoint - m_subsystem.getLength()) <= kPositionTolerance)
        && (Math.abs(m_subsystem.getVelocity()) <= kVelocityTolerance);
  }

}
