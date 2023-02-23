// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pid;

import static frc.robot.Constants.QuartetConstants.ArmConstants.*;
import frc.robot.subsystems.motor.ArmSubsystem;

public class ArmSetLength extends PIDPositionSet {

  /**
   * Constructs a new {@link PIDPositionSet} for the {@link ArmSubsystem} that
   * will move the robot to the specified length, then finish
   * 
   * @param setpoint  (inches) The setpoint for the arm
   * @param subsystem the {@link ArmSubsystem} for the position to be set
   */
  public ArmSetLength(double setpoint, ArmSubsystem subsystem) {
    super(setpoint, subsystem);
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(m_setpoint - m_subsystem.getLength()) <= kPositionTolerance)
        && (Math.abs(m_subsystem.getVelocity()) <= kVelocityTolerance);
  }
}
