// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pid;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.motor.MotorTemplate;

public abstract class PIDPositionSet extends CommandBase {
  protected final MotorTemplate m_subsystem;
  protected final double m_setpoint;

  /** sets motor to length (in or degrees), then finishes */
  public PIDPositionSet(double setpoint, MotorTemplate subsystem) {
    m_subsystem = subsystem;
    m_setpoint = setpoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    setName("PIDPosition" + m_subsystem.getName());
  }

  @Override
  public void initialize() {
    m_subsystem.setPosition(m_setpoint);
  }

  @Override
  public void execute() {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_setpoint - m_subsystem.getLength()) <= 0.25) && (Math.abs(m_subsystem.getVelocity()) <= 1);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopMotor();
  }
}
