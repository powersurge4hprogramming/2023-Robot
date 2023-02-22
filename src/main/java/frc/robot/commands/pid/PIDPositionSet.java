// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pid;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.motor.MotorTemplate;

public abstract class PIDPositionSet extends CommandBase {
  protected final MotorTemplate m_subsystem;
  protected final double m_setpoint;

  /**
   * Constructs a PID command that moves the {@link MotorTemplate} to the
   * specified length. Will not finish unless overridden by subclass.
   * 
   * @param setpoint  the setpoint in units that the {@link MotorTemplate}
   *                  position is in
   * @param subsystem the subsystem in which the PID should run
   */
  public PIDPositionSet(double setpoint, MotorTemplate subsystem) {
    m_subsystem = subsystem;
    m_setpoint = setpoint;

    addRequirements(subsystem);
    setName("PIDPosition" + m_subsystem.getName());
  }

  @Override
  public void initialize() {
    m_subsystem.setPosition(m_setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopMotor();
  }
}
