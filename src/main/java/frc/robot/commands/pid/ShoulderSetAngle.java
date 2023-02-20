// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pid;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderSetAngle extends CommandBase {
  private final ShoulderSubsystem m_shoulder;
  private final double m_setpoint;

  /** Creates a new TurretSetAngle. */
  public ShoulderSetAngle(double setpointAngle, ShoulderSubsystem shoulder) {
    m_shoulder = shoulder;
    m_setpoint = setpointAngle;

    addRequirements(m_shoulder);

  }

  @Override
  public void initialize() {
    m_shoulder.runShoulderPosition(m_setpoint);
  }

  @Override
  public void execute() {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_setpoint - m_shoulder.getAngle()) <= 0.5;
  }

  @Override
  public void end(boolean interrupted) {
    m_shoulder.stopShoulder();
  }
}
