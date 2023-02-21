// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pid;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSetLength extends CommandBase {
  private final ArmSubsystem m_arm;
  private final double m_setpoint;

  /** sets arm to length (in), then finishes */
  public ArmSetLength(double setpoint, ArmSubsystem arm) {
    m_arm = arm;
    m_setpoint = setpoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  @Override
  public void initialize() {
    m_arm.runArmPosition(m_setpoint);
  }

  @Override
  public void execute() {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_setpoint - m_arm.getLength()) <= 0.25) && (Math.abs(m_arm.getVelocity()) <= 1);
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.stopArm();
  }
}
