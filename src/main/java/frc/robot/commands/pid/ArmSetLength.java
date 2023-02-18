// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pid;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSetLength extends CommandBase {
  private final ArmSubsystem m_arm;
  private final ProfiledPIDController m_pidController = new ProfiledPIDController(0.05, 0, 0,
      new TrapezoidProfile.Constraints(1.75, 10));

  /** Creates a new ArmSetLength. */
  public ArmSetLength(double setpoint, ArmSubsystem arm) {
    m_arm = arm;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);

    m_pidController.setTolerance(0.25);
    m_pidController.setGoal(setpoint);
  }

  @Override
  public void execute() {
    double motorValue = m_pidController.calculate(m_arm.getLength());
    m_arm.runArm(motorValue);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.runArm(0.0);
  }
}
