// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pid;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretSetAngle extends CommandBase {
  private final TurretSubsystem m_turret;
  private final ProfiledPIDController m_pidController = new ProfiledPIDController(0.01, 0, 0.01,
      new TrapezoidProfile.Constraints(180, 360));

  /** Creates a new TurretSetAngle. */
  public TurretSetAngle(double setpointAngle, TurretSubsystem turret) {
    m_turret = turret;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_turret);

    m_pidController.setTolerance(1);
    m_pidController.enableContinuousInput(-360, 360);
    m_pidController.setGoal(setpointAngle);
  }

  @Override
  public void execute() {
    double motorValue = m_pidController.calculate(m_turret.getAngle());
    m_turret.runTurret(motorValue);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.runTurret(0.0);
  }
}
