// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Autobalance extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final PIDController m_pidController = new PIDController(0.025, 0, 0.001);

  /** Creates a new Autobalance. */
  public Autobalance(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidController.setSetpoint(0);
    m_pidController.setTolerance(1.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double mag = m_pidController.calculate(m_driveSubsystem.getPitch());
    m_driveSubsystem.tankDriveVolts(mag, mag);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driveSubsystem.isMoving() && m_pidController.atSetpoint();
  }
}
