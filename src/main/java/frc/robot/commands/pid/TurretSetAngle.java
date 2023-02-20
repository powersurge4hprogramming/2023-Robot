// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pid;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretSetAngle extends CommandBase {
  private final TurretSubsystem m_turret;
  private final double m_setpoint;

  /** Creates a new TurretSetAngle. */
  public TurretSetAngle(double setpointAngle, TurretSubsystem turret) {
    m_turret = turret;
    m_setpoint = setpointAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_turret);
  }

  @Override
  public void initialize() {
    double maxInput = 180;
    double minInput = -180;
    double errorBound = (maxInput - minInput) / 2.0;
    double positionError = MathUtil.inputModulus(m_setpoint - (m_turret.getRotations() * 360), -errorBound, errorBound); // TODO
                                                                                                                         // is
                                                                                                                         // this
                                                                                                                         // at
                                                                                                                         // all
                                                                                                                         // right?
    m_turret.runTurretPosition(positionError);
  }

  @Override
  public void execute() {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_setpoint - m_turret.getAngle()) <= 1;
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.stopTurret();
  }
}
