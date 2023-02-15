// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pid;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShoulderSetAngle extends CommandBase {
  private final ArmFeedforward m_armFeed = new ArmFeedforward(0, 1.75, 0);
  private final PIDController m_pidController = new PIDController(0, 0, 0);
  private final ShoulderSubsystem m_shoulder;
  /** Creates a new TurretSetAngle. */
  public ShoulderSetAngle(double setpointAngle, ShoulderSubsystem shoulder) {

    m_shoulder = shoulder;
    addRequirements(m_shoulder);

    m_pidController.setTolerance(0.5);
  }

  @Override
  public void execute() {
    final double feedback = m_pidController.calculate(m_shoulder.getAngle());
    final  double feedforward = m_armFeed.calculate(Math.toRadians(m_shoulder.getAngle()), 0); // TODO get degrees offset

    m_shoulder.runShoulder(feedback + feedforward);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint();
  }
}
