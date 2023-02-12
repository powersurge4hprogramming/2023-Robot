// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pid;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ShoulderSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShoulderSetAngle extends PIDCommand {
  /** Creates a new TurretSetAngle. */
  public ShoulderSetAngle(double setpointAngle, ShoulderSubsystem shoulder) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0), // TODO constantify
        // This should return the measurement
        () -> shoulder.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> setpointAngle,
        // This uses the output
        output -> {
          shoulder.runShoulder(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoulder);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
