// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class TemplateLEDCommand extends CommandBase {
  private final LEDSubsystem m_ledSubsystem;

  /** Creates a new ExampleLEDCommand. */
  public TemplateLEDCommand(LEDSubsystem ledSubsystem) {
    m_ledSubsystem = ledSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // call any code you want to run at once, like starting a timer or setting
    // initial LED colors

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // this code runs every 0.02 seconds, do not count anything here or use while,
    // you will break the robot. Instead use timers.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // this turns off the LEDs, ie sets them to transparent
    m_ledSubsystem.turnOff();
    // this sends the above line to the LEDs
    m_ledSubsystem.pushUpdates();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return true if you want the command to end and the competition command to
    // kick back in.
    return false;
  }
}
