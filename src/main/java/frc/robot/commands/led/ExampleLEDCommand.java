// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class ExampleLEDCommand extends CommandBase {
  // all of our colors are defined up here, starting with a "k". Colors can be
  // defined using Color.k<Color Here> or new Color(red, green, blue). Both are
  // included here.
  private static final Color kHotPink = Color.kHotPink;
  private static final Color kYellow = new Color(100, 28, 0);

  // this is the WPI timer, which allows us to tell time, which could help blink
  // the LEDs.
  private final Timer m_timer = new Timer();

  private final LEDSubsystem m_ledSubsystem;

  /** Creates a new TemplateLEDCommand. */
  public ExampleLEDCommand(LEDSubsystem ledSubsystem) {
    m_ledSubsystem = ledSubsystem;

    setName("PinkBlinkCommand");

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // call any code you want to run at once, like starting a timer or setting
    // initial LED colors

    m_timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // this code runs every 0.02 seconds, do not count anything here or use while,
    // you will break the robot. Instead use timers.
    if (m_timer.get() < 0.5) {
      // do nothing in first half second
    } else if (m_timer.get() < 1.0) {
      // if timer is greater than 0.5 seconds but less than a second, turn pink LEDs
      // on. The List.of() contains the indexes of the LEDs we want to switch, in this
      // case 5,6,14, and 15.
      m_ledSubsystem.setIndexesColor(List.of(5, 6, 14, 15), kHotPink);

      // after calling setIndexesColor we must update the LEDs that we made this
      // change.
      m_ledSubsystem.pushUpdates();

    } else if (m_timer.get() < 1.5) {
      // if the timer is greater than 1.0 but less than 1.5, turn off the lights
      m_ledSubsystem.turnOff();
      m_ledSubsystem.pushUpdates();
    } else if (m_timer.get() < 2.0) {
      // if the timer is greater than 1.5 but less than 2.0, set the lights to yellow
      m_ledSubsystem.setIndexesColor(List.of(5, 6, 14, 15), kYellow);
      m_ledSubsystem.pushUpdates();
    } else {
      // if the timer is greater than 2.0, reset it and turn off the lights
      m_ledSubsystem.turnOff();
      m_ledSubsystem.pushUpdates();

      m_timer.restart();
    }
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
