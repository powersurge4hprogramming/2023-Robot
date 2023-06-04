// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import java.util.List;
import java.util.concurrent.locks.Condition;

import javax.security.auth.kerberos.KerberosPrincipal;
import javax.swing.plaf.synth.ColorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class CaydenLEDCommand extends CommandBase {
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
  public CaydenLEDCommand(LEDSubsystem ledSubsystem) {
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
   if (m_timer.get() < 1.0) {
      // if timer is greater than 0.5 seconds but less than a second, turn pink LEDs
      // on. The List.of() contains the indexes of the LEDs we want to switch, in this
      // case 5,6,14, and 15.
      m_ledSubsystem.setIndexesColor(List.of( 1,3,5,7,9,11,13,15,17,19,21), kHotPink);
      m_ledSubsystem.setIndexesColor(List.of(0,2,4,6,8,10,12,14,16,18,20), Color.kCornflowerBlue);

      // after calling setIndexesColor we must update the LEDs that we made this
      // change.
      m_ledSubsystem.pushUpdates();

    } else if (m_timer.get() < 2.0) {
      // if the timer is greater than 1.5 but less than 2.0, set the lights to yellow
      m_ledSubsystem.setIndexesColor(List.of(0,2,4,6,8,10,12,14,16,18,20), Color.kHotPink);
      m_ledSubsystem.setIndexesColor(List.of(1,3,5,7,9,11,13,15,17,19,21), Color.kCornflowerBlue);
      m_ledSubsystem.pushUpdates();
    } else if (m_timer.get() < 3.0) { 
      m_ledSubsystem.setIndexesColor(List.of(0,2,4,6,8,10,12,14,16,18,20), Color.kAqua);
      m_ledSubsystem.setIndexesColor(List.of(1,3,5,7,9,11,13,15,17,19,21), Color.kLimeGreen);
      m_ledSubsystem.pushUpdates();
    } else if (m_timer.get() < 4.0) { 
      m_ledSubsystem.setIndexesColor(List.of( 1,3,5,7,9,11,13,15,17,19,21), Color .kAqua);
      m_ledSubsystem.setIndexesColor(List.of(0,2,4,6,8,10,12,14,16,18,20), Color.kLimeGreen);
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 5.0) {
      m_ledSubsystem.setIndexesColor(List.of(0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21),new Color(0, 0, 0));    
        m_ledSubsystem.setIndexesColor(List.of(0),Color.kHotPink);        
        m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 5.25) { 
      m_ledSubsystem.setIndexesColor(List.of(0),new Color(0, 0, 0));
        m_ledSubsystem.setIndexesColor(List.of(1),Color.kHotPink);        
        m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 5.50) { 
      m_ledSubsystem.setIndexesColor(List.of(1),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(2),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 5.75) { 
      m_ledSubsystem.setIndexesColor(List.of(2),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(3),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 6) { 
      m_ledSubsystem.setIndexesColor(List.of(3),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(4),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 6.25) { 
      m_ledSubsystem.setIndexesColor(List.of(4),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(5),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 6.50) { 
      m_ledSubsystem.setIndexesColor(List.of(5),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(6),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 6.75) { 
      m_ledSubsystem.setIndexesColor(List.of(6),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(7),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 7) { 
      m_ledSubsystem.setIndexesColor(List.of(7),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(8),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 7.25) { 
      m_ledSubsystem.setIndexesColor(List.of(8),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(9),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 7.50) { 
      m_ledSubsystem.setIndexesColor(List.of(9),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(10),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 7.75) { 
      m_ledSubsystem.setIndexesColor(List.of(10),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(11),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 8) { 
      m_ledSubsystem.setIndexesColor(List.of(11),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(12),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 8.25) { 
      m_ledSubsystem.setIndexesColor(List.of(12),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(13),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() <8.50) { 
      m_ledSubsystem.setIndexesColor(List.of(13),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(14),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 8.75) { 
      m_ledSubsystem.setIndexesColor(List.of(14),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(15),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 9) { 
      m_ledSubsystem.setIndexesColor(List.of(15),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(16),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 9.25) { 
      m_ledSubsystem.setIndexesColor(List.of(16),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(17),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 9.50) {
      m_ledSubsystem.setIndexesColor(List.of(17),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(18),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 9.75) { 
      m_ledSubsystem.setIndexesColor(List.of(18),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(19),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 10) { 
      m_ledSubsystem.setIndexesColor(List.of(19),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(20),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
    }  else if (m_timer.get() < 10.25) { 
      m_ledSubsystem.setIndexesColor(List.of(20),new Color(0, 0, 0));
      m_ledSubsystem.setIndexesColor(List.of(21),Color.kHotPink);        
      m_ledSubsystem.pushUpdates();
      

    }
    /* else {
      m_timer.restart();
    } 
    */

    /*
    List<Actions> actions = List.of(
    new Actions(
      List.of(
        Color.kHotPink, 
        Color.kOrange
        ), 
      List.of(
        List.of(1,2,3), 
        List.of(17,18,19)
        ), 
        0.25),

    new Actions(List.of(Color.kYellow), List.of(List.of(1,2,3)), 0.2),
    new Actions(List.of(Color.kBisque), List.of(List.of(1,2,3,4,5,6)), 0.4)
    );
    
    for (int i = 0; i < actions.size(); ) {
      Actions action = actions.get(i);
      if (m_timer.get() < action.m_time) {
        for (int j = 0; j < action.m_indexes.size(); j++) {
          System.out.println(j);
          m_ledSubsystem.setIndexesColor(action.m_indexes.get(j), action.m_color.get(j));
        }
        m_ledSubsystem.pushUpdates();
      } else {
        m_ledSubsystem.turnOff();
        m_ledSubsystem.pushUpdates();
        m_timer.restart();
        i++;
      }
    }
    */
     if (m_timer.get() < 3) {
      for (int i = 0; i <= 21; i = i + 1) {
        if (i % 2 == 1) {
          m_ledSubsystem.setIndexColor(i, kHotPink);
        } else {
          m_ledSubsystem.setIndexColor(i, Color.kCornflowerBlue);
        }
      }
    } else if (m_timer.get() < 6) {
      for (int i = 0; i <= 21; i = i + 1) {
        if (i % 2 == 1) {
          m_ledSubsystem.setIndexColor(i, Color.kCornflowerBlue);
        } else {
          m_ledSubsystem.setIndexColor(i, Color.kHotPink);
        }

      }
    }
    else{
      m_timer.reset();
    }
    m_ledSubsystem.pushUpdates();

    
    //m_ledSubsystem.setIndexesColor(List.of( 1,3,5,7,9,11,13,15,17,19,21), kHotPink);
    //m_ledSubsystem.setIndexesColor(List.of(0,2,4,6,8,10,12,14,16,18,20), Color.kCornflowerBlue);
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

class Actions {
  public final List<Color> m_color;
  public final List<List<Integer>> m_indexes;
  public final double m_time;

  Actions(List<Color> color, List<List<Integer>> indexes, double time) {
    m_color = color;
    m_indexes = indexes;
    m_time = time;
  }


}
