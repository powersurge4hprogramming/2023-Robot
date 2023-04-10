// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.QuartetConstants.ClawConstants.PickupMode;
import frc.robot.subsystems.LEDSubsystem;

public class LEDCompetition extends CommandBase {
  private static final Color kRedAllianceColor = new Color(145, 0, 0);
  private static final Color kBlueAllianceColor = new Color(0, 0, 80);
  private static final Color kInvalidAllianceColor = Color.kHotPink;

  private static final List<Integer> kAllianceLEDIndexes = List.of(0, 8, 9, 10, 11, 20, 21);

  private static final Color kConeColor = new Color(100, 28, 0);
  private static final Color kCubeColor = new Color(100, 0, 60);
  private static final Color kStoppybarColor = new Color(0, 140, 0); // "robot" green

  private static final List<Integer> kPickupLEDIndexes = List.of(1, 2, 3, 4, 5, 6, 7, 12, 13, 14, 15, 16, 17, 18, 19);

  private static final Color kTransparentColor = new Color(0, 0, 0);

  private final LEDSubsystem m_ledSubsystem;

  private final BooleanSupplier m_stoppyHook;
  private boolean m_stoppyLast;

  private final Supplier<PickupMode> m_clawHook;
  private PickupMode m_clawLast;

  /** Creates a new LEDCompetition. */
  public LEDCompetition(LEDSubsystem ledSubsystem, BooleanSupplier stoppyHook, Supplier<PickupMode> clawHook) {
    m_ledSubsystem = ledSubsystem;

    m_stoppyHook = stoppyHook;
    m_stoppyLast = m_stoppyHook.getAsBoolean();

    m_clawHook = clawHook;
    m_clawLast = m_clawHook.get();

    setName("LEDCompetitionCommand");

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (DriverStation.getAlliance()) {
      case Blue:
        m_ledSubsystem.setIndexesColor(kAllianceLEDIndexes, kBlueAllianceColor);
        break;
      case Red:
        m_ledSubsystem.setIndexesColor(kAllianceLEDIndexes, kRedAllianceColor);
        break;
      case Invalid:
      default:
        m_ledSubsystem.setIndexesColor(kAllianceLEDIndexes, kInvalidAllianceColor);
        break;

    }
    m_ledSubsystem.pushUpdates();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_stoppyHook.getAsBoolean() != m_stoppyLast) {
      m_stoppyLast = m_stoppyHook.getAsBoolean();
      if (m_stoppyLast) {
        m_ledSubsystem.setIndexesColor(kPickupLEDIndexes, kStoppybarColor);
      } else {
        m_ledSubsystem.setIndexesColor(kPickupLEDIndexes, kTransparentColor);
      }
      m_ledSubsystem.pushUpdates();
    }

    if (m_clawHook.get() != m_clawLast) {
      m_clawLast = m_clawHook.get();
      Color color;
      switch (m_clawLast) {
        case Cone:
          color = kConeColor;
          break;
        case Cube:
          color = kCubeColor;
          break;
        case None:
        default:
          color = kTransparentColor;
          break;
      }
      m_ledSubsystem.setIndexesColor(kPickupLEDIndexes, color);
      m_ledSubsystem.pushUpdates();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ledSubsystem.turnOff();
    m_ledSubsystem.pushUpdates();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
