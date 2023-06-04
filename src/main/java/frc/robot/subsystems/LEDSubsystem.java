// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LEDConstants.*;

import java.util.List;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_led = new AddressableLED(kLEDPWMPort);

  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(kNumberOfLEDs);

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    m_led.setLength(kNumberOfLEDs);

    new AddressableLEDSim(m_led);

    setName("LEDSubsystem");
  }

  public void pushUpdates() {
    m_led.setData(m_ledBuffer);
  }

  /**
   * Sets the LEDs to the specified color.
   * 
   * @param indexes the indexes of LEDs to be switched
   * @param color   the color to be sent to the LEDs (unconverted)
   */
  public void setIndexesColor(List<Integer> indexes, Color color) {
    // Buffer converts RGB to BGR, and we want BRG. So this will do RBG -> GBR?
    if (RobotBase.isReal()) {
      color = new Color(color.red, color.blue, color.green);
    }
    for (int index : indexes) {
      m_ledBuffer.setLED(index, color);
    }
  }
  
  /**
   * Sets the LEDs to the specified color.
   * 
   * @param indexes the indexes of LEDs to be switched
   * @param color   the color to be sent to the LEDs (unconverted)
   */
  public void setIndexColor(int index, Color color) {
    // Buffer converts RGB to BGR, and we want BRG. So this will do RBG -> GBR?
    if (RobotBase.isReal()) {
      color = new Color(color.red, color.blue, color.green);
    }
      m_ledBuffer.setLED(index, color);
  }

  private void start() {

    m_led.start();
  }

  /** Start LED outputs, switching the alliance LEDs to the correct color. */
  public CommandBase startCommand() {
    return this.runOnce(() -> start()).withName("StartLEDs");
  }

  private void stop() {
    turnOff();
    m_led.setData(m_ledBuffer);
    m_led.stop();
  }

  /**
   * Makes transparent, blocks, then stops sending data to the LEDs. This will
   * most likely turn them off?
   */
  public CommandBase stopCommand() {
    return this.runOnce(() -> stop()).withName("StopLEDs");
  }

  /**
   * Set all of the LEDs to transparent
   */
  public void turnOff() {
    setIndexesColor(List.of(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21),
        new Color(0, 0, 0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("");
    builder.addStringProperty("Cmd", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none", null);
  }
}
