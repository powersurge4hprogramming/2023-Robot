// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.structs;

import java.util.List;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ClawSubsystem.PickupMode;

public class LEDManager {
    private final static AddressableLED m_led = new AddressableLED(LEDConstants.kLEDPWMPort);

    @SuppressWarnings("unused")
    private final static AddressableLEDSim p_ledSim = new AddressableLEDSim(m_led);

    private final static AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kNumberOfLEDs);
    private static boolean initialized = false;

    private static void setIndexesColor(List<Integer> indexes, Color color) {
        if (!initialized) {
            initialize();
        }
        for (int index : indexes) {
            m_ledBuffer.setLED(index, color);
        }
        m_led.setData(m_ledBuffer);
    }

    /** prepare LEDs, expensive, call this once! */
    public static void initialize() {
        if (!initialized) {
            m_led.setLength(LEDConstants.kNumberOfLEDs);
            m_led.setData(m_ledBuffer);
            initialized = true;
        }
    }

    /** start LED outputs */
    public static void start() {
        if (!initialized) {
            initialize();
        }
        m_led.start();
        List<Integer> indexes = List.of(0, 1, 2, 10, 11, 12, 13, 14, 15, 23, 24, 25);
        switch (DriverStation.getAlliance()) {
            case Blue:
                setIndexesColor(indexes, Color.kBlue);
                break;
            case Red:
                setIndexesColor(indexes, Color.kRed);
                break;
            case Invalid:
            default:
                setIndexesColor(indexes, Color.kHotPink);
                break;

        }
    }

    public static void stop() {
        if (!initialized) {
            initialize();
        }
        m_led.stop();
    }

    /*
     * private static void setAllColor(Color color) {
     * if (!initialized) {
     * initialize();
     * }
     * for (var i = 0; i < ledBuffer.getLength(); i++) {
     * ledBuffer.setLED(i, color);
     * }
     * led.setData(ledBuffer);
     * }
     */

    public static void setPickupLEDs(PickupMode mode) {
        Color color;
        switch (mode) {
            case Cone:
                color = Color.kYellow;
                break;
            case Cube:
                color = Color.kPurple;
                break;
            case None:
                color = new Color(0, 0, 0);
            default:
                color = new Color(0, 0, 0);
                break;
        }
        setIndexesColor(List.of(3, 4, 5, 6, 7, 8, 9, 16, 17, 18, 19, 20, 21, 22), color);
    }

    public static void setStoppyBarLEDs(boolean on) {
        Color color;
        if (on) {
            color = new Color(0, 192, 0);
        } else {
            color = new Color(0, 0, 0);
        }
        setIndexesColor(List.of(3, 4, 5, 6, 7, 8, 9, 16, 17, 18, 19, 20, 21, 22), color);
    }
}