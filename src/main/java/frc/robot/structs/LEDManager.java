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

/**
 * The static manager for the LEDs. Call {@code initialize()} before running!
 */
public class LEDManager {
    private final static AddressableLED m_led = new AddressableLED(LEDConstants.kLEDPWMPort);

    @SuppressWarnings("unused")
    private final static AddressableLEDSim p_ledSim = new AddressableLEDSim(m_led);

    private final static AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kNumberOfLEDs);

    private static boolean initialized = false;

    /**
     * Sets the LEDs to the specified color.
     * 
     * @param indexes the indexes of LEDs to be switched
     * @param color   the color to be sent to the LEDs (unconverted)
     */
    private static void setIndexesColor(List<Integer> indexes, Color color) {
        /// Buffer converts RGB to BGR, and we want BRG. So this will do RBG -> GBR?
        if (!initialized) {
            initialize();
        }

        color = new Color(color.red, color.blue, color.green);
        for (int index : indexes) {
            m_ledBuffer.setLED(index, color);
        }
        m_led.setData(m_ledBuffer);
    }

    /** Prepare the LEDs. This is expensive, so call this only once! */
    public static void initialize() {
        if (!initialized) {
            m_led.setLength(LEDConstants.kNumberOfLEDs);
            m_led.setData(m_ledBuffer);
            initialized = true;
        }
    }

    /** Start LED outputs, switching the alliance LEDs to the correct color. */
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

    /** Stop sending data to the LEDs. This will most likely turn them off? */
    public static void stop() {
        if (!initialized) {
            initialize();
        }

        m_led.stop();
    }

    /**
     * Sets the pickup LEDs to the corresponding color.
     * 
     * @param mode the pickup mode the robot is currently in
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

    /** Sets the pickup LEDs based on whether the stoppy bars are down.
     * 
     * @param on whether the stoppy bar is activated
     */
    public static void setStoppyBarLEDs(boolean on) {
        Color color;
        if (on) {
            color = new Color(0, 192, 0); // "robot" green
        } else {
            color = new Color(0, 0, 0);
        }
        setIndexesColor(List.of(3, 4, 5, 6, 7, 8, 9, 16, 17, 18, 19, 20, 21, 22), color);
    }
}