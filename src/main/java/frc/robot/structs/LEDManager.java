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

    private static boolean m_initialized = false;

    /**
     * Sets the LEDs to the specified color.
     * 
     * @param indexes the indexes of LEDs to be switched
     * @param color   the color to be sent to the LEDs (unconverted)
     */
    private static void setIndexesColor(List<Integer> indexes, Color color) {
        if (!m_initialized) {
            initialize();
        }

        // Buffer converts RGB to BGR, and we want BRG. So this will do RBG -> GBR?
        color = new Color(color.red, color.blue, color.green);
        for (int index : indexes) {
            m_ledBuffer.setLED(index, color);
        }
        m_led.setData(m_ledBuffer);
    }

    /** Prepare the LEDs. This is expensive, so call this only once! */
    public static void initialize() {
        if (!m_initialized) {
            m_led.setLength(LEDConstants.kNumberOfLEDs);
            m_led.setData(m_ledBuffer);
            m_initialized = true;
        }
    }

    /** Start LED outputs, switching the alliance LEDs to the correct color. */
    public static void start() {
        if (!m_initialized) {
            initialize();
        }

        m_led.start();
        switch (DriverStation.getAlliance()) {
            case Blue:
                setIndexesColor(LEDConstants.kAllianceLEDIndexes, LEDConstants.kBlueAllianceColor);
                break;
            case Red:
                setIndexesColor(LEDConstants.kAllianceLEDIndexes, LEDConstants.kRedAllianceColor);
                break;
            case Invalid:
            default:
                setIndexesColor(LEDConstants.kAllianceLEDIndexes, LEDConstants.kInvalidAllianceColor);
                break;

        }
    }

    /** Stop sending data to the LEDs. This will most likely turn them off? */
    public static void stop() {
        if (!m_initialized) {
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
                color = LEDConstants.kConeColor;
                break;
            case Cube:
                color = LEDConstants.kCubeColor;
                break;
            case None:
                color = LEDConstants.kTransparentColor;
            default:
                color = LEDConstants.kTransparentColor;
                break;
        }
        setIndexesColor(LEDConstants.kPickupLEDIndexes, color);
    }

    /**
     * Sets the pickup LEDs based on whether the stoppy bars are down.
     * 
     * @param on whether the stoppy bar is activated
     */
    public static void setStoppyBarLEDs(boolean on) {
        Color color;
        if (on) {
            color = LEDConstants.kStoppybarColor;
        } else {
            color = LEDConstants.kTransparentColor;
        }
        setIndexesColor(LEDConstants.kPickupLEDIndexes, color);
    }
}