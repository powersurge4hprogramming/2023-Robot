// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.structs;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ClawSubsystem.PickupMode;

public class LEDManager {
    private final static AddressableLED led = new AddressableLED(LEDConstants.kLEDPWMPort);
    private final static AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.kNumberOfLEDs);
    private static boolean initialized = false;

    /** compose a list, inclusive of end index */
    private static List<Integer> composeList(int startIndex, int endIndex) {
        List<Integer> list = new ArrayList<Integer>();
        for (int i = startIndex; i <= endIndex; i++) {
            list.add(i);
        }
        return list;
    }

    private static void setIndexesColor(List<Integer> indexes, Color color) {
        if (!initialized) {
            initialize();
        }
        for (int index : indexes) {
            ledBuffer.setLED(index, color);
        }
        led.setData(ledBuffer);
    }

    /** prepare LEDs, expensive, call this once! */
    public static void initialize() {
        if (!initialized) {
            led.setLength(LEDConstants.kNumberOfLEDs);
            led.setData(ledBuffer);
            initialized = true;
        }
    }

    /** start LED outputs */
    public static void start() {
        if (!initialized) {
            initialize();
        }
        led.start();
    }

    public static void stop() {
        if (!initialized) {
            initialize();
        }
        led.stop();
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
        setIndexesColor(composeList(10, 19), color);
    }

    public static void setStoppyBarLEDs(boolean on) {
        Color color;
        if (on) {
            color = Color.kRed;
        } else {
            color = new Color(0, 0, 0);
        }
        List<Integer> list = composeList(0, 4);
        list.addAll(composeList(22, 25));
        setIndexesColor(list, color);
    }
}