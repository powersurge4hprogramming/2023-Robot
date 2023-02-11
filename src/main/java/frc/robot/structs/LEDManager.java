// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.structs;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ClawSubsystem.PickupMode;

public class LEDManager {
    private final static AddressableLED led = new AddressableLED(LEDConstants.kLEDPWMPort);
    private final static AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.kNumberOfLEDs);
    private static boolean initialized = false;

    public static void initialize() {
        led.setLength(LEDConstants.kNumberOfLEDs);
        led.setData(ledBuffer);
        initialized = true;
    }

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

    public static void setAllColor(Color8Bit color) {
        if (!initialized) {
            initialize();
        }
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, color.red, color.green, color.blue);
        }
        led.setData(ledBuffer);
    }

    private static void setIndexesColor(List<Integer> indexes, Color8Bit color) {
        if (!initialized) {
            initialize();
        }
        for (int index : indexes) {
            ledBuffer.setRGB(index, color.red, color.green, color.blue);
        }
        led.setData(ledBuffer);
    }

    public static void setPickupLEDs(PickupMode mode) {
        List<Integer> list = new ArrayList<Integer>();
        for (int i = 10; i <= 19; i++) {
            list.add(i);
        }
        Color color;
        switch (mode) {
            case Cone:
                color = Color.kYellow;
                break;
            case Cube:
                color = Color.kPurple;
                break;
            case None:
            default:
                color = null;
                break;
        }
        if (color != null) {
            setIndexesColor(list, null);
        }
    }
}