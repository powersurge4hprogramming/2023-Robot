// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.structs.hid;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

/**
 * Handle input from a USB Guitar Hero (Activision Guitar N15505) Controller
 * connected to the
 * Driver Station.
 * 
 *
 * <p>
 * This class handles Guitar input that comes from the Driver Station. Each time
 * a
 * value is
 * requested the most recent value is returned. There is a single class instance
 * for each controller
 * and the mapping of ports to hardware buttons depends on the code in the
 * Driver Station.
 * 
 * <p>
 * POV is the Strum Bar:
 * (from bottom) left = 0, right = 180
 * 
 */
public class GuitarHeroController extends GenericHID {
    /**
     * Represents a digital button on an GuitarHeroController. Top is defined as
     * near the head of
     * the guitar.
     */
    public enum Button {
        kTopYellow(1),
        kTopGreen(0),
        kBlue(3),
        kBottomGreen(2),
        kBottomYellow(6),
        // kStarPower(), // Don't use, changes mode setting!
        kPlus(9);

        public final int value;

        Button(int value) {
            this.value = value;
        }

        /**
         * Get the human-friendly name of the button, matching the relevant methods.
         * This is done by
         * stripping the leading `k`, and appending `Button`.
         *
         * <p>
         * Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the button.
         */
        @Override
        public String toString() {
            var name = this.name().substring(1); // Remove leading `k`

            return name + "Button";
        }
    }

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is
     *             plugged into.
     */
    public GuitarHeroController(final int port) {
        super(port);

        HAL.report(tResourceType.kResourceType_Controller, port + 1);
    }

    /**
     * Read the value of the top yellow button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getTopYellowButton() {
        return getRawButton(Button.kTopYellow.value);
    }

    /**
     * Whether the top yellow button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getTopYellowButtonPressed() {
        return getRawButtonPressed(Button.kTopYellow.value);
    }

    /**
     * Whether the top yellow button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getTopYellowButtonReleased() {
        return getRawButtonReleased(Button.kTopYellow.value);
    }

    /**
     * Constructs an event instance around the top yellow button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the top yellow button's digital signal
     *         attached
     *         to the given
     *         loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent topYellow(EventLoop loop) {
        return new BooleanEvent(loop, this::getTopYellowButton);
    }

    /**
     * Read the value of the top green button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getTopGreenButton() {
        return getRawButton(Button.kTopGreen.value);
    }

    /**
     * Whether the top green button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getTopGreenButtonPressed() {
        return getRawButtonPressed(Button.kTopGreen.value);
    }

    /**
     * Whether the top green button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getTopGreenButtonReleased() {
        return getRawButtonReleased(Button.kTopGreen.value);
    }

    /**
     * Constructs an event instance around the top green button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the top green button's digital signal
     *         attached
     *         to the given
     *         loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent topGreen(EventLoop loop) {
        return new BooleanEvent(loop, this::getTopGreenButton);
    }

    /**
     * Read the value of the blue button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBlueButton() {
        return getRawButton(Button.kBlue.value);
    }

    /**
     * Whether the blue button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBlueButtonPressed() {
        return getRawButtonPressed(Button.kBlue.value);
    }

    /**
     * Whether the blue button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBlueButtonReleased() {
        return getRawButtonReleased(Button.kBlue.value);
    }

    /**
     * Constructs an event instance around the blue button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the blue button's digital signal
     *         attached
     *         to the given
     *         loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent blue(EventLoop loop) {
        return new BooleanEvent(loop, this::getBlueButton);
    }

    /**
     * Read the value of the bottom green button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBottomGreenButton() {
        return getRawButton(Button.kBottomGreen.value);
    }

    /**
     * Whether the bottom green button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBottomGreenButtonPressed() {
        return getRawButtonPressed(Button.kBottomGreen.value);
    }

    /**
     * Whether the bottom green button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBottomGreenButtonReleased() {
        return getRawButtonReleased(Button.kBottomGreen.value);
    }

    /**
     * Constructs an event instance around the bottom green button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the bottom green button's digital
     *         signal attached
     *         to the given
     *         loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent bottomGreen(EventLoop loop) {
        return new BooleanEvent(loop, this::getBottomGreenButton);
    }

    /**
     * Read the value of the bottom yellow button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBottomYellowButton() {
        return getRawButton(Button.kBottomYellow.value);
    }

    /**
     * Whether the bottom yellow button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBottomYellowButtonPressed() {
        return getRawButtonPressed(Button.kBottomYellow.value);
    }

    /**
     * Whether the bottom yellow button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBottomYellowButtonReleased() {
        return getRawButtonReleased(Button.kBottomYellow.value);
    }

    /**
     * Constructs an event instance around the bottom yellow button's digital
     * signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the bottom yellow button's digital
     *         signal attached
     *         to the given
     *         loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent bottomYellow(EventLoop loop) {
        return new BooleanEvent(loop, this::getBottomYellowButton);
    }

    /**
     * Read the value of the plus button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getPlusButton() {
        return getRawButton(Button.kPlus.value);
    }

    /**
     * Whether the plus button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getPlusButtonPressed() {
        return getRawButtonPressed(Button.kPlus.value);
    }

    /**
     * Whether the plus button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getPlusButtonReleased() {
        return getRawButtonReleased(Button.kPlus.value);
    }

    /**
     * Constructs an event instance around the plus button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the plus button's digital signal
     *         attached
     *         to the given
     *         loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent plus(EventLoop loop) {
        return new BooleanEvent(loop, this::getPlusButton);
    }
}