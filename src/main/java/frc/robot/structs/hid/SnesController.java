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
 * Handle input from Nintendo Super Nintendo (SNES) Controller connected to the
 * Driver Station.
 * 
 *
 * <p>
 * This class handles snes input that comes from the Driver Station. Each time
 * a
 * value is
 * requested the most recent value is returned. There is a single class instance
 * for each controller
 * and the mapping of ports to hardware buttons depends on the code in the
 * Driver Station.
 */
public class SnesController extends GenericHID {
    /** Represents a digital button on an SnesController. */
    public enum Button {
        kA(1),
        kB(0),
        kX(3),
        kY(2),
        kLb(4),
        kRb(5),
        // kSelect() Don't use, changes mode setting!
        kStart(9);

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
    public SnesController(final int port) {
        super(port);

        HAL.report(tResourceType.kResourceType_Controller, port + 1);
    }

    /**
     * Read the value of the A button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getAButton() {
        return getRawButton(Button.kA.value);
    }

    /**
     * Whether the A button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getAButtonPressed() {
        return getRawButtonPressed(Button.kA.value);
    }

    /**
     * Whether the A button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getAButtonReleased() {
        return getRawButtonReleased(Button.kA.value);
    }

    /**
     * Constructs an event instance around the A button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the A button's digital signal attached
     *         to the given
     *         loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent a(EventLoop loop) {
        return new BooleanEvent(loop, this::getAButton);
    }

    /**
     * Read the value of the B button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBButton() {
        return getRawButton(Button.kB.value);
    }

    /**
     * Whether the B button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBButtonPressed() {
        return getRawButtonPressed(Button.kB.value);
    }

    /**
     * Whether the B button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBButtonReleased() {
        return getRawButtonReleased(Button.kB.value);
    }

    /**
     * Constructs an event instance around the B button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the B button's digital signal attached
     *         to the given
     *         loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent b(EventLoop loop) {
        return new BooleanEvent(loop, this::getBButton);
    }

    /**
     * Read the value of the X button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getXButton() {
        return getRawButton(Button.kX.value);
    }

    /**
     * Whether the X button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getXButtonPressed() {
        return getRawButtonPressed(Button.kX.value);
    }

    /**
     * Whether the X button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getXButtonReleased() {
        return getRawButtonReleased(Button.kX.value);
    }

    /**
     * Constructs an event instance around the X button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the X button's digital signal attached
     *         to the given
     *         loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent x(EventLoop loop) {
        return new BooleanEvent(loop, this::getXButton);
    }

    /**
     * Read the value of the Y button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getYButton() {
        return getRawButton(Button.kY.value);
    }

    /**
     * Whether the Y button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getYButtonPressed() {
        return getRawButtonPressed(Button.kY.value);
    }

    /**
     * Whether the Y button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getYButtonReleased() {
        return getRawButtonReleased(Button.kY.value);
    }

    /**
     * Constructs an event instance around the Y button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the Y button's digital signal attached
     *         to the given
     *         loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent y(EventLoop loop) {
        return new BooleanEvent(loop, this::getYButton);
    }

    /**
     * Read the value of the left bumper on the controller.
     *
     * @return The state of the button.
     */
    public boolean getLeftBumper() {
        return getRawButton(Button.kLb.value);
    }

    /**
     * Whether the left bumper was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getLeftBumperPressed() {
        return getRawButtonPressed(Button.kLb.value);
    }

    /**
     * Whether the left bumper was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getLeftBumperReleased() {
        return getRawButtonReleased(Button.kLb.value);
    }

    /**
     * Constructs an event instance around the left bumper's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left bumper's digital signal
     *         attached to the given
     *         loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent leftBumper(EventLoop loop) {
        return new BooleanEvent(loop, this::getLeftBumper);
    }

    /**
     * Read the value of the right bumper on the controller.
     *
     * @return The state of the button.
     */
    public boolean getRightBumper() {
        return getRawButton(Button.kRb.value);
    }

    /**
     * Whether the right bumper was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getRightBumperPressed() {
        return getRawButtonPressed(Button.kRb.value);
    }

    /**
     * Whether the right bumper was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getRightBumperReleased() {
        return getRawButtonReleased(Button.kRb.value);
    }

    /**
     * Constructs an event instance around the right bumper's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right bumper's digital signal
     *         attached to the given
     *         loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent rightBumper(EventLoop loop) {
        return new BooleanEvent(loop, this::getRightBumper);
    }

    /**
     * Read the value of the start button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getStartButton() {
        return getRawButton(Button.kStart.value);
    }

    /**
     * Whether the start button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getStartButtonPressed() {
        return getRawButtonPressed(Button.kStart.value);
    }

    /**
     * Whether the start button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getStartButtonReleased() {
        return getRawButtonReleased(Button.kStart.value);
    }

    /**
     * Constructs an event instance around the start button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the start button's digital signal
     *         attached to the given
     *         loop.
     */
    public BooleanEvent start(EventLoop loop) {
        return new BooleanEvent(loop, this::getStartButton);
    }
}