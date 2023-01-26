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
 * Handle input from Sidewinder 2 Microsoft Controller connected to the
 * Driver Station.
 * 
 * This is in JOYSTICK form, POV form not supported!
 *
 * <p>
 * This class handles arcade input that comes from the Driver Station. Each time
 * a
 * value is
 * requested the most recent value is returned. There is a single class instance
 * for each controller
 * and the mapping of ports to hardware buttons depends on the code in the
 * Driver Station.
 */
public class SidewinderController extends GenericHID {
    /** Represents a digital button on an SidewinderController. */
    public enum Button {
        kTrigger(1),
        k2(2),
        k3(3),
        k4(4),
        k5(5),
        k6(6),
        k7(7),
        k8(8);

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

    /** Represents an axis on an SidewinderController. +1 is down and to the right. */
    public enum Axis {
        kJoystickX(0),
        kJoystickY(1),
        kJoystickZ(2),
        kThrottle(3);

        public final int value;

        Axis(int value) {
            this.value = value;
        }

        /**
         * Get the human-friendly name of the axis, matching the relevant methods. This
         * is done by
         * stripping the leading `k`.
         *
         * <p>
         * Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the axis.
         */
        @Override
        public String toString() {
            var name = this.name().substring(1); // Remove leading `k`

            return name;
        }
    }

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is
     *             plugged into.
     */
    public SidewinderController(final int port) {
        super(port);

        HAL.report(tResourceType.kResourceType_Controller, port + 1);
    }

    /**
     * Get the X axis value of the controller joystick.
     *
     * @return The axis value.
     */
    public double getJoystickX() {
        return getRawAxis(Axis.kJoystickX.value);
    }

    /**
     * Get the Y axis value of the controller joystick.
     *
     * @return The axis value.
     */
    public double getJoystickY() {
        return getRawAxis(Axis.kJoystickY.value);
    }

    /**
     * Get the Z axis value of the controller joystick.
     *
     * @return The axis value.
     */
    public double getJoystickZ() {
        return getRawAxis(Axis.kJoystickZ.value);
    }

    /**
     * Get the throttle value of the controller joystick.
     *
     * @return The axis value.
     */
    public double getThrottle() {
        return getRawAxis(Axis.kThrottle.value);
    }


    /**
     * Read the value of the trigger on the controller.
     *
     * @return The state of the button.
     */
    public boolean getTriggerButton() {
        return getRawButton(Button.kTrigger.value);
    }

    /**
     * Whether the trigger was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getTriggerButtonPressed() {
        return getRawButtonPressed(Button.kTrigger.value);
    }

    /**
     * Whether the trigger was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getTriggerButtonReleased() {
        return getRawButtonReleased(Button.kTrigger.value);
    }

    /**
     * Constructs an event instance around the trigger's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the 2 button's digital signal attached
     *         to the given
     *         loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent trigger(EventLoop loop) {
        return new BooleanEvent(loop, this::getTriggerButton);
    }

    /**
     * Read the value of the 2 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean get2Button() {
        return getRawButton(Button.k2.value);
    }

    /**
     * Whether the 2 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean get2ButtonPressed() {
        return getRawButtonPressed(Button.k2.value);
    }

    /**
     * Whether the 2 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean get2ButtonReleased() {
        return getRawButtonReleased(Button.k2.value);
    }

    /**
     * Constructs an event instance around the 2 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the 2 button's digital signal attached
     *         to the given
     *         loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent b2(EventLoop loop) {
        return new BooleanEvent(loop, this::get2Button);
    }

    /**
     * Read the value of the 3 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean get3Button() {
        return getRawButton(Button.k3.value);
    }

    /**
     * Whether the 3 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean get3ButtonPressed() {
        return getRawButtonPressed(Button.k3.value);
    }

    /**
     * Whether the 3 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean get3ButtonReleased() {
        return getRawButtonReleased(Button.k3.value);
    }

    /**
     * Constructs an event instance around the 3 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the 3 button's digital signal attached
     *         to the given
     *         loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent b3(EventLoop loop) {
        return new BooleanEvent(loop, this::get3Button);
    }

    /**
     * Read the value of the 4 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean get4Button() {
        return getRawButton(Button.k4.value);
    }

    /**
     * Whether the 4 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean get4ButtonPressed() {
        return getRawButtonPressed(Button.k4.value);
    }

    /**
     * Whether the 4 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean get4ButtonReleased() {
        return getRawButtonReleased(Button.k4.value);
    }

    /**
     * Constructs an event instance around the 4 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the 4 button's digital signal attached
     *         to the given
     *         loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent b4(EventLoop loop) {
        return new BooleanEvent(loop, this::get4Button);
    }

    /**
     * Read the value of the 5 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean get5Button() {
        return getRawButton(Button.k5.value);
    }

    /**
     * Whether the 5 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean get5ButtonPressed() {
        return getRawButtonPressed(Button.k5.value);
    }

    /**
     * Whether the 5 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean get5ButtonReleased() {
        return getRawButtonReleased(Button.k5.value);
    }

    /**
     * Constructs an event instance around the 5 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the 5 button's digital signal attached
     *         to the given
     *         loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent b5(EventLoop loop) {
        return new BooleanEvent(loop, this::get5Button);
    }

    /**
     * Read the value of the 6 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean get6Button() {
        return getRawButton(Button.k6.value);
    }

    /**
     * Whether the 6 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean get6ButtonPressed() {
        return getRawButtonPressed(Button.k6.value);
    }

    /**
     * Whether the 6 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean get6ButtonReleased() {
        return getRawButtonReleased(Button.k6.value);
    }

    /**
     * Constructs an event instance around the 6 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the 6 button's digital signal attached
     *         to the given
     *         loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent b6(EventLoop loop) {
        return new BooleanEvent(loop, this::get6Button);
    }
    
    /**
     * Read the value of the 7 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean get7Button() {
        return getRawButton(Button.k7.value);
    }

    /**
     * Whether the 7 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean get7ButtonPressed() {
        return getRawButtonPressed(Button.k7.value);
    }

    /**
     * Whether the 7 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean get7ButtonReleased() {
        return getRawButtonReleased(Button.k7.value);
    }

    /**
     * Constructs an event instance around the 7 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the 7 button's digital signal
     *         attached to the given
     *         loop.
     */
    public BooleanEvent b7(EventLoop loop) {
        return new BooleanEvent(loop, this::get7Button);
    }

    /**
     * Read the value of the 8 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean get8Button() {
        return getRawButton(Button.k8.value);
    }

    /**
     * Whether the 8 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean get8ButtonPressed() {
        return getRawButtonPressed(Button.k8.value);
    }

    /**
     * Whether the 8 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean get8ButtonReleased() {
        return getRawButtonReleased(Button.k8.value);
    }

    /**
     * Constructs an event instance around the 8 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the 8 button's digital signal
     *         attached to the given
     *         loop.
     */
    public BooleanEvent b8(EventLoop loop) {
        return new BooleanEvent(loop, this::get8Button);
    }
}