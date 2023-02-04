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
 * Handle input from X Series Six PXN Flight Arcade Controller connected to the
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
public class PXNArcadeStickController extends GenericHID {
    /** Represents a digital button on an PXNArcadeStickController. */
    public enum Button {
        kA(1),
        kB(2),
        kX(3),
        kY(4),
        kLb(5),
        kRb(6),
        kShare(7),
        kOptions(8),
        kL3(9),
        kR3(10);

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

    /** Represents an axis on an PXNArcadeStickController. */
    public enum Axis {
        kJoystickX(0),
        kJoystickY(1),
        kLeftTrigger(2),
        kRightTrigger(3);

        public final int value;

        Axis(int value) {
            this.value = value;
        }

        /**
         * Get the human-friendly name of the axis, matching the relevant methods. This
         * is done by
         * stripping the leading `k`, and if a trigger axis append `Axis`.
         *
         * <p>
         * Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the axis.
         */
        @Override
        public String toString() {
            var name = this.name().substring(1); // Remove leading `k`
            if (name.endsWith("Trigger")) {
                return name + "Axis";
            }
            return name;
        }
    }

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is
     *             plugged into.
     */
    public PXNArcadeStickController(final int port) {
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
     * Get the left trigger (LT) axis value of the controller. Note that this axis
     * is bound to the
     * range of [0, 1] as opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getLeftTriggerAxis() {
        return getRawAxis(Axis.kLeftTrigger.value);
    }

    /**
     * Get the right trigger (RT) axis value of the controller. Note that this axis
     * is bound to the
     * range of [0, 1] as opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getRightTriggerAxis() {
        return getRawAxis(Axis.kRightTrigger.value);
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
     * Read the value of the share button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getShareButton() {
        return getRawButton(Button.kShare.value);
    }

    /**
     * Whether the share button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getShareButtonPressed() {
        return getRawButtonPressed(Button.kShare.value);
    }

    /**
     * Whether the share button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getShareButtonReleased() {
        return getRawButtonReleased(Button.kShare.value);
    }

    /**
     * Constructs an event instance around the share button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the share button's digital signal
     *         attached to the given
     *         loop.
     */
    public BooleanEvent share(EventLoop loop) {
        return new BooleanEvent(loop, this::getShareButton);
    }

    /**
     * Read the value of the options button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getOptionsButton() {
        return getRawButton(Button.kOptions.value);
    }

    /**
     * Whether the options button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getOptionsButtonPressed() {
        return getRawButtonPressed(Button.kOptions.value);
    }

    /**
     * Whether the options button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getOptionsButtonReleased() {
        return getRawButtonReleased(Button.kOptions.value);
    }

    /**
     * Constructs an event instance around the options button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the options button's digital signal
     *         attached to the given
     *         loop.
     */
    public BooleanEvent options(EventLoop loop) {
        return new BooleanEvent(loop, this::getOptionsButton);
    }

    /**
     * Read the value of the L3 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getL3Button() {
        return getRawButton(Button.kL3.value);
    }

    /**
     * Whether the L3 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getL3ButtonPressed() {
        return getRawButtonPressed(Button.kL3.value);
    }

    /**
     * Whether the L3 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getL3ButtonReleased() {
        return getRawButtonReleased(Button.kL3.value);
    }

    /**
     * Constructs an event instance around the L3 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the L3 button's digital signal
     *         attached to the given
     *         loop.
     */
    public BooleanEvent L3(EventLoop loop) {
        return new BooleanEvent(loop, this::getL3Button);
    }

    /**
     * Read the value of the R3 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getR3Button() {
        return getRawButton(Button.kR3.value);
    }

    /**
     * Whether the R3 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getR3ButtonPressed() {
        return getRawButtonPressed(Button.kR3.value);
    }

    /**
     * Whether the R3 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getR3ButtonReleased() {
        return getRawButtonReleased(Button.kR3.value);
    }

    /**
     * Constructs an event instance around the R3 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the L3 button's digital signal
     *         attached to the given
     *         loop.
     */
    public BooleanEvent R3(EventLoop loop) {
        return new BooleanEvent(loop, this::getR3Button);
    }

    /**
     * Constructs an event instance around the axis value of the left trigger. The
     * returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link BooleanEvent}
     *                  to be true. This
     *                  value should be in the range [0, 1] where 0 is the unpressed
     *                  state of the axis.
     * @param loop      the event loop instance to attach the event to.
     * @return an event instance that is true when the left trigger's axis exceeds
     *         the provided
     *         threshold, attached to the given event loop
     */
    public BooleanEvent leftTrigger(double threshold, EventLoop loop) {
        return new BooleanEvent(loop, () -> getLeftTriggerAxis() > threshold);
    }

    /**
     * Constructs an event instance around the axis value of the left trigger. The
     * returned trigger
     * will be true when the axis value is greater than 0.5.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the left trigger's axis exceeds
     *         the provided
     *         threshold, attached to the given event loop
     */
    public BooleanEvent leftTrigger(EventLoop loop) {
        return leftTrigger(0.5, loop);
    }

    /**
     * Constructs an event instance around the axis value of the right trigger. The
     * returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link BooleanEvent}
     *                  to be true. This
     *                  value should be in the range [0, 1] where 0 is the unpressed
     *                  state of the axis.
     * @param loop      the event loop instance to attach the event to.
     * @return an event instance that is true when the right trigger's axis exceeds
     *         the provided
     *         threshold, attached to the given event loop
     */
    public BooleanEvent rightTrigger(double threshold, EventLoop loop) {
        return new BooleanEvent(loop, () -> getRightTriggerAxis() > threshold);
    }

    /**
     * Constructs an event instance around the axis value of the right trigger. The
     * returned trigger
     * will be true when the axis value is greater than 0.5.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the right trigger's axis exceeds
     *         the provided
     *         threshold, attached to the given event loop
     */
    public BooleanEvent rightTrigger(EventLoop loop) {
        return rightTrigger(0.5, loop);
    }
}