// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pid;

import frc.robot.subsystems.ArmSubsystem;

/** Keeps setpoint where the arm currently is. */
public class ArmStopMovement extends ArmSetLength {

    /** sets arm to current length, does not finish */
    public ArmStopMovement(ArmSubsystem arm) {
        super(arm.getLength(), arm);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
