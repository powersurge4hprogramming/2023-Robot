// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pid;

import frc.robot.subsystems.motor.ArmSubsystem;

public class ArmStopMovement extends ArmSetLength {

    /**
     * Constructs a new {@link ArmSetLength} for the {@link ArmSubsystem} that
     * will hold the robot at the position it was upon construction, and will not finish.
     * 
     * @param subsystem the {@link ArmSubsystem} for the position to be set
     */
    public ArmStopMovement(ArmSubsystem arm) {
        super(arm.getLength(), arm);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
