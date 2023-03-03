// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pid;

import frc.robot.subsystems.motor.ShoulderSubsystem;

public class ShoulderStopMovement extends ShoulderSetAngle {

    /**
     * Constructs a new {@link ShoulderSetAngle } for the {@link ShoulderSetAngle}
     * that
     * will hold the robot at the position it was upon construction, and will not
     * finish.
     * 
     * @param subsystem the {@link ShoulderSetAngle} for the position to be set
     */
    public ShoulderStopMovement(ShoulderSubsystem shoulder) {
        super(shoulder.getLength(), shoulder);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
