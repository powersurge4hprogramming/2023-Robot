// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.structs.LimelightHelpers;

public class LimeCameraReader extends CommandBase {

  public LimeCameraReader() {
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Drive Mode", true);
    LimelightHelpers.setLEDMode_ForceOff(null);
  }

  @Override
  public void execute() {
    if (SmartDashboard.getBoolean("Drive Mode", true)) {
      LimelightHelpers.setCameraMode_Driver(null);
    } else {
      LimelightHelpers.setCameraMode_Processor(null);
    }
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Drive Mode", true);
    LimelightHelpers.setCameraMode_Driver(null);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
