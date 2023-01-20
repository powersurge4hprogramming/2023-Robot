// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.structs.PhotonCameraWrapper;

public class PhotonCameraReader extends CommandBase {
  private final PhotonCameraWrapper m_photonCamera;

  /** Creates a new PhotonCameraReader. */
  public PhotonCameraReader(PhotonCameraWrapper photonCamera) {
    m_photonCamera = photonCamera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Drive Mode", true);
    SmartDashboard.putNumber("#  Of Targets", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_photonCamera.setDriveMode(SmartDashboard.getBoolean("Drive Mode", true));
    PhotonPipelineResult result = m_photonCamera.getResult();
    if (result.hasTargets()) {
      SmartDashboard.putNumber("# Of Targets", result.getTargets().size());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Drive Mode", true);
    m_photonCamera.setDriveMode(true);
    SmartDashboard.putNumber("# Of Targets", 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
