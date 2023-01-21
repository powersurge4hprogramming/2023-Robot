package frc.robot.structs;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonCameraWrapper {
  private final PhotonCamera photonCamera;
  private final PhotonPoseEstimator photonPoseEstimator;

  public PhotonCameraWrapper() {
    // load the april tag field layout
    AprilTagFieldLayout field2023;
    try {
      field2023 = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      System.out.println("Fatal: Could not load 2023 Field!!");
      System.exit(1);
      field2023 = null;
    }

    // turret camera
    photonCamera = new PhotonCamera(VisionConstants.kPhotonCameraName);

    // Create pose estimator
    photonPoseEstimator = new PhotonPoseEstimator(field2023, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, VisionConstants.kRobotToCamera);

  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return A pair of the fused camera observations to a single Pose2d on the
   *         field, and the time
   *         of the observation. Assumes a planar field and the robot is always
   *         firmly on the ground
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  /**
   * Set the drive mode
   */
  public void setDriveMode(boolean driveMode) {
    photonCamera.setDriverMode(driveMode);
  }

  public PhotonPipelineResult getResult() {
    return photonCamera.getLatestResult();
  }
}