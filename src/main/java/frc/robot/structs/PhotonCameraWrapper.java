package frc.robot.structs;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

public class PhotonCameraWrapper {
  public final PhotonCamera photonCamera;
  public final RobotPoseEstimator photonPoseEstimator;

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
    photonPoseEstimator = new RobotPoseEstimator(
        field2023, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        List.of(new Pair<PhotonCamera, Transform3d>(photonCamera, VisionConstants.kRobotToCamera)));
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return A pair of the fused camera observations to a single Pose2d on the
   *         field, and the time
   *         of the observation. Assumes a planar field and the robot is always
   *         firmly on the ground
   */
  public Optional<Pair<Pose3d, Double>> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }
}