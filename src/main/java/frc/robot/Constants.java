// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotorLeaderPort = 6; // TODO
    public static final int kLeftMotorFollowerPort = 8; // TODO
    public static final int kRightMotorLeaderPort = 13; // TODO
    public static final int kRightMotorFollowerPort = 4; // TODO

    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final double kDriveSpeedLimit = 0.15;

    public static final double kTrackWidthMeters = 0.530352;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackWidthMeters);

    // public static final int kEncoderCPR = 42;
    public static final double kGearRatio = 7.31;
    public static final double kWheelRadiusMeters = 0.0762; // 3 inches
    public static final double kEncoderDistancePerPulse = (2 * kWheelRadiusMeters * Math.PI) / kGearRatio; // CPR
                                                                                                           // already
                                                                                                           // taken into
                                                                                                           // account!

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining
    // these
    // values for your robot. TODO
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    public static final double kPDriveVel = 8.5; // TODO
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kArcadeBoardControllerPort = 2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 2.0; // TODO, 4.48056*0.7 is actual
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.75; // TODO

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final String kAutoSelectionKey = "Auto Selector";

    // auto selections based on PathPlanner, from ./deploy/pathplanner dir
    public static final String kDefaultAuto = "S1H-P1Cu-S2H-C";
    public static final List<String> kAutoList = List.of("S1H-P1Cu-S2H-P1Co-S3H", "S9H-P1Cu-S8H-C", "S9H-P4Cu-S8H-P3Co-S7H");

  }

  public static final class VisionConstants {
    // from photonvision webUI
    public static final String kPhotonCameraName = "photonvision"; // TODO

    // Camera location from center of robot
    public static final Transform3d kRobotToCamera = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
        new Rotation3d(0, 0, 0)); // TODO Cam mounted facing forward, half a meter forward of center, half a meter
                                  // up from center.

  }
}
