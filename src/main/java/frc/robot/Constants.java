// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;

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
  public static final class QuartetConstants {
    public static final class TurretConstants {
      // for encoder
      private static final double kGearToothRatio = 20.0 / 147;
      private static final double kGearboxRatio = 1.0 / 12;
      private static final double kLossMultipler = 1.0 - (2.25 / 90);
      public static final double kDegreesPerRev = kGearToothRatio * kGearboxRatio * 360 * kLossMultipler;

      public static final int kMotorPort = 12;

      public static final double kPositionTolerance = 0.25;
      public static final double kVelocityTolerance = 1;
      public static final double kP = 0.01;
      public static final double kD = 0.01;
      public static final double kMin = -0.15;
      public static final double kMax = 0.15;

      public static final double kStartingDegrees = 180;

      public static final double kMaxRotations = 2; // TODO

    }

    public static final class ShoulderConstants {
      // for encoder
      private static final double kGearToothRatio = 1 / 100.0;
      private static final double kGearboxRatio = 1 / 6.0;
      public static final double kDegreesPerRev = kGearToothRatio * kGearboxRatio * 360;

      public static final int kMotorPort = 17;

      public static final double kPositionTolerance = 0.5;
      public static final double kVelocityTolerance = 1;
      public static final double kP = 0.01;
      public static final double kD = 0.00;
      public static final double kMin = -0.15;
      public static final double kMax = 0.15;

      public static final int kMaxDegrees = 70; // TODO
      public static final int kMinDegrees = -30; // TODO
      public static final int kStartingDegrees = 0; // TODO

      public static final double kHighGoalShoulderAngle = 0; // TODO
      public static final double kLowGoalShoulderAngle = 0; // TODO
      public static final double kGroundPickupShoulderAngle = 0; // TODO
      public static final double kSubstationPickupShoulderAngle = 0; // TODO

    }

    public static final class ArmConstants {
      // for enconder
      // private static final double kGearRatio = 1.0 / 20;
      // private static final double kSpoolRadius = 2* Math.PI * (3.0 / 4 / 2.0) +
      // 0.040;
      // public static final double kDistancePerRevInches = kGearRatio * kSpoolRadius;
      public static final double kDistancePerRevInches = 0.1481470777; // This is empirically determined via testing

      public static final int kMotorPort = 16;

      public static final double kPositionTolerance = 0.25;
      public static final double kVelocityTolerance = 1;
      public static final double kP = 0.05;
      public static final double kD = 0.00;
      public static final double kMin = -0.6;
      public static final double kMax = 0.4;

      public static final int kLockSolenoidFwd = 9; // TODO
      public static final int kLockSolenoidBkwd = 8; // TODO

      public static final double kMaxPosInches = 46.23; // TODO (51 is real max)
      public static final double kMinPosInches = 0.0; //

      public static final double kHighGoalArmLength = 0; // TODO
      public static final double kLowGoalArmLength = 0; // TODO
      public static final double kGroundPickupArmLength = 0; // TODO
      public static final double kSubstationPickupArmLength = 0; // TODO

    }

    public static final class ClawConstants {
      public static final int kClawUpstreamFwd = 3; // 30 PSI
      public static final int kClawUpstreamBkwd = 2; // 60 PSI

      public static final int kClawDownstreamFwd = 4;
      public static final int kClawDownstreamBkwd = 5;

      public static final int kSwivelMotor = 18;
    }
  }

  public static final class DriveConstants {
    public static final int kLeftMotorLeaderPort = 13;
    public static final int kLeftMotorFollowerPort = 8;
    public static final int kRightMotorLeaderPort = 6;
    public static final int kRightMotorFollowerPort = 9;

    public static final double kDriveSpeedLimit = 0.10; // TODO

    public static final double kTrackWidthMeters = 0.530352;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackWidthMeters);

    // private static final int kEncoderCPR = 42;
    private static final double kGearRatio = 7.31;
    private static final double kWheelRadiusMeters = 0.0762; // 3 inches
    public static final double kEncoderDistancePerPulse = (2 * kWheelRadiusMeters * Math.PI) / kGearRatio; // CPR
                                                                                                           // already
                                                                                                           // taken into
                                                                                                           // account!
    public static final double kEncoderVelocityConversion = (2 * kWheelRadiusMeters * Math.PI) / (60 * kGearRatio); // TODO
                                                                                                                    // ensure
                                                                                                                    // this
                                                                                                                    // is
                                                                                                                    // correct

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

  public static final class StoppyBarConstants {
    public static final int kFowardSolenoidPort = 0;
    public static final int kBackwardSolenoidPort = 1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kArcadePort = 2;
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
    public static final List<String> kAutoList = List.of("S1H-P1Cu-S2H-P1Co-S3H", "S9H-P1Cu-S8H-C",
        "S9H-P4Cu-S8H-P3Co-S7H"); // TODO

  }

  public static final class VisionConstants {
    // from photonvision webUI
    public static final String kPhotonCameraName = "photonvision"; // TODO

    // Camera location from center of robot
    public static final Transform3d kRobotToCamera = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
        new Rotation3d(0, 0, 0)); // TODO Cam mounted facing forward, half a meter forward of center, half a meter
                                  // up from center.

  }

  public static final class LEDConstants {
    public static final int kNumberOfLEDs = 24;
    public static final int kLEDPWMPort = 0;

    public static final Color kRedAllianceColor = new Color(145, 0, 0);
    public static final Color kBlueAllianceColor = new Color(0, 0, 80);
    public static final Color kInvalidAllianceColor = Color.kHotPink;

    public static final List<Integer> kAllianceLEDIndexes = List.of(0, 1, 2, 10, 11, 12, 13, 14, 15, 23);

    public static final Color kConeColor = new Color(100, 28, 0);
    public static final Color kCubeColor = new Color(100, 0, 60);
    public static final Color kStoppybarColor = new Color(0, 140, 0); // "robot" green

    public static final List<Integer> kPickupLEDIndexes = List.of(3, 4, 5, 6, 7, 8, 9, 16, 17, 18, 19, 20, 21, 22);

    public static final Color kTransparentColor = new Color(0, 0, 0);

  }
}
