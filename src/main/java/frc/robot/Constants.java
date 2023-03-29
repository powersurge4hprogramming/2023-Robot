// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

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

    public static enum LocationType {
      SubstationHigh(-58, 11),
      Chute(-58, 15),
      Hybrid(21, 8.5),
      LowCube(-41, 38),
      LowCone(-52, 40),
      HighCone(-51, 76),
      HighCube(-43, 76),
      ChargeStation(0, 0),
      Starting(-104, 0.0);

      public final double shoulderDegrees;
      public final double armInches;

      private LocationType(double shoulderDegrees, double armInches) {
        this.shoulderDegrees = shoulderDegrees;
        this.armInches = armInches;
      }
    }

    public static final class TurretConstants {
      // for encoder
      private static final double kGearToothRatio = 20.0 / 147;
      private static final double kGearboxRatio = 1.0 / 12;
      private static final double kLossMultipler = 1.0 - (2.25 / 90);
      public static final double kDegreesPerRev = kGearToothRatio * kGearboxRatio * 360 * kLossMultipler;

      public static final int kMotorPort = 12;

      public static final double kPositionTolerance = 1;
      public static final double kVelocityTolerance = 0.01;
      public static final double kP = 0.02;
      public static final double kD = 0.01;
      public static final double kF = 0.00;
      public static final double kMin = -0.21;
      public static final double kMax = 0.21;

      public static final double kStartingDegrees = 180;

      public static final double kMaxRotations = 1.5;

    }

    public static final class ShoulderConstants {
      // for encoder
      private static final double kGearToothRatio = 1 / 6.0;
      private static final double kGearboxRatio = 1 / 100.0;
      // private static final double kSprocketRatio = 1 / 15.0;
      public static final double kDegreesPerRev = kGearToothRatio * kGearboxRatio * 360;

      public static final int kMotorPort = 17;

      public static final double kPositionTolerance = 1.25;
      public static final double kVelocityTolerance = 0.01;
      public static final double kP = 0.05;
      public static final double kD = 0.00;
      public static final double kF = 0.00;
      public static final double kMin = -0.50;
      public static final double kMax = 0.40;

      public static final double kMaxDegrees = 32;
      public static final double kMinDegrees = -104;

    }

    public static final class ArmConstants {
      // for enconder
      // private static final double kGearRatio = 1.0 / 20;
      // private static final double kSpoolRadius = 2* Math.PI * (3.0 / 4 / 2.0) +
      // 0.040;
      // public static final double kDistancePerRevInches = kGearRatio * kSpoolRadius;
      public static final double kDistancePerRevInches = 0.1481470777; // This is empirically determined via testing

      public static final int kMotorPort = 16;

      public static final double kPositionTolerance = 1.25;
      public static final double kVelocityTolerance = 60;
      public static final double kP = 0.1;
      public static final double kD = 0.06;
      public static final double kF = 0.00;
      public static final double kMin = -0.95;
      public static final double kMax = 0.90;

      public static final int kLockSolenoidFwd = 14;
      public static final int kLockSolenoidBkwd = 15;

      public static final double kMaxPosInches = 76;
      // public static final double kMinPosInches = -3; fix offset

      public static final int kLimitSwitchPort = 0;
    }

    public static final class ClawConstants {
      public static final int kClawUpstreamFwd = 3; // 30 PSI
      public static final int kClawUpstreamBkwd = 2; // 60 PSI

      public static final int kClawDownstreamFwd = 5;
      public static final int kClawDownstreamBkwd = 4;

      /** Enum for determining the mode of robot pickup, for pressures. */
      public static enum PickupMode {
        Cone,
        Cube,
        None
      }
    }
  }

  public static final class DriveConstants {
    public static final int kLeftMotorLeaderPort = 13;
    public static final int kLeftMotorFollowerPort = 8;
    public static final int kRightMotorLeaderPort = 6;
    public static final int kRightMotorFollowerPort = 9;

    public static final double kDriveSpeedLimit = 0.65;

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
    public static final double kEncoderVelocityConversion = (2 * kWheelRadiusMeters * Math.PI) / (60 * kGearRatio);

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

    public static enum DriveProfiles {
      BrakeNoRamp,
      CoastRamp,
      CoastNoRamp
    }

    public static final DriveProfiles kDriveProfileDefault = DriveProfiles.CoastNoRamp;
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
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.25;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final String kAutoSelectionKey = "Auto Selector";

    // auto selections based on PathPlanner, from ./deploy/pathplanner dir
    public static final String kDefaultAuto = "S1-P1Cu";
    public static final List<String> kAutoList = List.of("S9-P1Cu", "S6H-O-C", "test");

  }

  public static final class LEDConstants {
    public static final int kNumberOfLEDs = 22;
    public static final int kLEDPWMPort = 0;

    public static final Color kRedAllianceColor = new Color(145, 0, 0);
    public static final Color kBlueAllianceColor = new Color(0, 0, 80);
    public static final Color kInvalidAllianceColor = Color.kHotPink;

    public static final List<Integer> kAllianceLEDIndexes = List.of(0, 8, 9, 10, 11, 20, 21);

    public static final Color kConeColor = new Color(100, 28, 0);
    public static final Color kCubeColor = new Color(100, 0, 60);
    public static final Color kStoppybarColor = new Color(0, 140, 0); // "robot" green

    public static final List<Integer> kPickupLEDIndexes = List.of(1, 2, 3, 4, 5, 6, 7, 12, 13, 14, 15, 16, 17, 18, 19);

    public static final Color kTransparentColor = new Color(0, 0, 0);

  }
}
