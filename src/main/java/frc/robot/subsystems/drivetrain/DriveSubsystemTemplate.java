// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.DriveProfiles;

public abstract class DriveSubsystemTemplate extends SubsystemBase {
  // The gyro sensor
  protected final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // Field for visualizing robot odometry
  protected final Field2d m_field = new Field2d();

  // the brake mode
  protected DriveProfiles m_driveProfile = DriveProfiles.CoastNoRamp;

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public abstract Pose2d getPose();

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public abstract DifferentialDriveWheelSpeeds getWheelSpeeds();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public abstract void resetOdometry(Pose2d pose);

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public abstract void arcadeDrive(double fwd, double rot);

  /**
   * Drives the robot using tank controls.
   *
   * @param left  the left movement
   * @param right the right movement
   */
  public abstract void tankDrive(double left, double right);

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public abstract void tankDriveVolts(double leftVolts, double rightVolts);

  /**
   * Limits robot
   * 
   * @param limit the -1 to 1 limit
   **/
  public abstract void limit(double limit);

  /** Resets the drive encoders to currently read a position of 0. */
  public abstract void resetEncoders();

  /** Toggles brake mode to true or false */
  public abstract void setDriveProfile(DriveProfiles driveProfile);

  /**
   * Toggles brake mode to true or false
   * 
   * @return a command which toggles the brake mode and then finishes
   */
  public abstract CommandBase setDriveProfileCmd(DriveProfiles driveProfile);

  /** Calibrate gyro (takes 5 seconds, robot MUST not move) */
  public void calibrateGyro() {
    System.out.println("Starting calibration");
    m_gyro.calibrate();
    System.out.println("Finishing calibration");

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("");
    builder.addStringProperty("Drive Brake", () -> m_driveProfile.toString(), null);
  }
}
