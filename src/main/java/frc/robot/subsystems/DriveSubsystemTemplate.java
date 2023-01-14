// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DriveSubsystemTemplate extends SubsystemBase{
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
   * @param fwd the left movement
   * @param rot the right movement
   */
  public abstract void tankDrive(double left, double right, double max);

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public abstract void tankDriveVolts(double leftVolts, double rightVolts);

  /** Resets the drive encoders to currently read a position of 0. */
  public abstract void resetEncoders();
}
