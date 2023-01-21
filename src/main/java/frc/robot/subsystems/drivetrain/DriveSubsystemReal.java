// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import frc.robot.structs.PhotonCameraWrapper;
import frc.robot.utilities.MathU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystemReal extends DriveSubsystemTemplate {

  // left motors
  private final CANSparkMax m_leftMotorLeader = new CANSparkMax(DriveConstants.kLeftMotorLeaderPort,
      MotorType.kBrushless);
  private final CANSparkMax m_leftMotorFollower = new CANSparkMax(DriveConstants.kLeftMotorFollowerPort,
      MotorType.kBrushless);

  // right motors
  private final CANSparkMax m_rightMotorLeader = new CANSparkMax(DriveConstants.kRightMotorLeaderPort,
      MotorType.kBrushless);
  private final CANSparkMax m_rightMotorFollower = new CANSparkMax(DriveConstants.kRightMotorFollowerPort,
      MotorType.kBrushless);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotorLeader, m_rightMotorLeader);

  // The drive encoders
  private final RelativeEncoder m_leftEncoder = m_leftMotorLeader.getEncoder(); // TODO multiple encoders?
  private final RelativeEncoder m_rightEncoder = m_rightMotorLeader.getEncoder(); // TODO multiple encoders?

  // The gyro sensor
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDrivePoseEstimator m_odometry = new DifferentialDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(),
      new Pose2d());

  // Field for visualizing robot odometry
  private final Field2d m_field = new Field2d();

  // wrapper for Global! PhotonCamera
  private final PhotonCameraWrapper m_photonWrapper;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystemReal(PhotonCameraWrapper photonWrapper) {
    m_photonWrapper = photonWrapper;

    m_leftMotorFollower.follow(m_leftMotorLeader);
    m_rightMotorFollower.follow(m_rightMotorLeader);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotorLeader.setInverted(true);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);

    // reset robot to (0,0) and encoders
    resetOdometry(new Pose2d());

    SmartDashboard.putData(m_field);
    SmartDashboard.putData(m_drive);

  }

  @Override
  public void periodic() {
    // This will get the simulated sensor readings that we set
    // in the previous article while in simulation, but will use
    // real values on the robot itself.
    m_odometry.update(m_gyro.getRotation2d(),
        m_leftEncoder.getPosition(),
        m_rightEncoder.getPosition());

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    Optional<EstimatedRobotPose> result = m_photonWrapper.getEstimatedGlobalPose(m_odometry.getEstimatedPosition());

    if (result.isPresent()) {
      EstimatedRobotPose camPose = result.get();
      m_odometry.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
      m_field.getObject("Cam Est Pos").setPose(camPose.estimatedPose.toPose2d());
    } else {
      // move it way off the screen to make it disappear
      m_field.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
    }

    m_field.setRobotPose(m_odometry.getEstimatedPosition());
  }

  @Override
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  @Override
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  // reset the initial pose to something other than the default constructor used
  // in this classes constructor, for use before auto to know where we are
  @Override
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
  }

  @Override
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  @Override
  public void tankDriveLimit(double left, double right, double max) {
    left = MathU.signLerp(0, max, left);
    right = MathU.signLerp(0, max, right);
    m_drive.tankDrive(left, right);
  }

  @Override
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotorLeader.setVoltage(leftVolts);
    m_rightMotorLeader.setVoltage(rightVolts);
    m_drive.feed();
  }

  @Override
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }
}
