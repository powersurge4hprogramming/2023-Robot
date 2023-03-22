// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveSubsystemReal extends DriveSubsystemTemplate {

  // left motors
  private final CANSparkMax m_leftMotorLeader = new CANSparkMax(DriveConstants.kLeftMotorLeaderPort,
      MotorType.kBrushless);
  private final CANSparkMax m_leftMotorFollower = new CANSparkMax(DriveConstants.kLeftMotorFollowerPort,
      MotorType.kBrushless); // should not be called outside of the constructor

  // right motors
  private final CANSparkMax m_rightMotorLeader = new CANSparkMax(DriveConstants.kRightMotorLeaderPort,
      MotorType.kBrushless);
  private final CANSparkMax m_rightMotorFollower = new CANSparkMax(DriveConstants.kRightMotorFollowerPort,
      MotorType.kBrushless); // should not be called outside of the constructor

  // The drive encoders
  private final RelativeEncoder m_leftEncoder = m_leftMotorLeader.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightMotorLeader.getEncoder();

  // Odometry class for tracking robot pose
  private final DifferentialDrivePoseEstimator m_odometry = new DifferentialDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(),
      new Pose2d());

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotorLeader, m_rightMotorLeader);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystemReal() {
    m_leftMotorLeader.restoreFactoryDefaults();
    m_leftMotorFollower.restoreFactoryDefaults();
    m_rightMotorLeader.restoreFactoryDefaults();
    m_rightMotorFollower.restoreFactoryDefaults();

    m_leftMotorFollower.follow(m_leftMotorLeader);
    m_rightMotorFollower.follow(m_rightMotorLeader);

    // m_leftMotorLeader.getPIDController().setOutputRange(-0.9, 0.9);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotorLeader.setInverted(true);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);

    // Set the velocity converter for the encoders
    m_leftEncoder.setVelocityConversionFactor(DriveConstants.kEncoderVelocityConversion);
    m_rightEncoder.setVelocityConversionFactor(DriveConstants.kEncoderVelocityConversion);

    // reset robot to (0,0) and encoders
    resetOdometry(new Pose2d());

    calibrateGyro();

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

    SmartDashboard.putBoolean("Drive Brake", m_brake);
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
  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  @Override
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotorLeader.setVoltage(leftVolts);
    m_rightMotorLeader.setVoltage(rightVolts);
    m_drive.feed();
  }

  @Override
  public void limit(double limit) {
    m_drive.setMaxOutput(limit);
  }

  @Override
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  @Override
  public void setBrakeMode(boolean brakeMode) {
    m_brake = brakeMode;
    updateBrakeMode();
  }

  @Override
  public CommandBase setBrakeModeCommand(boolean brakeMode) {
    return this.runOnce(() -> {
      setBrakeMode(brakeMode);
    }).withName("ToggleBrakeMode");
  }

  /** Updates CANSparkMax brake mode based on {@code m_brake} */
  private void updateBrakeMode() {
    if (m_brake) {
      m_leftMotorLeader.setIdleMode(IdleMode.kBrake);
      m_leftMotorFollower.setIdleMode(IdleMode.kBrake);
      m_rightMotorLeader.setIdleMode(IdleMode.kBrake);
      m_rightMotorFollower.setIdleMode(IdleMode.kBrake);
    } else {
      m_leftMotorLeader.setIdleMode(IdleMode.kCoast);
      m_leftMotorFollower.setIdleMode(IdleMode.kCoast);
      m_rightMotorLeader.setIdleMode(IdleMode.kCoast);
      m_rightMotorFollower.setIdleMode(IdleMode.kCoast);
    }
  }
}
