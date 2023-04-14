// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveConstants.*;

public class DriveSubsystem extends SubsystemBase {

  // left motors
  private final CANSparkMax m_leftMotorLeader = new CANSparkMax(kLeftMotorLeaderPort,
      MotorType.kBrushless);
  private final CANSparkMax m_leftMotorFollower = new CANSparkMax(kLeftMotorFollowerPort,
      MotorType.kBrushless); // should not be called outside of the constructor

  // right motors
  private final CANSparkMax m_rightMotorLeader = new CANSparkMax(kRightMotorLeaderPort,
      MotorType.kBrushless);
  private final CANSparkMax m_rightMotorFollower = new CANSparkMax(kRightMotorFollowerPort,
      MotorType.kBrushless); // should not be called outside of the constructor

  // The drive encoders
  private final RelativeEncoder m_leftEncoder = m_leftMotorLeader.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightMotorLeader.getEncoder();

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotorLeader, m_rightMotorLeader);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Field for visualizing robot odometry
  private final Field2d m_field = new Field2d();

  // the brake mode
  private DriveProfiles m_driveProfile = DriveProfiles.CoastNoRamp;

  // Odometry class for tracking robot pose
  private final DifferentialDrivePoseEstimator m_odometry = new DifferentialDrivePoseEstimator(
      kDriveKinematics,
      m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(),
      new Pose2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
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
    m_leftEncoder.setPositionConversionFactor(kEncoderDistancePerPulse);
    m_rightEncoder.setPositionConversionFactor(kEncoderDistancePerPulse);

    // Set the velocity converter for the encoders
    m_leftEncoder.setVelocityConversionFactor(kEncoderVelocityConversion);
    m_rightEncoder.setVelocityConversionFactor(kEncoderVelocityConversion);

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
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_gyro.reset();
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);

    if (true) {
      double leftSpeed = m_leftMotorLeader.get();
      if (Math.abs(leftSpeed) > 0.04) {
        m_leftMotorLeader.set(Math.signum(leftSpeed) * (Math.abs(leftSpeed)) - 0.03);
      } else {
        m_leftMotorLeader.set(Math.signum(leftSpeed) * (Math.abs(leftSpeed)) - 0.01);
      }
    }

  }

  /**
   * Drives the robot using tank controls.
   *
   * @param left  the left movement
   * @param right the right movement
   */
  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotorLeader.setVoltage(leftVolts);
    m_rightMotorLeader.setVoltage(rightVolts);
    m_drive.feed();
  }

  /**
   * Limits robot
   * 
   * @param limit the -1 to 1 limit
   **/
  public void limit(double limit) {
    m_drive.setMaxOutput(limit);
  }

  private void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public CommandBase resetEncodersCommand() {
    return this.runOnce(() -> resetEncoders()).withName("ResetDriveEncoders");
  }

  private void disableMotors() {
    tankDriveVolts(0, 0);

    m_leftMotorLeader.setIdleMode(IdleMode.kCoast);
    m_leftMotorFollower.setIdleMode(IdleMode.kCoast);
    m_rightMotorLeader.setIdleMode(IdleMode.kCoast);
    m_rightMotorFollower.setIdleMode(IdleMode.kCoast);

    m_leftMotorLeader.stopMotor();
    m_leftMotorFollower.stopMotor();
    m_rightMotorLeader.stopMotor();
    m_rightMotorFollower.stopMotor();
  }

  public CommandBase disableMotorsCommand() {
    return this.runOnce(() -> disableMotors()).withName("DisableDriveMotors");
  }

  private void setDriveProfile(DriveProfiles driveProfile) {
    m_driveProfile = driveProfile;
    updateBrakeMode();
  }

  public CommandBase setDriveProfileCmd(DriveProfiles driveProfile) {
    return this.runOnce(() -> setDriveProfile(driveProfile)).withName("SetDriveProfile");
  }

  /** Updates CANSparkMax brake mode based on {@code m_brake} */
  private void updateBrakeMode() {
    switch (m_driveProfile) {
      case BrakeNoRamp:
        m_leftMotorLeader.setIdleMode(IdleMode.kBrake);
        m_leftMotorFollower.setIdleMode(IdleMode.kBrake);
        m_rightMotorLeader.setIdleMode(IdleMode.kBrake);
        m_rightMotorFollower.setIdleMode(IdleMode.kBrake);

        m_leftMotorLeader.setOpenLoopRampRate(0.0);
        m_leftMotorFollower.setOpenLoopRampRate(0.0);
        m_rightMotorLeader.setOpenLoopRampRate(0.0);
        m_rightMotorFollower.setOpenLoopRampRate(0.0);
        break;
      case CoastRamp:
        m_leftMotorLeader.setIdleMode(IdleMode.kCoast);
        m_leftMotorFollower.setIdleMode(IdleMode.kCoast);
        m_rightMotorLeader.setIdleMode(IdleMode.kCoast);
        m_rightMotorFollower.setIdleMode(IdleMode.kCoast);

        m_leftMotorLeader.setOpenLoopRampRate(0.5);
        m_leftMotorFollower.setOpenLoopRampRate(0.5);
        m_rightMotorLeader.setOpenLoopRampRate(0.5);
        m_rightMotorFollower.setOpenLoopRampRate(0.5);
        break;
      case CoastNoRamp:
      default:
        m_leftMotorLeader.setIdleMode(IdleMode.kCoast);
        m_leftMotorFollower.setIdleMode(IdleMode.kCoast);
        m_rightMotorLeader.setIdleMode(IdleMode.kCoast);
        m_rightMotorFollower.setIdleMode(IdleMode.kCoast);

        m_leftMotorLeader.setOpenLoopRampRate(0.0);
        m_leftMotorFollower.setOpenLoopRampRate(0.0);
        m_rightMotorLeader.setOpenLoopRampRate(0.0);
        m_rightMotorFollower.setOpenLoopRampRate(0.0);
        break;
    }
  }

  public double getAngle() {
    return m_gyro.getAngle();
  }

  public double getPitch() {
    return m_gyro.getPitch();
  }
  
  public boolean isMoving() {
    return m_gyro.isMoving();
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("");
    builder.addStringProperty("Drive Profile", () -> m_driveProfile.toString(), null);
    builder.addBooleanProperty("Gyro Connected", () -> m_gyro.isConnected(), null);
    builder.addBooleanProperty("Gyro Calibrating", () -> m_gyro.isCalibrating(), null);
    builder.addDoubleProperty("Gyro Angle", () -> m_gyro.getAngle(), null);
    builder.addDoubleProperty("Gyro Yaw", () -> m_gyro.getYaw(), null);
    builder.addDoubleProperty("Gyro Roll", () -> m_gyro.getRoll(), null);
    builder.addDoubleProperty("Gyro Pitch", () -> m_gyro.getPitch(), null);
    builder.addBooleanProperty("Is Moving?", () -> m_gyro.isMoving(), null);
    builder.addBooleanProperty("Is Rotating?", () -> m_gyro.isRotating(), null);
  }
}
