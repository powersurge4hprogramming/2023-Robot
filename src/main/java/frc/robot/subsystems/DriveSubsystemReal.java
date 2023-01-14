// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import frc.robot.utilities.MathU;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystemReal extends DriveSubsystemTemplate {

  // left motors
  private final CANSparkMax m_leftMotorLeader = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  private final CANSparkMax m_leftMotorFollower = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);

  // ;eft motor group
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotorLeader, m_leftMotorFollower);

  // right motors
  private final CANSparkMax m_rightMotorLeader = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
  private final CANSparkMax m_rightMotorFollower = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

  // right motor group
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMotorLeader, m_rightMotorFollower);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The drive encoders
  private final RelativeEncoder m_leftEncoder = m_leftMotorLeader.getEncoder(); // TODO multiple encoders?
  private final RelativeEncoder m_rightEncoder = m_rightMotorLeader.getEncoder(); // TODO multiple encoders?

  // The gyro sensor
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // Field for visualizing robot odometry
  private Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystemReal() {

    m_leftMotorFollower.follow(m_leftMotorLeader);
    m_rightMotorFollower.follow(m_rightMotorLeader);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(
        m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());

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
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

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
  public void tankDrive(double left, double right, double max) {
    left = MathU.signLerp(0, max, left);
    right = MathU.signLerp(0, max, right);
    m_drive.tankDrive(left, right);
  }

  @Override
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  @Override
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }
}
