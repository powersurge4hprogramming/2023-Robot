// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystemReal;
import frc.robot.subsystems.DriveSubsystemSim;
import frc.robot.subsystems.DriveSubsystemTemplate;
import frc.robot.utilities.AutoTemplate;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final DriveSubsystemTemplate m_robotDrive;

        // The driver's controller
        XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

        // A chooser for autonomous commands
        SendableChooser<AutoCombo> m_chooser = new SendableChooser<>();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // create sim or real object
                if (RobotBase.isSimulation()) {
                        m_robotDrive = new DriveSubsystemSim();
                } else {
                        m_robotDrive = new DriveSubsystemReal();
                }


                // add all items to Auto Selector
                m_chooser.setDefaultOption("Simple Auto", templateAuto());

                SmartDashboard.putData("Auto Selector", m_chooser);

                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                // Set the default drive command to split-stick arcade drive
                /*
                 * m_robotDrive.setDefaultCommand(
                 * // A split-stick arcade command, with forward/backward controlled by the left
                 * // hand, and turning controlled by the right.
                 * new RunCommand(
                 * () ->
                 * m_robotDrive.arcadeDrive(
                 * -m_driverController.getLeftY(), -m_driverController.getRightX()),
                 * m_robotDrive));
                 */

                if (RobotBase.isSimulation()) {
                        m_robotDrive.setDefaultCommand(
                                        // Tank drive
                                        new RunCommand(
                                                        () -> m_robotDrive.tankDrive(
                                                                        -m_driverController.getLeftY(),
                                                                        -m_driverController.getRightY(), 0.7),
                                                        m_robotDrive));
                } else {
                        m_robotDrive.setDefaultCommand(
                                        // Tank drive
                                        new RunCommand(
                                                        () -> m_robotDrive.tankDrive(
                                                                        -m_driverController.getLeftY(),
                                                                        -m_driverController.getRightY(), 0.7),
                                                        m_robotDrive));
                }
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {

                AutoCombo runAuto = m_chooser.getSelected();

                Field2d m_field = new Field2d();
                SmartDashboard.putData(m_field);

                // Push the trajectory to Field2d.
                m_field.getObject("traj").setTrajectory(runAuto.concatTrajectories);
                SmartDashboard.putNumber("Drive ETA", runAuto.concatTrajectories.getTotalTimeSeconds());

                // Reset odometry to the starting pose of the trajectory.
                m_robotDrive.resetOdometry(runAuto.initialPose);

                return runAuto.auto.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
        }

        AutoCombo templateAuto() {
                Pose2d mid12 = new Pose2d(3, 0, new Rotation2d(0));
                Trajectory traj1 = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                                // End 3 meters straight ahead of where we started, facing forward
                                mid12,
                                // Pass config
                                AutoTemplate.kDefaultAutoTrajConfig);

                RamseteCommand move1 = AutoTemplate.ramseteCommandWithDefaults(traj1, m_robotDrive);

                Trajectory traj2 = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                mid12,
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(2, 1), new Translation2d(4, 4)),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(3, 2, new Rotation2d(0)),
                                // Pass config
                                AutoTemplate.kDefaultAutoTrajConfig);

                RamseteCommand move2 = AutoTemplate.ramseteCommandWithDefaults(traj2, m_robotDrive);

                return new AutoCombo(traj1.getInitialPose(), traj1.concatenate(traj2), move1.andThen(move2));
        }

        class AutoCombo {
                public final Pose2d initialPose;
                public final Trajectory concatTrajectories;
                public final Command auto;

                AutoCombo(Pose2d initialPose, Trajectory concatTrajectories, Command auto) {
                        this.initialPose = initialPose;
                        this.concatTrajectories = concatTrajectories;
                        this.auto = auto;
                }
        }

}
