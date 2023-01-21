// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.PhotonCameraReader;
import frc.robot.structs.PXNArcadeStickController;
import frc.robot.structs.PhotonCameraWrapper;
import frc.robot.subsystems.drivetrain.DriveSubsystemReal;
import frc.robot.subsystems.drivetrain.DriveSubsystemSim;
import frc.robot.subsystems.drivetrain.DriveSubsystemTemplate;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

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

        // the PhotonCamera global wrapper class
        private final PhotonCameraWrapper m_photonCamera = new PhotonCameraWrapper();
        // the PhotonCamera Smartdashboard sending class
        private final PhotonCameraReader m_photonCameraReader = new PhotonCameraReader(m_photonCamera);

        // The driver's controller
        private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
        // The PXN arcade stick board
        private final PXNArcadeStickController m_arcadeBoard = new PXNArcadeStickController(
                        OIConstants.kArcadeBoardControllerPort);

        // A chooser for autonomous commands
        private final SendableChooser<String> m_chooser = new SendableChooser<>();

        // the Autonomous builder for Path planning, doesn't have trajectory
        // constructor, just run .fullAuto(trajectory)
        private final RamseteAutoBuilder m_autoBuilder;

        /*
         * The map of events for PathPlanner usage. Each key corresponds to a command
         * that can be used at a waypoint.
         */
        private final HashMap<String, Command> m_hashMap = new HashMap<>();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // create sim or real object
                if (RobotBase.isSimulation()) {
                        m_robotDrive = new DriveSubsystemSim();
                } else {
                        m_robotDrive = new DriveSubsystemReal(m_photonCamera);
                }

                m_hashMap.put("printHelloAfter3sec", new SequentialCommandGroup(
                                new WaitCommand(3),
                                new PrintCommand("Hello World")));

                m_hashMap.put("collect0Cu", new PrintCommand("collect0Cu"));
                m_hashMap.put("collect0Co", new PrintCommand("collect0Co"));
                m_hashMap.put("place180H", new PrintCommand("place180H"));
                m_hashMap.put("place90H", new PrintCommand("place90H"));

                // add all items to Auto Selector
                m_chooser.setDefaultOption(AutoConstants.kDefaultAuto, AutoConstants.kDefaultAuto);

                for (

                String opt : AutoConstants.kAutoList) {
                        m_chooser.addOption(opt, opt);
                }

                m_autoBuilder = new RamseteAutoBuilder(m_robotDrive::getPose, m_robotDrive::resetOdometry,
                                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                                DriveConstants.kDriveKinematics,
                                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                m_robotDrive::getWheelSpeeds, new PIDConstants(0, 0, 0), m_robotDrive::tankDriveVolts,
                                m_hashMap, true, m_robotDrive);

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

                // Tank drive (each stick is one side)
                m_robotDrive.setDefaultCommand(
                                new RunCommand(() -> m_robotDrive.tankDriveLimit(-m_driverController.getLeftY(),
                                                -m_driverController.getRightY(), 0.7), m_robotDrive));

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

                // Get the PathPlanner key from the SendableChooser
                final String runAuto = m_chooser.getSelected();

                // load the Path group from the deploy pathplanner directory, with the default
                // constraints outlined in AutoConstants
                final List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(runAuto,
                                new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
                                                AutoConstants.kMaxAccelerationMetersPerSecondSquared));

                return m_autoBuilder.fullAuto(pathGroup).andThen(() -> m_robotDrive.tankDriveVolts(0, 0), m_robotDrive);
        }

}
