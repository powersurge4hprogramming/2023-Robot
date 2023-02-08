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
import frc.robot.Constants.QuartetConstants.ArmConstants;
import frc.robot.Constants.QuartetConstants.ShoulderConstants;
import frc.robot.commands.pid.ArmSetLength;
import frc.robot.commands.pid.ShoulderSetAngle;
import frc.robot.commands.pid.TurretSetAngle;
import frc.robot.structs.PhotonCameraWrapper;
import frc.robot.structs.hid.CommandPXNArcadeStickController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.StoppyBarSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ClawSubsystem.PickupMode;
import frc.robot.subsystems.drivetrain.DriveSubsystemReal;
import frc.robot.subsystems.drivetrain.DriveSubsystemSim;
import frc.robot.subsystems.drivetrain.DriveSubsystemTemplate;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;

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
        // <-- SUBSYSTEMS -->
        private final DriveSubsystemTemplate m_driveSubsystem;

        private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
        private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();
        private final ShoulderSubsystem m_shoulderSubsystem = new ShoulderSubsystem();
        private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();

        private final StoppyBarSubsystem m_stoppyBarSubsystem = new StoppyBarSubsystem();

        // <-- END SUBSYSTEMS -->

        // <-- STRUCTS --> (HID, controllers, etc)

        // the PhotonCamera global wrapper class
        private final PhotonCameraWrapper m_photonCamera = new PhotonCameraWrapper();

        // The driver's controller
        private final CommandXboxController m_driverController = new CommandXboxController(
                        OIConstants.kDriverControllerPort);

        // The operator's controller
        private final CommandXboxController m_operatorController = new CommandXboxController(
                        OIConstants.kOperatorControllerPort);

        // The arcade pad
        private final CommandPXNArcadeStickController m_arcadePad = new CommandPXNArcadeStickController(
                        OIConstants.kOperatorArcadePort);

        // <-- END STRUCTS --> (HID, controllers, etc)

        // <-- COMMANDS -->

        // the PhotonCamera Smartdashboard sending class
        // private final PhotonCameraReader m_photonCameraReader = new
        // PhotonCameraReader(m_photonCamera);

        // <-- END COMMANDS -->

        /*
         * The map of events for PathPlanner usage. Each key corresponds to a command
         * that can be used at a waypoint.
         */
        private final HashMap<String, Command> m_hashMap = new HashMap<>();

        // A chooser for autonomous commands
        private final SendableChooser<String> m_chooser = new SendableChooser<>();

        // the Autonomous builder for Path planning, doesn't have trajectory
        // constructor, just run .fullAuto(trajectory)
        private final RamseteAutoBuilder m_autoBuilder;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // create sim or real object
                if (RobotBase.isSimulation()) {
                        m_driveSubsystem = new DriveSubsystemSim();
                } else {
                        m_driveSubsystem = new DriveSubsystemReal(m_photonCamera);
                }

                m_hashMap.put("collect0GPrep", new ParallelCommandGroup(
                                new TurretSetAngle(0, m_turretSubsystem),
                                new ArmSetLength(ArmConstants.kGroundPickupArmLength, m_armSubsystem),
                                new ShoulderSetAngle(ShoulderConstants.kGroundPickupShoulderAngle,
                                                m_shoulderSubsystem)));
                m_hashMap.put("place180HPrep",
                                new ParallelCommandGroup(
                                                new TurretSetAngle(180, m_turretSubsystem),
                                                new ArmSetLength(ArmConstants.kHighGoalArmLength, m_armSubsystem),
                                                new ShoulderSetAngle(ShoulderConstants.kHighGoalShoulderAngle,
                                                                m_shoulderSubsystem)));
                m_hashMap.put("place270HPrep",
                                new ParallelCommandGroup(
                                                new TurretSetAngle(270, m_turretSubsystem),
                                                new ArmSetLength(ArmConstants.kHighGoalArmLength, m_armSubsystem),
                                                new ShoulderSetAngle(ShoulderConstants.kHighGoalShoulderAngle,
                                                                m_shoulderSubsystem)));
                m_hashMap.put("place0HPrep",
                                new ParallelCommandGroup(
                                                new TurretSetAngle(0, m_turretSubsystem),
                                                new ArmSetLength(ArmConstants.kHighGoalArmLength, m_armSubsystem),
                                                new ShoulderSetAngle(ShoulderConstants.kHighGoalShoulderAngle,
                                                                m_shoulderSubsystem)));
                m_hashMap.put("grabCu",
                                new InstantCommand(() -> m_clawSubsystem.grab(PickupMode.Cube), m_clawSubsystem));
                m_hashMap.put("grabCo", new RunCommand(() -> m_clawSubsystem.grab(PickupMode.Cube), m_clawSubsystem));
                m_hashMap.put("release", new InstantCommand(() -> m_clawSubsystem.release(), m_clawSubsystem));
                m_hashMap.put("tractionOn",
                                new InstantCommand(() -> m_driveSubsystem.tractionMode(true), m_driveSubsystem));
                m_hashMap.put("tractionOff",
                                new InstantCommand(() -> m_driveSubsystem.tractionMode(false), m_driveSubsystem));

                // add all items to Auto Selector
                m_chooser.setDefaultOption(AutoConstants.kDefaultAuto, AutoConstants.kDefaultAuto);
                for (String opt : AutoConstants.kAutoList) {
                        m_chooser.addOption(opt, opt);
                }

                m_autoBuilder = new RamseteAutoBuilder(m_driveSubsystem::getPose, m_driveSubsystem::resetOdometry,
                                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                                DriveConstants.kDriveKinematics,
                                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                m_driveSubsystem::getWheelSpeeds, new PIDConstants(0, 0, 0),
                                m_driveSubsystem::tankDriveVolts,
                                m_hashMap, true, m_driveSubsystem);

                SmartDashboard.putData("Auto Selector", m_chooser);

                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands

                // Set the drive limit
                m_driveSubsystem.limit(DriveConstants.kDriveSpeedLimit);

                m_driveSubsystem.setDefaultCommand(
                                // A split-stick arcade command, with forward/backward controlled by the left
                                // hand, and turning controlled by the right.
                                new RunCommand(
                                                () -> m_driveSubsystem.arcadeDrive(
                                                                -m_driverController.getLeftY(),
                                                                -m_driverController.getRightX()),
                                                m_driveSubsystem));

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
                // nuclear codes (for endgame solenoids)
                m_operatorController.rightStick().and(m_driverController.back()).onTrue(
                                new InstantCommand(() -> {
                                        m_stoppyBarSubsystem.setStop(true);
                                        m_clawSubsystem.setPickupMode(PickupMode.Error);
                                }, m_stoppyBarSubsystem));
                m_driverController.start().onTrue(
                                new InstantCommand(() -> {
                                        m_stoppyBarSubsystem.setStop(false);
                                        m_clawSubsystem.setPickupMode(PickupMode.None);
                                }, m_stoppyBarSubsystem));

                // Dumb bindings (non distance based)
                // Drive bindings

                // Operator controller bindings
                m_operatorController.leftBumper()
                                .whileTrue(new RunCommand(() -> m_turretSubsystem.run(-0.25), m_turretSubsystem));
                m_operatorController.rightBumper()
                                .whileTrue(new RunCommand(() -> m_turretSubsystem.run(0.25), m_turretSubsystem));
                m_operatorController.leftTrigger()
                                .whileTrue(new RunCommand(() -> m_armSubsystem.run(-0.01), m_armSubsystem));
                m_operatorController.rightTrigger()
                                .whileTrue(new RunCommand(() -> m_armSubsystem.run(0.01), m_armSubsystem));
                m_operatorController.x().onTrue(new InstantCommand(() -> m_clawSubsystem.grab(), m_clawSubsystem));
                m_operatorController.b().onTrue(new InstantCommand(() -> m_clawSubsystem.release(), m_clawSubsystem));
                m_operatorController.pov(0)
                                .whileTrue(new RunCommand(() -> m_shoulderSubsystem.run(0.1), m_shoulderSubsystem));
                m_operatorController.pov(180)
                                .whileTrue(new RunCommand(() -> m_shoulderSubsystem.run(-0.1), m_shoulderSubsystem));
                m_operatorController.y().onTrue(new InstantCommand(() -> m_clawSubsystem.setPickupMode(PickupMode.Cone),
                                m_clawSubsystem));
                m_operatorController.a().onTrue(new InstantCommand(() -> m_clawSubsystem.setPickupMode(PickupMode.Cube),
                                m_clawSubsystem));
                m_operatorController.back()
                                .whileTrue(new RunCommand(() -> m_clawSubsystem.runSwivel(-0.1), m_clawSubsystem));
                m_operatorController.start()
                                .whileTrue(new RunCommand(() -> m_clawSubsystem.runSwivel(0.1), m_clawSubsystem));

                // arcade pad
                // enable/disable brake mode
                m_arcadePad.share().onTrue(
                                new InstantCommand(() -> m_driveSubsystem.tractionMode(true), m_driveSubsystem));
                m_arcadePad.options().onTrue(
                                new InstantCommand(() -> m_driveSubsystem.tractionMode(false), m_driveSubsystem));
                m_arcadePad.rightTrigger().onTrue(new InstantCommand(() -> m_clawSubsystem.release(), m_clawSubsystem));

                // Smart bindings -->
                // Operator controller bindings

                // Turret directional
                BooleanSupplier arcadePadPOVSupplier = () -> (m_arcadePad.getHID().getPOV() != -1);
                new Trigger(arcadePadPOVSupplier)
                                .whileTrue(new TurretSetAngle(m_arcadePad.getHID().getPOV(), m_turretSubsystem));

                // Set shoulder and arm to HIGH GOAL
                m_arcadePad.x().whileTrue(new ParallelCommandGroup(
                                new ShoulderSetAngle(ShoulderConstants.kHighGoalShoulderAngle, m_shoulderSubsystem),
                                new ArmSetLength(ArmConstants.kHighGoalArmLength, m_armSubsystem)));

                // Set shoulder and arm to LOW GOAL
                m_arcadePad.y().whileTrue(new ParallelCommandGroup(
                                new ShoulderSetAngle(ShoulderConstants.kLowGoalShoulderAngle, m_shoulderSubsystem),
                                new ArmSetLength(ArmConstants.kLowGoalArmLength, m_armSubsystem)));

                // Set shoulder and arm to GROUND PICKUP/PLACE
                m_arcadePad.rightBumper().whileTrue(new ParallelCommandGroup(
                                new ShoulderSetAngle(ShoulderConstants.kGroundPickupShoulderAngle, m_shoulderSubsystem),
                                new ArmSetLength(ArmConstants.kGroundPickupArmLength, m_armSubsystem)));

                // Set shoulder and arm to SUBSTATION PICKUP
                m_arcadePad.a().whileTrue(new ParallelCommandGroup(
                                new ShoulderSetAngle(ShoulderConstants.kSubstationPickupShoulderAngle,
                                                m_shoulderSubsystem),
                                new ArmSetLength(ArmConstants.kSubstationPickupArmLength, m_armSubsystem)));

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

                return m_autoBuilder.fullAuto(pathGroup).andThen(() -> m_driveSubsystem.tankDriveVolts(0, 0),
                                m_driveSubsystem);
        }

        /**
         * Calibrate the encoder, takes 5 seconds and can be done while disabled
         */
        public void calibrateRioGyro() {
                m_driveSubsystem.calibrateGyro();
        }

        /**
         * Turn off tractionMode in driveSubsystem
         */
        public void setTractionMode(boolean brakeMode) {
                m_driveSubsystem.tractionMode(brakeMode);
        }

}
