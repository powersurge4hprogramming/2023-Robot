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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

                m_hashMap.put("collect0GPrep", Commands.parallel(
                                new TurretSetAngle(0, m_turretSubsystem),
                                new ArmSetLength(ArmConstants.kGroundPickupArmLength, m_armSubsystem),
                                new ShoulderSetAngle(ShoulderConstants.kGroundPickupShoulderAngle,
                                                m_shoulderSubsystem)));
                m_hashMap.put("place180HPrep",
                                Commands.parallel(
                                                new TurretSetAngle(180, m_turretSubsystem),
                                                new ArmSetLength(ArmConstants.kHighGoalArmLength, m_armSubsystem),
                                                new ShoulderSetAngle(ShoulderConstants.kHighGoalShoulderAngle,
                                                                m_shoulderSubsystem)));
                m_hashMap.put("place270HPrep",
                                Commands.parallel(
                                                new TurretSetAngle(270, m_turretSubsystem),
                                                new ArmSetLength(ArmConstants.kHighGoalArmLength, m_armSubsystem),
                                                new ShoulderSetAngle(ShoulderConstants.kHighGoalShoulderAngle,
                                                                m_shoulderSubsystem)));
                m_hashMap.put("place0HPrep",
                                Commands.parallel(
                                                new TurretSetAngle(0, m_turretSubsystem),
                                                new ArmSetLength(ArmConstants.kHighGoalArmLength, m_armSubsystem),
                                                new ShoulderSetAngle(ShoulderConstants.kHighGoalShoulderAngle,
                                                                m_shoulderSubsystem)));
                m_hashMap.put("grabCu", m_clawSubsystem.grabCommand(PickupMode.Cube));
                m_hashMap.put("grabCo", m_clawSubsystem.grabCommand(PickupMode.Cone));
                m_hashMap.put("release", m_clawSubsystem.releaseCommand());

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
                                m_driveSubsystem::getWheelSpeeds, new PIDConstants(DriveConstants.kPDriveVel, 0, 0),
                                m_driveSubsystem::tankDriveVolts,
                                m_hashMap, true, m_driveSubsystem);

                SmartDashboard.putData("Auto Selector", m_chooser);

                // Configure the button bindings
                configureButtonBindings();

                // Set the drive limit
                m_driveSubsystem.limit(DriveConstants.kDriveSpeedLimit);

                // Configure default commands
                m_driveSubsystem.setDefaultCommand(
                                // A split-stick arcade command, with forward/backward controlled by the left
                                // hand, and turning controlled by the right.
                                m_driveSubsystem.runEnd(
                                                () -> m_driveSubsystem.arcadeDrive(
                                                                -m_driverController.getLeftY(),
                                                                -m_driverController.getRightX()),
                                                () -> m_driveSubsystem.tankDriveVolts(0, 0)));

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
                                m_stoppyBarSubsystem.setStop(true));
                m_driverController.start().onTrue(
                                m_stoppyBarSubsystem.setStop(false));

                // Dumb bindings (non distance based)
                // Drive bindings

                // Operator controller bindings
                m_operatorController.leftBumper()
                                .whileTrue(m_turretSubsystem.runTurretCommand(-0.25));
                m_operatorController.rightBumper()
                                .whileTrue(m_turretSubsystem.runTurretCommand(0.25));
                m_operatorController.leftTrigger()
                                .whileTrue(m_armSubsystem.runArmCommand(-0.01));
                m_operatorController.rightTrigger()
                                .whileTrue(m_armSubsystem.runArmCommand(0.01));
                m_operatorController.x().onTrue(m_clawSubsystem.grabCommand());
                m_operatorController.b().onTrue(m_clawSubsystem.grabCommand());
                m_operatorController.pov(0)
                                .whileTrue(m_shoulderSubsystem.runShoulderCommand(0.1));
                m_operatorController.pov(180)
                                .whileTrue(m_shoulderSubsystem.runShoulderCommand(-0.1));
                m_operatorController.y().onTrue(m_clawSubsystem.setPickupModeCommand(PickupMode.Cone));
                m_operatorController.a().onTrue(m_clawSubsystem.setPickupModeCommand(PickupMode.Cone));
                m_operatorController.back()
                                .whileTrue(m_clawSubsystem.runSwivelCommand(-0.1));
                m_operatorController.start()
                                .whileTrue(m_clawSubsystem.runSwivelCommand(0.1));

                // arcade pad
                // enable/disable brake mode
                m_arcadePad.share().onTrue(
                                new InstantCommand(() -> m_driveSubsystem.tractionMode(true), m_driveSubsystem));
                m_arcadePad.options().onTrue(
                                new InstantCommand(() -> m_driveSubsystem.tractionMode(false), m_driveSubsystem));
                m_arcadePad.rightTrigger().onTrue(m_clawSubsystem.releaseCommand());

                // Smart bindings -->
                // Operator controller bindings

                // Turret directional
                BooleanSupplier arcadePadPOVSupplier = () -> (m_arcadePad.getHID().getPOV() != -1);
                new Trigger(arcadePadPOVSupplier)
                                .whileTrue(new TurretSetAngle(m_arcadePad.getHID().getPOV(), m_turretSubsystem));

                // Set shoulder and arm to HIGH GOAL
                m_arcadePad.x().whileTrue(Commands.parallel(
                                new ShoulderSetAngle(ShoulderConstants.kHighGoalShoulderAngle, m_shoulderSubsystem),
                                new ArmSetLength(ArmConstants.kHighGoalArmLength, m_armSubsystem)));

                // Set shoulder and arm to LOW GOAL
                m_arcadePad.y().whileTrue(Commands.parallel(
                                new ShoulderSetAngle(ShoulderConstants.kLowGoalShoulderAngle, m_shoulderSubsystem),
                                new ArmSetLength(ArmConstants.kLowGoalArmLength, m_armSubsystem)));

                // Set shoulder and arm to GROUND PICKUP/PLACE
                m_arcadePad.rightBumper().whileTrue(Commands.parallel(
                                new ShoulderSetAngle(ShoulderConstants.kGroundPickupShoulderAngle, m_shoulderSubsystem),
                                new ArmSetLength(ArmConstants.kGroundPickupArmLength, m_armSubsystem)));

                // Set shoulder and arm to SUBSTATION PICKUP
                m_arcadePad.a().whileTrue(Commands.parallel(
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
