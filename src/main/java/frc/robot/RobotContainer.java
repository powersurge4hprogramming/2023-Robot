// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.QuartetConstants.LocationType;
import frc.robot.Constants.QuartetConstants.ClawConstants.PickupMode;
import frc.robot.commands.TurretDynamicAngle;
import frc.robot.structs.LEDManager;
import frc.robot.structs.LimelightHelpers;
import frc.robot.structs.hid.CommandPXNArcadeStickController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.MechQuartetSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.StoppyBarSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.drivetrain.DriveSubsystemReal;
import frc.robot.subsystems.drivetrain.DriveSubsystemSim;
import frc.robot.subsystems.drivetrain.DriveSubsystemTemplate;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;
import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

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

        private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();
        private final ShoulderSubsystem m_shoulderSubsystem = new ShoulderSubsystem();
        private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
        private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();

        private final StoppyBarSubsystem m_stoppyBarSubsystem = new StoppyBarSubsystem();

        // <-- END SUBSYSTEMS -->

        // <-- STRUCTS --> (HID, controllers, etc)

        // The driver's controller
        private final CommandXboxController m_driverController = new CommandXboxController(
                        OIConstants.kDriverControllerPort);

        // The operator's controller
        private final CommandXboxController m_operatorController = new CommandXboxController(
                        OIConstants.kOperatorControllerPort);

        // The arcade pad
        private final CommandPXNArcadeStickController m_arcadePad = new CommandPXNArcadeStickController(
                        OIConstants.kArcadePort);

        // <-- END STRUCTS --> (HID, controllers, etc)

        // <-- COMMANDS -->

        // the PhotonCamera Smartdashboard sending class
        // private final LimeCameraReader m_limelightReader = new LimeCameraReader();

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

        private int m_smartIndex = 0;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                new MechQuartetSubsystem(m_armSubsystem::getLength,
                                m_shoulderSubsystem::getAngle);

                // create sim or real object
                if (RobotBase.isSimulation()) {
                        m_driveSubsystem = new DriveSubsystemSim();
                } else {
                        m_driveSubsystem = new DriveSubsystemReal();
                }

                m_hashMap.put("collect0GPrep", Commands.parallel(
                                m_turretSubsystem.moveToAngle(0),
                                m_armSubsystem.moveToLocation(LocationType.Hybrid),
                                m_shoulderSubsystem.moveToLocation(LocationType.Hybrid))
                                .withName("collect0GPrep"));
                m_hashMap.put("collect180GPrep",
                                Commands.parallel(
                                                m_turretSubsystem.moveToAngle(180),
                                                m_armSubsystem.moveToLocation(LocationType.Hybrid),
                                                m_shoulderSubsystem.moveToLocation(LocationType.Hybrid))
                                                .withName("collect180GPrep"));
                m_hashMap.put("place270HPrep",
                                Commands.parallel(
                                                m_turretSubsystem.moveToAngle(270),
                                                m_armSubsystem.moveToLocation(LocationType.High),
                                                m_shoulderSubsystem.moveToLocation(LocationType.High))
                                                .withName("place270HPrep"));
                m_hashMap.put("place0HPrep",
                                Commands.parallel(
                                                m_turretSubsystem.moveToAngle(0),
                                                m_armSubsystem.moveToLocation(LocationType.High),
                                                m_shoulderSubsystem.moveToLocation(LocationType.High))
                                                .withName("place0HPrep"));
                m_hashMap.put("retract",
                                Commands.parallel(
                                                m_turretSubsystem.moveToAngle(0),
                                                m_armSubsystem.moveToLocation(LocationType.Starting),
                                                m_shoulderSubsystem.moveToLocation(LocationType.Starting))
                                                .withName("retract"));
                m_hashMap.put("grabCu", m_clawSubsystem.grabCommand(PickupMode.Cube));
                m_hashMap.put("grabCo", m_clawSubsystem.grabCommand(PickupMode.Cone));
                m_hashMap.put("release", m_clawSubsystem.releaseCommand());
                m_hashMap.put("climb", m_stoppyBarSubsystem.setStop(true));

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

                SmartDashboard.putData("Arm", m_armSubsystem);
                SmartDashboard.putData("Shoulder", m_shoulderSubsystem);
                SmartDashboard.putData("Claw", m_clawSubsystem);
                SmartDashboard.putData("StoppyBar", m_stoppyBarSubsystem);
                SmartDashboard.putData("Turret", m_turretSubsystem);
                SmartDashboard.putData("Drive", m_driveSubsystem);

                SmartDashboard.putBoolean("PIDs On", false);
                SmartDashboard.putData(CommandScheduler.getInstance());

                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                m_driveSubsystem.setDefaultCommand(
                                // A split-stick arcade command, with forward/backward controlled by the left
                                // hand, and turning controlled by the right.
                                m_driveSubsystem.runEnd(
                                                () -> m_driveSubsystem.arcadeDrive(
                                                                -m_driverController.getLeftY(),
                                                                -m_driverController.getRightX()),
                                                () -> m_driveSubsystem.tankDriveVolts(0, 0)).withName("DriveArcade"));

                m_armSubsystem.lockPosition().schedule(); // TODO see if will work
                m_shoulderSubsystem.lockPosition().schedule();
                m_turretSubsystem.lockPosition().schedule();

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
                m_driverController.back().onTrue(
                                m_stoppyBarSubsystem.setStop(true));
                m_driverController.start().onTrue(
                                m_stoppyBarSubsystem.setStop(false));

                // Dumb bindings (non distance based)
                // Drive bindings

                // Set brake mode, with a debounce of 0.5 seconds to prevent accidental left
                // stick activation
                m_driverController.leftStick().debounce(0.25).onTrue(m_driveSubsystem.setBrakeModeCommand(true));
                m_driverController.leftStick().onFalse(m_driveSubsystem.setBrakeModeCommand(false));

                // Operator controller bindings
                m_operatorController.leftBumper()
                                .onTrue(m_turretSubsystem.incrementPosition(-3));
                m_operatorController.rightBumper()
                                .onTrue(m_turretSubsystem.incrementPosition(3));
                m_operatorController.leftTrigger()
                                .onTrue(m_armSubsystem.incrementPosition(-1));
                m_operatorController.rightTrigger()
                                .whileTrue(m_armSubsystem.incrementPosition(1));
                m_operatorController.x().onTrue(m_clawSubsystem.grabCommand());
                m_operatorController.b().onTrue(m_clawSubsystem.releaseCommand());
                m_operatorController.pov(0)
                                .whileTrue(m_shoulderSubsystem.incrementPosition(1));
                m_operatorController.pov(180)
                                .whileTrue(m_shoulderSubsystem.incrementPosition(-1));
                m_operatorController.y().onTrue(m_clawSubsystem.setPickupModeCommand(PickupMode.Cone));
                m_operatorController.a().onTrue(m_clawSubsystem.setPickupModeCommand(PickupMode.Cube));

                m_operatorController.leftStick().onTrue(m_armSubsystem.toggleArmLock());

                // arcade pad
                m_arcadePad.rightTrigger().onTrue(m_clawSubsystem.releaseCommand());
                m_arcadePad.L3().onTrue(Commands.run(() -> LEDManager.start(), new Subsystem[0]).withName("StartLEDs"));
                m_arcadePad.R3().onTrue(Commands.run(() -> LEDManager.stop(), new Subsystem[0]).withName("StopLEDs"));

                // destructive!
                m_arcadePad.share().onTrue(Commands.run(() -> {
                        m_driveSubsystem.resetEncoders();
                        m_driveSubsystem.calibrateGyro();
                        m_armSubsystem.resetEncoders();
                        m_shoulderSubsystem.resetEncoders();
                        m_turretSubsystem.resetEncoders();
                }, m_armSubsystem, m_turretSubsystem, m_shoulderSubsystem, m_driveSubsystem).withName("ResetEncoders"));

                m_arcadePad.options().onTrue(Commands.run(() -> {
                        m_driveSubsystem.tankDriveVolts(0, 0);
                        m_driveSubsystem.setBrakeMode(false);
                        m_armSubsystem.disableMotor();
                        m_shoulderSubsystem.disableMotor();
                        m_turretSubsystem.disableMotor();
                }, m_armSubsystem, m_turretSubsystem, m_shoulderSubsystem, m_driveSubsystem).withName("ReleaseMotors"));

                // Smart bindings -->
                // Operator controller bindings

                // Turret directional
                new Trigger(() -> (m_arcadePad.getHID().getPOV() != -1))
                                .whileTrue(new TurretDynamicAngle(() -> m_arcadePad.getHID().getPOV(),
                                                m_turretSubsystem)
                                                .beforeStarting(this::pidUp, new Subsystem[0])
                                                .finallyDo(this::pidDown).withName("SetTurretPOV"));

                // Set shoulder and arm to HIGH GOAL
                m_arcadePad.x().whileTrue(Commands.parallel(
                                m_shoulderSubsystem.moveToLocation(LocationType.High),
                                m_armSubsystem.moveToLocation(LocationType.High))
                                .beforeStarting(this::pidUp, new Subsystem[0])
                                .finallyDo(this::pidDown).withName("HighGoal"));

                // Set shoulder and arm to LOW GOAL
                m_arcadePad.y().whileTrue((Commands.parallel(
                                m_shoulderSubsystem.moveToLocation(LocationType.Low),
                                m_armSubsystem.moveToLocation(LocationType.Low))
                                .beforeStarting(this::pidUp, new Subsystem[0])).finallyDo(this::pidDown)
                                .withName("LowGoal"));

                // Set shoulder and arm to GROUND PICKUP/PLACE
                m_arcadePad.rightBumper().whileTrue(Commands.parallel(
                                m_shoulderSubsystem.moveToLocation(LocationType.Hybrid),
                                m_armSubsystem.moveToLocation(LocationType.Hybrid))
                                .beforeStarting(this::pidUp, new Subsystem[0])
                                .finallyDo(this::pidDown).withName("Ground"));

                // Set shoulder and arm to SUBSTATION PICKUP
                m_arcadePad.a().whileTrue(Commands.parallel(
                                m_shoulderSubsystem.moveToLocation(LocationType.SubstationHigh),
                                m_armSubsystem.moveToLocation(LocationType.SubstationHigh))
                                .beforeStarting(this::pidUp, new Subsystem[0])
                                .finallyDo(this::pidDown).withName("Substation"));

                // Set shoulder and arm to CHUTE PICKUP
                m_arcadePad.leftBumper().whileTrue(Commands.parallel(
                                m_shoulderSubsystem.moveToLocation(LocationType.Chute),
                                m_armSubsystem.moveToLocation(LocationType.Chute))
                                .beforeStarting(this::pidUp, new Subsystem[0])
                                .finallyDo(this::pidDown).withName("Chute"));

                // Set shoulder and arm to full starting/finishing retraction
                m_arcadePad.b().whileTrue(Commands.parallel(
                                m_shoulderSubsystem.moveToLocation(LocationType.Starting),
                                m_armSubsystem.moveToLocation(LocationType.Starting)).withName("ResetStarting")
                                .beforeStarting(this::pidUp, new Subsystem[0])
                                .finallyDo(this::pidDown));

        }

        private void pidUp() {
                m_smartIndex++;
                setRumble();
        }

        private void pidDown(boolean end) {
                m_smartIndex--;
                setRumble();
        }

        private void setRumble() {
                if (m_smartIndex > 0) {
                        m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.5);
                        SmartDashboard.putBoolean("PIDs On", true);
                } else {
                        m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                        SmartDashboard.putBoolean("PIDs On", false);
                }
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

        public void robotInit() {
                PathPlannerServer.startServer(5811); // TODO disable for competition

                m_driveSubsystem.calibrateGyro();

                // Set the drive limit
                m_driveSubsystem.limit(DriveConstants.kDriveSpeedLimit);
        }

        /**
         * Init autonomous, before the auto command is scheduled
         */
        public void autonomousInit() {
                LEDManager.initialize();
                LEDManager.start();

                m_armSubsystem.setArmLock(false);
                m_driveSubsystem.setBrakeMode(true);
        }

        public void teleopInit() {
                LEDManager.initialize();
                LEDManager.start();

                m_armSubsystem.setArmLock(false);
                m_driveSubsystem.setBrakeMode(m_driverController.leftStick().getAsBoolean());
                LimelightHelpers.setCameraMode_Driver(null);

        }

}
