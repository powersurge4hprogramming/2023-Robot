// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AutoConstants.Auto;
import frc.robot.Constants.AutoConstants.AutoType;
import frc.robot.Constants.DriveConstants.DriveProfiles;
import frc.robot.Constants.QuartetConstants.LocationType;
import frc.robot.Constants.QuartetConstants.ClawConstants.PickupMode;
import frc.robot.commands.Autobalance;
import frc.robot.commands.TurretDynamicAngle;
import frc.robot.commands.led.CaydenLEDCommand;
import frc.robot.commands.led.LEDCompetition;
import frc.robot.structs.LimelightHelpers;
import frc.robot.structs.hid.CommandPXNArcadeStickController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.StoppyBarSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
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
        private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

        private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();
        private final ShoulderSubsystem m_shoulderSubsystem = new ShoulderSubsystem();
        private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
        private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();

        private final StoppyBarSubsystem m_stoppyBarSubsystem = new StoppyBarSubsystem();

        private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

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
        private final HashMap<String, Command> m_autoCmdMap = new HashMap<>();

        // A chooser for autonomous commands
        private final SendableChooser<Auto> m_autoChooser = new SendableChooser<>();

        // the Autonomous builder for Path planning, doesn't have trajectory
        // constructor, just run .fullAuto(trajectory)
        private final RamseteAutoBuilder m_autoBuilder = new RamseteAutoBuilder(m_driveSubsystem::getPose,
                        m_driveSubsystem::resetOdometry,
                        AutoConstants.kRameseteController,
                        DriveConstants.kDriveKinematics,
                        DriveConstants.kDriveFeedforward,
                        m_driveSubsystem::getWheelSpeeds, DriveConstants.kDrivePID,
                        m_driveSubsystem::tankDriveVolts,
                        m_autoCmdMap, true, m_driveSubsystem);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                /*
                 * new MechQuartetSubsystem(m_armSubsystem::getLength,
                 * m_shoulderSubsystem::getAngle);
                 */

                m_autoCmdMap.put("collect0GPrep", Commands.parallel(
                                m_turretSubsystem.moveToAngle(0),
                                m_armSubsystem.moveToLocation(LocationType.Hybrid),
                                m_shoulderSubsystem.moveToLocation(LocationType.Hybrid))
                                .withName("collect0GPrep"));
                m_autoCmdMap.put("collect180GPrep",
                                Commands.parallel(
                                                m_turretSubsystem.moveToAngle(180),
                                                m_armSubsystem.moveToLocation(LocationType.Hybrid),
                                                m_shoulderSubsystem.moveToLocation(LocationType.Hybrid))
                                                .withName("collect180GPrep"));
                m_autoCmdMap.put("place270HPrep",
                                Commands.parallel(
                                                m_turretSubsystem.moveToAngle(270),
                                                Commands.either(Commands.parallel(
                                                                m_armSubsystem.moveToLocation(LocationType.HighCone),
                                                                m_shoulderSubsystem
                                                                                .moveToLocation(LocationType.HighCone)),
                                                                Commands.parallel(
                                                                                m_armSubsystem.moveToLocation(
                                                                                                LocationType.HighCube),
                                                                                m_shoulderSubsystem.moveToLocation(
                                                                                                LocationType.HighCube)),
                                                                () -> m_clawSubsystem
                                                                                .getPickupMode() == PickupMode.Cone))
                                                .withName("place270HPrep"));
                m_autoCmdMap.put("place180HPrep",
                                Commands.parallel(
                                                m_turretSubsystem.moveToAngle(180),
                                                Commands.either(Commands.parallel(
                                                                m_armSubsystem.moveToLocation(LocationType.HighCone),
                                                                m_shoulderSubsystem
                                                                                .moveToLocation(LocationType.HighCone)),
                                                                Commands.parallel(
                                                                                m_armSubsystem.moveToLocation(
                                                                                                LocationType.HighCube),
                                                                                m_shoulderSubsystem.moveToLocation(
                                                                                                LocationType.HighCube)),
                                                                () -> m_clawSubsystem
                                                                                .getPickupMode() == PickupMode.Cone))
                                                .withName("place180HPrep"));
                m_autoCmdMap.put("place180MPrep",
                                Commands.parallel(
                                                m_turretSubsystem.moveToAngle(180),
                                                Commands.either(Commands.parallel(
                                                                m_armSubsystem.moveToLocation(LocationType.LowCone),
                                                                m_shoulderSubsystem
                                                                                .moveToLocation(LocationType.LowCone)),
                                                                Commands.parallel(
                                                                                m_armSubsystem.moveToLocation(
                                                                                                LocationType.LowCube),
                                                                                m_shoulderSubsystem.moveToLocation(
                                                                                                LocationType.LowCube)),
                                                                () -> m_clawSubsystem
                                                                                .getPickupMode() == PickupMode.Cone))
                                                .withName("place180MPrep"));
                m_autoCmdMap.put("place0HPrep",
                                Commands.parallel(
                                                m_turretSubsystem.moveToAngle(0),
                                                Commands.either(Commands.parallel(
                                                                m_armSubsystem.moveToLocation(LocationType.HighCone),
                                                                m_shoulderSubsystem
                                                                                .moveToLocation(LocationType.HighCone)),
                                                                Commands.parallel(
                                                                                m_armSubsystem.moveToLocation(
                                                                                                LocationType.HighCube),
                                                                                m_shoulderSubsystem.moveToLocation(
                                                                                                LocationType.HighCube)),
                                                                () -> m_clawSubsystem
                                                                                .getPickupMode() == PickupMode.Cone))
                                                .withName("place0HPrep"));
                m_autoCmdMap.put("retract",
                                Commands.sequence(
                                                Commands.either(m_armSubsystem
                                                                .moveToLocation(LocationType.RetractSlightOnlyArm),
                                                                Commands.none(),
                                                                () -> (m_armSubsystem
                                                                                .getLength() > LocationType.RetractSlightOnlyArm.armInches)),
                                                Commands.parallel(
                                                                m_shoulderSubsystem
                                                                                .moveToLocation(LocationType.Starting),
                                                                m_armSubsystem.moveToLocation(LocationType.Starting)))
                                                .withName("ResetStarting"));
                m_autoCmdMap.put("chargeStation",
                                Commands.parallel(m_turretSubsystem.moveToAngle(0),
                                                m_armSubsystem.moveToLocation(LocationType.ChargeStation),
                                                m_shoulderSubsystem.moveToLocation(LocationType.ChargeStation))
                                                .withName("chargeStation"));

                m_autoCmdMap.put("turret0", m_turretSubsystem.moveToAngle(0));
                m_autoCmdMap.put("grabCu",
                                m_clawSubsystem.grabCommand(PickupMode.Cube).andThen(Commands.waitSeconds(0.5)));
                m_autoCmdMap.put("grabCo",
                                m_clawSubsystem.grabCommand(PickupMode.Cone).andThen(Commands.waitSeconds(0.5)));
                m_autoCmdMap.put("release", m_clawSubsystem.releaseCommand().andThen(Commands.waitSeconds(0.75)));
                m_autoCmdMap.put("climb", m_stoppyBarSubsystem.setStopCommand(true).andThen(Commands.waitSeconds(1.5)));

                // add all items to Auto Selector
                m_autoChooser.setDefaultOption(AutoConstants.kDefaultAuto.prettyName, AutoConstants.kDefaultAuto);
                for (Auto opt : AutoConstants.kAutoList) {
                        m_autoChooser.addOption(opt.pathName, opt);
                }

                SmartDashboard.putData("Auto Selector", m_autoChooser);

                SmartDashboard.putData("Arm", m_armSubsystem);
                SmartDashboard.putData("Shoulder", m_shoulderSubsystem);
                SmartDashboard.putData("Claw", m_clawSubsystem);
                SmartDashboard.putData("StoppyBar", m_stoppyBarSubsystem);
                SmartDashboard.putData("Turret", m_turretSubsystem);
                SmartDashboard.putData("Drive", m_driveSubsystem);
                SmartDashboard.putData("LEDs", m_ledSubsystem);

                SmartDashboard.putBoolean("PIDs On", false);
                SmartDashboard.putData(CommandScheduler.getInstance());

                // Configure default commands
                m_driveSubsystem.setDefaultCommand(
                                // A split-stick arcade command, with forward/backward controlled by the left
                                // hand, and turning controlled by the right.
                                m_driveSubsystem.runEnd(
                                                () -> m_driveSubsystem.arcadeDrive(
                                                                -m_driverController.getLeftY(),
                                                                (-m_driverController.getRightX() * 0.8)),
                                                () -> m_driveSubsystem.tankDriveVolts(0, 0)).withName("DriveArcade"));

           /*      m_ledSubsystem.setDefaultCommand(new LEDCompetition(m_ledSubsystem, m_stoppyBarSubsystem::stoppyOn,
                                m_clawSubsystem::getPickupMode)); */
                
                                m_ledSubsystem.setDefaultCommand(new CaydenLEDCommand(m_ledSubsystem));  

                // in order the command all the time, make a new one set as default.
                // m_ledSubsystem.setDefaultCommand(new ExampleLEDCommand(m_ledSubsystem));

                // --> on init/cross subsystem triggers:

                // startup code
                if (!DriverStation.isFMSAttached()) {
                        PathPlannerServer.startServer(5811);
                }
                m_driveSubsystem.limit(DriveConstants.kDriveSpeedLimit);

                // autoInit events
                Trigger autoTrigger = new Trigger(() -> RobotState.isEnabled() && RobotState.isAutonomous());

                autoTrigger.onTrue(Commands.sequence(m_armSubsystem.lockLength(),
                                m_armSubsystem.setArmLockCommand(false)).withName("AutoInitArmLock"));
                autoTrigger.onTrue(Commands.sequence(
                                Commands.run(() -> m_driveSubsystem.resetGyro(), m_driveSubsystem),
                                m_driveSubsystem.setDriveProfileCmd(DriveProfiles.BrakeNoRamp))
                                .withName("AutoInitDriveStartup"));

                // teleopInit events
                Trigger teleopTrigger = new Trigger(() -> RobotState.isEnabled() && RobotState.isTeleop());

                teleopTrigger.and(m_operatorController.leftStick()).onTrue(
                                Commands.sequence(m_armSubsystem.lockLength(), m_armSubsystem.setArmLockCommand(true))
                                                .withName("TeleopInitArmLock"));
                teleopTrigger.and(m_operatorController.leftStick().negate())
                                .onTrue(Commands.sequence(m_armSubsystem.lockLength(),
                                                m_armSubsystem.setArmLockCommand(false))
                                                .withName("TeleopInitArmUnlock"));
                teleopTrigger.onTrue(Commands.sequence(
                                Commands.runOnce(() -> m_driveSubsystem.resetGyro(), m_driveSubsystem),
                                m_driveSubsystem.setDriveProfileCmd(DriveConstants.kDriveProfileDefault))
                                .withName("TeleopInitDriveStartup"));
                teleopTrigger.onTrue(
                                Commands.runOnce(() -> LimelightHelpers.setCameraMode_Driver(null), new Subsystem[0]));

                // whole time teleop events
                teleopTrigger.whileTrue(Commands.run(() -> {
                        if (m_turretSubsystem.atSetpoint() && m_shoulderSubsystem.atSetpoint()
                                        && m_armSubsystem.atSetpoint()) {
                                m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                                SmartDashboard.putBoolean("PIDs On", false);
                        } else {
                                m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.5);
                                SmartDashboard.putBoolean("PIDs On", true);
                        }
                }, new Subsystem[0]).withName("UpdatePIDStatus"));

                // teleopInit or autoInit events
                teleopTrigger.or(autoTrigger).onTrue(Commands.parallel(
                                m_ledSubsystem.startCommand(),
                                m_shoulderSubsystem.lockAngle(),
                                m_turretSubsystem.lockAngle()).withName("EnabledInitSequence"));

                // Configure the button bindings
                configureButtonBindings();

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
                                m_stoppyBarSubsystem.setStopCommand(true));
                m_driverController.start().onTrue(
                                m_stoppyBarSubsystem.setStopCommand(false));

                // drive bindings
                m_driverController.leftTrigger().whileTrue(m_driveSubsystem.runEnd(
                                () -> {
                                        if (m_driveSubsystem.getUltrasonicDistance() > 0.70) {
                                                m_driveSubsystem.arcadeDrive(
                                                                -m_driverController.getLeftY(),
                                                                (-m_driverController.getRightX() * 0.8));
                                        } else {
                                                m_driveSubsystem.tankDriveVolts(0, 0);
                                        }
                                },
                                () -> m_driveSubsystem.tankDriveVolts(0, 0)).withName("DriveWithDistBlock"));

                m_driverController.a().onTrue(m_driveSubsystem.setDriveProfileCmd(DriveProfiles.CoastNoRamp));
                m_driverController.b().onTrue(m_driveSubsystem.setDriveProfileCmd(DriveProfiles.BrakeNoRamp));
                m_driverController.y().onTrue(m_driveSubsystem.setDriveProfileCmd(DriveProfiles.CoastRamp));

                // m_driverController.leftBumper().whileTrue((new
                // Autobalance(m_driveSubsystem)));

                // Operator controller bindings
                m_operatorController.leftBumper()
                                .onTrue(m_turretSubsystem.incrementAngle(-3));
                m_operatorController.rightBumper()
                                .onTrue(m_turretSubsystem.incrementAngle(3));
                m_operatorController.leftTrigger()
                                .onTrue(m_armSubsystem.incrementLength(-1));
                m_operatorController.rightTrigger()
                                .onTrue(m_armSubsystem.incrementLength(1));
                m_operatorController.x().onTrue(m_clawSubsystem.grabCommand());
                m_operatorController.b().onTrue(m_clawSubsystem.releaseCommand());
                m_operatorController.pov(0)
                                .onTrue(m_shoulderSubsystem.incrementAngle(-2));
                m_operatorController.pov(180)
                                .onTrue(m_shoulderSubsystem.incrementAngle(2));
                m_operatorController.y().onTrue(m_clawSubsystem.setPickupModeCommand(PickupMode.Cone));
                m_operatorController.a().onTrue(m_clawSubsystem.setPickupModeCommand(PickupMode.Cube));

                // for toggle switch equipped f310, if switch code must be reverted!
                m_operatorController.leftStick()
                                .onTrue(m_armSubsystem.setArmLockCommand(true));
                m_operatorController.leftStick()
                                .onFalse(m_armSubsystem.setArmLockCommand(false));

                m_operatorController.rightStick()
                                .onTrue(m_driveSubsystem.runOnce(() -> m_driveSubsystem.resetGyro()));

                // Speed Overrides
                m_operatorController.start().and(m_operatorController.leftBumper())
                                .whileTrue(m_turretSubsystem.setSpeedCmd(-0.17));
                m_operatorController.start().and(m_operatorController.rightBumper())
                                .whileTrue(m_turretSubsystem.setSpeedCmd(0.17));
                m_operatorController.start().and(m_operatorController.leftTrigger())
                                .whileTrue(m_armSubsystem.setSpeedCmd(-0.4));
                m_operatorController.start().and(m_operatorController.rightTrigger())
                                .whileTrue(m_armSubsystem.setSpeedCmd(0.3));
                m_operatorController.start().and(m_operatorController.pov(0))
                                .whileTrue(m_shoulderSubsystem.setSpeedCmd(-0.3));
                m_operatorController.start().and(m_operatorController.pov(180))
                                .whileTrue(m_shoulderSubsystem.setSpeedCmd(0.2));

                // arcade pad
                m_arcadePad.L3().onTrue(m_ledSubsystem.startCommand());
                m_arcadePad.R3().onTrue(
                                m_ledSubsystem.stopCommand());

                // destructive!
                m_arcadePad.share().onTrue(Commands.parallel(
                                m_driveSubsystem.resetEncodersCommand(),
                                m_armSubsystem.resetEncoderCommand(),
                                m_shoulderSubsystem.resetEncoderCommand(),
                                m_turretSubsystem.resetEncoderCommand()).withName("ResetAllEncoders"));

                // destructive!
                m_arcadePad.options().onTrue(Commands.parallel(
                                m_driveSubsystem.disableMotorsCommand(),
                                m_armSubsystem.disableMotorCommand(),
                                m_shoulderSubsystem.disableMotorCommand(),
                                m_turretSubsystem.disableMotorCommand()).withName("DisableMotors"));

                // Smart bindings -->
                // Operator controller bindings

                // Turret directional
                new Trigger(() -> (m_arcadePad.getHID().getPOV() != -1))
                                .whileTrue(new TurretDynamicAngle(() -> m_arcadePad.getHID().getPOV(),
                                                m_driveSubsystem::getAngle, m_turretSubsystem)
                                                .withName("SetTurretPOV"));

                // Set shoulder and arm to HIGH GOAL
                m_arcadePad.x().whileTrue(Commands.either(Commands.parallel(
                                m_armSubsystem.moveToLocation(LocationType.HighCone),
                                m_shoulderSubsystem
                                                .moveToLocation(LocationType.HighCone)),
                                Commands.parallel(
                                                m_armSubsystem.moveToLocation(
                                                                LocationType.HighCube),
                                                m_shoulderSubsystem.moveToLocation(
                                                                LocationType.HighCube)),
                                () -> m_clawSubsystem.getPickupMode() == PickupMode.Cone)
                                .withName("HighGoal"));

                // Set shoulder and arm to LOW GOAL
                m_arcadePad.y().whileTrue(Commands.either(Commands.parallel(
                                m_armSubsystem.moveToLocation(LocationType.LowCone),
                                m_shoulderSubsystem
                                                .moveToLocation(LocationType.LowCone)),
                                Commands.parallel(
                                                m_armSubsystem.moveToLocation(
                                                                LocationType.LowCube),
                                                m_shoulderSubsystem.moveToLocation(
                                                                LocationType.LowCube)),
                                () -> m_clawSubsystem.getPickupMode() == PickupMode.Cone)
                                .withName("LowGoal"));

                // Set shoulder and arm to GROUND PICKUP/PLACE
                m_arcadePad.rightBumper().whileTrue(Commands.parallel(
                                m_shoulderSubsystem.moveToLocation(LocationType.Hybrid),
                                m_armSubsystem.moveToLocation(LocationType.Hybrid))
                                .withName("Ground"));

                // Set shoulder and arm to SUBSTATION PICKUP
                m_arcadePad.a().whileTrue(Commands.parallel(
                                m_shoulderSubsystem.moveToLocation(LocationType.SubstationHigh),
                                m_armSubsystem.moveToLocation(LocationType.SubstationHigh))
                                .withName("Substation"));

                // Set shoulder and arm to SUBSTATION PICKUP
                m_arcadePad.leftTrigger().whileTrue(Commands.parallel(
                                m_shoulderSubsystem.moveToLocation(LocationType.SubstationHigh),
                                m_armSubsystem.moveToLocation(LocationType.SubstationHigh))
                                .withName("Substation"));

                // Set shoulder and arm to full starting/finishing retraction
                m_arcadePad.b().whileTrue(Commands.sequence(
                                Commands.either(m_armSubsystem.moveToLocation(LocationType.RetractSlightOnlyArm),
                                                Commands.none(),
                                                () -> (m_armSubsystem
                                                                .getLength() > LocationType.RetractSlightOnlyArm.armInches)),
                                Commands.parallel(
                                                m_shoulderSubsystem.moveToLocation(LocationType.Starting),
                                                m_armSubsystem.moveToLocation(LocationType.Starting)),
                                m_turretSubsystem.absoluteReset())
                                .withName("ResetStarting"));

                m_arcadePad.leftBumper().whileTrue(Commands.parallel(
                                m_shoulderSubsystem.moveToLocation(LocationType.ChargeStation),
                                m_armSubsystem.moveToLocation(LocationType.ChargeStation))
                                .withName("ChargeStation"));

                m_operatorController.back()
                                .whileTrue(m_turretSubsystem.absoluteReset()
                                                .withName("TurretAbsReset"));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {

                // Get the PathPlanner key from the SendableChooser
                final Auto runAuto = m_autoChooser.getSelected();

                if (runAuto.runType == AutoType.Nothing) {
                        return null;
                }

                // load the Path group from the deploy pathplanner directory, with the default
                // constraints outlined in AutoConstants
                final List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(runAuto.pathName,
                                AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared);

                if (runAuto.runType == AutoType.StartEventOnly) {
                        return m_autoBuilder.stopEventGroup(pathGroup.get(0).getStartStopEvent());
                }

                return m_autoBuilder.fullAuto(pathGroup).andThen(() -> m_driveSubsystem.tankDriveVolts(0, 0),
                                m_driveSubsystem);

        }

}
