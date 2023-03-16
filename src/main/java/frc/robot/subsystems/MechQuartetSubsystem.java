// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.QuartetConstants.LocationType;

/** Add your docs here. */
public class MechQuartetSubsystem extends SubsystemBase {
    private final DoubleSupplier m_armLength;
    private final DoubleSupplier m_shoulderAngle;

    private static final double kArmBaseLength = 20; // minus part on other side
    private static final double kHeight = 54;
    private static final double kWidth = 110;

    private static final double kBumperWidth = 33.5;
    private static final double kBumperHeight = 5;

    private final Mechanism2d m_mechanism = new Mechanism2d(kWidth, kHeight);

    private final MechanismRoot2d m_shoulderPoint = m_mechanism.getRoot("Quartet", 18.25, 31); // y=31
    private final MechanismLigament2d m_arm = m_shoulderPoint
            .append(new MechanismLigament2d("Arm", kArmBaseLength, LocationType.Starting.shoulderDegrees, 6,
                    new Color8Bit(Color.kLightGreen)));

    public MechQuartetSubsystem(DoubleSupplier armLength, DoubleSupplier shoulderAngle) {
        m_armLength = armLength;
        m_shoulderAngle = shoulderAngle;

        SmartDashboard.putData("Arm2d", m_mechanism);

        setName("Arm2dSubsystem");

        if (RobotBase.isSimulation()) {
            SmartDashboard.putNumber("SHOULDER ENC.", 0);
            SmartDashboard.putNumber("ARM ENC.", 0);
        }

        Color8Bit bumperColor;
        switch (DriverStation.getAlliance()) {
            case Blue:
                bumperColor = new Color8Bit(Color.kBlue);
                break;
            case Red:
                bumperColor = new Color8Bit(Color.kRed);
                break;
            case Invalid:
            default:
                bumperColor = new Color8Bit(Color.kHotPink);
                break;
        }

        m_mechanism.getRoot("Bumper MidpointH", 5, 6.625)
                .append(new MechanismLigament2d("Bumper T", kBumperWidth, 0.0, 6, bumperColor))
                .append(new MechanismLigament2d("Bumper R", kBumperHeight, -90, 6, bumperColor))
                .append(new MechanismLigament2d("Bumper B", kBumperWidth, -90, 6, bumperColor))
                .append(new MechanismLigament2d("Bumper L", kBumperHeight, -90, 6, bumperColor));
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            m_arm.setLength(kArmBaseLength + SmartDashboard.getNumber("ARM ENC.", 0));
            m_arm.setAngle(-SmartDashboard.getNumber("SHOULDER ENC.", 0));
        } else {
            m_arm.setLength(kArmBaseLength + m_armLength.getAsDouble());
            m_arm.setAngle(m_shoulderAngle.getAsDouble());
        }
    }
}
