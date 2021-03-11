/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class AutoAvoidCones extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain m_driveTrain;
    private final DoubleSupplier m_leftOutput, m_rightOutput;
    private double currentLeftOutput, currentRightOutput;
    private final Translation2d[] m_conePoses;

    private final double robotLength = Units.inchesToMeters(38);
    private final double robotWidth = Units.inchesToMeters(33);
    private final double deltaT = 0.02;
    private final double coneRadius = Units.inchesToMeters(6);
    private final double tolerance = Units.inchesToMeters(6);

    public AutoAvoidCones(DriveTrain driveTrain, DoubleSupplier leftOutput, DoubleSupplier rightOutput, Translation2d[] conePoses) {
        m_driveTrain = driveTrain;
        m_leftOutput = leftOutput;
        m_rightOutput = rightOutput;
        m_conePoses = conePoses;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        currentLeftOutput = m_leftOutput.getAsDouble();
        currentRightOutput = m_rightOutput.getAsDouble();
        if (aboutToHitCone()) {
            m_driveTrain.setMotorTankDrive(0, 0);
        } else {
            m_driveTrain.setMotorTankDrive(currentLeftOutput, currentRightOutput);
        }
    }

    private boolean aboutToHitCone() {
        Pose2d robotPose = m_driveTrain.getRobotPose();
        
        for (Translation2d cone : m_conePoses) {
            if (cone.getDistance(robotPose.getTranslation()) < robotLength / 2 + tolerance + coneRadius) {
                SmartDashboardTab.putBoolean("DriveTrain", "Will hit Cone", true);
                return true;
            }
        }
        SmartDashboardTab.putBoolean("DriveTrain", "Will hit Cone", false);
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
