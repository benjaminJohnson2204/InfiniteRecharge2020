package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.ResetOdometry;
import frc.robot.commands.drivetrain.SetDriveShifters;
import frc.robot.commands.intake.ControlledIntake;
import frc.robot.commands.intake.SetIntakePiston;
import frc.robot.commands.shooter.AutoRapidFireSetpoint;
import frc.robot.commands.shooter.SetAndHoldRpmSetpoint;
import frc.robot.commands.turret.AutoUseVisionCorrection;
import frc.robot.commands.turret.SetTurretRobotRelativeAngle;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;

public class AllyTrenchPathSpline extends SequentialCommandGroup {
    public AllyTrenchPathSpline(DriveTrain driveTrain, Intake intake, Indexer indexer, Turret turret, Shooter shooter, Vision vision) {
        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(20));
        configA.setReversed(true);
        configA.setEndVelocity(0);
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), Units.feetToMeters(12)));
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),11));
        var startToTrenchPath = TrajectoryUtils.readCsvTrajectory("init1Ally2");
        var startToTrenchCommand = TrajectoryUtils.generateRamseteCommand(driveTrain, startToTrenchPath, configA);

        var configB = new TrajectoryConfig(Units.feetToMeters(8), Units.feetToMeters(4));
        configB.setReversed(false);
        configB.setEndVelocity(0);
        configB.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), Units.feetToMeters(8)));
        configB.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),11));
        var trenchToShootPath = TrajectoryUtils.readCsvTrajectory("ally2Ally3");
        var trenchToShootCommand = TrajectoryUtils.generateRamseteCommand(driveTrain, trenchToShootPath, configB);

        addCommands(
                new ResetOdometry(driveTrain),
                new SetDriveShifters(driveTrain, false),
                new SetAndHoldRpmSetpoint(shooter, vision, 3500),
                new SetTurretRobotRelativeAngle(turret, -25),
                new AutoUseVisionCorrection(turret, vision),
                new AutoRapidFireSetpoint(shooter, indexer, intake, 1),
                new SetIntakePiston(intake, true),
                new SetDriveShifters(driveTrain, false),
                new ParallelDeadlineGroup(
                        startToTrenchCommand,
                        new ControlledIntake(intake, indexer)
                ),
                new SetIntakePiston(intake, false),
                new ParallelDeadlineGroup(
                        trenchToShootCommand,
                        new SetTurretRobotRelativeAngle(turret, 0),
                        new SetAndHoldRpmSetpoint(shooter, vision, 3600)
                ).andThen(()->driveTrain.setMotorTankDrive(0,0)),
                new AutoUseVisionCorrection(turret, vision),
                new WaitCommand(0.5),
                new ConditionalCommand(new AutoRapidFireSetpoint(shooter, indexer, intake, 6),
                                       new WaitCommand(0),
                                       () -> vision.getValidTarget())
        );
    }
}

