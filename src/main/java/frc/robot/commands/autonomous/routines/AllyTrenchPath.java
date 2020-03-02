package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.TimedIntake;
import frc.robot.commands.shooter.RapidFire;
import frc.robot.commands.shooter.SetAndHoldRpmSetpoint;
import frc.robot.commands.shooter.SetRpmSetpoint;
import frc.robot.commands.turret.SetTurretRobotRelativeAngle;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;

public class AllyTrenchPath extends SequentialCommandGroup {
    public AllyTrenchPath(DriveTrain driveTrain, Intake intake, Indexer indexer, Turret turret, Shooter shooter, Vision vision) {
        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(4), Units.feetToMeters(2));
        configA.setReversed(true);
        var startToTrenchPath = TrajectoryUtils.readCsvTrajectory("init1Ally2");
        var startToTrenchCommand = TrajectoryUtils.generateRamseteCommand(driveTrain, startToTrenchPath, configA);

        var configB = new TrajectoryConfig(Units.feetToMeters(4), Units.feetToMeters(2));
        configB.setReversed(false);
        var trenchToShootPath = TrajectoryUtils.readCsvTrajectory("ally2Ally3");
        var trenchToShootCommand = TrajectoryUtils.generateRamseteCommand(driveTrain, trenchToShootPath, configB);

        addCommands(
                new ParallelCommandGroup(
                        startToTrenchCommand,
                        new TimedIntake(intake, indexer, 4)
                ),
                new ParallelCommandGroup(
                        trenchToShootCommand,
                        new SetTurretRobotRelativeAngle(turret, 0),
                        new SetAndHoldRpmSetpoint(shooter, vision, 3500)
                ),
                new RapidFire(shooter, indexer, intake, 3500)
        );
    }
}
