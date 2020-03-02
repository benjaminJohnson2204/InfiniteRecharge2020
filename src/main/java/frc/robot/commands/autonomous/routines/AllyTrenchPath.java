package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomous.ReadTrajectory;
import frc.robot.commands.intake.SetIntakePiston;
import frc.robot.commands.intake.TimedIntake;
import frc.robot.commands.shooter.RapidFire;
import frc.robot.commands.turret.SetTurretRobotRelativeAngle;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;

public class AllyTrenchPath extends SequentialCommandGroup {
    public AllyTrenchPath(DriveTrain driveTrain, Shooter shooter, Indexer indexer, Intake intake, Turret turret) {
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(4), Units.feetToMeters(2));
        config.setReversed(true);
        var startToTrenchPath = TrajectoryUtils.readCsvTrajectory("init1Ally2");
        var startToTrenchCommand = TrajectoryUtils.generateRamseteCommand(driveTrain, startToTrenchPath, config);

        var config2 = config;
        config2.setReversed(false);
        var startToSootPath = TrajectoryUtils.readCsvTrajectory("ally2Ally3");
        var trenchToShootCommand = TrajectoryUtils.generateRamseteCommand(driveTrain, startToSootPath, config2);

        addCommands(
                new ParallelCommandGroup(
                        startToTrenchCommand,
                        new TimedIntake(intake, indexer, 4)
                ),
                new ParallelCommandGroup(
                        trenchToShootCommand,
                        new SetTurretRobotRelativeAngle(turret, 0)
                ),
                new RapidFire(shooter, indexer, intake, 3500)
        );
    }
}
