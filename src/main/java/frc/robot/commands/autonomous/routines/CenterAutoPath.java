package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.ReadTrajectoryOld;
import frc.robot.commands.intake.TimedIntake;
import frc.robot.subsystems.*;

public class CenterAutoPath extends SequentialCommandGroup {
    public CenterAutoPath(DriveTrain driveTrain, Shooter shooter, Indexer indexer, Intake intake, Turret turret) {
        addCommands(
                //start 12 feet from side of field with enemy trench
                new ParallelCommandGroup(
                        new ReadTrajectoryOld(driveTrain, "init3Ren45", true),
                        new TimedIntake(intake, indexer, 6)
                ),
                new ReadTrajectoryOld(driveTrain, "ren45Pivot"),
                new ParallelCommandGroup(
                        new ReadTrajectoryOld(driveTrain, "pivotRen3", true),
                        new TimedIntake(intake, indexer, 4.5)
                )

        );
    }
}
