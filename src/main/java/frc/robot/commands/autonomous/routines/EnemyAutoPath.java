package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class EnemyAutoPath extends SequentialCommandGroup {
    public EnemyAutoPath(DriveTrain driveTrain, Shooter shooter, Indexer indexer, Intake intake, Turret turret) {
        addCommands(
//                new ParallelCommandGroup(
//                        new ReadTrajectory(driveTrain, "init4Enemy1", true),
//                        new TimedIntake(intake, indexer, 6)
//                ),
//                new ResetOdometry(driveTrain),
//                new ReadTrajectory(driveTrain, "enemy1Shooting1")


        );
    }
}
