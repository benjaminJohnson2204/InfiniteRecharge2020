package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.ReadTrajectoryOld;
import frc.robot.commands.drivetrain.ResetOdometry;
import frc.robot.commands.intake.TimedIntake;
import frc.robot.subsystems.*;

public class EnemyAutoPath extends SequentialCommandGroup {
    public EnemyAutoPath(DriveTrain driveTrain, Shooter shooter, Indexer indexer, Intake intake, Turret turret) {
        addCommands(
                new ParallelCommandGroup(
                        new ReadTrajectoryOld(driveTrain, "init4Enemy1", true),
                        new TimedIntake(intake, indexer, 6)
                ),
                new ResetOdometry(driveTrain),
                new ReadTrajectoryOld(driveTrain, "enemy1Shooting1")


        );
    }
}
