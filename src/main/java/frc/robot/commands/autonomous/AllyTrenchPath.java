package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.TimedIntake;
import frc.robot.commands.shooter.RapidFire;
import frc.robot.commands.turret.SetTurretRobotRelativeAngle;
import frc.robot.subsystems.*;

public class AllyTrenchPath extends SequentialCommandGroup {
    public AllyTrenchPath(DriveTrain driveTrain, Shooter shooter, Indexer indexer, Intake intake, Turret turret) {
        addCommands(
                    new ParallelCommandGroup(
                            new ReadTrajectory(driveTrain, "init1Ally2", true),
                            new SequentialCommandGroup(
                                    new WaitCommand(1),
                                    new TimedIntake(intake, indexer, 6)
                            )
                    ),
                    new WaitCommand(3),
                    new ParallelCommandGroup(
                            new ReadTrajectory(driveTrain, "ally2Ally3", true),
                            new TimedIntake(intake, indexer, 2)
                    )
//                    new SetTurretRobotRelativeAngle(turret, 45),
//                    new RapidFire(shooter, indexer, intake, 3500)

        );
    }
}
