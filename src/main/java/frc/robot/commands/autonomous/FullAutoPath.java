package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class FullAutoPath extends SequentialCommandGroup {
    public FullAutoPath(DriveTrain driveTrain, Shooter shooter, Indexer indexer, Intake intake) {
        addCommands(
                    new AutoShoot(shooter, indexer, intake, 2),
                    new ReadTrajectory(driveTrain, "enemyTrench", true)

        );
    }
}
