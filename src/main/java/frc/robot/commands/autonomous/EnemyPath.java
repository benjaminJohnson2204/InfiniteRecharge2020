package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;

public class EnemyPath extends SequentialCommandGroup {
    public EnemyPath(DriveTrain m_driveTrain) {
        addCommands(
                    new ReadTrajectory(m_driveTrain, "enemyTrench", true),
                    new WaitCommand(2),
                    new ReadTrajectory(m_driveTrain, "turnShoot")
        );
    }
}
