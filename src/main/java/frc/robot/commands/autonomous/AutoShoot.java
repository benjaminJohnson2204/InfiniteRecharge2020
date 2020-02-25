package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.shooter.RapidFire;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends SequentialCommandGroup {

    Timer autoShootTimer = new Timer();
    double m_time;
    public AutoShoot(Shooter shooter, Indexer indexer, Intake intake, double time) {
        m_time = time;
        addCommands(
            new RapidFire(shooter, indexer, intake, 3500)
        );
    }
    @Override
    public boolean isFinished() {
        return autoShootTimer.getFPGATimestamp()>m_time;
    }

}
