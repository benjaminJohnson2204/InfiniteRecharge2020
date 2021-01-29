/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/**
 * An example command that uses an example subsystem.
 */
public class AccuracyChallengeShoot extends CommandBase { // Used in the Accuracy Challenge to set the shooter to the right speed to hit the target
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final Indexer m_indexer;
    private final Intake m_intake;
    private final Vision m_vision;
    private double targetDistance, shootSpeed, shootRPM;

    private final double metersPerSecondToRPM = 315; // How much to multiply a speed by to get RPM. This is only a preliminary estimate

    /**
     * Creates a new ExampleCommand.
     *
     * @param RobotContainer.m_shooter The subsystem used by this command.
     */
    public AccuracyChallengeShoot(Shooter shooter, Turret turret, Indexer indexer, Intake intake, Vision vision) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_shooter = shooter;
        m_turret = turret;
        m_indexer = indexer;
        m_intake = intake;
        m_vision = vision;
        addRequirements(shooter);
        addRequirements(indexer);
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        targetDistance = Units.feetToMeters(m_vision.getTargetDistance());
        shootSpeed = targetDistance * Math.sqrt(0.5 * Constants.g / (targetDistance * Math.tan(Constants.verticalShooterAngle) - Constants.verticalTargetDistance)) / Math.cos(Constants.verticalShooterAngle);
        shootRPM = shootSpeed * metersPerSecondToRPM;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_shooter.setRPM(shootRPM);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
