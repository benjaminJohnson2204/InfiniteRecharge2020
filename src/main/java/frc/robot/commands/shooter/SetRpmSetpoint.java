/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * An example command that uses an example subsystem.
 */
public class SetRpmSetpoint extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter m_shooter;
  private final Vision m_vision;
  private double m_RPM;
  /**
   * Creates a new ExampleCommand.
   *
   */
  public SetRpmSetpoint(Shooter shooter, Vision vision, double RPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_RPM = RPM;
    m_vision = vision;
//    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_vision.ledsOn();
    m_vision.setLastValidTargetTime();
    m_shooter.setRPM(m_RPM);
    m_vision.ledsOn();
    m_vision.setLastValidTargetTime();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setRPM(-1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (false);
  }
}
