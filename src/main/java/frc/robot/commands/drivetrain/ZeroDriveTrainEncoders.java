/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/**
 * An example command that uses an example subsystem.
 */
public class ZeroDriveTrainEncoders extends CommandBase {
  //@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_driveTrain;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ZeroDriveTrainEncoders(DriveTrain subsystem) {
    m_driveTrain = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetEncoderCounts();
    m_driveTrain.resetOdometry(new Pose2d(), new Rotation2d());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
