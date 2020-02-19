/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class InvertDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_driveTrain;
  private final DoubleSupplier m_throttle, m_turn;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public InvertDrive(DriveTrain subsystem, DoubleSupplier throttle, DoubleSupplier turn) {
    m_driveTrain = subsystem;
    m_throttle = throttle;
    m_turn = turn;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_driveTrain.setMotorArcadeDrive(-m_throttle.getAsDouble(), m_turn.getAsDouble());
//    if (Robot.climber.climbMode == 1) {
//      double operatorThrottle = Math.abs(Robot.m_oi.getXBoxRightY()) > 0.05 ? Robot.m_oi.getXBoxRightY() : 0;
//      Robot.driveTrain.setClimbMotorPercentOutput(Math.min(throttle + operatorThrottle, 0.5));
////            Robot.driveTrain.setClimbMotorCurrentOutput(30 * Math.min(throttle + operatorThrottle, 0.5));
//      throttle = Math.max(Math.min(throttle, 0.25), -0.25);
//      turn = Math.max(Math.min(turn, 0.4), -0.4);
//      Robot.driveTrain.setMotorArcadeDrive(throttle, turn);
//    } else {
//      if (Robot.elevator.controlMode == 1)
//        throttle = Robot.elevator.getHeight() > 30 ? Math.min(Math.max(throttle, -0.4), 0.5) : throttle;
//      Robot.driveTrain.setMotorArcadeDrive(throttle, turn);
//    }
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
}
