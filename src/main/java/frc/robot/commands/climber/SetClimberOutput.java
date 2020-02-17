/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class SetClimberOutput extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_climber;
  private DoubleSupplier m_input;

  /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetClimberOutput(Climber climber, DoubleSupplier input) {
    m_climber = climber;
    m_input = input;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  @Override
  public void execute() {
    if(m_climber.getClimbState())
      m_climber.setClimber(m_input.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setClimber(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
