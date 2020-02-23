/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
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

  private boolean curretDirection, movable;
  private double timestamp;
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
    double input = Math.abs(m_input.getAsDouble()) > 0.2 ? m_input.getAsDouble() : 0;
    boolean direction = input > 0;
//    if(m_climber.getClimbState()) {

//      if(direction != curretDirection) {
//        timestamp = Timer.getFPGATimestamp();
//        movable = false;
//        if (direction)
//          climberReleaseSequence();
//        else
//          climberRetractSequence();
//      }

//      if(movable)
        m_climber.setClimberOutput(input);
//    }
  }

  private void climberReleaseSequence() {
    m_climber.setClimbPiston(false);

    if(Math.abs(Timer.getFPGATimestamp() - timestamp) < 0.2)
      m_climber.setClimberOutput(-0.25);
    else if(Math.abs(Timer.getFPGATimestamp() - timestamp) < 0.4)
      m_climber.setClimberOutput(0.25);
    else {
      m_climber.setClimberOutput(0);
      movable = true;
      curretDirection = true;
    }
  }

  private void climberRetractSequence() {
    m_climber.setClimbPiston(true);
    if(Math.abs(Timer.getFPGATimestamp() - timestamp) < 0.2)
      m_climber.setClimberOutput(-0.25);
    else {
      m_climber.setClimberOutput(0);
      movable = true;
      curretDirection = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setClimberOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
