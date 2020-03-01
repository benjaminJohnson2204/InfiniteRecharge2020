/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.controlPanel;

import frc.robot.subsystems.ColorSensor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class PositionControl extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ColorSensor m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  double output;
  int throttle;
  public PositionControl(ColorSensor subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.working = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if(m_subsystem.panelColor() != 0)
      throttle = ((m_subsystem.getFMSColor() + 2) % 4 - m_subsystem.panelColor());
    else if(throttle == 0)
      throttle = 1;
    m_subsystem.setOutput(0.115 * throttle);*/
    m_subsystem.setOutput(0.085 * ((m_subsystem.getFMSColor() + 2) % 4 - m_subsystem.panelColor()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.working = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isColor = ((m_subsystem.getFMSColor() + 2) % 4 == m_subsystem.panelColor());
    if(isColor){
      Timer.delay(0.3);
      if(isColor){
        return true;
      }
    }
    return false;
  }
}
