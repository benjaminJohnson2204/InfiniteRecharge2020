/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LED;

/**
 * An example command that uses an example subsystem.
 */
public class GetSubsystemStates extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LED m_led;
  private final Indexer m_indexer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param The subsystem used by this command.
   */
  public GetSubsystemStates(LED led, Indexer indexer) {
    m_led = led;
    m_indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.    
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.setRGB(0, 125, 0);
    m_led.setSolidColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_led.setRGB(75, 20, 150);
    m_led.setSolidColor();
    if(/*climb mode enabled*/false){
      m_led.setState(0);
    }
    else if(/*Turret is aligning to target*/false || /*Flywheel is spinning, but is not at the required velocity*/false){
      m_led.setState(1);
    }
    else if(m_indexer.topSensor()) {
      m_led.setState(2);
    }
    else if(m_indexer.newBall()) {
      m_led.setState(3);
    }
    else
      m_led.setState(-1);
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
  public boolean runsWhenDisabled(){
    return true;
  }
}
