/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.LED.WhiteFlash;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class IndexerCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Indexer m_indexer;
  private final LED m_led;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  int tripped = -1;
  double setpoint, startTime;
  public IndexerCommand(Indexer indexer, LED led) {
    m_indexer = indexer;
    m_led = led;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint = m_indexer.getPosition() * 7 / (1.25 * Math.PI) * 20;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_indexer.sensorTripped() && tripped == -1){
      tripped = 1;
      startTime = Timer.getFPGATimestamp();
    }
    else if(!m_indexer.sensorTripped())
      tripped = -1;
    if(Timer.getFPGATimestamp() - startTime > 0.1 && tripped == 1) {
      CommandScheduler.getInstance().schedule(new IncrementIndexer(m_indexer));
      CommandScheduler.getInstance().schedule(new WhiteFlash(m_led));
      tripped = 0;
    }
    if(m_indexer.indexerFull()){
      m_led.setRGB(255, 0, 0);
      m_led.setBlinkingColor(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
