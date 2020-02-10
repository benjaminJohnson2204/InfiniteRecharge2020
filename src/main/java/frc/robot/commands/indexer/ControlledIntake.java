/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.indexer;

import frc.robot.constants.Enums.IntakeStates;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class ControlledIntake extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Indexer m_indexer;
  private final Intake m_intake;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  double m_setpoint;
  private double startTime;
  private IntakeStates intakeState = IntakeStates.INTAKE_EMPTY;
  boolean setpointCommand, timerLatch;
  public ControlledIntake(Indexer indexer, Intake intake) {
    m_indexer = indexer;
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeState = IntakeStates.INTAKE_EMPTY;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_indexer.topSensor() && m_indexer.secondSensor() && m_indexer.sensorTripped())
      intakeState = IntakeStates.INTAKE_FIVE_BALLS;
    else if(m_indexer.topSensor())
      intakeState = IntakeStates.INTAKE_FOUR_BALLS;
    else if(m_indexer.secondSensor())
      intakeState = IntakeStates.INTAKE_ONE_BALL;

    SmartDashboard.putString("Intake State", intakeState.toString());
    
    if(intakeState != IntakeStates.INTAKE_FIVE_BALLS){
      m_intake.setIntakePercentOutput(0.75);
      m_indexer.setKickerOutput(-0.2);
    }
    if(!setpointCommand){
      if(intakeState == IntakeStates.INTAKE_EMPTY){
        if(m_indexer.sensorTripped()){
          m_setpoint = m_indexer.getPosition() + 12 / (1.25 * Math.PI) * 20;
          m_indexer.incrementIndexer(m_setpoint);
          setpointCommand = true;
        }
      }
      else if(intakeState == IntakeStates.INTAKE_ONE_BALL){
        if(m_indexer.sensorTripped()){
          if(!timerLatch) {
            startTime = Timer.getFPGATimestamp();
            timerLatch = true;
          } else if(timerLatch && Timer.getFPGATimestamp() - startTime > 0.1) {
            m_setpoint = m_indexer.getPosition() + 4 / (1.25 * Math.PI) * 20;
            m_indexer.incrementIndexer(m_setpoint);
            setpointCommand = true;
            timerLatch = false;
          }
        }
      }
      else if(intakeState == IntakeStates.INTAKE_FOUR_BALLS){
        if(m_indexer.sensorTripped()){
          m_setpoint = m_indexer.getPosition() + 1 / (1.25 * Math.PI) * 20;
          m_indexer.incrementIndexer(m_setpoint);
          setpointCommand = true;
        }
      }
    }
    if(m_indexer.onTarget() )
      setpointCommand = false;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_indexer.setKickerOutput(0);
    m_intake.setIntakePercentOutput(0);
    SmartDashboard.putNumber("Execution Time", Timer.getFPGATimestamp() - startTime);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
