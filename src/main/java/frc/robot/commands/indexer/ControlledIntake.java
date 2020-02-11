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

  private double intakeRPM = 400;
  private double startTime;
  private boolean intaking;

  private IntakeStates intakeState = IntakeStates.INTAKE_EMPTY;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
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
    if(m_indexer.getIntakeSensor() && m_indexer.getIndexerBottomSensor() && m_indexer.getIndexerTopSensor())
      intakeState = IntakeStates.INTAKE_FIVE_BALLS;
    else if(m_indexer.getIndexerBottomSensor() && m_indexer.getIndexerTopSensor())
      intakeState = IntakeStates.INTAKE_FOUR_BALLS;
    else if(m_indexer.getIndexerBottomSensor())
      intakeState = IntakeStates.INTAKE_ONE_BALL;
    else
      intakeState = IntakeStates.INTAKE_EMPTY;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("Intake State", intakeState.toString());

    switch (intakeState) {
      case INTAKE_FIVE_BALLS:
        m_intake.setRPM(0);
        m_indexer.setKickerOutput(0);
        m_indexer.setIndexerOutput(0);
        break;
      case INTAKE_FOUR_BALLS:
        m_intake.setRPM(intakeRPM);
        if (m_indexer.getIntakeSensor()) {
          startTime = Timer.getFPGATimestamp();
          intaking = true;
        }
        if(Timer.getFPGATimestamp() - startTime > 0.1) {
          intaking = false;
          intakeState = IntakeStates.INTAKE_FIVE_BALLS;
        }
        break;
      case INTAKE_ONE_BALL:
        m_intake.setRPM(intakeRPM);
        if (m_indexer.getIntakeSensor() && !intaking) {
          m_indexer.setRPM(intakeRPM);
          startTime = Timer.getFPGATimestamp();
          intaking = true;
        } else if (Timer.getFPGATimestamp() - startTime > 0.1) {
          if (m_indexer.getIndexerBottomSensor()) {
            m_indexer.setRPM(0);
            intaking = false;
          }
        }
        if(m_indexer.getIndexerTopSensor() && m_indexer.getIndexerBottomSensor())
          intakeState = IntakeStates.INTAKE_FOUR_BALLS;
        break;
      case INTAKE_EMPTY:
      default:
        m_intake.setRPM(intakeRPM);
        if (m_indexer.getIntakeSensor()) {
          m_indexer.setRPM(intakeRPM);
          intaking = true;
        } else if (m_indexer.getIndexerBottomSensor()) {
          m_indexer.setRPM(0);
          intaking = false;
          intakeState = IntakeStates.INTAKE_ONE_BALL;
        }
        break;
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakePercentOutput(0);
    m_indexer.setIndexerOutput(0);
    m_indexer.setKickerOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
