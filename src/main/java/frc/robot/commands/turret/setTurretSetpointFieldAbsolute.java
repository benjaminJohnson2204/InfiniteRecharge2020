/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/**
 * An example command that uses an example subsystem.
 */
public class setTurretSetpointFieldAbsolute extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Turret m_turret;
  private final DriveTrain m_driveTrain;
  private final Vision m_vision;
  double setpoint;
  private final double deadZone = 0.1;
  private Timer timer = new Timer();
  boolean timeout = false;
  boolean limelightDisabled = false;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public setTurretSetpointFieldAbsolute(Turret subsystem, DriveTrain driveTrainSubsystem, Vision visionSybsystem) {
    m_turret = subsystem;
    m_driveTrain = driveTrainSubsystem;
    m_vision = visionSybsystem;
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
    if(RobotContainer.getLeftJoystickX() >= deadZone || RobotContainer.getLeftJoystickY() >= deadZone) {
      setpoint = Math.toDegrees(Math.tan(RobotContainer.getLeftJoystickY() / RobotContainer.getLeftJoystickX())) - (90 + m_driveTrain.getAngle());
      limelightDisabled = false; //enable limelight control
    } else if (!limelightDisabled){
      if(Constants.limelightTempDisabled){
        if(m_turret.atTarget()&&Constants.canSeeVisionTarget){
          Constants.limelightTempDisabled = false;
        }
      } else if (Constants.canSeeVisionTarget){ //if you can see the target, set setpoint to vision target's angle and reset timer if activated.
        setpoint = m_turret.getAngle() + m_vision.getTargetX();
        if(timeout) {
          timer.stop();
          timer.reset();
        }
      } else { //if you can't see the target for 1 second, then disable the limelight
        timer.start();
        timeout = true;
        if(timer.get()>1) {
          timer.stop();
          timer.reset();
          limelightDisabled = true;
        }
      }
    }
    m_turret.setSetpoint(setpoint);
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
