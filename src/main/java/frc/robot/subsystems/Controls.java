/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import badlog.lib.BadLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Controls extends SubsystemBase {
  private PowerDistributionPanel pdp = new PowerDistributionPanel(0);
  private DriveTrain m_driveTrain;
  private Shooter m_shooter;
  private boolean init = false;
  /**
   * Creates a new ExampleSubsystem.
   */
  public Controls(DriveTrain driveTrain, Shooter shooter) {
    m_driveTrain = driveTrain;
    m_shooter = shooter;

  }

  private void initLogging() {
//    BadLog.createTopic("DriveTrain/Left Front Input Current", "A",
//            () -> m_driveTrain.getMotorInputCurrent(0),
//            "hide", "join:DriveTrain/Input Currents");
//    BadLog.createTopic("DriveTrain/Left Rear Input Current", "A",
//            () -> m_driveTrain.getMotorInputCurrent(1),
//            "hide", "join:DriveTrain/Input Currents");
//    BadLog.createTopic("DriveTrain/Right Front Input Current", "A",
//            () -> m_driveTrain.getMotorInputCurrent(2),
//            "hide", "join:DriveTrain/Input Currents");
//    BadLog.createTopic("DriveTrain/Right Front Input Current", "A",
//            () -> m_driveTrain.getMotorInputCurrent(3),
//            "hide", "join:DriveTrain/Input Currents");
//    BadLog.createTopic("Shooter/Left Input Current", "A",1
//            () -> m_shooter.getMotorInputCurrent(0),
//            "hide", "join:Shooter/Input Currents");
//    BadLog.createTopic("Shooter/Right Input Current", "A",
//            () -> m_shooter.getMotorInputCurrent(1),
//            "hide", "join:Shooter/Input Currents");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(!init && (m_shooter != null) && (m_driveTrain != null)) {
      initLogging();
      init = true;
    }
  }
}
