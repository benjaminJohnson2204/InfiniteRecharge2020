/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import badlog.lib.BadLog;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.vitruvianlib.I2C.I2CLCD;

public class Controls extends SubsystemBase {
  private PowerDistributionPanel pdp = new PowerDistributionPanel(0);
  private DriveTrain m_driveTrain;
  private Shooter m_shooter;
  private Turret m_turret;
  private boolean init = false;
  /**
   * Creates a new ExampleSubsystem.
   */
  I2CLCD LCDDisplay = new I2CLCD(I2C.Port.kOnboard, 0x27);

  AnalogInput PressureSensor = new AnalogInput(0);

  public Controls(DriveTrain driveTrain, Shooter shooter, Turret turret) {
    m_driveTrain = driveTrain;
    m_shooter = shooter;
    m_turret = turret;

    LCDDisplay.init();
  }

  public void initLogging() {
    BadLog.createTopic("DriveTrain/Left Front Input Current", "A",
            () -> m_driveTrain.getMotorInputCurrent(0),
            "hide", "join:DriveTrain/Input Currents");
    BadLog.createTopic("DriveTrain/Left Rear Input Current", "A",
            () -> m_driveTrain.getMotorInputCurrent(1),
            "hide", "join:DriveTrain/Input Currents");
    BadLog.createTopic("DriveTrain/Right Front Input Current", "A",
            () -> m_driveTrain.getMotorInputCurrent(2),
            "hide", "join:DriveTrain/Input Currents");
    BadLog.createTopic("DriveTrain/Right Rear Input Current", "A",
            () -> m_driveTrain.getMotorInputCurrent(3),
            "hide", "join:DriveTrain/Input Currents");
    BadLog.createTopic("Shooter/Left Input Current", "A",
            () -> m_shooter.getMotorInputCurrent(0),
            "hide", "join:Shooter/Input Currents");
    BadLog.createTopic("Shooter/Right Input Current", "A",
            () -> m_shooter.getMotorInputCurrent(1),
            "hide", "join:Shooter/Input Currents");
  }

  public double getPressure (){
    return PressureSensor.getAverageVoltage() * (200.0/5.0);
  }

  public String isPressureGood (){
    if(getPressure()>40){
      return "Closed";
    }else{
      return "Open";
    }
  }

  public String isPoseGood (){
    if(Math.abs(m_driveTrain.getRobotPose().getTranslation().getX()) < 1 &&
            Math.abs(m_driveTrain.getHeading().getDegrees()) < 1){
      return "Good";
    }else{
      return "Not Good";
    }
  }

  private void updateSmartDashboard() {
    SmartDashboardTab.putString("LCDDisplay","turret angle",
            "Angle:" + Math.floor(m_turret.getTurretAngle() * 10) / 10);
    SmartDashboardTab.putString("LCDDisplay", "Pressure",
            Math.floor(getPressure() * 10) / 10 + " "+ isPressureGood());
    SmartDashboardTab.putString("LCDDisplay", "Odometry", "Odometry:" + isPoseGood());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: Turn off LCD backlight if robot is enabled
    if(DriverStation.getInstance().isDisabled()) {
      LCDDisplay.display_string("Angle:" + Math.floor(m_turret.getTurretAngle() * 10) / 10, 1);
      //angle of the robot's turret in degrees. returned as exp. "angle:169.8"
      // the angle value should be maximum 6 characters including the decimal point and maybe a negitive sign
      LCDDisplay.display_string("PSI:" + Math.floor(getPressure() * 10) / 10 + " " + isPressureGood(), 2);
      //there should be a reserved space for a pressure value measured in psi
      //exp: "psi:128" there should be no decimal point as it will be measured as a whole number
      LCDDisplay.display_string("Odometry:" + isPoseGood(), 3);
      //if both the position of the robot and the angle it's facing are 0 then it will return "good". alse it will return
      //bad, the function allows the error to be within 1 unit of 0
      LCDDisplay.display_string("Purple is best soda", 4);
      //silly stuff
      updateSmartDashboard();
    }
  }
}
