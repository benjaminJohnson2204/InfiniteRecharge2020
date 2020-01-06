/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.tables.IRemote;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  public ColorSensorV3 sensor = new ColorSensorV3(I2C.Port.kOnboard);
  
  public ColorSensor() {

  }

  public Color getColor() {
    return sensor.getColor();
  }

  public double getIR() {
    return sensor.getIR();
  }

  public int getProximity() {
    return sensor.getProximity();
  }

  public int panelColor(){ // none = 0; red = 1; green = 2; blue = 3; yellow = 4
    if(getColor().red > getColor().blue * 3 && getColor().red > getColor().green * 1.33){
      return 1;
    }
    else if(getColor().green > getColor().red * 2.75 && getColor().green > getColor().blue * 1.8){
      return 2;
    }
    else if(getColor().blue < getColor().green * 1.15 && getColor().green < getColor().blue * 1.15 && getColor().blue > getColor().red * 2.5){
      return 3;
    }
    else if(getColor().green < getColor().red * 1.8 && getColor().green > getColor().red * 1.65){
      return 4;
    }
    else {
      return 0;
    }
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Red", getColor().red);
    SmartDashboard.putNumber("Green", getColor().green);
    SmartDashboard.putNumber("Blue", getColor().blue);
    SmartDashboard.putNumber("IR", getIR());
    SmartDashboard.putNumber("Proximity", getProximity());
    SmartDashboard.putNumber("Panel Color", panelColor());
    //SmartDashboard.putString("Color", getColorString());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
