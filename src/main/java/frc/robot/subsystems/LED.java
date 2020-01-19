/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  AddressableLED LEDStrip;
  AddressableLEDBuffer LEDBuffer;
  public LED() {
    LEDStrip = new AddressableLED(0);
    LEDBuffer = new AddressableLEDBuffer(5);
    LEDStrip.setLength(LEDBuffer.getLength());
    LEDStrip.setData(LEDBuffer);
    LEDStrip.start();
    setSolidColor(75, 20, 160);
  }

  public void setSolidColor(int red, int green, int blue){
    for(int i = 0; i < LEDBuffer.getLength(); i++){
      LEDBuffer.setRGB(i, red, green, blue);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
