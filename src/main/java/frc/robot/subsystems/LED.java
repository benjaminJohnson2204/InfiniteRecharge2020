/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  AddressableLED LEDStrip;
  AddressableLEDBuffer LEDBuffer;
  int start = (int)Timer.getFPGATimestamp() * 5;

  private int red, green, blue;

  public LED() {
    LEDStrip = new AddressableLED(0);
    LEDBuffer = new AddressableLEDBuffer(60);
    LEDStrip.setLength(LEDBuffer.getLength());
    LEDStrip.setData(LEDBuffer);
    LEDStrip.start();
    
    SmartDashboard.putNumber("Rainbows", rainbows);
    SmartDashboard.putNumber("Speed", speed);
  }

  public void setRGB(int red, int green, int blue){
    this.red = red;
    this.blue = blue;
    this.green = green;
  };

  public void setSolidColor(int red, int green, int blue){
    for(int i = 0; i < LEDBuffer.getLength(); i++){
      LEDBuffer.setRGB(i, red, green, blue);
    }
  }

  public void resetLED(){
    for(int i = 0; i < LEDBuffer.getLength(); i++){
      LEDBuffer.setRGB(i, 0, 0, 0);
    }
  }

  public void setBlinkingColor(boolean blinkType, int red, int green, int blue){
    double time = (int)(5 * Timer.getFPGATimestamp());
    if(!blinkType){
      if(time / 2 == Math.floor(time / 2)){
        for(int i = 0; i < LEDBuffer.getLength(); i++){
          LEDBuffer.setRGB(i, red, green, blue);
        }
      }
      else
        resetLED();
    }
    else {
      if(time / 2 == Math.floor(time / 2)){
        resetLED();
        for(int i = 0; i < LEDBuffer.getLength(); i += 2){
          LEDBuffer.setRGB(i, red, green, blue);
        }
      }
      else {
        resetLED();
        for(int i = 1; i < LEDBuffer.getLength(); i += 2){
          LEDBuffer.setRGB(i, red, green, blue);
        }
      }
    }
  }

  double hueOffset = 0;
  public void setRainbow(double iterations, double speed){
    for(int i = 0; i < LEDBuffer.getLength(); i++){
      LEDBuffer.setHSV(i, (int)(180 * iterations * i / LEDBuffer.getLength() + hueOffset) % 180, 255, 255);
    }
    hueOffset = (hueOffset + 3 * speed * iterations) % 180;
    Timer.delay(0.05);
  }

  int head = 0;
  public void coolDesign(int interval, int trail, int red, int green, int blue){
    resetLED();
    for(int i = head; i < LEDBuffer.getLength(); i += (interval + trail)){
      LEDBuffer.setRGB(i, red, green, blue);
      /*for(int ii = trail; ii > 0; ii--){
        double mag = ii / (trail + 1);
        LEDBuffer.setRGB(i - (trail - ii + 1), (int)(mag * red), (int)(mag * green), (int)(mag * blue));
      }*/
    }
    
    head = (head + 1) % (interval + trail);
    Timer.delay(0.03);
  }

  double rainbows = 3;
  double speed = 8;
  boolean uptakeSensorTripped = false;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setRainbow(rainbows, speed);
    rainbows = SmartDashboard.getNumber("Rainbows", 0);
    speed = SmartDashboard.getNumber("Speed", 0);
    LEDStrip.setData(LEDBuffer);
  }
}
