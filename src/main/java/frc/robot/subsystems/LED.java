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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;

public class LED extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private AddressableLED LEDStrip;
  private AddressableLEDBuffer LEDBuffer;
  int start = (int)Timer.getFPGATimestamp() * 5;

  private int red, green, blue;
  double rainbows = 3;
  double speed = 8;

  public LED() {
    LEDStrip = new AddressableLED(Constants.ledPortA);
//    LEDStripB = new AddressableLED(Constants.ledPortB);
    LEDBuffer = new AddressableLEDBuffer(50 * 2);
    LEDStrip.setLength(LEDBuffer.getLength());
    LEDStrip.setData(LEDBuffer);
    LEDStrip.start();
//    LEDStripB.setLength(LEDBuffer.getLength());
//    LEDStripB.setData(LEDBuffer);
//    LEDStripB.start();

    red = 0;
    green = 125;
    blue = 0;
    setSolidColor();
//    SmartDashboard.putNumber("Rainbows", rainbows);
//    SmartDashboard.putNumber("Speed", speed);
  }

  public void setRGB(int red, int green, int blue){
    this.red = (int) (red * 0.5);
    this.blue = (int) (blue * 0.5);
    this.green = (int) (green * 0.5);
  }

  public void setSolidColor(){
    for(int i = 0; i < LEDBuffer.getLength(); i++){
      LEDBuffer.setRGB(i, red, green, blue);
    }
  }

  public void resetLED(){
    for(int i = 0; i < LEDBuffer.getLength(); i++){
      LEDBuffer.setRGB(i, 0, 0, 0);
    }
  }

  public void setBlinkingColor(boolean blinkType){
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
      LEDBuffer.setHSV(i, (int)(180 * iterations * i / 0.5 * LEDBuffer.getLength() + hueOffset) % 180, 255, 255);
    }
    hueOffset = (hueOffset + 3 * speed * iterations) % 180;
    Timer.delay(0.05);
  }

  int head = 0;
  public void trail(int interval){
    resetLED();
    for(int i = head; i < LEDBuffer.getLength(); i += interval){
      LEDBuffer.setRGB(i % 100, red, green, blue);
      LEDBuffer.setRGB((i + 50) % 100, red, green, blue);
    }
    Timer.delay(0.03);
    head++;
  }

  public void flash(){
    setSolidColor();
    Timer.delay(0.1);
    resetLED();
    Timer.delay(0.1);
    setSolidColor();
    Timer.delay(0.25);
  }

  public void setBuffer() {
    LEDStrip.setData(LEDBuffer);
//    LEDStripB.setData(LEDBuffer);
  }

  int state = -1;
  public void setLED(){
    switch(state){
      case 0:
        setRainbow(3, 8);
        break;
      case 1:
        setRGB(255, 200, 0);
        setBlinkingColor(true);
        break;
      case 2:
        setRGB(255, 0, 0);
        setBlinkingColor(true);
        break;
      case 3:
        setRGB(255, 255, 255);
        flash();
        break;
      case 4:
        setRGB(20, 255, 110);
        trail(8);
      case 5:
        setRGB(0, 110, 255);
        setBlinkingColor(true);
      case 6:
        setRGB(255, 110, 0);
        trail(8);
      default:
        setRGB(106, 90, 205);
        setBlinkingColor(true);
    }
    LEDStrip.setData(LEDBuffer);
  }

  public void setState(int state){
    this.state = state;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //setRainbow(rainbows, speed);
    /*rainbows = SmartDashboard.getNumber("Rainbows", 0);
    speed = SmartDashboard.getNumber("Speed", 0);
    setBuffer();*/
    setLED();
  }
}
