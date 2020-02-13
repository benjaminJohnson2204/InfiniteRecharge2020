/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.music.Orchestra;
import frc.robot.constants.Constants;

public class Orchestra extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   * https://www.ctr-electronics.com/downloads/api/java/html/classcom_1_1ctre_1_1phoenix_1_1music_1_1_orchestra.html
   */
  public Orchestra() {
    //using the leftRearDriveMotor as the speaker motor
    addInstrument(Constants.leftRearDriveMotor);
  }

  public void loadSong(String path){
    loadMusic(path);
  }

  public void playSong() {
    play();
  }

  public void stopSong() {
    stop();
    //note: dont import the badLog stuff, thats the wrong thing
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}