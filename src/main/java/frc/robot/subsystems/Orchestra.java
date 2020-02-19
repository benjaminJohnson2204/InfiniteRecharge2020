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

  private final TalonFX[] talons = {
          new TalonFX(Constants.leftRearDriveMotor),
          new TalonFX(Constants.rightRearDriveMotor),
          new TalonFX(Constants.rightFrontDriveMotor),
          new TalonFX(Constants.leftFrontDriveMotor),
          new TalonFX(Constants.flywheelMotorA),
          new TalonFX(Constants.flywheelMotorB)
  };

  private Orchestra orchestra;

  public Orchestra() {
    for (TalonFX talon : talons) {
      orchestra.addInstrument(talon);
    }
  }

  public void song(String name) {
    if (!orchestra.isPlaying()) {
      orchestra.loadMusic(name);
      orchestra.play();
    }
  }

  public void stopSong(){
    stop();
  }

  @Override
  public void periodic() {
  }
}