/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.constants.Constants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private CANSparkMax[] outtakeMotors = {
      new CANSparkMax(Constants.flywheelMotorA, MotorType.kBrushless), //0 and 1 are the actual shooting motors
      new CANSparkMax(Constants.flywheelMotorB, MotorType.kBrushless)
  };

  public Shooter() {
    super(); //do not know what this does

    for(CANSparkMax outtakeMotor : outtakeMotors){
      outtakeMotor.restoreFactoryDefaults(); //configure the motors
      outtakeMotor.setIdleMode(IdleMode.kCoast);
    }
    outtakeMotors[0].setInverted(false);
    outtakeMotors[1].setInverted(true);
    outtakeMotors[1].follow(outtakeMotors[0]);
  }

  public void startSpin(double output){outtakeMotors[0].set(output);}

  public void angleHood(double distance){
    //calculate angle necessary to shoot
    //determine difference in angles
    //use either pid loop or simpler loop to correct error
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
