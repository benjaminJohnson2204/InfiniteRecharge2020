/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private TalonSRX[] outtakeMotors = {
          new TalonSRX(40); //subject to change, 40 is placeholder
          //initialize motors here
  }
  public Shooter() {
    super("Shooter"); //do not know what this does

    for(TalonSRX outtakeMotors:outtakeMotor){
      outtakeMotor.configFactoryDefault(); //configure the motors
      outtakeMotor.setNeutralMode(NeutralMode.Coast)
    }
    //set any motors inverted/follow if necessary
  }

  public void startSpin(double output){outtakeMotors[0].set(ControlMode.PercentOutput, output);}

  public void insertCell(){/*dont know how to do this mechanically*/}

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
