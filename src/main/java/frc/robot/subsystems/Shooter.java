/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */


  private TalonFX[] outtakeMotors = {
      new TalonFX(Constants.flywheelMotorA), //0 and 1 are the actual shooting motors
      new TalonFX(Constants.flywheelMotorB)
  };

  double output = 0.5;
  public Shooter() {
    super(); //do not know what this does

    for(TalonFX outtakeMotor : outtakeMotors){
      outtakeMotor.configFactoryDefault(); //configure the motors
      outtakeMotor.setNeutralMode(NeutralMode.Coast);
    }
    outtakeMotors[0].setInverted(true);
    outtakeMotors[1].setInverted(false);
    outtakeMotors[1].follow(outtakeMotors[0]);
    SmartDashboard.putNumber("FW Output", output);
  }

  public void startSpin(double nothing){outtakeMotors[0].set(ControlMode.PercentOutput, output);}

  public void stopSpin(){outtakeMotors[0].set(ControlMode.PercentOutput, 0);}
  public void angleHood(double distance){
    //calculate angle necessary to shoot
    //determine difference in angles
    //use either pid loop or simpler loop to correct error
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    output = SmartDashboard.getNumber("FW Output", 0);
  }
}
