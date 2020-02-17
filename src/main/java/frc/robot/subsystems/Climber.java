/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Climber extends SubsystemBase {

  //defining the climb motor as a motor that uses a spark max
  private CANSparkMax climbMotor = new CANSparkMax(Constants.climbMotorA, CANSparkMaxLowLevel.MotorType.kBrushless);

  // defining the climb piston as a double solenoid
  DoubleSolenoid climbPiston = new DoubleSolenoid(Constants.pcmOne,  Constants.climbPistonForward, Constants.climbPistonReverse);

  public boolean getClimbPistonExtendStatus(){
    //comments the pistons current postion
    return climbPiston.get() == DoubleSolenoid.Value.kForward ? true : false;
  }

  public void setClimbPiston(boolean state){
    //sets the piston to the opposite postition to its current
    climbPiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  public Climber() {
    //configuring the motor
    climbMotor.restoreFactoryDefaults();
    //setting the motor to nuetral
    climbMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    //not inverting the motor
    climbMotor.setInverted(false);
  }

  public void setClimber(double value) {
    //setting the climber motor to a value
    climbMotor.set(value);
  }

  public void stop() {
    //stops the motor
    setClimber(0);
  }

  @Override
  public void periodic() {

  }
}