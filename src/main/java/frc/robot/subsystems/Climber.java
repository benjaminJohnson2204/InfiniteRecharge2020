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

  private CANSparkMax climbMotor = new CANSparkMax(Constants.climbMotor, CANSparkMaxLowLevel.MotorType.kBrushless);

  DoubleSolenoid climbPiston = new DoubleSolenoid(Constants.pcmOne,  Constants.climbPistonForward, Constants.climbPistonReverse);

  public boolean getClimbPistonExtendStatus(){
    return climbPiston.get() == DoubleSolenoid.Value.kForward ? true : false;
  }

  public void setClimbPiston(boolean state){
    climbPiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  public Climber() {
    climbMotor.restoreFactoryDefaults();
    climbMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    climbMotor.setInverted(false);
  }

  public void setClimber(double value) {
    climbMotor.set(value);
  }
  public void stop() {
    setClimber(0);
  }
}

//  private CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
//
//  DoubleSolenoid intakePiston = new DoubleSolenoid(Constants.pcmOne, Constants.intakePistonForward, Constants.intakePistonReverse);
//
//  public boolean getIntakePistonExtendStatus(){
//    return intakePiston.get() == DoubleSolenoid.Value.kForward ? true : false;
//  }
//
//  public void setIntakePiston(boolean state){
//    intakePiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
//  }
//
//  public Intake() {
//    intakeMotor.restoreFactoryDefaults();
//    intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
//    intakeMotor.setInverted(false);
//  }
//
//  public void setIntake(double value){
//    intakeMotor.set(value);
//  }
//
//  public void stop(){
//    setIntake(0);
//  }
//
////  public double getIntakeVoltage(){
////    return intakeMotor.getBusVoltage();
////  }
//
//  @Override
//  public void periodic() {
//  }
//}