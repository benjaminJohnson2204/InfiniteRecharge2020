/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Climber extends SubsystemBase {

  private CANSparkMax climbMotor = new CANSparkMax(Constants.climbMotorA, CANSparkMaxLowLevel.MotorType.kBrushless);

  DoubleSolenoid climbPiston = new DoubleSolenoid(Constants.pcmOne,  Constants.climbPistonForward, Constants.climbPistonReverse);

  private boolean climbState;

  public Climber() {
    climbMotor.restoreFactoryDefaults();
    climbMotor.setIdleMode(IdleMode.kBrake);
    climbMotor.setInverted(true);
  }

  public boolean getClimbPistonExtendStatus(){
    return climbPiston.get() == DoubleSolenoid.Value.kForward ? true : false;
  }

  public void setClimbPiston(boolean state){
    climbPiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  public boolean getClimbState() {
    return climbState;
  }

  public void setClimbState(boolean climbState) {
    this.climbState = climbState;
  }

  public void setClimber(double value) {
    climbMotor.set(value);
  }

  @Override
  public void periodic() {

  }
}