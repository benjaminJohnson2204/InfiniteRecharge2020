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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Climber extends SubsystemBase {

  private TalonSRX[] climberMotors = {
          new TalonSRX(Constants.climbMotorA),
          new TalonSRX(Constants.climbMotorB),
  };

  DoubleSolenoid climbPiston = new DoubleSolenoid(Constants.pcmOne,  Constants.climbPistonForward, Constants.climbPistonReverse);

  public boolean getClimbPistonExtendStatus(){
    return climbPiston.get() == DoubleSolenoid.Value.kForward ? true : false;
  }

  public void setClimbPiston(boolean state){
    climbPiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  public Climber() {
    climberMotors[0].configFactoryDefault();
    climberMotors[0].setNeutralMode(NeutralMode.Coast);
    climberMotors[1].configFactoryDefault();
    climberMotors[1].setNeutralMode(NeutralMode.Coast);
  }

  public void setClimber(float value) {
    climberMotors[0].set(ControlMode.PercentOutput, value);
    climberMotors[0].set(ControlMode.PercentOutput, value);
  }
  public void stop() {
    setClimber(0);
  }
}
