/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Climber extends SubsystemBase {

  private CANSparkMax[] climberMotors = {
      new CANSparkMax(Constants.climbMotorA, MotorType.kBrushless),
      new CANSparkMax(Constants.climbMotorB, MotorType.kBrushless)
  };
  public Climber() {
    for(CANSparkMax motor: climberMotors) {
      motor.restoreFactoryDefaults();
      motor.setIdleMode(IdleMode.kBrake);
    }
    climberMotors[1].follow(climberMotors[0], true);
  }

  public void setClimber(double value) {
    climberMotors[0].set(value);
  }

  public void stop() {
    setClimber(0);
  }

  @Override
  public void periodic() {

  }
}
