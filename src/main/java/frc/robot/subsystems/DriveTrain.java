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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  private TalonSRX[] driveMotors ={
            new TalonSRX(Constants.leftDriveMotor),
            new TalonSRX(Constants.rightDriveMotor)

  };
  public DriveTrain() {
    for (TalonSRX motor : driveMotors){
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Coast);
    }
  driveMotors[0].setInverted(true);
  driveMotors[1].setInverted(false);
}
  public void tankDrive(double leftValue, double rightValue) {
    driveMotors[0].set(ControlMode.PercentOutput, leftValue);
    driveMotors[1].set(ControlMode.PercentOutput, rightValue);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler rub
  }
}
