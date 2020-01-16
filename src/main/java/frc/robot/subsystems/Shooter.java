/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
 * @return 
   */
  public double kP = 0.004;
  public double kI = 0;
  public double kD = 0.0016;

  private CANSparkMax[] outtakeMotors = {
          new CANSparkMax(40, MotorType.kBrushless), //0 and 1 are the actual shooting motors
          new CANSparkMax(41, MotorType.kBrushless)
  };

  private CANEncoder shooterEncoder = new CANEncoder(outtakeMotors[0]);

  private PIDController shooterController = new PIDController(kP, kI, kD);

  public Shooter() {
    super(); //do not know what this does

    for(CANSparkMax outtakeMotor : outtakeMotors){
      outtakeMotor.restoreFactoryDefaults(); //configure the motors
      outtakeMotor.setIdleMode(IdleMode.kCoast);
    }
    outtakeMotors[0].setInverted(false);
    outtakeMotors[1].setInverted(true);

    shooterController.setIntegratorRange(-500, 500);
    shooterController.setTolerance(100);

  }

  

  public void startSpin(double output){
    outtakeMotors[0].set(output);
    outtakeMotors[1].set(output);
  }

  public void setRPM(double setpoint){
    double output = shooterController.calculate(shooterEncoder.getVelocity(), setpoint);
    startSpin(output);
  }

  public boolean encoderAtSetpoint(){
    return shooterController.atSetpoint();
  }

  public void updateShuffleboard(){
    SmartDashboard.putNumber("RPM", shooterEncoder.getVelocity());
  }

  public void insertCell(){}



  @Override
  public void periodic() {
    updateShuffleboard();
    // This method will be called once per scheduler run
  }
}
