/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Indexer extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  CANSparkMax master = new CANSparkMax(Constants.indexerMotor, MotorType.kBrushless);
  CANEncoder encoder = master.getEncoder();
  VictorSPX kicker = new VictorSPX(Constants.kickerMotor);
  CANPIDController pidController = master.getPIDController();
  DigitalInput sensor = new DigitalInput(Constants.indexSensor);
  DigitalInput limitSensor = new DigitalInput(Constants.indexLimitSensor);
  DigitalInput secondSensor = new DigitalInput(Constants.secondIndexSensor);
  private double targetSetpoint;
//  private double kF = 1.67;
//  private double kP = 2.36;
//  private double kI = 0;
//  private double kD = 1070;
  private double kF = 0.0001;
  private double kP = 0.000001;
  private double kI = 80;
  private double kD = 0.0001;

  private int controlMode = 1;

  public Indexer() {
    master.restoreFactoryDefaults();
    pidController.setFF(kF);
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setSmartMotionMaxVelocity(1.1e4, 0); // Formerly 1.1e4
    pidController.setSmartMotionMaxAccel(1e6, 0); // Formerly 1e6
    pidController.setSmartMotionAllowedClosedLoopError(1, 0);
    pidController.setIZone(1);
    master.setInverted(true);
    kicker.setInverted(true);
    master.setIdleMode(IdleMode.kBrake);

//    SmartDashboard.putNumber("kF", kF);
//    SmartDashboard.putNumber("kP", kP);
//    SmartDashboard.putNumber("kI", kI);
//    SmartDashboard.putNumber("kD", kD);
  }

  public void toggleControlMode() {
    if(controlMode == 0)
      controlMode = 1;
    else
      controlMode = 0;
  }

  public int getControlMode() {
    return controlMode;
  }

  public boolean sensorTripped(){
    return (!sensor.get());
  }

  public boolean secondSensor(){
    return !secondSensor.get();
  }

  boolean pTripped = false;
  public boolean newBall(){
    boolean returnVal;
    if(pTripped == false && sensorTripped()){
      returnVal = true;
    }
    else 
      returnVal = false;
    if(sensorTripped())
      pTripped = true;
    else
      pTripped = false;
    return returnVal;
  }

  public boolean topSensor(){
    return !limitSensor.get();
  }

  public void incrementIndexer(double setpoint){
    targetSetpoint = setpoint;
    SmartDashboard.putNumber("Target Setpoint", targetSetpoint);
    pidController.setReference(targetSetpoint, ControlType.kSmartMotion);
  }

  public void resetEncoderPosition(){
    encoder.setPosition(0);
  }

  public double getPosition(){
    return encoder.getPosition();
  }

  public boolean onTarget() {
    return Math.abs(encoder.getPosition() - targetSetpoint) < 1; 
  }

  public boolean indexerFull(){
    return !limitSensor.get();
  }

  public void setKickerOutput(double output) {
    kicker.set(ControlMode.PercentOutput, output);
  }

  public void setIndexerOutput(double output) {
    master.set(output);
  }

  private void updateSmartDashboard(){
    SmartDashboard.putNumber("Motor Output", master.getAppliedOutput());
    SmartDashboard.putBoolean("Indexing Sensor Tripped", sensorTripped());
    SmartDashboard.putNumber("Motor Position", getPosition());
    
  }

  private void updatePIDValues() {
    kF = SmartDashboard.getNumber("kF", 0);
    kP = SmartDashboard.getNumber("kP", 0);
    kI = SmartDashboard.getNumber("kI", 0);
    kD = SmartDashboard.getNumber("kD", 0);
    pidController.setFF(kF);
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //updateSmartDashboard();
    //updatePIDValues();
  }
}
