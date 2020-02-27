/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Climber extends SubsystemBase {

  private double gearRatio = 1.0/18.0;
  public double pulleyDiameter = 2.0; // inches

  private TalonFX climbMotor = new TalonFX(Constants.climbMotorA);

  DoubleSolenoid climbPiston = new DoubleSolenoid(Constants.pcmOne, Constants.climbPistonForward, Constants.climbPistonReverse);

  private boolean climbState;

  public Climber() {
    climbMotor.configFactoryDefault();
    climbMotor.setSelectedSensorPosition(0);
    climbMotor.setNeutralMode(NeutralMode.Brake);
    climbMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
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

  public void setClimberOutput(double value) {
    // Prevent backdrive
    climbMotor.set(ControlMode.PercentOutput, value);
  }
  
  public int getClimberPosition() {
	  return climbMotor.getSelectedSensorPosition();
  }

  public void setClimberPosition(double position) {
    double setpoint = inchesToEncoderUnits(position);
    climbMotor.set(ControlMode.Position, setpoint);
  }

  private double inchesToEncoderUnits(double inches) {
    return inches * gearRatio * (2048.0 / (Math.PI * pulleyDiameter));
  }

  private double encoderUnitsToInches(double encoderUnits) {
    return encoderUnits * (1/gearRatio) * ((Math.PI * pulleyDiameter) / 2048.0);
  }

  private void updateShuffleboard(){
    SmartDashboard.putBoolean("Climb Mode", climbState);

    SmartDashboardTab.putNumber("Climber", "Position", encoderUnitsToInches(climbMotor.getSelectedSensorPosition()));
    SmartDashboardTab.putBoolean("Climber", "Climb Mode", climbState);
    SmartDashboardTab.putBoolean("Climber", "Climb Pistons", getClimbPistonExtendStatus());
//    try {
//      SmartDashboardTab.putString("Climber", "Current Command", this.getCurrentCommand().getName());
//    }  catch(Exception e) {
//
//    }
  }
  @Override
  public void periodic() {
    updateShuffleboard();
  }
}