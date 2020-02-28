/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
 * @return 
   */
  public double kF = 0.0523;   //Gree: 0.0475;
  public double kP = 0.6; 	  //0.00047
  public double kI = 0.0; 	  //0.0000287
  public double kD = 0.0;

  public int kI_Zone = 100;
  public int kAllowableError = 50;

  private TalonFX[] outtakeMotors = {
        new TalonFX(Constants.flywheelMotorA),
        new TalonFX(Constants.flywheelMotorB),
  };

  public double rpmOutput;
  public double rpmTolerance = 25.0;

  public Shooter() {

    for(TalonFX outtakeMotor : outtakeMotors){
      outtakeMotor.configFactoryDefault();
      outtakeMotor.setNeutralMode(NeutralMode.Coast);
      outtakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,30,0,0));
    }
    outtakeMotors[0].setInverted(true);
    outtakeMotors[1].follow(outtakeMotors[0], FollowerType.PercentOutput);

    outtakeMotors[0].config_kF(0, kF);
    outtakeMotors[0].config_kP(0, kP);
    outtakeMotors[0].config_kI(0, kI);
    outtakeMotors[0].config_IntegralZone(0,kI_Zone);
    outtakeMotors[0].config_kD(0, kD);
    outtakeMotors[0].configAllowableClosedloopError(0, kAllowableError);
    outtakeMotors[0].configClosedloopRamp(0.1);
    outtakeMotors[1].configClosedloopRamp(0);
    outtakeMotors[1].configOpenloopRamp(0);

    initShuffleboard();
  }

  public double getMotorInputCurrent(int motorIndex) {
    return outtakeMotors[motorIndex].getSupplyCurrent();
  }

  public void setPower(double output) {
    outtakeMotors[0].set(ControlMode.PercentOutput, output);
  }

  public void setRPM(double setpoint){
    outtakeMotors[0].set(ControlMode.Velocity, RPMtoFalconUnits(setpoint));
  }

  public void setTestRPM(){
    outtakeMotors[0].set(ControlMode.Velocity, RPMtoFalconUnits(rpmOutput));
  }

  public double getTestRPM() {
    return rpmOutput;
  }

  public double getRPMTolerance() {
    return rpmTolerance;
  }

  public boolean encoderAtSetpoint(int motorIndex){
    return (Math.abs(outtakeMotors[motorIndex].getClosedLoopError()) < 100.0);
  }
  public double getRPM(int motorIndex) {
    return falconUnitsToRPM(outtakeMotors[motorIndex].getSelectedSensorVelocity());
  }
  public double falconUnitsToRPM(double sensorUnits) {
    return (sensorUnits / 2048.0) * 600.0;
  }

  public double RPMtoFalconUnits(double RPM) {
    return (RPM / 600.0) * 2048.0;
  }

  private void initShuffleboard() {
    // Unstable. Don''t use until WPILib fixes this
//    Shuffleboard.getTab("Shooter").addNumber("RPM Primary", () -> this.getRPM(0));
//    Shuffleboard.getTab("Shooter").addNumber("RPM Secondary", () -> this.getRPM(1));
//    Shuffleboard.getTab("Shooter").addNumber("Power", () -> this.outtakeMotors[0].getMotorOutputPercent());

    SmartDashboardTab.putNumber("Shooter","RPM Output", rpmOutput);
    SmartDashboardTab.putNumber("Shooter","Flywheel kF", kF);
    SmartDashboardTab.putNumber("Shooter","Flywheel kP", kP);
    SmartDashboardTab.putNumber("Shooter","Flywheel kI", kI);
    SmartDashboardTab.putNumber("Shooter","Flywheel kD", kD);
    SmartDashboardTab.putNumber("Shooter","Flywheel kI_Zone", kI_Zone);
    SmartDashboardTab.putNumber("Shooter","Flywheel kAllowableError", kAllowableError);
    SmartDashboardTab.putNumber("Shooter","Flywheel RPM Tolerance", rpmTolerance);
  }

  private void updateShuffleboard(){
    SmartDashboard.putNumber("RPM", falconUnitsToRPM(outtakeMotors[0].getSelectedSensorVelocity()));

    SmartDashboardTab.putNumber("Shooter", "RPM Primary", getRPM(0));
    SmartDashboardTab.putNumber("Shooter", "RPM Secondary", getRPM(1));
    SmartDashboardTab.putNumber("Shooter", "Power", outtakeMotors[0].getMotorOutputPercent());


  }

  public void updatePIDValues() {
    rpmOutput = SmartDashboardTab.getNumber("Shooter","RPM Output", 0);
    rpmTolerance = SmartDashboardTab.getNumber("Shooter","Flywheel RPM Tolerance", 0);

    outtakeMotors[0].config_kF(0, SmartDashboardTab.getNumber("Shooter","Flywheel kF", 0));
    outtakeMotors[0].config_kP(0, SmartDashboardTab.getNumber("Shooter","Flywheel kP", 0));
    outtakeMotors[0].config_kI(0, SmartDashboardTab.getNumber("Shooter","Flywheel kI", 0));
    outtakeMotors[0].config_IntegralZone(0, (int) SmartDashboardTab.getNumber("Shooter","Flywheel kI_Zone", 0));
    outtakeMotors[0].config_kD(0, SmartDashboardTab.getNumber("Shooter","Flywheel kD", 0));
    outtakeMotors[0].configAllowableClosedloopError(0, (int) SmartDashboardTab.getNumber("Shooter","Flywheel kAllowableError", 0));
  }


  @Override
  public void periodic() {
    updateShuffleboard();
    //updatePIDValues();
  }
}
