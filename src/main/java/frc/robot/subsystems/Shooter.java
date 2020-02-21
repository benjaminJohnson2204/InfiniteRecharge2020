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
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
 * @return 
   */
  public double kF = 0.0475;  //0.0558
  public double kP = 0.4; 	  //0.00047
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
    //super();

    for(TalonFX outtakeMotor : outtakeMotors){
      outtakeMotor.configFactoryDefault();
      outtakeMotor.setNeutralMode(NeutralMode.Coast);
      outtakeMotor.enableVoltageCompensation(false);
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

    //initShuffleboard();
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
    Shuffleboard.getTab("Shooter").addNumber("RPM Primary", () -> this.getRPM(0));
    Shuffleboard.getTab("Shooter").addNumber("RPM Secondary", () -> this.getRPM(1));
    Shuffleboard.getTab("Shooter").addNumber("Power", () -> this.outtakeMotors[0].getMotorOutputPercent());

//    SmartDashboard.putNumber("RPM Output", rpmOutput);
//    SmartDashboard.putNumber("Flywheel kF", kF);
//    SmartDashboard.putNumber("Flywheel kP", kP);
//    SmartDashboard.putNumber("Flywheel kI", kI);
//    SmartDashboard.putNumber("Flywheel kD", kD);
//    SmartDashboard.putNumber("Flywheel kI_Zone", kI_Zone);
//    SmartDashboard.putNumber("Flywheel kAllowableError", kAllowableError);
//    SmartDashboard.putNumber("Flywheel RPM Tolerance", rpmTolerance);
  }

  private void updateShuffleboard(){
    SmartDashboard.putNumber("RPM", falconUnitsToRPM(outtakeMotors[0].getSelectedSensorVelocity()));

    SmartDashboardTab.putNumber("Shooter", "RPM Primary", getRPM(0));
    SmartDashboardTab.putNumber("Shooter", "RPM Secondary", getRPM(1));
    SmartDashboardTab.putNumber("Shooter", "Power", outtakeMotors[0].getMotorOutputPercent());
  }

  public void updatePIDValues() {
    rpmOutput = SmartDashboard.getNumber("RPM Output", 0);
    rpmTolerance = SmartDashboard.getNumber("Flywheel RPM Tolerance", 0);

    outtakeMotors[0].config_kF(0, SmartDashboard.getNumber("Flywheel kF", 0));
    outtakeMotors[0].config_kP(0, SmartDashboard.getNumber("Flywheel kP", 0));
    outtakeMotors[0].config_kI(0, SmartDashboard.getNumber("Flywheel kI", 0));
    outtakeMotors[0].config_IntegralZone(0, (int) SmartDashboard.getNumber("Flywheel kI_Zone", 0));
    outtakeMotors[0].config_kD(0, SmartDashboard.getNumber("Flywheel kD", 0));
    outtakeMotors[0].configAllowableClosedloopError(0, (int) SmartDashboard.getNumber("Flywheel kAllowableError", 0));
  }


  @Override
  public void periodic() {
    updateShuffleboard();
//    updatePIDValues();
    // This method will be called once per scheduler run
//    rpmOutput = SmartDashboard.getNumber("FW Output", 0);
  }
}
