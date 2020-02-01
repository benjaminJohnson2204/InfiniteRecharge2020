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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
 * @return 
   */
  public double kF = 0.0558;
  public double kP = 0.000452;
  public double kI = 0.0000287;
  public double kD = 0.0;
  public double power = 1;

  private TalonFX[] outtakeMotors = {
          new TalonFX(40),
          new TalonFX(41),
  };

  //private VictorSPX turretMotor = new VictorSPX(42);


  private double rpmOutput;
  public Shooter() {
    //super();

    for(TalonFX outtakeMotor : outtakeMotors){
      outtakeMotor.configFactoryDefault();
      outtakeMotor.setNeutralMode(NeutralMode.Coast);
    }
    outtakeMotors[0].setInverted(true);
    outtakeMotors[1].follow(outtakeMotors[0], FollowerType.PercentOutput);


    outtakeMotors[0].config_kF(0, kF);
    outtakeMotors[0].config_kP(0, kP);
    outtakeMotors[0].config_kI(0, kI);
    outtakeMotors[0].config_IntegralZone(0,100);
    outtakeMotors[0].config_kD(0, kD);
    outtakeMotors[0].configAllowableClosedloopError(0, 100);

    outtakeMotors[1].configClosedloopRamp(0);
    outtakeMotors[1].configOpenloopRamp(0);




    SmartDashboard.putNumber("RPM Output", rpmOutput);

  }
  

  public void startSpin(double output){
    outtakeMotors[0].set(ControlMode.PercentOutput, rpmOutput);
  }

  public void setPower(double output) {
    outtakeMotors[0].set(ControlMode.PercentOutput, output/1.2);
  }
  public void spinTurret(double output, int direction){
    //turretMotor.set(ControlMode.PercentOutput, output*direction);
  }


  public void setRPM(double setpoint){
    outtakeMotors[0].set(ControlMode.Velocity, rpmOutput);
  }

  public boolean encoderAtSetpoint(){
    return (Math.abs(outtakeMotors[0].getClosedLoopError()) < 100);
  }

  public void updateShuffleboard(){
    SmartDashboard.putNumber("RPM", outtakeMotors[0].getSelectedSensorVelocity());
    SmartDashboard.putNumber("RPM 2", outtakeMotors[1].getSelectedSensorVelocity());
    SmartDashboard.putNumber("Voltage", outtakeMotors[0].getBusVoltage());
    SmartDashboard.putNumber("Power", power);
    SmartDashboard.putNumber("Position", outtakeMotors[0].getSelectedSensorPosition());
    rpmOutput = SmartDashboard.getNumber("RPM Output", 0);
  }




  @Override
  public void periodic() {
    updateShuffleboard();
    // This method will be called once per scheduler run
  }
}
