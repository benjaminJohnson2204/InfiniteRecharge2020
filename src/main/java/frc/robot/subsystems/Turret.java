/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;

public class Turret extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  double kP = 0.0171;
  double kI = 0;
  double kD = 0.00781;
  double kS = 0;
  double kV = 0.00103;
  double kA = 0.000164;
  double maxAngle = 290;
  double minAngle = -110;
  double gearRatio = 18.0 / 120.0;
  double setpoint = 0; //angle
  public int controlMode = 1;

  private Timer timeout = new Timer();

  private CANCoder encoder = new CANCoder(Constants.turretEncoder);

  private VictorSPX turretMotor = new VictorSPX(Constants.turretMotor);

  private SimpleMotorFeedforward turretFF = new SimpleMotorFeedforward(kS, kV, kA);
  private PIDController turretPID = new PIDController(kP, kI, kD);

  public Turret() {
    turretMotor.configFactoryDefault();
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.setInverted(false);
    //encoder.configFactoryDefault();
    encoder.setPositionToAbsolute();
  }

  public void resetEncoder(){
    encoder.setPosition(0);
  }

  public double getAngle(){
    return gearRatio * encoder.getPosition();
  }

  public void setPercentOutput(double output){
    turretMotor.set(ControlMode.PercentOutput, output);
  }

  public void incrementSetpoint(double increment){
    setpoint = setpoint + increment;
    if(Math.abs(setpoint)>=maxAngle) {
      if (setpoint < 0) {
        setpoint = setpoint + 360;
      } else {
        setpoint = setpoint - 360;
      }
      Constants.limelightTempDisabled = true;
    }
  }

  public void setSetpoint(double setpoint){ //use degrees
    if(Math.abs(setpoint)>=maxAngle) {
      if (setpoint < 0) {
        setpoint = setpoint + 360;
      } else {
        setpoint = setpoint - 360;
      }
      Constants.limelightTempDisabled = true;
    }
    this.setpoint = setpoint;
  }

  public void setClosedLoopPosition(){
    setPercentOutput(turretPID.calculate(getAngle(), setpoint));
  }

  public boolean atTarget(){
    return turretPID.atSetpoint();
  }

  public void updateSmartdashboard() {
    SmartDashboard.putNumber("Turret Angle", getAngle());

    //Shuffleboard.getTab("Turret").addNumber("Turret Angle", this::getAngle);
    //Shuffleboard.getTab("Turret").addNumber("Position", this::getPosition);
  }

  @Override
  public void periodic() {
    if(controlMode == 1)
      setClosedLoopPosition();
    // This method will be called once per scheduler run
    updateSmartdashboard();
  }
}