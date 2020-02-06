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
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Turret extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  double kF = 0.00295;
  double kP = 0.00295;
  double kI = 0;
  double kD = 0.00135;
  double maxAngle = 290;
  double minAngle = -10;
  double gearRatio = 18.0 / 120.0;
  double setpoint = 0; //angle

  private int encoderUnitsPerRotation = 4096;
  private int controlMode = 1;

  private final DriveTrain m_driveTrain;

  private Timer timeout = new Timer();

  private CANCoder encoder = new CANCoder(Constants.turretEncoder);

  private VictorSPX turretMotor = new VictorSPX(Constants.turretMotor);

  public Turret(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    turretMotor.configFactoryDefault();
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.setInverted(true);
    turretMotor.configRemoteFeedbackFilter(61, RemoteSensorSource.CANCoder, 0, 0);
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    turretMotor.config_kF(0, kF);
    turretMotor.config_kP(0, kP);
    turretMotor.config_kI(0, kI);
    turretMotor.config_kD(0, kD);
    turretMotor.configMotionCruiseVelocity(14000);
    turretMotor.configMotionAcceleration(140000);
    turretMotor.configAllowableClosedloopError(0, 50);
    turretMotor.selectProfileSlot(0,0);
    //encoder.configFactoryDefault();
    encoder.configFactoryDefault();
    //encoder.setPositionToAbsolute();

    initShuffleboard();
  }

  public void resetEncoder(){
    turretMotor.setSelectedSensorPosition(0);
    encoder.setPosition(0);
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

  public double getTurretAngle(){
    return gearRatio * encoder.getPosition();
  }

  public double getFieldRelativeAngle(){
    return getTurretAngle()-m_driveTrain.navX.getAngle();
  }

  public void setPercentOutput(double output){
    turretMotor.set(ControlMode.PercentOutput, output);
  }

  public void setSetpoint(double setpoint){ //use degrees
    // TODO: Add logic for overwrap
    double distanceToSetpoint = Math.abs(setpoint - getTurretAngle());

    // Case B: If inverse of the setpoint is closer, use that as the setpoint
    distanceToSetpoint = Math.abs(setpoint - getTurretAngle());
    if(Math.abs((setpoint - 360) - getTurretAngle()) <= distanceToSetpoint)
      setpoint -= 360;
    else if(Math.abs((setpoint + 360) - getTurretAngle()) <= distanceToSetpoint)
      setpoint += 360;

	// Case A: If the setpoint exceeds the min/max angle, use the inverse setpoint
    if(setpoint < minAngle)
      setpoint += 360;
    else if(setpoint > maxAngle)
      setpoint -= 360;

    this.setpoint = setpoint;
  }

  public void setClosedLoopPosition(){
    turretMotor.set(ControlMode.MotionMagic, degreesToEncoderUnits(setpoint));
  }

  public int degreesToEncoderUnits(double degrees) {
    return (int)(degrees * (1.0 / gearRatio) * (encoderUnitsPerRotation / 360.0));
  }

  public double encoderUnitsToDegrees(double encoderUnits) {
    return encoderUnits * gearRatio * (360.0 / encoderUnitsPerRotation);
  }

  public boolean atTarget(){
    return Math.abs(turretMotor.getClosedLoopError()) < 50;
  }

  public void initShuffleboard() {
    Shuffleboard.getTab("SmartDashboard").addNumber("Turret Motor Output", turretMotor::getMotorOutputPercent);
  }


  public void updateSmartdashboard() {
    SmartDashboard.putNumber("Turret Encoder Units", turretMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Turret Setpoint Degrees", setpoint);
    SmartDashboard.putNumber("Turret Setpoint Units", degreesToEncoderUnits(setpoint));
    SmartDashboard.putNumber("Turret Target Degrees", encoderUnitsToDegrees(turretMotor.getClosedLoopTarget()));
    SmartDashboard.putNumber("Turret Target Units", turretMotor.getClosedLoopTarget());

    SmartDashboard.putNumber("Robot Relative Turret Angle", getTurretAngle());
    SmartDashboard.putNumber("Field Relative Turret Angle", getFieldRelativeAngle());
  }

  @Override
  public void periodic() {
//    if(controlMode == 1)
//      setClosedLoopPosition();
//    else
//      setPercentOutput(RobotContainer.getXBoxLeftX());
    // This method will be called once per scheduler run
    updateSmartdashboard();
  }
}
