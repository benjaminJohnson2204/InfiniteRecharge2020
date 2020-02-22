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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Turret extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  double kF = 0.05;     //0.05
  double kP = 0.155;    //0.155
  double kI = 0.0001;    //0.00075
  double kD = 0.00766;  //0.00766
  int kI_Zone = 900;    //900
  int kMaxIAccum = 500000;    //900
  int kErrorBand = 50;
  double maxAngle = 315;
  double minAngle = -135;
  double gearRatio = 18.0 / 120.0;
  private double setpoint = 0; //angle

  private int encoderUnitsPerRotation = 4096;
  private int controlMode = 0;

  private final DriveTrain m_driveTrain;

  private Timer timeout = new Timer();

  private CANCoder encoder = new CANCoder(Constants.turretEncoder);

  private VictorSPX turretMotor = new VictorSPX(Constants.turretMotor);

  private DigitalInput turretHomeSensor = new DigitalInput(Constants.turretHomeSensor);
  private boolean turretHomeSensorLatch = false;

  public Turret(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    encoder.configFactoryDefault();
    encoder.setPositionToAbsolute();
    encoder.configSensorDirection(true);

    turretMotor.configFactoryDefault();
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.setInverted(true);
    turretMotor.setSelectedSensorPosition(0);
    turretMotor.configRemoteFeedbackFilter(61, RemoteSensorSource.CANCoder, 0, 0);
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    turretMotor.config_kF(0, kF);
    turretMotor.config_kP(0, kP);
    turretMotor.config_kI(0, kI);
    turretMotor.config_IntegralZone(0, kI_Zone);
    turretMotor.configMaxIntegralAccumulator(0, kMaxIAccum);
    turretMotor.config_kD(0, kD);
    turretMotor.configMotionCruiseVelocity(140000);
    turretMotor.configMotionAcceleration(1400000);
    turretMotor.configAllowableClosedloopError(0, kErrorBand);


    //turretPID.enableContinuousInput(0, 360);

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
    return encoderUnitsToDegrees(turretMotor.getSelectedSensorPosition());
  }

  public double getFieldRelativeAngle(){
    return getTurretAngle() - m_driveTrain.navX.getAngle();
  }

  public double getMaxAngle() {
    return maxAngle;
  }

  public double getMinAngle() {
    return minAngle;
  }

  public boolean getTurretHome() {
    return !turretHomeSensor.get();
  }

  public void setPercentOutput(double output){
    turretMotor.set(ControlMode.PercentOutput, output);
  }

  public void setSetpoint(double setpoint){
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
    return Math.abs(turretMotor.getClosedLoopError()) < kErrorBand;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public void clearIAccum() {
    turretMotor.setIntegralAccumulator(0);
  }

  private void initShuffleboard() {
    Shuffleboard.getTab("Turret").addNumber("Turret Motor Output", turretMotor::getMotorOutputPercent);
    Shuffleboard.getTab("Turret").addNumber("Turret Robot Relative Angle", this::getTurretAngle);
    Shuffleboard.getTab("Turret").addNumber("Turret Field Relative Angle", this::getFieldRelativeAngle);
    Shuffleboard.getTab("Turret").addNumber("Turret Setpoint", this::getSetpoint);
    Shuffleboard.getTab("Turret").addNumber("Turret Error", turretMotor::getClosedLoopError);
    Shuffleboard.getTab("Turret").addNumber("Turret IAccum", turretMotor::getIntegralAccumulator);
    Shuffleboard.getTab("Turret").addBoolean("Home", this::getTurretHome);
  }

  private void updateSmartdashboard() {
    SmartDashboard.putNumber("Robot Relative Turret Angle", getTurretAngle());
    SmartDashboard.putNumber("Field Relative Turret Angle", getFieldRelativeAngle());
    SmartDashboard.putNumber("Turret Setpoint", setpoint);
    SmartDashboard.putNumber("Turret Motor Output", turretMotor.getMotorOutputPercent());

    SmartDashboard.getBoolean("Turret Home", getTurretHome());
  }

  @Override
  public void periodic() {
    if(getControlMode() == 1)
      setClosedLoopPosition();

    // This method will be called once per scheduler run
    if(!turretHomeSensorLatch && getTurretHome()) {
      turretMotor.setSelectedSensorPosition(0);
      turretHomeSensorLatch = true;
    } else if(turretHomeSensorLatch && !getTurretHome())
      turretHomeSensorLatch = false;

    //updateSmartdashboard();
  }
}
