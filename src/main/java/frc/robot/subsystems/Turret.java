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
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Turret extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  double kP = 0.00295;
  double kI = 0;
  double kD = 0.00135;
  double kS = 0;
  double kV = 0.00295;
  double kA = 0.000186;
  double maxAngle = 290;
  double minAngle = -110;
  double gearRatio = 18.0 / 120.0;
  double setpoint = 0; //angle

  private double degreesToUnits = 75.85185185185186;
  private int controlMode = 1;

  private final DriveTrain m_driveTrain;

  private Timer timeout = new Timer();

  private CANCoder encoder = new CANCoder(Constants.turretEncoder);

  private VictorSPX turretMotor = new VictorSPX(Constants.turretMotor);

  private SimpleMotorFeedforward turretFF = new SimpleMotorFeedforward(kS, kV, kA);
  private PIDController turretPID = new PIDController(kP, kI, kD);

  public Turret(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    turretMotor.configFactoryDefault();
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.setInverted(true);
    turretMotor.configRemoteFeedbackFilter(61, RemoteSensorSource.CANCoder, 0, 0);
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    turretMotor.config_kF(0, kV);
    turretMotor.config_kP(0, kP);
    turretMotor.config_kI(0, kI);
    turretMotor.config_kD(0, kD);
    turretMotor.configMotionCruiseVelocity(2 * 4096);
    turretMotor.configMotionAcceleration(20 * 4096);
    turretMotor.configAllowableClosedloopError(0, 50);
    turretMotor.selectProfileSlot(0,0);
    //encoder.configFactoryDefault();
    encoder.configFactoryDefault();
    //encoder.setPositionToAbsolute();

    initShuffleboard();
  }

  public void resetEncoder(){
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
    return gearRatio * -encoder.getPosition();
  }

  public double getFieldRelativeAngle(){
    return getTurretAngle()-m_driveTrain.navX.getAngle();
  }

  public void setPercentOutput(double output){
    turretMotor.set(ControlMode.PercentOutput, output);

  }

  public void setSetpoint(double setpoint){ //use degrees
    if(setpoint>=maxAngle) {
        setpoint = setpoint - 360;
    } else if(setpoint<=minAngle) {
        setpoint = setpoint + 360;
    }
    this.setpoint = setpoint;
  }

  public void setClosedLoopPosition(){
    SmartDashboard.putNumber("Turret Setpoint", setpoint * degreesToUnits);
    turretMotor.set(ControlMode.MotionMagic, setpoint * degreesToUnits);
    //setPercentOutput(turretPID.calculate(getTurretAngle(), setpoint));
  }

  public boolean atTarget(){
    return turretPID.atSetpoint();
  }

  public void initShuffleboard() {
    Shuffleboard.getTab("SmartDashboard").addNumber("Turret Motor Output", turretMotor::getMotorOutputPercent);
  }


  public void updateSmartdashboard() {
    SmartDashboard.putNumber("Turret Encoder Units", turretMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Robot Relative Turret Angle", getTurretAngle());
    SmartDashboard.putNumber("Field Relative Turret Angle", getFieldRelativeAngle());
//    SmartDashboard.putNumber("Turret Setpoint", setpoint);
//    SmartDashboard.putNumber("Turret Motor Output", turretMotor.getMotorOutputPercent());


    //Shuffleboard.getTab("Turret").addNumber("Turret Angle", this::getAngle);
    //Shuffleboard.getTab("Turret").addNumber("Position", this::getPosition);
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
