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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PCM_ONE;
import frc.robot.Constants.DriveMotors;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private TalonSRX[] driveMotors = {
        new TalonSRX(DriveMotors.leftFrontDriveMotor),
        new TalonSRX(DriveMotors.leftRearDriveMotor),
        new TalonSRX(DriveMotors.rightFrontDriveMotor),
        new TalonSRX(DriveMotors.rightRearDriveMotor),
        new TalonSRX(DriveMotors.climbDriveMotor)
  };

  DoubleSolenoid driveTrainShifters = new DoubleSolenoid(PCM_ONE.CAN_ADDRESS, PCM_ONE.DRIVETRAIN_SIFTER.FORWARD, PCM_ONE.DRIVETRAIN_SIFTER.REVERSE);
  public AHRS navX = new AHRS(SerialPort.Port.kMXP);

  public int controlMode = 0;

  public DriveTrain() {

    for (TalonSRX motor : driveMotors) {
      motor.configFactoryDefault();
//      motor.config_kP(0, 0.25, 30);
//      motor.config_kI(0, 0, 30);
//      motor.config_kD(0, 10, 30);
//      motor.config_kF(0, 1023.0 / 72000.0, 30);
      motor.configVoltageCompSaturation(12);
      motor.enableVoltageCompensation(true);
      motor.configContinuousCurrentLimit(30);
      motor.configPeakCurrentLimit(40);
      motor.configPeakCurrentDuration(1000);
      motor.enableCurrentLimit(true);
      motor.configOpenloopRamp(0.1);
      motor.configClosedloopRamp(0.1);
      motor.setNeutralMode(NeutralMode.Coast);
      motor.configForwardSoftLimitEnable(false);
      motor.configReverseSoftLimitEnable(false);
    }

    driveMotors[0].setInverted(true);
    driveMotors[1].setInverted(true);
    driveMotors[2].setInverted(false);
    driveMotors[3].setInverted(false);
    driveMotors[4].setInverted(true);

    driveMotors[0].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    driveMotors[2].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    driveMotors[0].setSensorPhase(false);
    driveMotors[2].setSensorPhase(false);

    driveMotors[1].set(ControlMode.Follower, driveMotors[0].getDeviceID());
    driveMotors[3].set(ControlMode.Follower, driveMotors[2].getDeviceID());

    driveMotors[4].configPeakOutputReverse(0);
  }
  public int getEncoderCount(int sensorIndex) {
    return driveMotors[sensorIndex].getSelectedSensorPosition();
  }

  public void setMotorArcadeDrive(double throttle, double turn) {
    double leftPWM = throttle + turn;
    double rightPWM = throttle - turn;

    if(rightPWM > 1.0) {
      leftPWM -= rightPWM - 1.0;
      rightPWM = 1.0;
    } else if(rightPWM < -1.0) {
      leftPWM -= rightPWM + 1.0;
      rightPWM = -1.0;
    } else if(leftPWM > 1.0) {
      rightPWM -= leftPWM - 1.0;
      leftPWM = 1.0;
    } else if(leftPWM < -1.0) {
      rightPWM -= leftPWM + 1.0;
      leftPWM = -1.0;
    }

//        if(Robot.climber.climbMode == 1)
//            setMotorCurrentOutput(20 *leftPWM, 20 * rightPWM);
//        else
    setMotorPercentOutput(leftPWM, rightPWM);
  }

  public void setMotorTankDrive(double leftOutput, double rightOutput) {
    setMotorPercentOutput(leftOutput, rightOutput);
  }

  public void setMotorPercentOutput(double leftOutput, double rightOutput) {
    driveMotors[0].set(ControlMode.PercentOutput, leftOutput);
    driveMotors[2].set(ControlMode.PercentOutput, rightOutput);
  }

  public boolean getDriveShifterStatus() {
    return (driveTrainShifters.get() == DoubleSolenoid.Value.kForward) ? true : false;
  }

  public void setDriveShifterStatus(boolean state) {
    driveTrainShifters.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Encoder", getEncoderCount(0));
    SmartDashboard.putNumber("Right Encoder", getEncoderCount(2));
  }
}
