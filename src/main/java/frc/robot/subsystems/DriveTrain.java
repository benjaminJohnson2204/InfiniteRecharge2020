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
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PCM_ONE;
import frc.robot.Constants.DriveMotors;

public class
DriveTrain extends SubsystemBase {
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
  private int gearRatio = 1; //gear ration between the encoder and the wheel
  private int wheelRadius = 3;

  //DifferentialDrive drive = new DifferentialDrive(driveMotors[0],driveMotors[2]);
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1,1, 1);

  PIDController leftPIDController = new PIDController(1, 0, 0);
  PIDController rightPIDController = new PIDController(1, 0,0);

  Pose2d pose;

  private int leftZeroOffset;
  private int rightZeroOffset;

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

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navX.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      driveMotors[0].getSelectedSensorVelocity() / gearRatio * 2 * Math.PI * Units.inchesToMeters(wheelRadius) / 60, //divide by gear ratio to make sure we have wheel speed
      driveMotors[2].getSelectedSensorVelocity() / gearRatio * 2 * Math.PI * Units.inchesToMeters(wheelRadius) / 60  //calculate for circumference and divide by 60 to convert from rpm to rps
    );
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public int getEncoderCount(int sensorIndex) {
    return driveMotors[sensorIndex].getSelectedSensorPosition();
  }

  public double getWheelDistanceMeters(int sensorIndex) {
    return driveMotors[sensorIndex].getSelectedSensorPosition() * Constants.ticksPerMeter;
  }

  public void resetEncoderCounts() {
    leftZeroOffset = driveMotors[0].getSelectedSensorPosition();
    rightZeroOffset = driveMotors[2].getSelectedSensorPosition();
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

  public void resetOdometry(){
    odometry.resetPosition(new Pose2d(0, 0, new Rotation2d(0)), new Rotation2d(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(getHeading(), getWheelDistanceMeters(0), getWheelDistanceMeters(2));
    SmartDashboard.putNumber("Left Encoder", getEncoderCount(0));
    SmartDashboard.putNumber("Right Encoder", getEncoderCount(2));

    SmartDashboard.putNumber("xCoordinate", Units.metersToInches(getRobotPose().getTranslation().getX()));
    SmartDashboard.putNumber("yCoordinate", Units.metersToInches(getRobotPose().getTranslation().getY()));
    SmartDashboard.putNumber("angle", getRobotPose().getRotation().getDegrees());
  }

  public Pose2d getRobotPose(){
    return pose;
  }
}
