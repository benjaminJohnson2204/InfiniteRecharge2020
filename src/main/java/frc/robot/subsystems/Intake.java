
package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private double kFF = 0.00068; //0.06; //0.122
  private double kP = 6e-5; //0.492
  private double kI = 0;
  private double kD = 0;

  private double kI_Zone = 0;
  private double allowableError = 50;
  private double maxVel = 5880;
  private double maxAccel = 58800;
  private double gearRatio = 1.0/3.0;

  private CANSparkMax intakeMotor =  new CANSparkMax(Constants.intakeMotor, MotorType.kBrushless);
  private CANEncoder intakeEncoder = intakeMotor.getEncoder();
  private CANPIDController canPidController = intakeMotor.getPIDController();

  DoubleSolenoid intakePiston = new DoubleSolenoid(Constants.pcmOne, Constants.intakePistonForward, Constants.intakePistonReverse);

  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    intakeMotor.setInverted(false);

    canPidController.setFF(kFF);
    canPidController.setP(kP);
    canPidController.setI(kI);
    canPidController.setIZone(kI_Zone);
    canPidController.setD(kD);
    canPidController.setSmartMotionMaxVelocity(maxVel, 0);
    canPidController.setSmartMotionMaxAccel(maxAccel, 0);
    canPidController.setSmartMotionAllowedClosedLoopError(allowableError, 0);
  }

  public boolean getintakePistonExtendStatus(){
    return intakePiston.get() == DoubleSolenoid.Value.kForward ? true : false;
  }

  public void setintakePiston(boolean state){
    intakePiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  public void setIntakePercentOutput(double value){
    intakeMotor.set(value);
  }

  public double getRPM(){
    return intakeEncoder.getVelocity() * gearRatio;
  }

  public void setRPM(double rpm){
    double setpoint =  rpm / gearRatio;
    canPidController.setReference(setpoint, ControlType.kSmartVelocity);
  }

  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Intake RPM", getRPM());
  }
  @Override
  public void periodic() {
    //updateSmartDashboard();
  }
}
