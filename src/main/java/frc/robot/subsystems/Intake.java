
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private TalonSRX intakeMotor = new TalonSRX(Constants.intake);

  public Intake() {
      intakeMotor.configFactoryDefault();
      intakeMotor.setNeutralMode(NeutralMode.Coast);
  }
  public void setIntake(float value){
    intakeMotor.set(ControlMode.PercentOutput, value);
  }
  public void intake(){
    setIntake(1);
  }
  public void stop(){
    setIntake(0);
  }
  public double getIntakeVoltage(){
    return intakeMotor.getMotorOutputVoltage();
  }
  @Override
  public void periodic() {
  }
}
