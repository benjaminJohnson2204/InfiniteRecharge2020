
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.*;
public class Intake extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private CANSparkMax intakeMotor = new CANSparkMax(Constants.intake, CANSparkMaxLowLevel.MotorType.kBrushless);

  public Intake() {
      intakeMotor.restoreFactoryDefaults();
      intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }
  public void setIntake(double value){
    intakeMotor.set(value);
  }
  public void intakeForward(){
    setIntake(.5);
  }
  public void intakeBackward(){
    setIntake(-.5);
  }
  public void stop(){
    setIntake(0);
  }
  public double getIntakeVoltage(){
    return intakeMotor.getBusVoltage();
  }
  @Override
  public void periodic() {
  }
}
