
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
<<<<<<< HEAD
import frc.robot.Constants;
import com.revrobotics.*;
=======
import frc.robot.constants.Constants;

>>>>>>> master
public class Intake extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

<<<<<<< HEAD
  private CANSparkMax intakeMotor = new CANSparkMax(Constants.intake, CANSparkMaxLowLevel.MotorType.kBrushless);
=======
  private TalonSRX intakeMotor = new TalonSRX(Constants.intakeMotor);
>>>>>>> master

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
