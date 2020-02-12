
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private CANSparkMax intakeMotor[] ={
        new CANSparkMax(Constants.intakeMotor, MotorType.kBrushless),
  };

  DoubleSolenoid intakePiston = new DoubleSolenoid(Constants.pcmOne, Constants.intakePistonForward, Constants.intakePistonReverse);

  public boolean getintakePistonExtendStatus(){
    return intakePiston.get() == DoubleSolenoid.Value.kForward ? true : false;
  }

  public void setintakePiston(boolean state){
    intakePiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  public Intake() {
    for(CANSparkMax intakeMotor: intakeMotor) {
      intakeMotor.restoreFactoryDefaults();
      intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }
    intakeMotor[0].setInverted(false);
  }

  public void setIntake(double value){
    intakeMotor[0].set(value);
  }

  @Override
  public void periodic() {
  }
}
