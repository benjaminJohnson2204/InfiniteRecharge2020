
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private TalonSRX[] intakeMotors = {
          new TalonSRX(30) // subject to change, 30 could be changed to whatever
          //add more motors here if needed
  }
  public Intake(){
    for(TalonSRX intakeMotors:intakeMotor){
      intakeMotors.configFactoryDefault();
      intakeMotor.setNeutalMode(NeutralMode.Coast)
    }
    //invert & follow motors here
  }
  funtion intakeSwitch = (bleh) =>{
    if(bleh==true){
      intakeMotor.value = vroom
    } else {
      if(bleh == false){
        intakemototr.value = sSREeeeeeeeeeeeqkkek
      }
//    }else{
//      return("hecc")
    }
  }

  @Override
  public void periodic() {