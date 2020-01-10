/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  TalonSRX master = new TalonSRX(Constants.indexer);

  public Indexer() {

  }

  public void setOutput(double output){
    master.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
