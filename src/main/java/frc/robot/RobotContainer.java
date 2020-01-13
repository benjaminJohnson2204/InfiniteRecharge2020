/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.BackwardIntake;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ForwardIntake;
import frc.robot.commands.setTankDrive;
import frc.robot.constants.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Climber m_climber = new Climber();
  private final ControlPanel m_controlPanel = new ControlPanel();
  private final Controls m_controls = new Controls();
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Indexer m_indexer = new Indexer();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Skyhook m_skyhook = new Skyhook();
  private final Vision m_vision = new Vision();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  static Joystick leftJoystick = new Joystick(Constants.leftJoystick);
  Button[] leftButtons = new Button[leftJoystick.getButtonCount()];

  static Joystick rightJoystick = new Joystick(Constants.rightJoystick);
  Button[] rightButtons = new Button[rightJoystick.getButtonCount()];
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_driveTrain.setDefaultCommand(new setTankDrive(m_driveTrain));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public static double getLeftJoystickY() {
    return leftJoystick.getY();
  }
  public static double getRightJoystickY() {
    return rightJoystick.getY();
  }
  private void configureButtonBindings() {
    for(int i = 0; i < leftButtons.length; i++){
      leftButtons[i] = new JoystickButton(leftJoystick, i + 1);
    }
    for(int i = 0; i < rightButtons.length; i++){
      rightButtons[i] = new JoystickButton(rightJoystick, i + 1);
    }

    leftButtons[0].whileHeld(new ForwardIntake(m_intake));
    rightButtons[0].whileHeld(new BackwardIntake(m_intake));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
