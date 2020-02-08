/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.LED.LEDCommand;
import frc.robot.commands.drivetrain.SetDriveShifters;
import frc.robot.commands.indexer.ToggleIndexerControlMode;
import frc.robot.commands.intake.SetIntakeManual;
import frc.robot.commands.shooter.SetShooterManual;
import frc.robot.commands.turret.SetTurretSetpointFieldAbsolute;
import frc.robot.commands.turret.ToggleTurretControlMode;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.autonomous.TestPathFollowing;
import frc.robot.commands.drivetrain.SetArcadeDrive;
import frc.robot.commands.drivetrain.ZeroDriveTrainEncoders;
import frc.robot.commands.indexer.IndexerCommand;
import frc.robot.commands.skyhook.SetSkyhookOutput;
import frc.robot.commands.turret.ZeroTurretEncoder;
import frc.robot.commands.vision.AlignToOuterPort;
import frc.robot.constants.Constants;
import frc.vitruvianlib.utils.JoystickWrapper;
import frc.vitruvianlib.utils.XBoxTrigger;

import java.util.Map;

import static java.util.Map.entry;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Climber m_climber = new Climber();
  private final Controls m_controls = new Controls();
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Skyhook m_skyhook = new Skyhook();
  private final Turret m_turret = new Turret(m_driveTrain);
  private final Vision m_vision = new Vision();
  public final Indexer m_indexer = new Indexer();
  private final LED m_led = new LED();

  static JoystickWrapper leftJoystick = new JoystickWrapper(Constants.leftJoystick);
  static JoystickWrapper rightJoystick = new JoystickWrapper(Constants.rightJoystick);
  static JoystickWrapper xBoxController = new JoystickWrapper(Constants.xBoxController);
  public Button[] leftButtons = new Button[7];
  public Button[] rightButtons = new Button[7];
  public Button[] xBoxButtons = new Button[10];
  public Button[] xBoxPOVButtons = new Button[8];
  public Button xBoxLeftTrigger, xBoxRightTrigger;

  private enum CommandSelector {
    DRIVE_STRAIGHT
  }

  SendableChooser<Integer> m_autoChooser = new SendableChooser();
  private SelectCommand m_autoCommand = new SelectCommand(
    Map.ofEntries(
      entry(CommandSelector.DRIVE_STRAIGHT, new TestPathFollowing(m_driveTrain))
    ),
    this::selectCommand
  );

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   * 
   */
  public RobotContainer() {
    m_autoChooser.addDefault("Drive Straight", CommandSelector.DRIVE_STRAIGHT.ordinal());
    for(Enum commandEnum : CommandSelector.values())
      if (commandEnum != CommandSelector.DRIVE_STRAIGHT)
        m_autoChooser.addOption(commandEnum.toString(), commandEnum.ordinal());

    SmartDashboard.putData(m_autoChooser);

    initializeSubsystems();
    // Configure the button bindings
    configureButtonBindings();
  }

  public void initializeSubsystems() {
    m_driveTrain.setDefaultCommand(new SetArcadeDrive(m_driveTrain,
            () -> leftJoystick.getRawAxis(1), () -> rightJoystick.getRawAxis(0)));
    CommandScheduler.getInstance().schedule(new ZeroDriveTrainEncoders(m_driveTrain));

//    m_intake.setDefaultCommand(new SetIntake(m_intake));
    m_indexer.setDefaultCommand(new IndexerCommand(m_indexer));

    m_turret.setDefaultCommand(new SetTurretSetpointFieldAbsolute(m_turret, m_driveTrain, m_vision,
            () -> xBoxController.getRawAxis(0),
            () -> xBoxController.getRawAxis(1)));
    m_skyhook.setDefaultCommand(new SetSkyhookOutput(m_skyhook, () -> xBoxController.getRawAxis(0)));
    m_led.setDefaultCommand(new LEDCommand(m_led));

    m_vision.initUSBCamera();
    m_vision.openSightInit();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    leftJoystick.invertRawAxis(1, false);
    rightJoystick.invertRawAxis(0, true);
    xBoxController.invertRawAxis(1, true);
    xBoxController.invertRawAxis(5, true);
    for (int i = 0; i < leftButtons.length; i++)
      leftButtons[i] = new JoystickButton(leftJoystick, (i + 1));
    for (int i = 0; i < rightButtons.length; i++)
      rightButtons[i] = new JoystickButton(rightJoystick, (i + 1));
    for (int i = 0; i < xBoxButtons.length; i++)
      xBoxButtons[i] = new JoystickButton(xBoxController, (i + 1));
    for (int i = 0; i < xBoxPOVButtons.length; i++)
      xBoxPOVButtons[i] = new POVButton(xBoxController, (i * 45));
    xBoxLeftTrigger = new XBoxTrigger(xBoxController, 2);
    xBoxRightTrigger = new XBoxTrigger(xBoxController, 3);

    leftButtons[0].whileHeld(new SetDriveShifters(m_driveTrain, true)); //Top (left) Button - Switch to high gear
    leftButtons[1].whileHeld(new SetDriveShifters(m_driveTrain, false)); //Bottom (right) Button - Switch to low gear

    rightButtons[0].whenPressed(new AlignToOuterPort(m_driveTrain, m_vision)); //Top (left) Button - Shoot power cells (kicker)
    //rightButtons[1].whenPressed(new Command()); //Bottom (right) Button - Turn to powercells (Automated vision targeting

    xBoxLeftTrigger.whileHeld(new SetIntakeManual(m_intake, m_indexer)); // Deploy intake
    xBoxButtons[4].whileHeld(new SetShooterManual(m_shooter, m_indexer));
    //xBoxLeftTrigger.whileHeld(new Command()); // Run Intake Motors
    //xBoxRightTrigger.whenPressed(new Command()); //flywheel on toggle
    //xBoxButtons[0].whenPressed(new Command()); //A - toggle driver climb mode
    //xBoxButtons[1].whenPressed(new Command()); //B - manual eject
    //xBoxButtons[2].whenPressed(new Command()); //X - manual move uptake
    //xBoxButtons[3].whenPressed(new Command()); //Y -
    //xBoxButtons[4].whileHeld(new Command()); //left bumper - winch up
    //xBoxButtons[5].whileHeld(new Command()); //right bumper - winch down
    xBoxButtons[6].whenPressed(new ToggleTurretControlMode(m_turret)); //start - toggle control mode turret
    xBoxButtons[7].whenPressed(new ToggleIndexerControlMode(m_indexer)); //select - toggle control mode uptake
    //xBoxButtons[8].whenPressed(new Command()); //left stick
    //xBoxButtons[9].whenPressed(new Command()); //right stick

    xBoxPOVButtons[4].whenPressed(new ZeroTurretEncoder(m_turret));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  private CommandSelector selectCommand() {
    return CommandSelector.values()[m_autoChooser.getSelected()];
  }
  public Command getAutonomousCommand() {
    return m_autoCommand;
  }

  public void robotPeriodic() {

  }

  public void teleOpInit() {
    m_driveTrain.resetEncoderCounts();
    m_driveTrain.resetOdometry(new Pose2d(), new Rotation2d());
  }
  public void teleOpPeriodic() {
  }
  public void autonomousInit() {
  }
  public void autonomousPeriodic(){
  }
}
