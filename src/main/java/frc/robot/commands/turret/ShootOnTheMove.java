/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.constants.Constants;
import java.lang.Math;

import java.util.function.DoubleSupplier;

/*
static double robotInitialXPosition = 1.7;
    static double robotInitialYPosition = 1.7;
    static double intialHeading = 1.7;
    static double targetXPosition = 1.7;
    static double targetYPosition = 1.7;

  */
/**
 * An example command that uses an example subsystem.
 */
public class ShootOnTheMove extends CommandBase {

  /*
  Project Doc:
    https://docs.google.com/document/d/1ksk5DAHCi1Ag2KrfKTEo95SJ_jbmWwSPbHlM2ZlTe-o/edit?usp=sharing
  */

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Turret m_turret;
  private final Shooter m_shooter;
  private final DriveTrain m_drivetrain;
  private final LED m_led;

  private boolean canShoot; // If we can shoot

  private ChassisSpeeds speeds; // contains initial straight and angular velocity of robot
  private double timeToShoot = 2; // Time from when command is called to when ball leaves robot
  private double robotLinearVelocity, robotAngularVelocity, // Linear velocity is meters per second straight ahead, angular velocity is rotation per second calculated from differences between wheels
  robotInitialXPosition, robotInitialYPosition, initialHeading, // Current robot position and where it's facing on the field relative to x-axis
    // TODO: Figure out how to convert robot position to shooter position

  deltaTheta, // Angle in radians that robot rotates during time to shoot
  distanceToTargetXY, // distance in meters from robot to target on xy-plane (field)
  targetTurretAngle, // Angle turret needs to rotate to in order for ball to hit target
  shooterBallMagnitude; // Magnitude of velocity shooter needs to shoot at in order for ball to hit target

  private Translation2d shooterBallVector; // xy components of shooter ball magnitude
  private Pose2d predictedPosition; // Where robot will be after time to shoot, including heading
  private double startTime; // Timestamp when command is called

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public ShootOnTheMove(Turret turret, Shooter shooter, DriveTrain drivetrain, LED led) {
    m_turret = turret;
    m_shooter = shooter;
    m_drivetrain = drivetrain;
    m_led = led;

    canShoot = true;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret, shooter, drivetrain);

    speeds = m_drivetrain.getDriveTrainKinematics().toChassisSpeeds(m_drivetrain.getSpeeds()); // Getting straight and angular velocity of drivetrain

    // Separating into linear and angular components
    robotLinearVelocity = speeds.vxMetersPerSecond;
    robotAngularVelocity = speeds.omegaRadiansPerSecond;

    Pose2d position = m_drivetrain.getRobotPose(); // Getting robot's position and heading through odometry || later implement vision calibration

    // Separating position into x, y, and heading components
    robotInitialXPosition = position.getTranslation().getX();
    robotInitialYPosition = position.getTranslation().getY();
    initialHeading = position.getRotation().getRadians();

    deltaTheta = robotAngularVelocity * timeToShoot; // Calculating how much robot's heading will change during time to shoot
    predictedPosition = predictPosition(); // Storing robot's predicted position and heading in a variable
    distanceToTargetXY = findDistance(Constants.targetXPosition - predictedPosition.getTranslation().getX(), 
    Constants.targetYPosition - predictedPosition.getTranslation().getY()); // Find out how far away target is from robot || later implement with vision's function
    shooterBallVector = calculateShootXYVelocityComponents(predictedPosition); // Calculating xy velocity components ball needs to be shot at
    
    targetTurretAngle = findAngle(shooterBallVector.getY(), shooterBallVector.getX()); // Calculates angle turret needs to rotate to
    shooterBallMagnitude = findDistance(shooterBallVector.getX(), shooterBallVector.getY()) // Gets xy magnitude of velocity balls needs to be shot at
     / Math.cos(Constants.verticalShooterAngle); // Project that xy-velocity into 3 dimensions
  }

  private double findDistance(double deltaX, double deltaY) { // Using Pythagorean
    return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
  } 

  private double findAngle(double deltaX, double deltaY){ // Uses arctangent but uses signs to determine quadrant
    double tangent = Math.atan(deltaY / deltaX); // Raw value in radians
      // 0 degrees is on the right (positive x-axis)
    if (deltaX >= 0) {
      return tangent; // Positive x; between -90 and 90
    } else if (deltaY < 0) {
      return tangent - Math.PI; // Negative x, negative y; between -180 and -90
    } else {
      return tangent + Math.PI; // Negative x, positive y; between 90 and 180
    }
  }

  private Pose2d predictPosition() { // Predicting position and heading of robot after time delay
    double radius = robotLinearVelocity / robotAngularVelocity; // Calculating distance from robot's position and center of robot's rotation

    // Calculating x and y position after delay using linear and angular velocities over time to shoot using trigonometry from center of rotation
    double predictedX = robotInitialXPosition + radius * (Math.sin(initialHeading - deltaTheta) - Math.sin(initialHeading));
    double predictedY = robotInitialYPosition + radius * (Math.cos(initialHeading - deltaTheta) - Math.cos(initialHeading));

    double robotPredictedHeading = Math.PI / 2 - (initialHeading + deltaTheta); // Heading of robot after time delay
    return new Pose2d(predictedX, predictedY, new Rotation2d(robotPredictedHeading)); // Returning x, y, and heading
  }

  private Translation2d calculateShootXYVelocityComponents(Pose2d values) { // Calculating xy velocity the shooter needs to give the ball
    // Calculating x and y components of velocity robot will have after time to shoot
    double robotPredictedXvel = robotLinearVelocity * Math.cos(values.getRotation().getRadians());
    double robotPredictedYvel = robotLinearVelocity * Math.sin(values.getRotation().getRadians());

    // Calculating xy-magnitude of total velocity ball needs to hit target (including what it gets from robot)
    double necessaryXYvel = Constants.airResistanceCoefficient * Math.sqrt( -Constants.g / (Constants.verticalTargetDistance - distanceToTargetXY * Math.tan(Constants.verticalShooterAngle)));

    double predictedAngle = findAngle(values.getTranslation().getX(), values.getTranslation().getY()); // xy Angle from robot to target

    return new Translation2d(
        necessaryXYvel * Math.cos(predictedAngle) - robotPredictedXvel,
        necessaryXYvel * Math.sin(predictedAngle) - robotPredictedYvel);
  } // return the velocity components that the shooter has to give to the ball in form (xVel, yVel)

  private double velocityToRPM(double velocity){ // Converts velocity ball needs to be shot at to RPM of shooter
    double flywheelRadius = 0.1; // Meters
    double RPM = velocity * 60 / (flywheelRadius * 2 * Math.PI); // Kind of works, probably a better way
    canShoot = RPM <= Constants.maxShooterRPM;
    return RPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp(); // Starting timer to run command
    
    m_turret.setRobotCentricSetpoint(targetTurretAngle); // Setting turret to turn to angle
    m_turret.setControlMode(1); // Enabling turret to turn to setpoint
    m_shooter.setRPM(velocityToRPM(shooterBallMagnitude)); // Spin the shooter to shoot the ball
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (canShoot && !interrupted) {
    m_turret.setControlMode(0); // Disabling turret turning to setpoint
    m_turret.setRobotCentricSetpoint(0); // Reset turret setpoint
      // TODO: Find out if we need to reset setpoint
    m_shooter.setRPM(0); // Stopping shooter
    } else {
      m_led.setState(10); // Sets LED to orange (error message)
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !canShoot || Timer.getFPGATimestamp() >= startTime + timeToShoot; // Find if time for command is over
  }
}
