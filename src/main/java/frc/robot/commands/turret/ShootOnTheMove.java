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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  // Should be re-calculated based on the maximum amount of time the turret and shooter can take to get to any given position and RPM
  private double timeToShoot = 2; // Time from when command is called to when ball leaves robot
  private double robotLinearVelocity, robotAngularVelocity, // Linear velocity is meters per second straight ahead, angular velocity is rotation per second calculated from differences between wheels
  robotInitialXPosition, robotInitialYPosition, initialHeading, // Current robot position and where it's facing on the field relative to x-axis

  robotPredictedXvel, robotPredictedYvel, // Predicted x and y components of robot velocity after delay
  deltaTheta, // Angle in radians that robot rotates during time to shoot
  distanceToInnerTargetXY, // distance in meters from robot to target on xy-plane (field)
  targetTurretAngle, // Angle turret needs to rotate to in order for ball to hit target
  shooterBallMagnitude, // Magnitude of velocity shooter needs to shoot at in order for ball to hit target
  necessaryXYvel, // xy-velocity needed to hit target
  xDistanceToInnerTarget, yDistanceToInnerTarget, // X and y distances to inner target
  distanceToOuterTargetXY;

  private Translation2d shooterBallVector; // xy components of shooter ball magnitude
  private Pose2d predictedPosition; // Where robot will be after time to shoot, including heading
  private double startTime; // Timestamp when command is called

  private double hexagonCenterCanHitHeight = Constants.outerTargetHeight - (2 * Constants.ballRadius) - (2 * Constants.ballTolerance); // Height of hexagon that center of ball must hit
  private double maxRatioWithWall = (Constants.ballRadius + Constants.ballTolerance) / (2 / Math.sqrt(3)) * hexagonCenterCanHitHeight; // Maximum ratio/angle for ball to go through outer target

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

    speeds = new ChassisSpeeds(3, 0, Math.PI / 12);// m_drivetrain.getDriveTrainKinematics().toChassisSpeeds(m_drivetrain.getSpeeds()); // Getting straight and angular velocity of drivetrain

    // Separating into linear and angular components
    robotLinearVelocity = speeds.vxMetersPerSecond;
    robotAngularVelocity = speeds.omegaRadiansPerSecond;

    Pose2d position = new Pose2d(new Translation2d(3, 10), new Rotation2d(Math.PI / 6))// m_drivetrain.getRobotPose(); // Getting robot's position and heading through odometry || later implement vision calibration

    // Separating position into x, y, and heading components, then adjusting based on offset and distance between navX and shooter
    initialHeading = position.getRotation().getRadians();
    robotInitialXPosition = position.getTranslation().getX() + Constants.navXToShooterDistance * Math.cos(Constants.navXToShooterAngle + initialHeading);
    robotInitialYPosition = position.getTranslation().getY() + Constants.navXToShooterDistance * Math.sin(Constants.navXToShooterAngle + initialHeading);
    
    deltaTheta = robotAngularVelocity * timeToShoot; // Calculating how much robot's heading will change during time to shoot
    SmartDashboard.putNumber("Change in angle", deltaTheta);
    predictedPosition = predictPosition(); // Storing robot's predicted position and heading in a variable
    xDistanceToInnerTarget = Constants.targetXPosition - predictedPosition.getTranslation().getX();
    yDistanceToInnerTarget = Constants.targetYPosition - predictedPosition.getTranslation().getY();
    distanceToInnerTargetXY = findDistance(xDistanceToInnerTarget, yDistanceToInnerTarget); // Find out how far away inner target is from robot || later implement with vision's function
    SmartDashboard.putNumber("XY Distance to inner target", distanceToInnerTargetXY);
    distanceToOuterTargetXY = findDistance(xDistanceToInnerTarget, yDistanceToInnerTarget - Constants.targetOffset); // Find out how far away outer target is from robot

    // Calculating x and y components of velocity robot will have after time to shoot
    robotPredictedXvel = robotLinearVelocity * Math.cos(predictedPosition.getRotation().getRadians());
    robotPredictedYvel = robotLinearVelocity * Math.sin(predictedPosition.getRotation().getRadians());

    shooterBallVector = calculateShootXYVelocityComponents(predictedPosition, true); // Calculating xy velocity components ball needs to be shot at
    if (!canGoThroughInnerTarget()) { // If ball can't go through inner target, aim for outer target
      SmartDashboard.putBoolean("Able to hit inner target", false);
      shooterBallVector = calculateShootXYVelocityComponents(predictedPosition, false); // Re-calculating xy velocity based on aiming for outer target
      canShoot = canGoThroughOuterTarget(); // Checking if ball can go through outer target
    }

    targetTurretAngle = findAngle(shooterBallVector.getX(), shooterBallVector.getY()); // Calculates angle turret needs to rotate to
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
    double robotPredictedHeading = initialHeading  + deltaTheta; // Heading of robot after time delay
    SmartDashboard.putNumber("Predicted heading", robotPredictedHeading);
    double predictedX = robotInitialXPosition + radius * (Math.sin(initialHeading + deltaTheta) - Math.sin(initialHeading));
    double predictedY = robotInitialYPosition - radius * (Math.cos(initialHeading + deltaTheta) - Math.cos(initialHeading));
    SmartDashboard.putNumber("Predicted x position", predictedX);
    SmartDashboard.putNumber("Predicted y position", predictedY);
    return new Pose2d(predictedX, predictedY, new Rotation2d(robotPredictedHeading)); // Returning x, y, and heading
  }

  private Translation2d calculateShootXYVelocityComponents(Pose2d predictedPositionValues, boolean aimingForInner) { // Calculating xy velocity the shooter needs to give the ball
    
    // Calculating xy-magnitude of total velocity ball needs to hit target (including what it gets from robot)
    necessaryXYvel = (aimingForInner ? distanceToInnerTargetXY : distanceToOuterTargetXY) * Constants.airResistanceCoefficient * Math.sqrt( (-Constants.g / 2) / (Constants.verticalTargetDistance - 
    (aimingForInner ? distanceToInnerTargetXY : distanceToOuterTargetXY) * Math.tan(Constants.verticalShooterAngle)));

    double predictedAngle = findAngle(xDistanceToInnerTarget, yDistanceToInnerTarget - (aimingForInner ? 0 : Constants.targetOffset)); // xy Angle from robot to target
    SmartDashboard.putNumber("XY angle to target", predictedAngle);

    return new Translation2d(
        necessaryXYvel * Math.cos(predictedAngle) - robotPredictedXvel,
        necessaryXYvel * Math.sin(predictedAngle) - robotPredictedYvel);
  } // return the velocity components that the shooter has to give to the ball in form (xVel, yVel)

  private double velocityToRPM(double velocity){ // Converts velocity ball needs to be shot at to RPM of shooter
    double flywheelRadius = 0.1; // Meters
    double RPM = velocity * 60 / (flywheelRadius * 2 * Math.PI); // Kind of works, probably a better way
    canShoot = RPM <= Constants.maxShooterRPM;
    SmartDashboard.putBoolean("Able to shoot", canShoot);
    return RPM;
  }

  private boolean canGoThroughInnerTarget() {
    double hitsTargetAt = Math.abs(((-Constants.g * distanceToInnerTargetXY) / necessaryXYvel + necessaryXYvel * Math.tan(Constants.verticalShooterAngle)) / necessaryXYvel); // How high up ball hits outer target
    boolean withinHeight = hitsTargetAt <= hexagonCenterCanHitHeight / 2 / Constants.targetOffset; // Vertical angle is within acceptable range

    // Vertical angle as a function of horizontal angle is acceptable
    boolean withinHexagonSlope = hitsTargetAt <= -Math.sqrt(3) * Constants.targetOffset * Math.abs(xDistanceToInnerTarget / yDistanceToInnerTarget) + hexagonCenterCanHitHeight;
    return withinHeight && withinHexagonSlope; // Both angles must be accpetable
  }

  private boolean canGoThroughOuterTarget() {
    return (yDistanceToInnerTarget - Constants.targetOffset) / xDistanceToInnerTarget >= // Tangent of angle to target
    maxRatioWithWall; // Maximum angle tangent without ball hitting outer wall
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp(); // Starting timer to run command
    SmartDashboard.putNumber("Velocity", shooterBallMagnitude);
    SmartDashboard.putNumber("RPM", velocityToRPM(shooterBallMagnitude));
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
