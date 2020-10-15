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

  private double ledState = 0, // 0 = can't shoot at all, 1 = can only hit outer, 2 = can hit inner
  timeStep = 0.02; // seconds between calls to command (time delay)

  private ChassisSpeeds speeds; // contains initial straight and angular velocity of robot

  // Should be re-calculated based on the maximum amount of time the turret and shooter can take to get to any given position and RPM
  private double robotLinearVelocity, robotAngularVelocity, // Linear velocity is meters per second straight ahead, angular velocity is rotation per second calculated from differences between wheels
  robotInitialXPosition, robotInitialYPosition, initialHeading, // Current robot position and where it's facing on the field relative to x-axis

  robotPredictedXvel, robotPredictedYvel, // Predicted x and y components of robot velocity after delay
  deltaTheta, // Angle in radians that robot rotates during time to shoot
  targetTurretAngle, // Angle turret needs to rotate to in order for ball to hit target
  shooterBallMagnitude, // Magnitude of velocity shooter needs to shoot at in order for ball to hit target
  necessaryXYvel, // xy-velocity needed to hit target
  RPM, // Needed RPM to shoot

  angleToOuter, angleToInner, // x-y angles to inner & outer targets
  distanceToInnerTargetXY, // distance in meters from robot to inner target on xy-plane (field)
  xDistanceToInnerTarget, yDistanceToInnerTarget, // X and y distances to inner target
  xDistanceToOuterTarget, yDistanceToOuterTarget, // X and y distances to outer target
  distanceToOuterTargetXY; // distance in meters from robot to outer target on xy-plane (field)

  private Translation2d shooterBallVector; // xy components of shooter ball magnitude
  private Pose2d predictedPosition; // Where robot will be after time to shoot, including heading
  private double startTime; // Timestamp when command is called

  private double hexagonCenterCanHitHeight = Constants.outerTargetHeight - (2 * Constants.ballRadius) - (2 * Constants.ballTolerance); // Height of hexagon that center of ball must hit

  private double maxHorizontalRatioOuter = (Constants.ballRadius + Constants.ballTolerance) / (2 / Math.sqrt(3)) * hexagonCenterCanHitHeight; // Maximum horizontal for ball to go through outer target
  private double maxVerticalRatioOuter = hexagonCenterCanHitHeight / 2 / (Constants.ballRadius + Constants.ballTolerance);

  /**
   * Creates a new ExampleCommand.
   *
   * @param turret The subsystem used by this command.
   */

  public ShootOnTheMove(Turret turret, Shooter shooter, DriveTrain drivetrain, LED led) {
    m_turret = turret;
    m_shooter = shooter;
    m_drivetrain = drivetrain;
    m_led = led;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret, shooter, drivetrain);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp(); // Starting timer to run command
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speeds = m_drivetrain.getDriveTrainKinematics().toChassisSpeeds(m_drivetrain.getSpeeds()); // Getting straight and angular velocity of drivetrain

    // Separating into linear and angular components
    robotLinearVelocity = speeds.vxMetersPerSecond;
    robotAngularVelocity = speeds.omegaRadiansPerSecond;

    Pose2d position = m_drivetrain.getRobotPose(); // Getting robot's position and heading through odometry || later implement vision calibration

    // Separating position into x, y, and heading components, then adjusting based on offset and distance between navX and shooter
    initialHeading = position.getRotation().getRadians();
    robotInitialXPosition = position.getTranslation().getX() + Constants.navXToShooterDistance * Math.cos(Constants.navXToShooterAngle + initialHeading);
    robotInitialYPosition = position.getTranslation().getY() + Constants.navXToShooterDistance * Math.sin(Constants.navXToShooterAngle + initialHeading);

    deltaTheta = robotAngularVelocity * timeStep; // Calculating how much robot's heading will change during time to shoot
    predictedPosition = predictPosition(); // Storing robot's predicted position and heading in a variable

    xDistanceToInnerTarget = Constants.targetXPosition - predictedPosition.getTranslation().getX();
    yDistanceToInnerTarget = Constants.targetYPosition - predictedPosition.getTranslation().getY();
    xDistanceToOuterTarget = xDistanceToInnerTarget;
    yDistanceToOuterTarget = yDistanceToInnerTarget - Constants.targetOffset;
    distanceToInnerTargetXY = findDistance(xDistanceToInnerTarget, yDistanceToInnerTarget); // Find out how far away inner target is from robot || later implement with vision's function
    distanceToOuterTargetXY = findDistance(xDistanceToOuterTarget, yDistanceToOuterTarget); // Find out how far away outer target is from robot

    // Angles to inner & outer targets
    angleToInner = findAngle(xDistanceToInnerTarget, yDistanceToInnerTarget);
    angleToOuter = findAngle(xDistanceToOuterTarget, yDistanceToOuterTarget);

    // Calculating x and y components of velocity robot will have after time to shoot
    robotPredictedXvel = robotLinearVelocity * Math.cos(predictedPosition.getRotation().getRadians());
    robotPredictedYvel = robotLinearVelocity * Math.sin(predictedPosition.getRotation().getRadians());

    if (canGoThroughInnerTarget()) {
      ledState = 2;
      necessaryXYvel = distanceToInnerTargetXY * Constants.airResistanceCoefficient * Math.sqrt( (-Constants.g / 2) / (Constants.verticalTargetDistance - 
      distanceToInnerTargetXY * Math.tan(Constants.verticalShooterAngle)));
      shooterBallVector = new Translation2d( // velocity components that the shooter has to give to the ball in form (xVel, yVel)
        necessaryXYvel * Math.cos(angleToInner) - robotPredictedXvel,
        necessaryXYvel * Math.sin(angleToInner) - robotPredictedYvel);
    } else if (canGoThroughOuterTarget()){
      ledState = 1;
      necessaryXYvel = distanceToOuterTargetXY * Constants.airResistanceCoefficient * Math.sqrt( (-Constants.g / 2) / (Constants.verticalTargetDistance - 
      distanceToOuterTargetXY * Math.tan(Constants.verticalShooterAngle)));
      shooterBallVector = new Translation2d( // velocity components that the shooter has to give to the ball in form (xVel, yVel)
        necessaryXYvel * Math.cos(angleToOuter) - robotPredictedXvel,
        necessaryXYvel * Math.sin(angleToOuter) - robotPredictedYvel);
    } else {
      ledState = 0;
    }

    if (ledState != 0) {
      targetTurretAngle = findAngle(shooterBallVector.getX(), shooterBallVector.getY()); // Calculates angle turret needs to rotate to
      shooterBallMagnitude = findDistance(shooterBallVector.getX(), shooterBallVector.getY()) // Gets xy magnitude of velocity balls needs to be shot at
      / Math.cos(Constants.verticalShooterAngle); // Project that xy-velocity into 3 dimensions
      RPM = shooterBallMagnitude * 60 / (Constants.flywheelRadius * 2 * Math.PI); // Kind of works, probably a better way
      if (RPM > Constants.maxShooterRPM) {
        ledState = 0;
      } else {
        m_turret.setRobotCentricSetpoint(targetTurretAngle); // Setting turret to turn to angle
        m_turret.setControlMode(1); // Enabling turret to turn to setpoint
        m_shooter.setRPM(RPM); // Spin the shooter to shoot the ball
        m_led.setState(ledState == 2 ? 10 : 11);
      }
    } else {
      m_led.setState(2); // Solid red, unable to shoot
    } 
    updateSmartDashboard();
  }

  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Predicted heading", predictedPosition.getRotation().getRadians());
    SmartDashboard.putNumber("Predicted x position", predictedPosition.getTranslation().getX());
    SmartDashboard.putNumber("Predicted y position", predictedPosition.getTranslation().getY());

    if (ledState == 2) {
      SmartDashboard.putBoolean("Can shoot", true);
      SmartDashboard.putString("Target", "Inner");
      SmartDashboard.putNumber("xy Distance to inner target", distanceToInnerTargetXY);
      SmartDashboard.putNumber("xy Angle to inner target", angleToInner);
    } else if (ledState == 1) {
      SmartDashboard.putBoolean("Can shoot", true);
      SmartDashboard.putString("Target", "Outer");
      SmartDashboard.putNumber("xy Distance to outer target", distanceToOuterTargetXY);
      SmartDashboard.putNumber("xy Angle to outer target", angleToOuter);
    } else {
      SmartDashboard.putBoolean("Can shoot", false);
      SmartDashboard.putString("Target", "None");
    }

    SmartDashboard.putNumber("Shooting at speed", shooterBallMagnitude);
    SmartDashboard.putNumber("Revving shooter to RPM", RPM);
    SmartDashboard.putNumber("Rotating turret to angle", targetTurretAngle);
  }

  private double findDistance(double deltaX, double deltaY) { // Using Pythagorean
    return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
  } 

  private double findAngle(double deltaX, double deltaY){ // Uses arctangent but uses signs to determine quadrant
    if (Math.abs(deltaX) <= 0.01) {
      return deltaY >= 0 ? Math.PI / 2 : - Math.PI / 2;
    }
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

    // Calculating x and y position after delay using linear and angular velocities over time using trigonometry from center of rotation
    return new Pose2d(
      robotInitialXPosition + radius * (Math.sin(initialHeading + deltaTheta) - Math.sin(initialHeading)), // predicted x position
      robotInitialYPosition - radius * (Math.cos(initialHeading + deltaTheta) - Math.cos(initialHeading)), // predicted y position
      new Rotation2d(initialHeading  + deltaTheta) // predicted heading
    );
  }

  private boolean canGoThroughInnerTarget() {
    double tangentOfFinalAngle = Math.abs(2 * Constants.verticalTargetDistance / distanceToInnerTargetXY - Math.tan(Constants.verticalShooterAngle)); 
    return tangentOfFinalAngle <= hexagonCenterCanHitHeight / Constants.targetOffset / 2 && // Vertical angle alone is fine
    tangentOfFinalAngle <= - Math.sqrt(3) * Constants.targetOffset / Math.abs(yDistanceToInnerTarget / xDistanceToInnerTarget) + hexagonCenterCanHitHeight; // Within sloped hexagon lines
  }

  private boolean canGoThroughOuterTarget() {
  double verticalTargetIntersection = Math.abs(2 * Constants.verticalTargetDistance / distanceToOuterTargetXY - Math.tan(Constants.verticalShooterAngle)) * Constants.ballRadius;
  return verticalTargetIntersection <= hexagonCenterCanHitHeight / 2 && 
  verticalTargetIntersection <= -Math.sqrt(3) * Math.abs(Constants.ballRadius * xDistanceToOuterTarget / yDistanceToOuterTarget) + hexagonCenterCanHitHeight;
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.setControlMode(0); // Disabling turret turning to setpoint
    m_turret.setRobotCentricSetpoint(0); // Reset turret setpoint
      // TODO: Find out if we need to reset setpoint
    m_shooter.setRPM(0); // Stopping shooter
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Fix this, not sure how long command should run for
  }
}
