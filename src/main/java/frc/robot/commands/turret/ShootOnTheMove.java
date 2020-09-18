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
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Turret m_turret;
  private final Shooter m_shooter;
  private final DriveTrain m_drivetrain;

  private ChassisSpeeds speeds;
  private double timeToShoot = 2;
  private double robotLinearVelocity, robotAngularVelocity, robotInitialXPosition, robotInitialYPosition, initialHeading, 
  deltaTheta, distanceToTarget, xyPlaneAngleToShoot, totalLaunchVelocity, shooterMagnitude, rotateTurretTo;
  private Translation2d shooterVelVector, shootBallTo;
  private Pose2d predictedPosition;
  private double startTime;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public ShootOnTheMove(Turret turret, Shooter shooter, DriveTrain drivetrain) {
    m_turret = turret;
    m_shooter = shooter;
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret, shooter, drivetrain);
    speeds = m_drivetrain.getDriveTrainKinematics().toChassisSpeeds(m_drivetrain.getSpeeds());
    robotLinearVelocity = speeds.vxMetersPerSecond;
    robotAngularVelocity = speeds.vyMetersPerSecond;
    Pose2d position = m_drivetrain.getRobotPose();
    robotInitialXPosition = position.getTranslation().getX();
    robotInitialYPosition = position.getTranslation().getY();
    initialHeading = position.getRotation().getRadians();

    deltaTheta = robotAngularVelocity * timeToShoot;
    predictedPosition = predictPosition();
    distanceToTarget = findDistance(Constants.targetXPosition - predictedPosition.getTranslation().getX(), Constants.targetYPosition - predictedPosition.getTranslation().getY());
    shooterVelVector = predictParabola(predictedPosition);
    
    xyPlaneAngleToShoot = findAngle(shooterVelVector.getY(), shooterVelVector.getX());
    totalLaunchVelocity = findDistance(shooterVelVector.getX(), shooterVelVector.getY()) / Math.cos(Constants.verticalShooterAngle);
  }

  private double findDistance(double deltaX, double deltaY) {
    return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
  } 

  private double findAngle(double deltaX, double deltaY){
    double tangent = Math.atan(deltaY / deltaX);
    if (deltaX >= 0) {
      return tangent;
    } else if (deltaY < 0) {
      return tangent - Math.PI;
    } else {
      return tangent + Math.PI;
    }
  }

  private Pose2d predictPosition() {
    double radius = robotLinearVelocity / robotAngularVelocity;
    double predictedX = robotInitialXPosition + radius * (Math.sin(initialHeading - deltaTheta) - Math.sin(initialHeading));
    double predictedY = robotInitialYPosition + radius * (Math.cos(initialHeading - deltaTheta) - Math.cos(initialHeading));
    double robotPredictedHeading = Math.PI / 2 - (initialHeading + deltaTheta);
    return new Pose2d(predictedX, predictedY, new Rotation2d(robotPredictedHeading));
  }

  private Translation2d predictParabola(Pose2d values) {
    double robotPredictedXvel = robotLinearVelocity * Math.cos(values.getRotation().getRadians());
    double robotPredictedYvel = robotLinearVelocity * Math.sin(values.getRotation().getRadians());
    double necessaryXYvel = distanceToTarget * Math.sqrt( -Constants.g / (Constants.verticalTargetDistance - distanceToTarget * Math.tan(Constants.verticalShooterAngle)));
    double predictedAngle = findAngle(values.getTranslation().getX(), values.getTranslation().getY());
    return new Translation2d(
        necessaryXYvel * Math.cos(predictedAngle) - robotPredictedXvel,
        necessaryXYvel * Math.sin(predictedAngle) - robotPredictedYvel);
  } // return the velocity components that the shooter has to give to the ball in form (Vel, Xvel)

  private double velocityToRPM(double velocity){
        return velocity; // Update this
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    shootBallTo = predictParabola(predictedPosition);
    shooterMagnitude = findDistance(shootBallTo.getY(), shootBallTo.getX()) / Math.cos(Constants.verticalShooterAngle);
    rotateTurretTo = findAngle(shootBallTo.getX(), shootBallTo.getY());
    m_turret.setControlMode(1);
    m_turret.setRobotCentricSetpoint(rotateTurretTo);
    m_shooter.setRPM(velocityToRPM(shooterMagnitude));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.setControlMode(0);
    m_turret.setRobotCentricSetpoint(0);
    m_shooter.setRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() >= startTime + timeToShoot;
  }
}
