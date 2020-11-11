/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.*;


/**
 * An example command that uses an example subsystem.
 */
public class ShootOnTheMove extends CommandBase {

  /*
  Project Doc:
    https://docs.google.com/document/d/1ksk5DAHCi1Ag2KrfKTEo95SJ_jbmWwSPbHlM2ZlTe-o/edit?usp=sharing

  Slides:
    https://docs.google.com/presentation/d/1hh_Wb-yfTFJWq16qWlhEl8ExGFHcJ4ZX2CQzLz0vmVg/edit?usp=sharing 
  */

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Turret m_turret;
    private final Shooter m_shooter;
    private final DriveTrain m_drivetrain;
    private final LED m_led;
    private final Vision m_vision;
    private final double hexagonCenterCanHitHeight = Constants.outerTargetHeight - (2 * Constants.ballRadius) - (2 * Constants.ballTolerance); // Height of hexagon that center of ball must hit
    private boolean isRunning; // Whether to run code
    private double ledState = 0; // 0 = can't shoot at all, 1 = can only hit outer, 2 = can hit inner
    private double initialHeading; // Current robot position and where it's facing on the field relative to x-axis
    private double deltaTheta; // Angle in radians that robot rotates during time to shoot
    private double targetTurretAngle; // Angle turret needs to rotate to in order for ball to hit target
    private double shooterBallMagnitude; // Magnitude of velocity shooter needs to shoot at in order for ball to hit target
    private double RPM; // Needed RPM to shoot
    private double angleToOuter;
    private double angleToInner; // x-y angles to inner & outer targets
    private double currentDistanceToOuterTargetXY;
    private double currentAngleToOuter;
    private double distanceToOuterTargetXY; // predicted distance in meters from robot to outer target
    private double distanceToInnerTargetXY; // predicted distance in meters from robot to inner target on xy-plane (field)ot to outer target on xy-plane (field)
    private double startTime; // the time it took to get to initialize
    private Translation2d shooterBallVector; // xy components of shooter ball magnitude

//  private final double maxHorizontalRatioOuter = (Constants.ballRadius + Constants.ballTolerance) / (2 / Math.sqrt(3)) * hexagonCenterCanHitHeight; // Maximum horizontal for ball to go through outer target
//  private double maxVerticalRatioOuter = hexagonCenterCanHitHeight / 2 / (Constants.ballRadius + Constants.ballTolerance);

    /**
     * Creates a new ExampleCommand.
     *
     * @param turret The subsystem used by this command.
     */

    public ShootOnTheMove(Turret turret, Shooter shooter, DriveTrain drivetrain, LED led, Vision vision) {
        m_turret = turret;
        m_shooter = shooter;
        m_drivetrain = drivetrain;
        m_led = led;
        m_vision = vision;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(turret, shooter, vision);
    }

    public void start() {
        isRunning = true;
    }

    public void stop() {
        isRunning = false;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Timestamp when command is called
        startTime = Timer.getFPGATimestamp(); // Starting timer to run command
        isRunning = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        final double timeStep = 0.1;
        double currentTime = Timer.getFPGATimestamp();
        if((currentTime - startTime) % 0.2 < 0.02 && isRunning) {

            // contains initial straight and angular velocity of robot
            ChassisSpeeds speeds = m_drivetrain.getDriveTrainKinematics().toChassisSpeeds(m_drivetrain.getSpeeds()); // Getting straight and angular velocity of drivetrain

            // Separating into linear and angular components
            // Should be re-calculated based on the maximum amount of time the turret and shooter can take to get to any given position and RPM
            double robotLinearVelocity = speeds.vxMetersPerSecond;
            // Linear velocity is meters per second straight ahead, angular velocity is rotation per second calculated from differences between wheels
            double robotAngularVelocity = speeds.omegaRadiansPerSecond;

            initialHeading = m_drivetrain.getHeading();
            currentDistanceToOuterTargetXY = m_vision.getTargetDistance();
            currentAngleToOuter = m_vision.getAngleToTarget();

            deltaTheta = robotAngularVelocity * timeStep; // Calculating how much robot's heading will change during time to shoot
            double radius = robotLinearVelocity / (robotAngularVelocity == 0 ? 0.01 : robotAngularVelocity); // Calculating distance from robot's position and center of robot's rotation

            distanceToOuterTargetXY = Math.sqrt(
                Math.pow(currentDistanceToOuterTargetXY, 2)
                + (4 * radius * radius * Math.pow(Math.sin(deltaTheta / 2), 2))
                - (4 * currentDistanceToOuterTargetXY * radius * Math.sin(deltaTheta / 2) * Math.cos(initialHeading - currentAngleToOuter + (deltaTheta / 2))));

            angleToOuter = Math.asin(
                currentDistanceToOuterTargetXY / distanceToOuterTargetXY
                * Math.sin(initialHeading - currentAngleToOuter + deltaTheta / 2))
                + Math.PI + initialHeading + deltaTheta / 2;

            distanceToInnerTargetXY = Math.sqrt(
                Math.pow(distanceToOuterTargetXY, 2) + Math.pow(Constants.targetOffset, 2)
                + 2 * distanceToOuterTargetXY * Constants.targetOffset * Math.sin(angleToOuter));

            angleToInner = angleToOuter - Math.asin(-Constants.targetOffset / distanceToInnerTargetXY * Math.cos(angleToOuter));

            // Calculating x and y components of velocity robot will have after time to shoot
            double robotPredictedXvel = robotLinearVelocity * Math.cos(initialHeading + deltaTheta);
            // Predicted x and y components of robot velocity after delay
            double robotPredictedYvel = robotLinearVelocity * Math.sin(initialHeading + deltaTheta);

            // xy-velocity needed to hit target
            double necessaryXYvel;
            if(canGoThroughInnerTarget()) {
                ledState = 2;
                double x = solveCubicEquation(
                    distanceToInnerTargetXY, -Constants.verticalTargetDistance, -Math.pow(distanceToInnerTargetXY, 2) * Constants.g / 2);
                shooterBallMagnitude = Math.sqrt(
                    x * x - 2 * x * robotLinearVelocity * Math.cos(initialHeading + deltaTheta - angleToInner + Math.pow(robotLinearVelocity, 2))
                    ) / Math.cos(Constants.verticalShooterAngle);
                targetTurretAngle = angleToInner + Math.asin(-robotLinearVelocity * Math.sin(initialHeading + deltaTheta - angleToInner) / shooterBallMagnitude / Math.cos(Constants.verticalShooterAngle));
                /*necessaryXYvel = distanceToInnerTargetXY * Constants.airResistanceCoefficient * Math.sqrt((- Constants.g / 2) / (Constants.verticalTargetDistance -
                        (distanceToInnerTargetXY == 0 ? 0.01 : distanceToInnerTargetXY) * Math.tan(Constants.verticalShooterAngle)));
                shooterBallVector = new Translation2d( // velocity components that the shooter has to give to the ball in form (xVel, yVel)
                        necessaryXYvel * Math.cos(angleToInner) - robotPredictedXvel,
                        necessaryXYvel * Math.sin(angleToInner) - robotPredictedYvel);*/
            } else if(canGoThroughOuterTarget()) {
                ledState = 1;
                double x = solveCubicEquation(
                    distanceToOuterTargetXY, -Constants.verticalTargetDistance, -Math.pow(distanceToOuterTargetXY, 2) * Constants.g / 2);
                shooterBallMagnitude = Math.sqrt(
                    x * x - 2 * x * robotLinearVelocity * Math.cos(initialHeading + deltaTheta - angleToOuter + Math.pow(robotLinearVelocity, 2))
                    ) / Math.cos(Constants.verticalShooterAngle);
                targetTurretAngle = angleToOuter + Math.asin(-robotLinearVelocity * Math.sin(initialHeading + deltaTheta - angleToOuter) / shooterBallMagnitude / Math.cos(Constants.verticalShooterAngle));
                /*necessaryXYvel = distanceToOuterTargetXY * Constants.airResistanceCoefficient * Math.sqrt((- Constants.g / 2) / (Constants.verticalTargetDistance -
                        (distanceToOuterTargetXY == 0 ? 0.01 : distanceToOuterTargetXY) * Math.tan(Constants.verticalShooterAngle)));
                shooterBallVector = new Translation2d( // velocity components that the shooter has to give to the ball in form (xVel, yVel)
                        necessaryXYvel * Math.cos(angleToOuter) - robotPredictedXvel,
                        necessaryXYvel * Math.sin(angleToOuter) - robotPredictedYvel);*/
            } else {
                ledState = 0;
            }

            if(ledState != 0) {
                /*targetTurretAngle = findAngle(shooterBallVector.getX(), shooterBallVector.getY()); // Calculates angle turret needs to rotate to
                shooterBallMagnitude = findDistance(shooterBallVector.getX(), shooterBallVector.getY()) // Gets xy magnitude of velocity balls needs to be shot at
                        / Math.cos(Constants.verticalShooterAngle); // Project that xy-velocity into 3 dimensions*/
                RPM = shooterBallMagnitude * 60 / (Constants.flywheelRadius * 2 * Math.PI); // Kind of works, probably a better way
                if(RPM > Constants.maxShooterRPM) {
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

    }

    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Predicted heading", initialHeading + deltaTheta);
        SmartDashboard.putNumber("Current distance to outer", currentDistanceToOuterTargetXY);
        SmartDashboard.putNumber("Current angle to outer", currentAngleToOuter);

        if(ledState == 2) {
            SmartDashboard.putBoolean("Can shoot", true);
            SmartDashboard.putString("Target", "Inner");
            SmartDashboard.putNumber("xy Distance to inner target", distanceToInnerTargetXY);
            SmartDashboard.putNumber("xy Angle to inner target", angleToInner);
        } else if(ledState == 1) {
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
        return Math.sqrt((deltaX * deltaX) + (deltaY * deltaY));
    }

    private double findAngle(double deltaX, double deltaY) { // Uses arctangent but uses signs to determine quadrant
        if(Math.abs(deltaX) <= 0.01) {
            return deltaY >= 0 ? Math.PI / 2 : - Math.PI / 2;
        }
        double tangent = Math.atan(deltaY / deltaX); // Raw value in radians
        // 0 degrees is on the right (positive x-axis)
        if(deltaX >= 0) {
            return tangent; // Positive x; between -90 and 90
        } else if(deltaY < 0) {
            return tangent - Math.PI; // Negative x, negative y; between -180 and -90
        } else {
            return tangent + Math.PI; // Negative x, positive y; between 90 and 180
        }
    }

    private boolean canGoThroughInnerTarget() {
        double tangentOfFinalAngle = Math.abs(2 * Constants.verticalTargetDistance / (distanceToInnerTargetXY == 0 ? 0.01 : distanceToInnerTargetXY) - Math.tan(Constants.verticalShooterAngle));
        return tangentOfFinalAngle <= hexagonCenterCanHitHeight / Constants.targetOffset / 2 && // Vertical angle alone is fine
                tangentOfFinalAngle <= - Math.sqrt(3) * Constants.targetOffset / Math.abs(Math.tan(angleToInner)) + hexagonCenterCanHitHeight; // Within sloped hexagon lines
    }

    private boolean canGoThroughOuterTarget() {
        double verticalTargetIntersection = Math.abs(2 * Constants.verticalTargetDistance / distanceToOuterTargetXY - Math.tan(Constants.verticalShooterAngle)) * Constants.ballRadius;
        return verticalTargetIntersection <= hexagonCenterCanHitHeight / 2 &&
                verticalTargetIntersection <= - Math.sqrt(3) * Math.abs(Constants.ballRadius / Math.tan(angleToOuter)) + hexagonCenterCanHitHeight;
    }

    private double solveCubicEquation(double a, double b, double d) { // Solves a cubic with x-coefficient 0
        double discriminant = Math.sqrt(Math.pow(Math.pow(b, 3) / 27 / Math.pow(a, 3) + d / 2 / a, 2) - Math.pow(b, 6) / 729 / Math.pow(a, 6));
        return Math.cbrt(-Math.pow(b, 3) / 27 / Math.pow(a, 3) - d / 2 / a + discriminant) 
        + Math.cbrt(-Math.pow(b, 3) / 27 / Math.pow(a, 3) - d / 2 / a - discriminant)
        - b / 3 / a;
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

//TODO: make it all work
