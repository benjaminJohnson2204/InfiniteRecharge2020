/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;
import frc.vitruvianlib.utils.ReadCsvTrajectory;

import java.util.ArrayList;

/**
 * An example command that uses an example subsystem.
 */
public class ReadTrajectory extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_driveTrain;
  private Trajectory trajectory;
  private String m_filename;
  private boolean m_isInverted = false;
  private Timer testTimer = new Timer();
  private double timeout, distance;
  private DifferentialDriveKinematicsConstraint m_kinematicsConstraint;
  ArrayList<Pose2d> trajectoryWaypoints = new ArrayList<Pose2d>();
  /**
   * Creates a new ExampleCommand.
   *
   * @param driveTrain The subsystem used by this command.
   */

  public ReadTrajectory(DriveTrain driveTrain, String filename, boolean isInverted, DifferentialDriveKinematicsConstraint kinematicsConstraint) {
    m_driveTrain = driveTrain;
    m_isInverted = isInverted;
    m_filename = filename;
    m_kinematicsConstraint = kinematicsConstraint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  public ReadTrajectory(DriveTrain driveTrain, String filename, boolean isInverted) {
    m_driveTrain = driveTrain;
    m_isInverted = isInverted;
    m_filename = filename;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  public ReadTrajectory(DriveTrain driveTrain, String filename) {
    m_driveTrain = driveTrain;
    m_filename = filename;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }


    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trajectoryWaypoints = new ArrayList<Pose2d>();
//    m_driveTrain.navX.reset();
    m_driveTrain.setDriveTrainNeutral();
//    m_driveTrain.resetOdometry(new Pose2d(), new Rotation2d());
//    m_driveTrain.resetEncoderCounts();
    // Start position for all trajectories will be where the robot is currently
    var startPosition = new Pose2d(m_driveTrain.getRobotPose().getTranslation().getX(),
                                   m_driveTrain.getRobotPose().getTranslation().getY(),
                                   Rotation2d.fromDegrees(m_driveTrain.navX.getAngle()));

    String filePath = Filesystem.getDeployDirectory().getAbsolutePath() + "/Trajectories/" + m_filename + ".csv";
    var fileTrajectory = ReadCsvTrajectory.readCsv(filePath);

    trajectoryWaypoints.add(startPosition);

    // All points we generate assume we start from (0,0). Take those points and shift it based on your starting position
    for(Pose2d point : fileTrajectory) {
      if(point.getTranslation().getX() == 0 && point.getTranslation().getY() == 0 && point.getRotation().getDegrees() ==0)
        continue;

      trajectoryWaypoints.add(new Pose2d(startPosition.getTranslation().getX() + point.getTranslation().getX(),
                                         startPosition.getTranslation().getY() + point.getTranslation().getY(),
                                            point.getRotation()));
    }

    var trajectoryConfig = new TrajectoryConfig(Units.feetToMeters(16),  Units.feetToMeters(8));

    distance = 0;
    for(int i = 0; i < trajectoryWaypoints.size() - 1; i++) {
      var pointA = trajectoryWaypoints.get(i);
      var pointB = trajectoryWaypoints.get(i + 1);

      double deltaX = pointB.getTranslation().getX() - pointA.getTranslation().getX();
      double deltaY = pointB.getTranslation().getY() - pointA.getTranslation().getY();
      double deltaDistance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
      distance += deltaDistance;
    }
    timeout = (distance / 16.0) + 2;

    if(m_kinematicsConstraint != null)
      trajectoryConfig.addConstraint(m_kinematicsConstraint);

    trajectoryConfig.setReversed(m_isInverted);

    trajectory = TrajectoryGenerator.generateTrajectory(trajectoryWaypoints, trajectoryConfig);

    RamseteCommand followTrajectory = new RamseteCommand(
            trajectory,
            m_driveTrain::getRobotPose,
            new RamseteController(),
            m_driveTrain.getFeedforward(),
            m_driveTrain.getDriveTrainKinematics(),
            m_driveTrain::getSpeeds,
            m_driveTrain.getLeftPIDController(),
            m_driveTrain.getRightPIDController(),
            m_driveTrain::setVoltageOutput,
            m_driveTrain
    );
    CommandScheduler.getInstance().schedule(followTrajectory);
    testTimer.start();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double deltaX = Math.abs(m_driveTrain.getRobotPose().getTranslation().getX() - trajectoryWaypoints.get(trajectoryWaypoints.size()).getTranslation().getX());
    double deltaY = Math.abs(m_driveTrain.getRobotPose().getTranslation().getY() - trajectoryWaypoints.get(trajectoryWaypoints.size()).getTranslation().getY());
    double deltaRot = Math.abs(m_driveTrain.getRobotPose().getRotation().getDegrees() - trajectoryWaypoints.get(trajectoryWaypoints.size()).getRotation().getDegrees());
    return ((deltaX < 0.25) && (deltaY < 0.25) && (deltaRot < 2)) || testTimer.get() > timeout;
  }

}
