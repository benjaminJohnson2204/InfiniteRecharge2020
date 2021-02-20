package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.autonomous.SmartdashboardCommand;
import frc.robot.commands.autonomous.TurnInPlace;
import frc.robot.commands.drivetrain.SetDriveNeutralMode;
import frc.robot.commands.drivetrain.SetDriveShifters;
import frc.robot.commands.drivetrain.SetOdometry;
import frc.robot.commands.intake.SetIntakeSpeed;
import frc.robot.commands.intake.SetIntakePiston;
import frc.robot.commands.intake.SetIntakeStates;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimulationShoot;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;
import java.util.ArrayList;
import java.util.List;

public class AutoNavSlalom extends SequentialCommandGroup {
    public AutoNavSlalom(DriveTrain driveTrain, FieldSim fieldSim) {
        /*Pose2d startPosition = new Pose2d();
        Pose2d point2 = new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(0)));
        Pose2d point3 = new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(0)));
        Pose2d point4 = new Pose2d(Units.inchesToMeters(240), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(-120)));
        Pose2d point5 = new Pose2d(Units.inchesToMeters(285), Units.inchesToMeters(4), new Rotation2d());
        Pose2d point6 = new Pose2d(Units.inchesToMeters(285), Units.inchesToMeters(56), new Rotation2d(Units.degreesToRadians(120)));
        Pose2d origin = new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(30)));*/
        Pose2d[] waypoints = {
                new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(60))),
                new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(240), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(-45))),
                new Pose2d(Units.inchesToMeters(315), Units.inchesToMeters(34), new Rotation2d(Units.degreesToRadians(30))),
                new Pose2d(Units.inchesToMeters(315), Units.inchesToMeters(86), new Rotation2d(Units.degreesToRadians(150))),
                new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(225))),
                new Pose2d(Units.inchesToMeters(240), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(180))),
                new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(180))),
                new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(120))),
                new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(150)))
        };
        Pose2d startPosition = waypoints[0];

        Trajectory[] trajectories = new Trajectory[waypoints.length - 1];
        Trajectory[] transformedTrajectories = new Trajectory[waypoints.length - 1];

        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(6), Units.feetToMeters(10));
        configA.setReversed(false);
        //configA.setEndVelocity(configA.getMaxVelocity());
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));
        configA.addConstraint(new CentripetalAccelerationConstraint(0.45));

        Trajectory tempTrajectory;
        Pose2d finalPose = waypoints[0];

        addCommands(new SetOdometry(driveTrain, fieldSim, startPosition),
                new SetDriveNeutralMode(driveTrain, 0));

        for(int i = 0; i < waypoints.length - 1; i++) {
            /*trajectories[i] = TrajectoryGenerator.generateTrajectory(waypoints[i],
                    List.of(),
                    waypoints[i + 1],
                    configA);
            Transform2d transform = finalPose.minus(trajectories[i].getInitialPose());
            transformedTrajectories[i] = trajectories[i];//.transformBy(transform);

            tempTrajectory = transformedTrajectories[i];
            finalPose = waypoints[i];//tempTrajectory.getStates().get(tempTrajectory.getStates().size() - 1).poseMeters;*/
                if (i != 0) {
                        configA.setEndVelocity(configA.getMaxVelocity());
                        configA.setStartVelocity(configA.getMaxVelocity());
                }
                if (i == waypoints.length - 2) {
                        configA.setEndVelocity(0);
                }
                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints[i],
                List.of(),
                waypoints[i + 1],
                configA);
            var command = TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory);
            addCommands(command);//.andThen(() -> SmartDashboard.putNumber("Final Pose angle", trajectory.)));
            /*SmartDashboard.putNumber("Initial Pose x", tempTrajectory.getInitialPose().getX());
            SmartDashboard.putNumber("Initial Pose y", tempTrajectory.getInitialPose().getY());
            SmartDashboard.putNumber("Initial Pose angle", tempTrajectory.getInitialPose().getRotation().getDegrees());*/
        }

        /*var startTo2Path = TrajectoryGenerator.generateTrajectory(startPosition,
                List.of(),
                point2,
                configA);
        Transform2d startTransform = origin.minus(startTo2Path.getInitialPose());
        Trajectory transformedPathstartTo2 = startTo2Path.transformBy(startTransform);

        //configA.setStartVelocity(configA.getMaxVelocity());
        var point2To3Path = TrajectoryGenerator.generateTrajectory(point2,
                List.of(),
                point3,
                configA);
        Transform2d point2To3Transform = origin.minus(point2To3Path.getInitialPose());
        Trajectory transformedPath2To3 = point2To3Path.transformBy(point2To3Transform);

        var point3To4Path = TrajectoryGenerator.generateTrajectory(point3,
                List.of(),
                point4,
                configA);
        Transform2d point3To4Transform = origin.minus(point3To4Path.getInitialPose());
        Trajectory transformedPath3To4 = point3To4Path.transformBy(point3To4Transform);

        var point4To5Path = TrajectoryGenerator.generateTrajectory(point4,
                List.of(),
                point5,
                configA);
        Transform2d point4To5Transform = origin.minus(point4To5Path.getInitialPose());
        Trajectory transformedPath4To5 = point4To5Path.transformBy(point4To5Transform);
        
        Transform2d point5To6Transform = point4To5Path.getInitialPose().minus(point5);
        Trajectory transformedPath5To6 = transformedPath4To5.transformBy(point5To6Transform);

        Transform2d point6To7Transform = point4To5Path.getInitialPose().minus(point6);
        Trajectory transformedPath6To7 = transformedPath4To5.transformBy(point6To7Transform);


        var startTo2Command = TrajectoryUtils.generateRamseteCommand(driveTrain, transformedPathstartTo2);
        var point2To3Command = TrajectoryUtils.generateRamseteCommand(driveTrain, transformedPath2To3);
        var point3To4Command = TrajectoryUtils.generateRamseteCommand(driveTrain, transformedPath3To4);
        var point4To5Command = TrajectoryUtils.generateRamseteCommand(driveTrain, transformedPath4To5);
        var point5To6Command = TrajectoryUtils.generateRamseteCommand(driveTrain, transformedPath5To6);
        var point6To7Command = TrajectoryUtils.generateRamseteCommand(driveTrain, transformedPath6To7);*/


        /*addCommands(
                new SetOdometry(driveTrain, fieldSim, point2To3Path.getInitialPose()),
                new SetDriveNeutralMode(driveTrain, 0),
                //startTo2Command,
                point2To3Command.andThen(() -> driveTrain.setVoltageOutput(0, 0))
                // point3To4Command,
                // point4To5Command,
                // point5To6Command,
                // point6To7Command
        );*/

    }
}

