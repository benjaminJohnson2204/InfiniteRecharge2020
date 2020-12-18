package frc.robot.commands.autonomous.routines.simulation;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.SetDriveShifters;
import frc.robot.commands.drivetrain.SetOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.List;

public class DriveStraight extends SequentialCommandGroup {

    public DriveStraight(DriveTrain driveTrain, Turret turret, FieldSim fieldSim) {
        Pose2d initPosition = new Pose2d(2,5, new Rotation2d());
        Pose2d endPosition = new Pose2d(Units.feetToMeters(40),5, new Rotation2d());

        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(8), Units.feetToMeters(40));
//        config.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), config.getMaxVelocity()));
//        config.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(), 10));

        Trajectory driveStraight = TrajectoryGenerator.generateTrajectory(initPosition,
                                                                            List.of(),
                                                                            endPosition,
                                                                            config);

        var driveStraightCommand = TrajectoryUtils.generateRamseteCommand(driveTrain, driveStraight);

        Pose2d position2 = new Pose2d(Units.feetToMeters(45),3, new Rotation2d());


        Trajectory splineTrajectory = TrajectoryGenerator.generateTrajectory(endPosition,
                List.of(),
                position2,
                config);

        var spline = TrajectoryUtils.generateRamseteCommand(driveTrain, splineTrajectory);

        addCommands(new SetOdometry(driveTrain, fieldSim, initPosition),
                    new SetDriveShifters(driveTrain, true),
                    driveStraightCommand,//.andThen(() -> driveTrain.setVoltageOutput(0,0)));
                    spline.andThen(() -> driveTrain.setVoltageOutput(0,0)));
    }
}
