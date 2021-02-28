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
import frc.robot.constants.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimulationShoot;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;
import java.util.ArrayList;
import java.util.List;

public class AutoNavBounce extends SequentialCommandGroup {
    private Pose2d[] startPoints, endPoints;
    public AutoNavBounce(DriveTrain driveTrain, FieldSim fieldSim) {
        if (RobotBase.isReal()) {
                Pose2d[] startingPoints = {
                        new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(140), new Rotation2d(Units.degreesToRadians(90))),
                        new Pose2d(Units.inchesToMeters(105), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(120))),
                        new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(180))),
                        new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(140), new Rotation2d(Units.degreesToRadians(-90))),
                        new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(255), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(140), new Rotation2d(Units.degreesToRadians(90))),
                };
                startPoints = startingPoints;

                Pose2d[] endingPoints = {
                        new Pose2d(Units.inchesToMeters(87), Units.inchesToMeters(140), new Rotation2d(Units.degreesToRadians(90))),
                        new Pose2d(Units.inchesToMeters(105), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(120))),
                        new Pose2d(Units.inchesToMeters(148), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(180))),
                        new Pose2d(Units.inchesToMeters(175), Units.inchesToMeters(140), new Rotation2d(Units.degreesToRadians(-92))),
                        new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(250), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(265), Units.inchesToMeters(140), new Rotation2d(Units.degreesToRadians(90))),
                        new Pose2d(Units.inchesToMeters(315), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(160))),
                };
                endPoints = endingPoints;
        } else {
                Pose2d[] startingPoints = {
                        new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(140), new Rotation2d(Units.degreesToRadians(90))),
                        new Pose2d(Units.inchesToMeters(105), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(120))),
                        new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(180))),
                        new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(140), new Rotation2d(Units.degreesToRadians(-90))),
                        new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(255), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(140), new Rotation2d(Units.degreesToRadians(90))),
                };
                startPoints = startingPoints;

                Pose2d[] endingPoints = {
                        new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(140), new Rotation2d(Units.degreesToRadians(90))),
                        new Pose2d(Units.inchesToMeters(105), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(120))),
                        new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(180))),
                        new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(140), new Rotation2d(Units.degreesToRadians(-90))),
                        new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(255), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(140), new Rotation2d(Units.degreesToRadians(90))),
                        new Pose2d(Units.inchesToMeters(315), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(160))),
                };
                endPoints = endingPoints;
        }

        boolean[] pathIsReversed = {false, true, true, true, false, false, false, true};
        Pose2d startPosition = startPoints[0];


        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(14), Units.feetToMeters(14));
        configA.setReversed(false);
        //configA.setEndVelocity(configA.getMaxVelocity());
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));
        configA.addConstraint(new CentripetalAccelerationConstraint(6));

        addCommands(new SetDriveShifters(driveTrain, Constants.DriveConstants.inSlowGear),
                new SetOdometry(driveTrain, fieldSim, startPosition),
                new SetDriveNeutralMode(driveTrain, 0));

        double[] startVelocities = {configA.getMaxVelocity(), 0, configA.getMaxVelocity(), configA.getMaxVelocity(), 0, 
                configA.getMaxVelocity(), configA.getMaxVelocity(), 0, 0};
        double[] endVelocities = {0, configA.getMaxVelocity(), configA.getMaxVelocity(), 0, configA.getMaxVelocity(), 
                configA.getMaxVelocity(), 0, 0};

        for(int i = 0; i < startPoints.length; i++) {
                configA.setStartVelocity(startVelocities[i]);
                configA.setEndVelocity(endVelocities[i]);
                configA.setReversed(pathIsReversed[i]);
                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startPoints[i],
                List.of(),
                endPoints[i],
                configA);
            var command = TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory);
            addCommands(command);
        }
    }
}

