package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
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

public class AutoNavBarrel extends SequentialCommandGroup {
    private Pose2d[] startPoints, endPoints;
    public AutoNavBarrel(DriveTrain driveTrain, FieldSim fieldSim) {
        if (RobotBase.isReal()) {
                Pose2d[] startingPoints = {
                        new Pose2d(Units.inchesToMeters(40), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(176), Units.inchesToMeters(45), new Rotation2d(Units.degreesToRadians(-120))),
                        new Pose2d(Units.inchesToMeters(124), Units.inchesToMeters(45), new Rotation2d(Units.degreesToRadians(120))),
                        new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(240), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(120), new Rotation2d(Units.degreesToRadians(90))),
                        new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(120), new Rotation2d(Units.degreesToRadians(-90))),
                        new Pose2d(Units.inchesToMeters(285), Units.inchesToMeters(34), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(90))),
                        new Pose2d(Units.inchesToMeters(285), Units.inchesToMeters(86), new Rotation2d(Units.degreesToRadians(180))),
                };
                startPoints = startingPoints;
                Pose2d[] endingPoints = {
                        startingPoints[1],
                        startingPoints[2],
                        startingPoints[3],
                        new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(6))),
                        startingPoints[5],
                        startingPoints[6],
                        startingPoints[7],
                        startingPoints[8],
                        startingPoints[9],
                        startingPoints[10],
                        new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(180))),
                };
                endPoints = endingPoints;
                /*Pose2d[] startingPoints = {
                        new Pose2d(Units.inchesToMeters(40), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(176), Units.inchesToMeters(45), new Rotation2d(Units.degreesToRadians(-120))),
                        new Pose2d(Units.inchesToMeters(124), Units.inchesToMeters(45), new Rotation2d(Units.degreesToRadians(120))),
                        new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(240), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(120), new Rotation2d(Units.degreesToRadians(90))),
                        new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(120), new Rotation2d(Units.degreesToRadians(-90))),
                        new Pose2d(Units.inchesToMeters(285), Units.inchesToMeters(34), new Rotation2d(Units.degreesToRadians(0))),
                        //new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(90))),
                        new Pose2d(Units.inchesToMeters(290), Units.inchesToMeters(75), new Rotation2d(Units.degreesToRadians(160))),
                        new Pose2d(Units.inchesToMeters(275), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(180))),
                };      
                startPoints = startingPoints;
                Pose2d[] endingPoints = {
                        new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(176), Units.inchesToMeters(40), new Rotation2d(Units.degreesToRadians(-120))),
                        new Pose2d(Units.inchesToMeters(124), Units.inchesToMeters(40), new Rotation2d(Units.degreesToRadians(118))),
                        new Pose2d(Units.inchesToMeters(156), Units.inchesToMeters(85), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(260), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(283), Units.inchesToMeters(105), new Rotation2d(Units.degreesToRadians(95))),
                        new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(-86))),
                        new Pose2d(Units.inchesToMeters(320), Units.inchesToMeters(15), new Rotation2d(Units.degreesToRadians(2))),
                        //new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(40), new Rotation2d(Units.degreesToRadians(100))),
                        new Pose2d(Units.inchesToMeters(290), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(190))),
                        new Pose2d(Units.inchesToMeters(275), Units.inchesToMeters(84), new Rotation2d(Units.degreesToRadians(185))),
                        new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(180))),
                };
                endPoints = endingPoints;*/
        } else {
                Pose2d[] startingPoints = {
                        new Pose2d(Units.inchesToMeters(40), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(176), Units.inchesToMeters(45), new Rotation2d(Units.degreesToRadians(-120))),
                        new Pose2d(Units.inchesToMeters(124), Units.inchesToMeters(45), new Rotation2d(Units.degreesToRadians(120))),
                        new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(240), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(120), new Rotation2d(Units.degreesToRadians(90))),
                        new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(120), new Rotation2d(Units.degreesToRadians(-90))),
                        new Pose2d(Units.inchesToMeters(285), Units.inchesToMeters(34), new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(90))),
                        new Pose2d(Units.inchesToMeters(285), Units.inchesToMeters(86), new Rotation2d(Units.degreesToRadians(180))),
                };
                startPoints = startingPoints;
                Pose2d[] endingPoints = {
                        startingPoints[1],
                        startingPoints[2],
                        startingPoints[3],
                        startingPoints[4],
                        startingPoints[5],
                        startingPoints[6],
                        startingPoints[7],
                        startingPoints[8],
                        startingPoints[9],
                        startingPoints[10],
                        new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(180))),
                };
                endPoints = endingPoints;
        }

        Pose2d startPosition = startPoints[0];

        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(10), Units.feetToMeters(6));
        configA.setReversed(false);
        //configA.setEndVelocity(configA.getMaxVelocity());
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));
        configA.addConstraint(new CentripetalAccelerationConstraint(1.2)); // This is what we can change when we're actually testing

        addCommands(new SetDriveShifters(driveTrain, Constants.DriveConstants.inSlowGear),
                new SetOdometry(driveTrain, fieldSim, startPosition),
                new SetDriveNeutralMode(driveTrain, 0));

        for(int i = 0; i < startPoints.length; i++) {
                if (i != 0) {
                        configA.setEndVelocity(configA.getMaxVelocity());
                        configA.setStartVelocity(configA.getMaxVelocity());
                }
                if (i == startPoints.length - 1) {
                        configA.setEndVelocity(0);
                }
                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startPoints[i],
                i == 8 ? List.of(new Translation2d(Units.inchesToMeters(313), Units.inchesToMeters(48))) : List.of(),
                endPoints[i],
                configA);
            var command = TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory);
            addCommands(command);
        }
    }
}

