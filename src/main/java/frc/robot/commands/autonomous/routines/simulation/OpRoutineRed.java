package frc.robot.commands.autonomous.routines.simulation;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.TurnInPlace;
import frc.robot.commands.drivetrain.SetDriveShifters;
import frc.robot.commands.drivetrain.SetOdometry;
import frc.robot.commands.intake.SetIntakeSpeed;
import frc.robot.commands.intake.SetIntakePiston;
import frc.robot.commands.intake.SetIntakeStates;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.ArrayList;
import java.util.List;

public class OpRoutineRed extends SequentialCommandGroup {
    /* Notes:
        Approximate total Distance to Travel (By counting Pixels) = 17.55m (57.6ft)
        That means we need to go ~ 3.8 ft/s at minimum. Lets try 6 ft/s for some headroom.

        Note: Using unreasonable limits in the Sim will result in weird behavior

     */
    public OpRoutineRed(DriveTrain driveTrain, Intake intake, Indexer indexer, Turret turret, Shooter shooter, Vision vision, FieldSim fieldSim) {
        Pose2d initPosition = new Pose2d(12.571505,6.425269, new Rotation2d(Units.degreesToRadians(0)));
        Pose2d startPosition = new Pose2d(12.571505,6.425269, new Rotation2d(Units.degreesToRadians(-22.5)));
        Pose2d midPos = new Pose2d(10.566546,7.75, new Rotation2d(Units.degreesToRadians(-35)));
        Pose2d blueTrenchIntakePos = new Pose2d(10.049316,7.166328, new  Rotation2d(Units.degreesToRadians(-55)));
        Pose2d midPoint = new Pose2d(11.25,6.425269, new Rotation2d(Units.degreesToRadians(-90)));
        Pose2d crossoverStopPos = new Pose2d(11.25,0.902087, new  Rotation2d(Units.degreesToRadians(-90)));
        Pose2d redTrenchIntakePos = new Pose2d(8.607954,0.883017, new  Rotation2d(Units.degreesToRadians(0)));
        Pose2d redCenterIntakePos = new Pose2d(9.541358,2.741749, new  Rotation2d(Units.degreesToRadians(-155)));

        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(15), Units.feetToMeters(60));
        configA.setReversed(true);
        /*  Note For some reason, a curved reverse path ends up looking like a question mark, where the robot turns outwards
            first before going to the balls. Don't know how to get around this, so using a simpler
         */
//        Trajectory startToBlueTrajectory = TrajectoryGenerator.generateTrajectory(initPosition,
//                                                                                List.of(),
//                                                                                blueTrenchIntakePos,
//                                                                                configA);

        Trajectory simpleReversePath = TrajectoryGenerator.generateTrajectory(new Pose2d(0,0, new Rotation2d(0)),
                List.of(),
                new Pose2d(Units.feetToMeters(-7.75), 0, new Rotation2d(0)),
                configA);
        Transform2d transform = startPosition.minus(simpleReversePath.getInitialPose());
        Trajectory startToBlueTrajectory = simpleReversePath.transformBy(transform);

        var startToBlueTrench = TrajectoryUtils.generateRamseteCommand(driveTrain, startToBlueTrajectory);
        int endIdx = startToBlueTrajectory.getStates().size() - 1;
        Pose2d endPointA = startToBlueTrajectory.getStates().get(endIdx).poseMeters;

//        var configB = new TrajectoryConfig(Units.feetToMeters(20), Units.feetToMeters(20));
        TrajectoryConfig configB = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(20));
        configB.setReversed(false);
        ArrayList<Pose2d> startToTrenchPath = new ArrayList();
        startToTrenchPath.add(endPointA);
        startToTrenchPath.add(midPoint);
        startToTrenchPath.add(crossoverStopPos);

        var blueTrenchCrossover = TrajectoryUtils.generateRamseteCommand(driveTrain, startToTrenchPath, configB);

//        configB.setEndVelocity(0);
//        configB.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configB.getMaxVelocity()));
//        configB.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));
//        configB.addConstraint(new CentripetalAccelerationConstraint(Units.feetToMeters(2)));
        //var trenchToShootPath = TrajectoryUtils.readCsvTrajectory("ally2Ally3");
//        ArrayList<Pose2d> trenchToShootPath = new ArrayList();
//        startToTrenchPath.add(redTrenchIntakePos);
//        Pose2d crossoverStopPos = new Pose2d(4.786788,7.268511, new  Rotation2d(Units.degreesToRadians(90)));
        //var trenchToShootCommand = TrajectoryUtils.generateRamseteCommand(driveTrain, trenchToShootPath, configB);


        addCommands(
                new SetOdometry(driveTrain, fieldSim, initPosition),
                new SetDriveShifters(driveTrain, false),
                new TurnInPlace(driveTrain, -22.5),
                new ParallelDeadlineGroup(startToBlueTrench,
                        new SetIntakeStates(intake, true, 1))
                        .andThen(() -> driveTrain.setVoltageOutput(0,0)),
                blueTrenchCrossover
//                new ParallelDeadlineGroup(new WaitCommand(2),
//                                          new SimulationShoot(fieldSim, true)),
//                startToRedTrench,
//                new WaitCommand(2)
//                .andThen(trenchToShootCommand)
//                .alongWith(new SetTurretRobotRelativeAngle(turret, 0))
//                .andThen(() -> driveTrain.setMotorTankDrive(0,0))
//                .andThen(new AutoUseVisionCorrection(turret, vision).withTimeout(0.75))
//                .andThen(new ParallelDeadlineGroup(new WaitCommand(2),
//                                                   new SimulationShoot(fieldSim, true)))
        );

    }
}

