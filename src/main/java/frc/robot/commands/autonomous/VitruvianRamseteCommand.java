/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj2.command;

import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.DriveTrain;
import frc.vitruvianlib.utils.TrajectoryUtils;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * A command that uses a RAMSETE controller ({@link RamseteController}) to follow a trajectory
 * {@link Trajectory} with a differential drive.
 *
 * <p>The command handles trajectory-following, PID calculations, and feedforwards internally.  This
 * is intended to be a more-or-less "complete solution" that can be used by teams without a great
 * deal of controls expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to use the onboard
 * PID functionality of a "smart" motor controller) may use the secondary constructor that omits
 * the PID and feedforward functionality, returning only the raw wheel speeds from the RAMSETE
 * controller.
 */
@SuppressWarnings("PMD.TooManyFields")
public class VitruvianRamseteCommand extends CommandBase {
    private final Timer m_timer = new Timer();
    private final boolean m_usePID;
    private Trajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final RamseteController m_follower;
    private final SimpleMotorFeedforward m_feedforward;
    private final DifferentialDriveKinematics m_kinematics;
    private final Supplier<DifferentialDriveWheelSpeeds> m_speeds;
    private final PIDController m_leftController;
    private final PIDController m_rightController;
    private final BiConsumer<Double, Double> m_output;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;

    private String m_filename;
    private final DriveTrain m_driveTrain;
    private final TrajectoryConfig m_config;
    ArrayList<Pose2d> m_path = new ArrayList<>();

    private final boolean generatePath;

    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
     * PID control and feedforward are handled internally, and outputs are scaled -12 to 12
     * representing units of volts.
     *
     * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
     * this
     * is left to the user, since it is not appropriate for paths with nonstationary endstates.
     *
     */
    @SuppressWarnings("PMD.ExcessiveParameterList")
    public VitruvianRamseteCommand(String filename, DriveTrain driveTrain, TrajectoryConfig config) {
        m_pose = driveTrain::getRobotPose;
        m_follower = new RamseteController();
        m_feedforward = driveTrain.getFeedforward();
        m_kinematics = driveTrain.getDriveTrainKinematics();
        m_speeds = driveTrain::getSpeeds;
        m_leftController = driveTrain.getLeftPIDController();
        m_rightController = driveTrain.getRightPIDController();
        m_output = driveTrain::setVoltageOutput;

        m_usePID = true;

        m_filename = filename;
        m_driveTrain = driveTrain;
        m_config = config;

        addRequirements(driveTrain);

        generatePath = true;
    }

    public VitruvianRamseteCommand(ArrayList<Pose2d> path, DriveTrain driveTrain, TrajectoryConfig config) {
        m_pose = driveTrain::getRobotPose;
        m_follower = new RamseteController();
        m_feedforward = driveTrain.getFeedforward();
        m_kinematics = driveTrain.getDriveTrainKinematics();
        m_speeds = driveTrain::getSpeeds;
        m_leftController = driveTrain.getLeftPIDController();
        m_rightController = driveTrain.getRightPIDController();
        m_output = driveTrain::setVoltageOutput;

        m_usePID = true;

        m_path = path;
        m_driveTrain = driveTrain;
        m_config = config;

        addRequirements(driveTrain);

        generatePath = false;
    }

    @Override
    public void initialize() {
        if(generatePath) {
            // Construct the trajectory and path given a .csv of the path and a trajectory config
            String filePath = Filesystem.getDeployDirectory().getAbsolutePath() + "/Trajectories/" + m_filename + ".csv";
            var fileTrajectory = TrajectoryUtils.readCsvTrajectory(filePath);

            var startPosition = new Pose2d(m_pose.get().getTranslation().getX(),
                    m_pose.get().getTranslation().getY(),
                    Rotation2d.fromDegrees(m_driveTrain.getAngle()));

            for (Pose2d point : fileTrajectory) {
//                System.out.printf("X: %.2f\tY: %.2f\n", point.getTranslation().getX(), point.getTranslation().getY());
                if (point.getTranslation().getX() == 0 && point.getTranslation().getY() == 0)
                    continue;

                m_path.add(new Pose2d(startPosition.getTranslation().getX() + startPosition.getTranslation().getX(),
                        startPosition.getTranslation().getY() + startPosition.getTranslation().getY(),
                        point.getRotation()));
            }
        }

        m_trajectory = TrajectoryGenerator.generateTrajectory(m_path, m_config);

        m_prevTime = 0;
        var initialState = m_trajectory.sample(0);
        m_prevSpeeds = m_kinematics.toWheelSpeeds(
                new ChassisSpeeds(initialState.velocityMetersPerSecond,
                        0,
                        initialState.curvatureRadPerMeter
                                * initialState.velocityMetersPerSecond));
        m_timer.reset();
        m_timer.start();
        if (m_usePID) {
            m_leftController.reset();
            m_rightController.reset();
        }
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;

        var targetWheelSpeeds = m_kinematics.toWheelSpeeds(
                m_follower.calculate(m_pose.get(), m_trajectory.sample(curTime)));

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftOutput;
        double rightOutput;

        if (m_usePID) {
            double leftFeedforward =
                    m_feedforward.calculate(leftSpeedSetpoint,
                            (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

            double rightFeedforward =
                    m_feedforward.calculate(rightSpeedSetpoint,
                            (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

            leftOutput = leftFeedforward
                    + m_leftController.calculate(m_speeds.get().leftMetersPerSecond,
                    leftSpeedSetpoint);

            rightOutput = rightFeedforward
                    + m_rightController.calculate(m_speeds.get().rightMetersPerSecond,
                    rightSpeedSetpoint);
        } else {
            leftOutput = leftSpeedSetpoint;
            rightOutput = rightSpeedSetpoint;
        }

        m_output.accept(leftOutput, rightOutput);

        m_prevTime = curTime;
        m_prevSpeeds = targetWheelSpeeds;
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        double deltaX = Units.metersToFeet(Math.abs(m_pose.get().getTranslation().getX() - m_path.get(m_path.size() - 1).getTranslation().getX()));
        double deltaY = Units.metersToFeet(Math.abs(m_pose.get().getTranslation().getY() - m_path.get(m_path.size() - 1).getTranslation().getY()));
        double deltaRot = Math.abs(m_pose.get().getRotation().getDegrees() - m_path.get(m_path.size() - 1).getRotation().getDegrees());
        boolean isFinished = ((deltaX < 0.25) && (deltaY < 0.25) && (deltaRot < 4));
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds()) || isFinished;
    }
}
