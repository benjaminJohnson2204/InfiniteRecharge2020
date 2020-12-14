package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.DriveTrain;

import java.util.ArrayList;

import static frc.robot.constants.Constants.DriveConstants.turn_kP;
import static frc.robot.constants.Constants.DriveConstants.turn_kI;
import static frc.robot.constants.Constants.DriveConstants.turn_kD;
import static frc.robot.constants.Constants.DriveConstants.turn_kDt;


public class TurnInPlace extends PIDCommand {
    private DriveTrain m_driveTrain;
    private final double kS = 0.19;
    private final double kV = 2.23;
    private final double kA = 0.0289;

    private double degreesTolerance = 3;
    private double degreesPerSecondTolerance = 3;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    private double m_startTime;
    private double m_timeout = 2;

    /**
     * Creates a new ExampleCommand.
     *
     * @param driveTrain The subsystem used by this command.
     */
    public TurnInPlace(DriveTrain driveTrain, double targetAngle) {
        super(new PIDController(turn_kP, turn_kI, turn_kD),
                driveTrain::getHeading,
                targetAngle,
                (output) -> driveTrain.setMotorArcadeDrive(0, Math.min(Math.max(output, 0.1), -0.1)),
                driveTrain);

        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(degreesTolerance, degreesPerSecondTolerance);

        m_driveTrain = driveTrain;

        SmartDashboard.putNumber("P", getController().getP());
        SmartDashboard.putNumber("I", getController().getI());
        SmartDashboard.putNumber("D", getController().getD());
    }

    @Override
    public void execute() {
        super.execute();
//        SmartDashboard.putNumber("PID Goal",getController().getGoal().position);
        SmartDashboard.putNumber("PID Setpoint",getController().getSetpoint());
        SmartDashboard.putNumber("PID Position Error",getController().getPositionError());
        SmartDashboard.putNumber("PID Velocity Error",getController().getVelocityError());
//        SmartDashboard.putNumber("PID Output");

        SmartDashboard.getNumber("P", getController().getP());
        SmartDashboard.getNumber("I", getController().getI());
        SmartDashboard.getNumber("D", getController().getD());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.setVoltageOutput(0,0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
