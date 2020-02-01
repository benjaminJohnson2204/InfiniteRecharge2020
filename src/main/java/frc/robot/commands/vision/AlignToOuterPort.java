/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.controller.PIDController;

public class AlignToOuterPort extends CommandBase
{
    private final double kP = 1;
    private final double kI = 0;
    private final double kD = 0;

    private final DriveTrain driveTrain;
    private final Vision vision;
    private PIDController visionPID = new PIDController(kP, kI, kD);

    /**
     * Creates a new AlignToOuterPort.
     */
    public AlignToOuterPort(DriveTrain subsystem, Vision visionSubsystem)
    {
        // Use addRequirements() here to declare subsystem dependencies.
        driveTrain = subsystem;
        vision = visionSubsystem;
        addRequirements(visionSubsystem);
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        // TODO: Correction might be too much, might be overcompensating sending robot into loop
        // Make sure we have a target
        if (!vision.hasTarget())
        {
            return;
        }

        visionPID.setSetpoint(0);

        while (!visionPID.atSetpoint())
        {
            double pidOut = visionPID.calculate(vision.getTargetX());
            if (pidOut < 0) // Target is to the left
            {
                driveTrain.setMotorTankDrive(0, pidOut);
            }
            else if (pidOut > 0) // Target is to the right
            {
                driveTrain.setMotorTankDrive(pidOut, 0);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
