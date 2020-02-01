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

public class AlignToOuterPort extends CommandBase
{
    private final DriveTrain driveTrain;
    private final Vision vision;

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
        // Make sure we have a target
        if (!vision.hasTarget())
        {
            return;
        }

        double xOffset = vision.getTargetX();

        while (xOffset > 3 || xOffset < -3)
        {
            if (xOffset < -3) // Target is to the left?
            {
                driveTrain.setMotorTankDrive(0, 0.5);
            }
            else if (xOffset > 3) // Target is to the right?
            {
                driveTrain.setMotorTankDrive(0.5, 0);
            }

            xOffset = vision.getTargetX();
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
