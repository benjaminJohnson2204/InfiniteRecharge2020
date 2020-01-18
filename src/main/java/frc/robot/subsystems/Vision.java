/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends SubsystemBase
{
    private NetworkTableInstance instance;
    private UsbCamera driverCam;

    // Variables for calculating distance
    private final double OUTER_PORT_HEIGHT = 98.25; // Outer port height above carpet in inches
    private final double LIMELIGHT_MOUNT_ANGLE = 5; // Angle that the Limelight is mounted at
    private final double LIMELIGHT_HEIGHT = 45; // Limelight height above the ground in inches

    public Vision()
    {
        instance = NetworkTableInstance.getDefault();
    }

    public double getLimelightValue(String key)
    {
        return instance.getDefault().getTable("limelight").getEntry(key).getDouble(0);
    }

    public double getTargetDistance()
    {
        double angleToTarget = getLimelightValue("ty");
        return (OUTER_PORT_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(LIMELIGHT_MOUNT_ANGLE + angleToTarget);
    }

    public void initUSBCamera()
    {
        try
        {
            driverCam = CameraServer.getInstance().startAutomaticCapture();
            driverCam.setResolution(160, 160);
            driverCam.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
            driverCam.setFPS(15);
            driverCam.setExposureManual(50);
        }
        catch (Exception e)
        {

        }
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
