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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase
{
    private NetworkTable limelight; //Field-of-View: 59.6 x 49.7 degrees. Tracking Resolution: 320 x 240 pixels.
    private UsbCamera driverCam;

    // Variables for calculating distance
    private final double TARGET_HEIGHT = 98.25; // Outer port height above carpet in inches
    private final double LIMELIGHT_MOUNT_ANGLE = 26.5; // Angle that the Limelight is mounted at
    private final double LIMELIGHT_HEIGHT = 37.31; // Limelight height above the ground in inches

    public Vision() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        setPipeline(1);
    }

    public double getTargetY() {
        return limelight.getEntry("ty").getDouble(0);
    }

    public double getTargetX() {
        return limelight.getEntry("tx").getDouble(0);
    }

    public boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0) == 1;
    }

    public double getTargetArea() {
        return limelight.getEntry("ta").getDouble(0);
    }

    public double getTargetSkew() {
        return limelight.getEntry("ts").getDouble(0);
    }

    public double getPipelineLatency() {
        return limelight.getEntry("tl").getDouble(0);
    }

    public double getTargetShort() {
        return limelight.getEntry("tshort").getDouble(0);
    }

    public double getTargetLong() {
        return limelight.getEntry("tlong").getDouble(0);
    }

    public double getHorizontalSidelength() {
        return limelight.getEntry("thor").getDouble(0);
    }

    public double getVerticalSidelength() {
        return limelight.getEntry("tvert").getDouble(0);
    }

    public double getPipeline() {
        return limelight.getEntry("getpipe").getDouble(0);
    }

    public void ledsOn() {
        limelight.getEntry("ledMode").setNumber(3);
    }

    public void ledsOff() {
        limelight.getEntry("ledMode").setNumber(1);
    }

    public void setPipeline(int pipeline) {
        limelight.getEntry("pipeline").setNumber(pipeline);
    }

    public double getTargetDistance() {
        double angleToTarget = getTargetY();
        return (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(LIMELIGHT_MOUNT_ANGLE + angleToTarget);
    }

    public void initUSBCamera() {
        try {
            driverCam = CameraServer.getInstance().startAutomaticCapture();
            driverCam.setResolution(160, 160);
            driverCam.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
            driverCam.setFPS(15);
            driverCam.setExposureManual(50);
        }
        catch (Exception e) {

        }
    }

    public void openSightInit()
    {
        // Seems to be necessary to get OpenSight cam to show up in Shuffleboard
        // CameraServer.getInstance().addAxisCamera("opensight", "opensight.local");
        CameraServer.getInstance().addServer("opensight.local");
    }

    public void updateSmartDashboard()
    {
        SmartDashboard.putBoolean("Can see target", hasTarget());
        SmartDashboard.putNumber("Limelight Target Distance", getTargetDistance());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateSmartDashboard();
    }
}
