/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.net.PortForwarder;
import frc.robot.simulation.SimConstants;

/*
Subsystem for interacting with the Limelight and OpenSight vision systems
 */

public class Vision extends SubsystemBase {
    // Variables for calculating distance
    private final double TARGET_HEIGHT = 98.25; // Outer port height above carpet in inches
    private final double LIMELIGHT_MOUNT_ANGLE = 32; // Angle that the Limelight is mounted at
    private final double LIMELIGHT_HEIGHT = 37.31; // Limelight height above the ground in inches

    private final double MIN_TARGET_DISTANCE = 1;
    private final double INNER_PORT_SLOPE = 1;
    private final double INNER_PORT_OFFSET = 1;

    private final double HORIZONTAL_TARGET_PIXEL_WIDTH = 1;
    private final double HORIZONTAL_TARGET_PIXEL_THRESHOLD = 1;
    private final double VERTICAL_TARGET_PIXEL_WIDTH = 1;
    private final double VERTICAL_TARGET_PIXEL_THRESHOLD = 1;

    // NetworkTables for reading vision data
    private final NetworkTable limelight;
    private final NetworkTable photonVision;
    private final PhotonCamera photonCamera;

    // Subsystems that will be controlled based on vision data
    private final DriveTrain m_driveTrain;
    private final Turret m_turret;
    double[] distances = new double[5];
    double[] counts = new double[5];
    int index = 0;
    // Filters to prevent target values from oscillating too much
    SlewRateLimiter targetXFilter = new SlewRateLimiter(20);
    SlewRateLimiter innerTargetXFilter = new SlewRateLimiter(20);
    UsbCamera camera;
    private boolean resetPose;
    private double lastValidTargetTime;
    private boolean validTarget;

    String camName = "MyCamera";
    double camDiagFOV = 75.0; // degrees
    double camPitch = -30;     // degrees
    Transform2d cameraToRobot = new Transform2d(new Translation2d(-0.838 / 2, 0.0), new Rotation2d()); // meters
    double camHeightOffGround = 0.85; // meters
    double maxLEDRange = 20;          // meters
    int camResolutionWidth = 640;     // pixels
    int camResolutionHeight = 480;    // pixels
    double minTargetArea = 10;        // square pixels

    SimVisionSystem visionSys = new SimVisionSystem(camName,
                                camDiagFOV,
                                camPitch,
                                cameraToRobot,
                                camHeightOffGround,
                                maxLEDRange,
                                camResolutionWidth,
                                camResolutionHeight,
                                minTargetArea);

    public Vision(DriveTrain driveTrain, Turret turret) {
        m_driveTrain = driveTrain;
        m_turret = turret;

		if(RobotBase.isReal()) {
//		camera = CameraServer.getInstance().startAutomaticCapture();
			camera = CameraServer.getInstance().startAutomaticCapture("intake", "/dev/video0");
			camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
			camera.setExposureManual(25);
			camera.setResolution(320, 240);
            camera.setPixelFormat(VideoMode.PixelFormat.kMJPEG);
		}
		//CameraServer.getInstance().addAxisCamera("opensight", "opensight.local");

        // TODO: What port does opensight use?
        PortForwarder.add(6000, "opensight.local", 22);
        PortForwarder.add(5800, "10.42.1.11", 5800);
        PortForwarder.add(5801, "10.42.1.11", 5801);
        PortForwarder.add(5805, "10.42.1.11", 5805);

        // Init vision NetworkTables
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        //openSight = NetworkTableInstance.getDefault().getTable("OpenSight");
        photonVision = NetworkTableInstance.getDefault().getTable("PhotonVision");
        photonCamera = new PhotonCamera("photonvision");

        setPipeline(0);

        //initShuffleboard();
    }

    private void updateValidTarget() {
        // Determine whether the limelight has detected a valid target and not a random reflection
        // If the target is seen for a specific amount of time it is marked as valid
        if(hasTarget()) {
            setLastValidTargetTime();
        }
        if((Timer.getFPGATimestamp() - lastValidTargetTime) < 3) {
            ledsOn();
            validTarget = true;
        } else {
            ledsOff();
            validTarget = false;
        }
    }

    public SimVisionSystem getVisionSim() {
        return visionSys;
    }

    public boolean getValidTarget() {
        return validTarget;
    }

    public void setLastValidTargetTime() {
        lastValidTargetTime = Timer.getFPGATimestamp();
    }

    // Limelight interaction functions
    public double getTargetY() {
        return limelight.getEntry("ty").getDouble(0);
    }

    public double getTargetX() {
        return limelight.getEntry("tx").getDouble(0);
    }

    public double getFilteredTargetX() {
        return targetXFilter.calculate(getTargetX());
    }

    public double getSmartTargetX() {
        if(getTargetDistance() > MIN_TARGET_DISTANCE) {
            double xDistance = Units.metersToFeet(m_driveTrain.getRobotPose().getTranslation().getX());
            double yDistance = Math.abs(Units.metersToFeet(m_driveTrain.getRobotPose().getTranslation().getY()));

            double maxYDistance = INNER_PORT_SLOPE * xDistance + INNER_PORT_OFFSET;

            if(yDistance < maxYDistance) {
                xDistance += 29.25 / 12.0;
                return innerTargetXFilter.calculate(Math.signum(getFilteredTargetX()) * Units.radiansToDegrees(Math.atan(xDistance / yDistance)));
            }
        }

        return getFilteredTargetX();
    }

    private void resetPoseByVision() {
        if(! resetPose) {
            if((Math.abs(getHorizontalSidelength() - HORIZONTAL_TARGET_PIXEL_WIDTH) < HORIZONTAL_TARGET_PIXEL_THRESHOLD) &&
                    (Math.abs(getVerticalSidelength() - VERTICAL_TARGET_PIXEL_WIDTH) < VERTICAL_TARGET_PIXEL_THRESHOLD)) {
                double targetRadians = Units.degreesToRadians(m_turret.getFieldRelativeAngle());
                double xDistance = Math.abs(Math.cos(targetRadians)) * getTargetDistance();
                double yDistance = - Math.signum(getFilteredTargetX()) * Math.abs(Math.sin(targetRadians)) * getTargetDistance();

                m_driveTrain.resetOdometry(new Pose2d(xDistance, yDistance, new Rotation2d()),
                        Rotation2d.fromDegrees(m_driveTrain.getHeading()));

                resetPose = true;
            }
        } else if(resetPose && ! hasTarget()) {
            resetPose = false;
        }
    }

    // More Limelight interaction functions

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

    public void setPipeline(int pipeline) {
        limelight.getEntry("pipeline").setNumber(pipeline);
    }

    public void ledsOn() {
        limelight.getEntry("ledMode").setNumber(3);
    }

    public void ledsOff() {
        limelight.getEntry("ledMode").setNumber(1);
    }

    // Calculate target distance based on field dimensions and the angle from the Limelight to the target
    public double getTargetDistance() {
        if (RobotBase.isReal()) {
            double angleToTarget = getPipeline() > 0 ? getTargetY() - 12.83 : getTargetY();

            double inches = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(LIMELIGHT_MOUNT_ANGLE + angleToTarget));
            distances[index++ % distances.length] = inches / 12.0;
    
            return computeMode(distances);
        } else {
            return Units.metersToFeet(m_turret.getIdealTargetDistance());
        }
        
    }

	public double getAngleToTarget() {
        if (RobotBase.isReal()) {
            return getPipeline() > 0 ? getTargetY() - 12.83 : getTargetY();
        } else {
            return m_turret.getIdealTurretAngle();
        }
    }
    
    // For Shoot on the Move, gets horizontal angle on field to target
    public double getHorizontalAngleToTarget() {
        return getTargetX();
            // TODO: Figure out what to add/subtract if we're zoomed in
    }

    // Used to find the most common value to provide accurate target data
    private double computeMode(double[] data) {
        // Compute mode
        this.counts = new double[data.length];
        for(int i = 0; i < data.length; i++) {
            for(double datum : data) {
                if(data[i] == datum) {
                    this.counts[i]++;
                }
            }
        }

        int highestIndex = 0;
        double previousHigh = 0;
        for(int i = 0; i < this.counts.length; i++) {
            if(this.counts[i] > previousHigh) {
                highestIndex = i;
                previousHigh = this.counts[i];
            }
        }

        return data[highestIndex]; // Final distance in feet
    }

    // Read ball position data from OpenSight (Raspberry Pi)
    // Returns angle in degrees from center of camera (right is positive)
    public double getPowerCellX() {
        // TODO: Calculate degrees from pixels?
        // return openSight.getEntry("found-x").getDouble(0) * 5.839; // 5.839 pixels per degree
        return photonVision.getEntry("targetYaw").getDouble(0);
    }

    // Decides, based on power cell locations, which color (red or blue) and path (A or B) to run
    // 0 = red A, 1 = blue A, 2 = red B, 3 = blue B, -1 = can't determine path
    public int galacticSearchPath() {
        if (!photonCamera.hasTargets()) {
            SmartDashboard.putString("Galactic Search Path", "None");
            return -1;
        }
        var targets = photonCamera.getLatestResult().getTargets();
        if (Math.abs(targets.get(0).getYaw()) < 10) {
            if (m_driveTrain.getHeading() > 3 && m_driveTrain.getHeading() < 40) {
                SmartDashboard.putString("Galactic Search Path", "Red B");
                return 2;
            }
            if (m_driveTrain.getHeading() > -3 && m_driveTrain.getHeading() < 3) {
                SmartDashboard.putString("Galactic Search Path", "Red A");
                return 0;
            }
        } else {
            if (m_driveTrain.getHeading() < -3 && m_driveTrain.getHeading() > -30) {
                SmartDashboard.putString("Galactic Search Path", "Blue A");
                return 1;
            }
            if (m_driveTrain.getHeading() > -3 && m_driveTrain.getHeading() < 3) {
                SmartDashboard.putString("Galactic Search Path", "Blue B");
                return 3;
            }
        }
        SmartDashboard.putString("Galactic Search Path", "None");
        return -1;
    }

    public boolean hasPowerCell() {
        //return photonVision.getEntry("found").getBoolean(false);
        return photonVision.getEntry("hasTarget").getBoolean(false);
    }

    private void initShuffleboard() {
        // Unstable. Don''t use until WPILib fixes this
        Shuffleboard.getTab("Turret").addBoolean("Vision Valid Output", this :: getValidTarget);
        Shuffleboard.getTab("Turret").addNumber("Vision Target X", this :: getFilteredTargetX);

    }

    public void updateSmartDashboard() {
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
        SmartDashboard.putNumber("Limelight Target X", getTargetX());
        SmartDashboard.putNumber("Limelight Target Distance", getTargetDistance());
        SmartDashboard.putNumber("Limelight Pipeline", getPipeline());

        SmartDashboardTab.putBoolean("Turret", "Vision Valid Output", getValidTarget());
        SmartDashboardTab.putNumber("Turret", "Vision Target X", getFilteredTargetX());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateSmartDashboard();
        updateValidTarget();
        galacticSearchPath();
        if (RobotBase.isSimulation())
            visionSys.processFrame(m_driveTrain.getRobotPose());

        //resetPoseByVision();
    }
}
