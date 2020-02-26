/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.net.PortForwarder;

public class Vision extends SubsystemBase {
	private NetworkTable limelight;
	private NetworkTable openSight;

	// Variables for calculating distance
	private final double TARGET_HEIGHT = 98.25; // Outer port height above carpet in inches
	private final double LIMELIGHT_MOUNT_ANGLE = 32; // Angle that the Limelight is mounted at
	private final double LIMELIGHT_HEIGHT = 37.31; // Limelight height above the ground in inches

	private double lastValidTargetTime;
	private boolean validTarget;

	double[] distances = new double[5];
	double[] counts = new double[5];
	int index = 0;

	SlewRateLimiter targetXFilter = new SlewRateLimiter(20);

	public Vision() {
	    CameraServer.getInstance().addAxisCamera("opensight", "opensight.local");

	    // TODO: What port does opensight use?
		PortForwarder.add(6000, "opensight.local", 80);
		PortForwarder.add(5800, "10.42.1.11", 5800);
		PortForwarder.add(5801, "10.42.1.11", 5801);

		limelight = NetworkTableInstance.getDefault().getTable("limelight");
		openSight = NetworkTableInstance.getDefault().getTable("OpenSight");
		setPipeline(1);

		//initShuffleboard();
	}

	private void updateValidTarget() {
		if (hasTarget()) {
			setLastValidTargetTime();
		}
		if ((Timer.getFPGATimestamp() - lastValidTargetTime) < 5) {
			ledsOn();
			validTarget = true;
		} else {
			//ledsOff();
			validTarget = false;
		}
	}

	public boolean getValidTarget() {
		return validTarget;
	}

	public void setLastValidTargetTime() {
		lastValidTargetTime = Timer.getFPGATimestamp();
	}

	public double getTargetY() {
		return limelight.getEntry("ty").getDouble(0);
	}

	public double getTargetX() {
		return limelight.getEntry("tx").getDouble(0);
	}

	public double getFilteredTargetX() {
		return targetXFilter.calculate(getTargetX());
	}
	
	public boolean isValidInnerPort() {
		// TODO: Solve this problem
		/* Imagine the valid area where you can aim at the 
		 * Inner Port as a triangle. Given the limelight's distance
		 * to the center of the Inner Port target and the angle it returns,
		 * how do I return true when I'm in the valid shooting area?
		 * .......*-------- Inner Port
		 * ....../ \
		 * ...../___\------ Outer Port
		 * ..../     \
		 * .../       \
		 * ../         \--- Valid Area to shoot from 
		 * ./           \
		 * /_____________\
		 * 
		 */
		double angle = 90.0 - Math.abs(getTargetY());
		double distance = getTargetDistance();
		
		double x = Math.sin(angle) * distance;
		double y = Math.cos(angle) * distance;
		
		double ratio = x / y;
		
		return false;
	}

	public double getInnerTargetX() {
		// TODO: Add adjustment for inner port
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
		double angleToTarget = getPipeline() > 0 ? getTargetY() - 12.83 : getTargetY();

		double inches = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(LIMELIGHT_MOUNT_ANGLE + angleToTarget));
		distances[index++ % distances.length] = inches / 12.0;

		return computeMode(distances);
	}

	private double computeMode(double[] data) {
		// Compute mode
		this.counts = new double[data.length];
		for (int i = 0; i < data.length; i++) {
			for (int j = 0; j < data.length; j++) {
				if (data[i] == data[j]) {
					this.counts[i]++;
				}
			}
		}

		int highestIndex = 0;
		double previousHigh = 0;
		for (int i = 0; i < this.counts.length; i++) {
			if (this.counts[i] > previousHigh) {
				highestIndex = i;
				previousHigh = this.counts[i];
			}
		}

		return data[highestIndex]; // Final distance in feet
	}

    public double getPowerCellX() {
        double pixels = openSight.getEntry("found-x").getDouble(0);

        // Convert to degrees (5.839 pixels per degree)
        return pixels * 5.839;
    }

	public boolean hasPowerCell() {
		return openSight.getEntry("found").getBoolean(false);
	}

	private void initShuffleboard() {
		// Unstable. Don''t use until WPILib fixes this
		Shuffleboard.getTab("Turret").addBoolean("Vision Valid Output", this::getValidTarget);
		Shuffleboard.getTab("Turret").addNumber("Vision Target X", this::getFilteredTargetX);

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
	}
}
