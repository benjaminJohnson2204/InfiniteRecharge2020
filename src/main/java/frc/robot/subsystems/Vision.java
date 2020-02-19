/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.net.PortForwarder;
import frc.robot.constants.Constants;

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

    private final String[] SOUNDS = {
            "target.chrp",
            "tip.chrp",
            "tip2.chrp",
            "enable.chrp",
            "wii.chrp",
            "megolovania.chrp",
            "imperial.chrp"
    };

    private final TalonFX[] TALONS = {
        new TalonFX(Constants.leftRearDriveMotor),
        new TalonFX(Constants.rightRearDriveMotor),
        new TalonFX(Constants.rightFrontDriveMotor),
        new TalonFX(Constants.leftFrontDriveMotor),
        new TalonFX(Constants.flywheelMotorA),
        new TalonFX(Constants.flywheelMotorB)
    };

    private Orchestra orchestra;

    public Vision() {
        PortForwarder.add(5800, "10.42.1.11", 5800);
        PortForwarder.add(5801, "10.42.1.11", 5801);

        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        openSight = NetworkTableInstance.getDefault().getTable("OpenSight");
        setPipeline(1);

        for (TalonFX talon : TALONS) {
            orchestra.addInstrument(talon);
        }
    }

    private void updateValidTarget() {
        if (hasTarget() || true) {
            setLastValidTargetTime();
        }
        if ((Timer.getFPGATimestamp() - lastValidTargetTime) < 5) {
            ledsOn();
            validTarget = true;
        } else {
            ledsOff();
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

    public void updateSmartDashboard() {
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
        SmartDashboard.putNumber("Limelight Target X", getTargetX());
        SmartDashboard.putNumber("Limelight Target Distance", getTargetDistance());
        SmartDashboard.putNumber("Limelight Pipeline", getPipeline());
        SmartDashboard.putNumber("Powercell X", getPowerCellX());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateSmartDashboard();
        updateValidTarget();
        song("megolovania.chrp");
        // targetSound();
    }

    private void targetSound() {
        if (hasPowerCell() || hasTarget()) {
            orchestra.stop();
            orchestra.loadMusic(SOUNDS[0]);
            orchestra.play();
        }
    }

    /* TODO
    public void randomSong() {
        // Inspired by https://www.reddit.com/r/FRC/comments/f4tek6/our_intimidation_factor_is_ready/
        orchestra.loadMusic(SOUNDS[((int)Math.random() % SOUNDS.length) + 1]);
    }
    */

    public void song(int index) {
        if (!orchestra.isPlaying()) {
            orchestra.loadMusic(SOUNDS[index]);
            orchestra.play();
        }
    }

    public void song(String name) {
        if (!orchestra.isPlaying()) {
            orchestra.loadMusic(name);
            orchestra.play();
        }
    }
}
