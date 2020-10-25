package frc.robot.commands.autonomous.routines;

import java.util.function.DoubleSupplier;

public class GetSOTMTestPowers{

    private double maxPower, h, aValue;

    // Uses a parabola to accelerate left and right motor powers
    // Max powers are height of vertex, time is width
    public GetSOTMTestPowers(double maxPower, double timeToFinish) {
        this.maxPower = maxPower;
        this.h = timeToFinish / 2;
        this.aValue = -maxPower / Math.pow(h, 2);
    }

    public double getVariablePower(double timePassed) {
        return this.aValue * Math.pow(timePassed - h, 2) + this.maxPower;
    }



}