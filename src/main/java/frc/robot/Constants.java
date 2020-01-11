/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static int leftJoystick = 0;
    public static int rightJoystick = 1;
    public static int xBoxController = 2;

    public static class DriveMotors {
        public static int leftFrontDriveMotor = 20;
        public static int leftRearDriveMotor = 21;
        public static int rightFrontDriveMotor = 22;
        public static int rightRearDriveMotor = 23;
        public static int climbDriveMotor = 24;
    }

    public static double ticksPerMeter = Units.inchesToMeters(6 * Math.PI) / 4096;

    public static enum PCM_ONE {
        DRIVETRAIN_SIFTER (0, 1),
        HATCH_EXTEND (4, 5),
        //    	HATCH_SECURE (4, 5),
        CLIMB_PISTONS (2, 3);

        public int FORWARD;
        public int REVERSE;

        private PCM_ONE(int forward, int reverse){
            this.FORWARD = forward;
            this.REVERSE = reverse;
        }

        public static int CAN_ADDRESS = 11;
    }

}
