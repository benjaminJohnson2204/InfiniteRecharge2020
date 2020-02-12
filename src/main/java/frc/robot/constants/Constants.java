/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

import java.io.File;
import java.net.NetworkInterface;
import java.net.SocketException;

import frc.robot.constants.Enums.ROBOT;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static void initRobot() {
    	String macAddress = getMacAddress();
    	readRobotConfig(macAddress);
    }
    
	private static final String greeMacAddress = "";
	private static final String jangoMacAddress = "";
	
    private String void getMacAddress() {
    	// Get MAC Address
        try {
            //Enumeration<NetworkInterface> netInterfaces = NetworkInterface.getNetworkInterfaces();
            NetworkInterface neti = NetworkInterface.getByName("eth0");
            byte[] mac = neti.getHardwareAddress();

            if(mac == null){ //happens on windows sometimes
                throw new SocketException();
            }

            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < mac.length; i++) 
                sb.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));        
            return sb.toString();
        } catch (SocketException e){
            return "";
        }
    }
    
    private void readRobotConfig(String macAddress) {
    	String robotName;
    	switch(macAddress) {
    		case greeMacAddress:
    			robotName = "Gree";
    				break;
    		default:
    		case jangoMacAddress:
    			robotName = "Jango";
    			break;
    	}
	 File iniFile = new File("/home/lvuser/deploy/" + robotName + ".ini");
	 robotConfigs = new Wini(iniFile);
    }
    
	private static ROBOT robot;
	private static Wini robotConfigs;
	
	// USB PORTS
    public static final int leftJoystick = robotConfigs.get("USB_PORTS", "leftJoystick");
    public static final int rightJoystick = robotConfigs.get("USB_PORTS", "rightJoystick");
    public static final int xBoxController = robotConfigs.get("USB_PORTS", "xBoxController");

    // CAN ADDRESSES
    public static final int pdp = 0;
    public static final int pcmOne = 11;
    public static final int leftFrontDriveMotor = 20;
    public static final int leftRearDriveMotor = 21;
    public static final int rightFrontDriveMotor = 22;
    public static final int rightRearDriveMotor = 23;
    public static final int intakeMotor = 30;
    public static final int indexerMotor = 35;
    public static final int kickerMotor = 36;
    public static final int flywheelMotorA = 40;
    public static final int flywheelMotorB = 41;
    public static final int climbMotorA = 50;
    public static final int climbMotorB = 51;
    public static final int skyhookMotor = 55;
    public static final int turretMotor = 60;
    public static final int turretEncoder = 61;

    // PWM
    public static final int ledPort = 0;

    // DIO
    public static final int intakeSensor = 0;
    public static final int indexerTopSensor = 1;
    public static final int indexerBottomSensor = 2;
    public static final int turretHomeSensor = 3;

    //Solenoid addresses
    public static final int driveTrainShiftersForward = 0;
    public static final int driveTrainShiftersReverse = 1;
    public static final int intakePistonForward = 4; // 2
    public static final int intakePistonReverse = 5; // 3
    public static final int climbPistonForward = 6;
    public static final int climbPistonReverse = 7;
}
