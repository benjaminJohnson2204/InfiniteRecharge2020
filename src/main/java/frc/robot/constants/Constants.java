/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

import java.io.File;
import java.io.IOException;
import java.net.NetworkInterface;
import java.net.SocketException;

import frc.robot.constants.Enums.ROBOT;
import org.ini4j.Wini;

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
    
	private static final String greeMacAddress = "A";
	private static final String jangoMacAddress = "B";
	
    private static String getMacAddress() {
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
    
    private static void readRobotConfig(String macAddress) {
    	String robotName;
    	switch(macAddress) {
    		case greeMacAddress:
    			robotName = "GREE";
    				break;
    		default:
    		case jangoMacAddress:
    			robotName = "JANGO";
    			break;
    	}
	 File iniFile = new File("/home/lvuser/deploy/" + robotName + ".ini");
        try {
            robotConfigs = new Wini(iniFile);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
	private static ROBOT robot;
	private static Wini robotConfigs;
	
	// USB PORTS
    public static final int leftJoystick = robotConfigs.get("USB_PORTS", "leftJoystick", int.class);
    public static final int rightJoystick = robotConfigs.get("USB_PORTS", "rightJoystick", int.class);
    public static final int xBoxController = robotConfigs.get("USB_PORTS", "xBoxController", int.class);

    // CAN ADDRESSES
    public static final int pdp = robotConfigs.get("CAN_ADDRESSES", "pdp", int.class);
    public static final int pcmOne = robotConfigs.get("CAN_ADDRESSES", "pcmOne", int.class);
    public static final int leftFrontDriveMotor = robotConfigs.get("CAN_ADDRESSES", "leftFrontDriveMotor", int.class);
    public static final int leftRearDriveMotor = robotConfigs.get("CAN_ADDRESSES", "leftRearDriveMotor", int.class);
    public static final int rightFrontDriveMotor = robotConfigs.get("CAN_ADDRESSES", "rightFrontDriveMotor", int.class);
    public static final int rightRearDriveMotor = robotConfigs.get("CAN_ADDRESSES", "rightRearDriveMotor", int.class);
    public static final int intakeMotor = robotConfigs.get("CAN_ADDRESSES", "intakeMotor", int.class);
    public static final int indexerMotor = robotConfigs.get("CAN_ADDRESSES", "indexerMotor", int.class);
    public static final int kickerMotor = robotConfigs.get("CAN_ADDRESSES", "kickerMotor", int.class);
    public static final int flywheelMotorA = robotConfigs.get("CAN_ADDRESSES", "flywheelMotorA", int.class);
    public static final int flywheelMotorB = robotConfigs.get("CAN_ADDRESSES", "flywheelMotorB", int.class);
    public static final int climbMotorA = robotConfigs.get("CAN_ADDRESSES", "climbMotorA", int.class);
    public static final int climbMotorB = robotConfigs.get("CAN_ADDRESSES", "climbMotorB", int.class);
    public static final int skyhookMotor = robotConfigs.get("CAN_ADDRESSES", "skyhookMotor", int.class);
    public static final int turretMotor = robotConfigs.get("CAN_ADDRESSES", "turretMotor", int.class);
    public static final int turretEncoder = robotConfigs.get("CAN_ADDRESSES", "turretEncoder", int.class);

    // PWM
    public static final int ledPort = robotConfigs.get("PWM_PORTS", "ledPort", int.class);

    // DIO
    public static final int intakeSensor = robotConfigs.get("DIO_PORTS", "intakeSensor", int.class);
    public static final int indexerTopSensor = robotConfigs.get("DIO_PORTS", "indexerTopSensor", int.class);
    public static final int indexerBottomSensor = robotConfigs.get("DIO_PORTS", "indexerBottomSensor", int.class);
    public static final int turretHomeSensor = robotConfigs.get("DIO_PORTS", "turretHomeSensor", int.class);

    //Solenoid addresses
    public static final int driveTrainShiftersForward = robotConfigs.get("PCMONE_ADDRESSES", "driveTrainShiftersForward", int.class);
    public static final int driveTrainShiftersReverse = robotConfigs.get("PCMONE_ADDRESSES", "driveTrainShiftersReverse", int.class);
    public static final int intakePistonForward = robotConfigs.get("PCMONE_ADDRESSES", "intakePistonForward", int.class);
    public static final int intakePistonReverse = robotConfigs.get("PCMONE_ADDRESSES", "intakePistonReverse", int.class);
    public static final int climbPistonForward = robotConfigs.get("PCMONE_ADDRESSES", "climbPistonForward", int.class);
    public static final int climbPistonReverse = robotConfigs.get("PCMONE_ADDRESSES", "climbPistonReverse", int.class);
}
