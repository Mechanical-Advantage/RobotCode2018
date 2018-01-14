/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6328.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	public static int rightMaster;
	public static int rightSlave;
	public static int rightSlave2;
	public static int leftMaster;
	public static int leftSlave;
	public static int leftSlave2;
	public static int minVelocity; // lower values will be treated as this value, RPM
	public static int maxVelocity; // maximum velocity when sticks are fully forward (value of 1), RPM
	public static int maxAcceleration;
	public static final boolean tuningMode = false;
	public static final RobotType robot = RobotType.PRACTICE;
	
	public RobotMap() {
		switch (robot) {
		case PRACTICE:
			rightMaster = 1;
			rightSlave = 2;
			leftMaster = 3;
			leftSlave = 4;
			maxVelocity = 950; // 950 native units per 100ms
			minVelocity = 40; // 40 native units per 100ms
			maxAcceleration = 300;
			break;
		case ROBOT_2017:
			rightMaster = 14;
			rightSlave = 13;
			rightSlave2 = 12;
			leftMaster = 15;
			leftSlave = 0;
			leftSlave2 = 1;
			maxVelocity = 525;
			minVelocity = 20;
			maxAcceleration = 300;
			break;
		case ROBOT_2018:
			break;
		default:
			break;
		}
	}
	
	public enum RobotType {
		ROBOT_2018,
		ROBOT_2017,
		PRACTICE
	}
}
