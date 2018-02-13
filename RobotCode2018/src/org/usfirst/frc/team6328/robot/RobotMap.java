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
	public static int scoringArm;
	public static int elevatorMaster;
	public static int elevatorSlave1;
	public static int elevatorSlave2;
	public static int elevatorSlave3;
	public static int elevatorBrakeSolenoid1;
	public static int elevatorBrakeSolenoid2;
	public static int elevatorBrakePCM;
	public static int elevatorStageLockSolenoid1;
	public static int elevatorStageLockSolenoid2;
	public static int elevatorStageLockPCM;
	public static int elevatorGearSolenoid1;
	public static int elevatorGearSolenoid2;
	public static int elevatorGearPCM;
	public static int intakeLeft;
	public static int intakeRight;
	public static int intakeWeak1Solenoid1;
	public static int intakeWeak1Solenoid2;
	public static int intakeWeak1PCM;
	public static int intakeWeak2Solenoid1;
	public static int intakeWeak2Solenoid2;
	public static int intakeWeak2PCM;
	public static int intakeStrongSolenoid;
	public static int intakeStrongPCM;
	public static int intakeSensor; // DIO
	public static int minVelocityLow; // lower values will be treated as this value, RPM
	public static int maxVelocityLow; // maximum velocity when sticks are fully forward (value of 1), RPM
	public static int maxVelocityHigh;
	public static int minVelocityHigh;
	public static int maxAcceleration;
	public static int topGearSolenoid1;
	public static int topGearSolenoid2;
	public static int gearExpellerSolenoid1;
	public static int gearExpellerSolenoid2;
	public static int leftDriveGearSolenoid1;
	public static int leftDriveGearSolenoid2;
	public static int leftDriveGearPCM;
	public static int rightDriveGearSolenoid1;
	public static int rightDriveGearSolenoid2;
	public static int rightDriveGearPCM;
	public static final boolean tuningMode = true;
	public static final RobotType robot = RobotType.ROBOT_2017;
	
	
	public RobotMap() {
		switch (robot) {
		case PRACTICE:
			rightMaster = 1;
			rightSlave = 2;
			leftMaster = 3;
			leftSlave = 4;
			maxVelocityLow = 950; // 950 native units per 100ms
			minVelocityLow = 40; // 40 native units per 100ms
			maxAcceleration = 300;
			break;
		case ROBOT_2017:
			rightMaster = 14;
			rightSlave = 13;
			rightSlave2 = 12;
			leftMaster = 15;
			leftSlave = 0;
			leftSlave2 = 1;
			maxVelocityLow = 3284; // 525 RPM
			minVelocityLow = 135; // 20 RPM
			maxAcceleration = 300;
			topGearSolenoid1 = 3;
			topGearSolenoid2 = 2;
			gearExpellerSolenoid1 = 0;
			gearExpellerSolenoid2 = 1;
			break;
		case ORIGINAL_ROBOT_2018:
			break;
		default:
			break;
		}
	}
	
	public enum RobotType {
		ORIGINAL_ROBOT_2018,
		EVERYBOT_2018,
		ROBOT_2017,
		PRACTICE
	}
}
