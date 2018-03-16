/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6328.robot;

import org.usfirst.frc.team6328.robot.RobotMap.RobotType;
import org.usfirst.frc.team6328.robot.commands.ArmMoveAndReset;
import org.usfirst.frc.team6328.robot.commands.Climb;
import org.usfirst.frc.team6328.robot.commands.DriveToCube;
import org.usfirst.frc.team6328.robot.commands.EjectCubeForTime;
import org.usfirst.frc.team6328.robot.commands.ExtendIntake;
import org.usfirst.frc.team6328.robot.commands.IntakeCube;
import org.usfirst.frc.team6328.robot.commands.ResetArm;
import org.usfirst.frc.team6328.robot.commands.RetractIntake;
import org.usfirst.frc.team6328.robot.commands.SetCamera;
import org.usfirst.frc.team6328.robot.commands.SetElevatorPosition;
import org.usfirst.frc.team6328.robot.commands.SwitchElevatorGear;
import org.usfirst.frc.team6328.robot.commands.SwitchGear;
import org.usfirst.frc.team6328.robot.commands.ThrowCube;
import org.usfirst.frc.team6328.robot.commands.ToggleGear;
import org.usfirst.frc.team6328.robot.commands.ToggleIntakeOpen;
import org.usfirst.frc.team6328.robot.subsystems.DriveTrain.DriveGear;
import org.usfirst.frc.team6328.robot.subsystems.Elevator.ElevatorGear;
import org.usfirst.frc.team6328.robot.subsystems.Elevator.ElevatorPosition;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
	
	/*
	 * Controls needed:
	 * Elevator gear switch
	 * Elevator joystick
	 * Elevator positions
	 * Intake manual force intake
	 * Intake manual grab
	 * Intake eject
	 * Intake normal intake+hold? (Might also be default command)
	 */
	private boolean joysticksReversed = false;
	
	// map left stick to ID 0 and right to ID 1 in driver station
	private Joystick leftController = new Joystick(0);
	private Joystick rightController = new Joystick(1);
	private Joystick oiController1 = new Joystick(2);
	@SuppressWarnings("unused")
	private Joystick oiController2 = new Joystick(3);
	
	private Button frontCameraButton = new JoystickButton(rightController, 3);
	private Button rearCameraButton = new JoystickButton(rightController, 2);
	@SuppressWarnings("unused")
	private Button joysticksForward = new JoystickButton(leftController, 3);
	@SuppressWarnings("unused")
	private Button joysticksBackward = new JoystickButton(leftController, 2);
	private Button sniperMode = new JoystickButton(rightController, 1);
	private Button toggleGear = new JoystickButton(leftController, 1);
	private Button openLoopDrive = new JoystickButton(oiController2, 10);
	private Button driveDisableSwitch = new JoystickButton(oiController2, 9);
	private Button shiftDisableSwitch = new JoystickButton(oiController2, 8);
	private Button cubeSenseDisableSwitch = new JoystickButton(oiController2, 7);
	private Button elevatorLimitDisableSwitch = new JoystickButton(oiController1, 12);
	private Button driveToCube = new JoystickButton(rightController, 4);
	private Button highGear = new JoystickButton(leftController, 5);
	private Button lowGear = new JoystickButton(leftController, 4);
	private Button highElevatorGear = new JoystickButton(oiController1, 9);
	private Button lowElevatorGear = new JoystickButton(oiController1, 10);
	private Button climb = new JoystickButton(oiController1, 11);
	
	// Elevator Heights
	private Button elevGround = new JoystickButton(oiController1, 6);
	private Button elevSwitch = new JoystickButton(oiController1, 5);
	private Button elevDrive = new JoystickButton(oiController1, 4);
	private Button elevScaleL = new JoystickButton(oiController1, 3);
	private Button elevScaleH = new JoystickButton(oiController1, 2);
	private Button elevClimbGrab = new JoystickButton(oiController1, 1);
	
	// Everybot
	private Button scoreCube = new JoystickButton(oiController1, 11);
	private Button raiseArm = new JoystickButton(oiController1, 9);
	private Button lowerArm = new JoystickButton(oiController1, 10);
	
	private Button startIntake = new JoystickButton(oiController2, 4);
	private Button stopIntake = new JoystickButton(oiController2, 3);
	private Button retractIntake = new JoystickButton(oiController2, 1);
	private Button ejectCubeTime = new JoystickButton(oiController2, 5);
	private Button toggleIntakeOpen = new JoystickButton(oiController2, 2);
	
	NetworkTable ledTable;
	NetworkTableEntry ledEntry;

	public OI() {
		ledTable = NetworkTableInstance.getDefault().getTable("LEDs");
		ledEntry = ledTable.getEntry("OI LEDs");
		
		frontCameraButton.whenPressed(new SetCamera(true));
		rearCameraButton.whenPressed(new SetCamera(false));
//		joysticksForward.whenPressed(new SetCamera(true));
//		joysticksBackward.whenPressed(new SetCamera(false));
//		joysticksForward.whenPressed(new ReverseJoysticks(false));
//		joysticksBackward.whenPressed(new ReverseJoysticks(true));
		driveToCube.whileHeld(new DriveToCube());
		highGear.whenPressed(new SwitchGear(DriveGear.HIGH));
		lowGear.whenPressed(new SwitchGear(DriveGear.LOW));
		highElevatorGear.whenPressed(new SwitchElevatorGear(ElevatorGear.HIGH));
		lowElevatorGear.whenPressed(new SwitchElevatorGear(ElevatorGear.LOW));
		toggleGear.whenPressed(new ToggleGear());
		if (RobotMap.robot == RobotType.EVERYBOT_2018) {
			scoreCube.whenPressed(new ArmMoveAndReset());
			raiseArm.whenPressed(new ThrowCube());
			lowerArm.whenPressed(new ResetArm());
		}
		IntakeCube intakeCommand = new IntakeCube(false);
		startIntake.whenPressed(intakeCommand);
		stopIntake.whenPressed(new ExtendIntake());
		retractIntake.whenPressed(new RetractIntake());
		toggleIntakeOpen.whenPressed(new ToggleIntakeOpen());
		ejectCubeTime.whenPressed(new EjectCubeForTime());
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			climb.whenPressed(new Climb());
		}
		
		elevGround.whenPressed(new SetElevatorPosition(ElevatorPosition.GROUND));
		elevSwitch.whenPressed(new SetElevatorPosition(ElevatorPosition.SWITCH));
		elevDrive.whenPressed(new SetElevatorPosition(ElevatorPosition.DRIVE));
		elevScaleL.whileHeld(new SetElevatorPosition(ElevatorPosition.SCALE_LOW, false));
		elevScaleL.whenReleased(new SetElevatorPosition(ElevatorPosition.DRIVE));
		elevScaleH.whileHeld(new SetElevatorPosition(ElevatorPosition.SCALE_HIGH, false));
		elevScaleH.whenReleased(new SetElevatorPosition(ElevatorPosition.DRIVE));
		elevClimbGrab.whenPressed(new SetElevatorPosition(ElevatorPosition.CLIMB_GRAB));
	}
	
	public double getLeftAxis() {
		if (joysticksReversed) {
			return rightController.getRawAxis(1)*-1;
		} else {
			return leftController.getRawAxis(1);
		}
	}
	public double getRightAxis() {
		if (joysticksReversed) {
			return leftController.getRawAxis(1)*-1;
		} else {
			return rightController.getRawAxis(1);
		}
	}
	
	public double getArmAxis() {
		return oiController1.getY();
	}
	
	// reversing the joysticks should not change which joystick to use for straight drive, use
	// different function to make that correct
	// Note: Brian is left-handed
	public double getSingleDriveAxis() {
		if (joysticksReversed) {
			return leftController.getRawAxis(1)*-1;
		} else {
			return leftController.getRawAxis(1);
		}
	}
	public double getHorizDriveAxis() {
		return rightController.getRawAxis(0);
	}
	
	public boolean getOpenLoop() {
		return openLoopDrive.get();
	}
	
	public boolean getDriveEnabled() {
		return !driveDisableSwitch.get();
	}
	
	public boolean getSniperMode() {
		return sniperMode.get();
	}
	
	public double getSniperLevel() {
		double sniperLimit = 0.5;
		return (1-((rightController.getRawAxis(2)+1)/2))*sniperLimit; // control returns -1 to 1, scale to 0 to 1, subtract from 1 so 1 is up
	}
	
	public double getIntakeLevel() {
		return (1-((leftController.getRawAxis(2)+1)/2)); // control returns -1 to 1, scale to 0 to 1, subtract from 1 so 1 is up
	}
	
	public void reverseJoysticks(boolean reverse) {
		joysticksReversed = reverse;
	}
	
	public double getElevatorJoystick() {
		return oiController1.getRawAxis(1);
	}
	
	public double getEjectForce() {
		return oiController2.getRawAxis(1)/2+0.5;
	}
	
	public boolean isShiftingEnabled() {
		return !shiftDisableSwitch.get();
	}
	
	public boolean isCubeSensorEnabled() {
		return !cubeSenseDisableSwitch.get();
	}
	
	public boolean isElevatorLimitEnabled() {
		return !elevatorLimitDisableSwitch.get();
	}
	
	
	public void updateLED(OILED led, boolean state) {
		boolean[] array = ledTable.getEntry("OI LEDs").getBooleanArray(new boolean[]{false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false});
		array[led.ordinal()] = state;
		ledEntry.setBooleanArray(array);
	}
	
	public enum OILED {
		CUBE_SENSE_1, CUBE_SENSE_2, CUBE_SENSE_3, INTAKE_OPEN, INTAKE_RETRACT, INTAKE_OFF, INTAKE_ON, ELEVATOR_LOW_GEAR, ELEVATOR_HIGH_GEAR,
		ELEVATOR_BRAKE, CLIMB, ELEVATOR_GROUND, ELEVATOR_SWITCH, ELEVATOR_DRIVE, ELEVATOR_SCALE_LOW, ELEVATOR_SCALE_HIGH, CLIMB_GRAB
	}
}
