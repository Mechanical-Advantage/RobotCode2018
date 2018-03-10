/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6328.robot;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.StringJoiner;

import org.usfirst.frc.team6328.robot.OI.OILED;
import org.usfirst.frc.team6328.robot.RobotMap.RobotType;
import org.usfirst.frc.team6328.robot.commands.CenterAuto;
import org.usfirst.frc.team6328.robot.commands.DriveDistanceOnHeading;
import org.usfirst.frc.team6328.robot.commands.DriveWithJoystick.JoystickMode;
import org.usfirst.frc.team6328.robot.commands.GenerateMotionProfiles;
import org.usfirst.frc.team6328.robot.commands.RunMotionProfileOnRio;
import org.usfirst.frc.team6328.robot.commands.SmartSideAuto;
import org.usfirst.frc.team6328.robot.commands.SwitchStraightDelivery;
import org.usfirst.frc.team6328.robot.commands.TurnToAngle;
import org.usfirst.frc.team6328.robot.commands.VelocityPIDTuner;
import org.usfirst.frc.team6328.robot.subsystems.CameraSystem;
import org.usfirst.frc.team6328.robot.subsystems.DriveTrain;
import org.usfirst.frc.team6328.robot.subsystems.DriveTrain.DriveGear;
import org.usfirst.frc.team6328.robot.subsystems.Elevator;
import org.usfirst.frc.team6328.robot.subsystems.Intake;
import org.usfirst.frc.team6328.robot.subsystems.PixyI2C;
import org.usfirst.frc.team6328.robot.subsystems.PixyI2C.PixyException;
import org.usfirst.frc.team6328.robot.subsystems.PixyI2C.PixyPacket;
import org.usfirst.frc.team6328.robot.subsystems.ScoringArm;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import openrio.powerup.MatchData;
import openrio.powerup.MatchData.GameFeature;
import openrio.powerup.MatchData.OwnedSide;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static final RobotMap robotMap = new RobotMap();
	
	public static final DriveTrain driveSubsystem = new DriveTrain();
	public static final Elevator elevator = new Elevator();
	public static final Intake intake = new Intake();
	public static final ScoringArm scoringArm = new ScoringArm();
	
	public static OI oi;
	public static final AHRS ahrs = new AHRS(SPI.Port.kMXP);
	public static final PixyI2C pixy = new PixyI2C(new I2C(I2C.Port.kOnboard, 0x54), new PixyPacket[5], new PixyException("Pixy Error"), new PixyPacket());
//	public static final MaxbotixUltrasonic ultrasonic = new MaxbotixUltrasonic(SerialPort.Port.kOnboard);
	public static final CameraSystem cameraSubsystem = new CameraSystem();
	public static DigitalInput tapeSensor;

	Command autoCommand;
	SendableChooser<Command> tuningModeChooser
	= new SendableChooser<>();
	public static SendableChooser<JoystickMode> joystickModeChooser;
	SendableChooser<StartingPosition> startingPositionChooser = new SendableChooser<StartingPosition>();
	SendableChooser<AutoMode> autoModeChooser = new SendableChooser<AutoMode>();
	SendableChooser<AutoPriority> autoPriorityChooser = new SendableChooser<AutoPriority>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		this.setPeriod(0.02);
		oi = new OI();
		elevator.initLEDs();
		joystickModeChooser = new SendableChooser<JoystickMode>();
		
		startingPositionChooser.addObject("Left", StartingPosition.LEFT);
		startingPositionChooser.addDefault("Center", StartingPosition.CENTER);
		startingPositionChooser.addObject("Right", StartingPosition.RIGHT);
		autoModeChooser.addDefault("Do Nothing", null);
		autoModeChooser.addObject("Cross line", AutoMode.CROSS_LINE);
		autoModeChooser.addObject("Simple switch delivery", AutoMode.SIMPLE_SWITCH);
		autoModeChooser.addObject("Smart Delivery", AutoMode.SMART);
		autoPriorityChooser.addDefault("None", AutoPriority.NONE);
		autoPriorityChooser.addObject("Switch", AutoPriority.SWITCH);
		autoPriorityChooser.addObject("Scale", AutoPriority.SCALE);
		tuningModeChooser.addDefault("Do Nothing", null);
		// chooser.addObject("My Auto", new MyAutoCommand());
		joystickModeChooser.addDefault("Tank", JoystickMode.Tank);
        joystickModeChooser.addObject("Split Arcade", JoystickMode.SplitArcade);
        
        if (RobotMap.tuningMode) {
        		tuningModeChooser.addObject("10 forward 5 right profile", new RunMotionProfileOnRio("test10forward5right", false, false, false, true));
        		tuningModeChooser.addObject("1 foot off edge to switch side profile", new RunMotionProfileOnRio("sideToSwitch", false, false, false, true));
        		tuningModeChooser.addObject("1 foot off edge to switch front profile", new RunMotionProfileOnRio("sideToSwitchFront", false, false, false, true));
        		tuningModeChooser.addObject("20 foot straight line", new DriveDistanceOnHeading(240));
        		tuningModeChooser.addObject("15 foot straight line", new DriveDistanceOnHeading(180));
        		tuningModeChooser.addObject("10 foot straight line", new DriveDistanceOnHeading(120));
        		tuningModeChooser.addObject("5 foot straight line", new DriveDistanceOnHeading(60));
        		tuningModeChooser.addObject("Velocity PID Tuner", new VelocityPIDTuner());
        		tuningModeChooser.addObject("side switch to start profile", new RunMotionProfileOnRio("backwardsTest", false, false, true, true));
        		tuningModeChooser.addObject("Turn 90 degrees", new TurnToAngle(90));
        		tuningModeChooser.addObject("8 foot straight profile", new RunMotionProfileOnRio("8straight", false, false, false, true));
        		tuningModeChooser.addObject("Profile Flip Test", new RunMotionProfileOnRio("centerToLeftSwitch", true, false, false, true));
         	SmartDashboard.putData("Tuning Auto Mode", tuningModeChooser);
         	autoModeChooser.addObject("Tuning auto", AutoMode.TUNING);
        }
        SmartDashboard.putData("Control Mode", joystickModeChooser);
        SmartDashboard.putData("Starting Position", startingPositionChooser);
        SmartDashboard.putData("Auto Mode", autoModeChooser);
        SmartDashboard.putBoolean("Cross Middle Allowed", true);
        SmartDashboard.putData("Auto Priority", autoPriorityChooser);
        SmartDashboard.putBoolean("Enable 2nd Cube", true);
        
     // if the current waypoint version is old, re-generate profiles
        BufferedReader waypointVersionReader;
        int lastWaypointVersion = 0;
		try {
			waypointVersionReader = new BufferedReader(new FileReader("/home/lvuser/lastWaypointVersion"));
			lastWaypointVersion = Integer.parseInt(waypointVersionReader.readLine());
			waypointVersionReader.close();
		} catch (NumberFormatException | IOException e) {
			// do nothing
		}
		if (org.usfirst.frc.team6328.robot.commands.GenerateMotionProfiles.waypointVersion > lastWaypointVersion) {
			GenerateMotionProfiles generateCommand = new GenerateMotionProfiles();
			generateCommand.setRunWhenDisabled(true);
			generateCommand.start();
		}
		// Force compressor to run by creating a pneumatics object
		@SuppressWarnings("unused")
		Compressor c = new Compressor();
		cameraSubsystem.useFrontCamera();
	}
	
	@Override
	public void robotPeriodic() {
		boolean tapeSensorValue = false;
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
//			tapeSensorValue = tapeSensor.get();
		} 
		SmartDashboard.putBoolean("Tape Sensor", tapeSensorValue);
		SmartDashboard.putBoolean("Cube Sensor", intake.getSensor());
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		if (driveSubsystem.getVelocityLeft() <= 2 && driveSubsystem.getVelocityRight() <= 2) {
			Robot.driveSubsystem.enableBrakeMode(false);
		}
		Robot.oi.updateLED(OILED.CUBE_SENSE_3, joystickModeChooser.getSelected() != JoystickMode.SplitArcade);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		Robot.driveSubsystem.enableBrakeMode(true);
		
		// Zero Gyro
		ahrs.zeroYaw();
		double startingUpdates = ahrs.getUpdateCount();
		while (ahrs.getUpdateCount()<startingUpdates+3) {
			try {
				Thread.sleep(5);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		
		// Wait for game data to arrive, check scale since we don't care about opposing switch
		int retries = 400;
		OwnedSide scaleSide = MatchData.getOwnedSide(GameFeature.SCALE);
		while (scaleSide == OwnedSide.UNKNOWN && retries > 0) {
			retries--;
			try {
                Thread.sleep(5);
            } catch (InterruptedException ie) {
                // Just ignore the interrupted exception
            }
			scaleSide = MatchData.getOwnedSide(GameFeature.SCALE);
		}
		OwnedSide switchSide = MatchData.getOwnedSide(GameFeature.SWITCH_NEAR);
		
		Command tuningCommand = tuningModeChooser.getSelected();
		StartingPosition startingPosition = startingPositionChooser.getSelected();
		AutoMode mode = autoModeChooser.getSelected();
		boolean crossMiddle = SmartDashboard.getBoolean("Cross Middle Allowed", true);
		AutoPriority priority = autoPriorityChooser.getSelected();
		boolean twoCubeEnabled = SmartDashboard.getBoolean("Enable 2nd Cube", true);
		
		Command crossLineCommand = new DriveDistanceOnHeading(120+12-RobotMap.robotLength, 0);
		
		switch (mode) {
		case CROSS_LINE:
			autoCommand = crossLineCommand;
			if (switchSide == OwnedSide.UNKNOWN || scaleSide == OwnedSide.UNKNOWN) {
				DriverStation.reportWarning("Failed to recieve data from FMS but it didn't matter", false);
			}
			break;
		case SIMPLE_SWITCH:
			if (startingPosition.equals(switchSide)) {
				autoCommand = new SwitchStraightDelivery();
			} else {
				autoCommand = crossLineCommand;
			}
			if (switchSide == OwnedSide.UNKNOWN) {
				DriverStation.reportError("Failed to recieve data from FMS about switch so not delivering", false);
			}
			if (scaleSide == OwnedSide.UNKNOWN) {
				DriverStation.reportWarning("Failed to recieve data from FMS about scale but it didn't matter", false);
			}
			break;
		case SMART:
			if (startingPosition == StartingPosition.CENTER) {
				if (switchSide == OwnedSide.UNKNOWN) {
					DriverStation.reportError("Failed to recieve data from FMS about switch, doing nothing", false);
				} else {
					autoCommand = new CenterAuto(switchSide);
				}
				if (scaleSide == OwnedSide.UNKNOWN) {
					DriverStation.reportWarning("Failed to recieve data from FMS about scale but it didn't matter", false);
				}	
			} else {
				if (switchSide == OwnedSide.UNKNOWN || scaleSide == OwnedSide.UNKNOWN) {
					DriverStation.reportError("Failed to recieve data from FMS, crossing line!", false);
					autoCommand = crossLineCommand;
				} else {
					AutoTruthTable.SelectedAutoDestinations dests = AutoTruthTable.findRow(
							startingPosition.equals(switchSide), startingPosition.equals(scaleSide), 
							crossMiddle, priority == AutoPriority.SWITCH, priority == AutoPriority.SCALE, 
							twoCubeEnabled);
					if (dests != null) {
						if (dests.getDest1() == null) {
							// No first destination means cross line
							autoCommand = crossLineCommand;
						} else {
							// SmartSideAuto will handle dest2 being null
							autoCommand = new SmartSideAuto(dests.getDest1(), dests.getDest2(), 
									startingPosition == StartingPosition.LEFT);
						}
					} else {
						// The truth table did not find a match
						autoCommand = crossLineCommand;
						DriverStation.reportError("Auto lookup failed, crossing line!", false);
					}
				}
			}
			break;
		case TUNING:
			if (RobotMap.tuningMode) {
				autoCommand = tuningCommand;
			} else {
				autoCommand = crossLineCommand;
				DriverStation.reportError("Tuning auto selected in competition mode, crossing line!", false);
			}
			break;
		default:
			if (switchSide == OwnedSide.UNKNOWN || scaleSide == OwnedSide.UNKNOWN) {
				DriverStation.reportWarning("Failed to recieve data from FMS but it didn't matter", false);
			}
			break;
		}

		// schedule the autonomous command (example)
		if (autoCommand != null) {
			autoCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		Robot.driveSubsystem.enableBrakeMode(true);
		Robot.driveSubsystem.switchGear(DriveGear.LOW);
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autoCommand != null) {
			autoCommand.cancel();
		}
		Robot.oi.updateLED(OILED.CUBE_SENSE_3, false);
		if (joystickModeChooser.getSelected() != JoystickMode.SplitArcade) {
			DriverStation.reportWarning("Not in split arcade mode", false);
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	public enum StartingPosition {
		LEFT, CENTER, RIGHT;
		
		public boolean equals(OwnedSide side) {
			return this.toString().equals(side.toString());
		}
	}
	
	public enum AutoMode {
		CROSS_LINE, SIMPLE_SWITCH, SMART, TUNING
	}
	
	public enum AutoPriority {
		NONE, SWITCH, SCALE
	}
	
	
	// Utility functions
	public static String genGraphStr(double...data) {
		StringJoiner sj = new StringJoiner(":");
		for (double item : data) {
			sj.add(String.valueOf(item));
		}
		return sj.toString();
	}
}
