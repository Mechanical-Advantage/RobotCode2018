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

import org.usfirst.frc.team6328.robot.commands.DriveDistanceOnHeading;
import org.usfirst.frc.team6328.robot.commands.DriveWithJoystick.JoystickMode;
import org.usfirst.frc.team6328.robot.commands.GenerateMotionProfiles;
import org.usfirst.frc.team6328.robot.commands.RunMotionProfileOnRioFromFile;
import org.usfirst.frc.team6328.robot.commands.VelocityPIDTuner;
import org.usfirst.frc.team6328.robot.subsystems.CameraSystem;
import org.usfirst.frc.team6328.robot.subsystems.DriveTrain;
import org.usfirst.frc.team6328.robot.subsystems.Elevator;
import org.usfirst.frc.team6328.robot.subsystems.Intake;
import org.usfirst.frc.team6328.robot.subsystems.PixyI2C;
import org.usfirst.frc.team6328.robot.subsystems.PixyI2C.PixyException;
import org.usfirst.frc.team6328.robot.subsystems.PixyI2C.PixyPacket;
import org.usfirst.frc.team6328.robot.subsystems.PnuematicsTest;
import org.usfirst.frc.team6328.robot.subsystems.PnuematicsTest2;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	public static final PnuematicsTest pnuematicsTest = new PnuematicsTest();
	public static final PnuematicsTest2 pnuematicsTest2 = new PnuematicsTest2();
	
	public static OI oi;
	public static final AHRS ahrs = new AHRS(SPI.Port.kMXP);
	public static final PixyI2C pixy = new PixyI2C(new I2C(I2C.Port.kOnboard, 0x54), new PixyPacket[5], new PixyException("Pixy Error"), new PixyPacket());
	public static final CameraSystem cameraSubsystem = new CameraSystem();

	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();
	public static SendableChooser<JoystickMode> joystickModeChooser;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		this.setPeriod(0.02);
		oi = new OI();
		joystickModeChooser = new SendableChooser<JoystickMode>();
		m_chooser.addDefault("Default Auto", null);
		// chooser.addObject("My Auto", new MyAutoCommand());
		joystickModeChooser.addDefault("Tank", JoystickMode.Tank);
        joystickModeChooser.addObject("Split Arcade", JoystickMode.SplitArcade);
        if (RobotMap.tuningMode) {
        		m_chooser.addObject("10 forward 5 right profile", new RunMotionProfileOnRioFromFile("test10forward5right", false, false));
        		m_chooser.addObject("1 foot off edge to switch side profile", new RunMotionProfileOnRioFromFile("sideToSwitch", false, false));
        		m_chooser.addObject("1 foot off edge to switch front profile", new RunMotionProfileOnRioFromFile("sideToSwitchFront", false, false));
        		m_chooser.addObject("20 foot straight line", new DriveDistanceOnHeading(240));
        		m_chooser.addObject("Velocity PID Tuner", new VelocityPIDTuner());
        }
		SmartDashboard.putData("Auto mode", m_chooser);
        SmartDashboard.putData("Control Mode", joystickModeChooser);
        
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
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		Robot.driveSubsystem.enableBrakeMode(false);
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
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
		m_autonomousCommand = m_chooser.getSelected();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
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
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
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
	
	
	// Utility functions
	public static String genGraphStr(double...data) {
		StringJoiner sj = new StringJoiner(":");
		for (double item : data) {
			sj.add(String.valueOf(item));
		}
		return sj.toString();
	}
}
