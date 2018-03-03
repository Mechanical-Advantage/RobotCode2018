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

import org.usfirst.frc.team6328.robot.RobotMap.RobotType;
import org.usfirst.frc.team6328.robot.commands.CenterAuto;
import org.usfirst.frc.team6328.robot.commands.DriveDistanceOnHeading;
import org.usfirst.frc.team6328.robot.commands.DriveWithJoystick.JoystickMode;
import org.usfirst.frc.team6328.robot.commands.GenerateMotionProfiles;
import org.usfirst.frc.team6328.robot.commands.RunMotionProfileOnRio;
import org.usfirst.frc.team6328.robot.commands.SideAutoScaleAndSwitch;
import org.usfirst.frc.team6328.robot.commands.SideAutoScaleAndSwitch.SideEnabledAutos;
import org.usfirst.frc.team6328.robot.commands.SideAutoSwitch;
import org.usfirst.frc.team6328.robot.commands.SimpleSideAutoSwitch;
import org.usfirst.frc.team6328.robot.commands.TurnToAngle;
import org.usfirst.frc.team6328.robot.commands.VelocityPIDTuner;
import org.usfirst.frc.team6328.robot.subsystems.CameraSystem;
import org.usfirst.frc.team6328.robot.subsystems.DriveTrain;
import org.usfirst.frc.team6328.robot.subsystems.Elevator;
import org.usfirst.frc.team6328.robot.subsystems.Intake;
import org.usfirst.frc.team6328.robot.subsystems.MaxbotixUltrasonic;
import org.usfirst.frc.team6328.robot.subsystems.PixyI2C;
import org.usfirst.frc.team6328.robot.subsystems.PixyI2C.PixyException;
import org.usfirst.frc.team6328.robot.subsystems.PixyI2C.PixyPacket;
import org.usfirst.frc.team6328.robot.subsystems.ScoringArm;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
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
		m_chooser.addDefault("Do Nothing", null);
		// chooser.addObject("My Auto", new MyAutoCommand());
		joystickModeChooser.addDefault("Tank", JoystickMode.Tank);
        joystickModeChooser.addObject("Split Arcade", JoystickMode.SplitArcade);
        if (RobotMap.tuningMode) {
        		m_chooser.addObject("10 forward 5 right profile", new RunMotionProfileOnRio("test10forward5right", false, false, false, true));
        		m_chooser.addObject("1 foot off edge to switch side profile", new RunMotionProfileOnRio("sideToSwitch", false, false, false, true));
        		m_chooser.addObject("1 foot off edge to switch front profile", new RunMotionProfileOnRio("sideToSwitchFront", false, false, false, true));
        		m_chooser.addObject("20 foot straight line", new DriveDistanceOnHeading(240));
        		m_chooser.addObject("15 foot straight line", new DriveDistanceOnHeading(180));
        		m_chooser.addObject("10 foot straight line", new DriveDistanceOnHeading(120));
        		m_chooser.addObject("5 foot straight line", new DriveDistanceOnHeading(60));
        		m_chooser.addObject("Velocity PID Tuner", new VelocityPIDTuner());
        		m_chooser.addObject("side switch to start profile", new RunMotionProfileOnRio("backwardsTest", false, false, true, true));
        		m_chooser.addObject("Turn 90 degrees", new TurnToAngle(90));
        		m_chooser.addObject("8 foot straight profile", new RunMotionProfileOnRio("8straight", false, false, false, true));
        		m_chooser.addObject("Profile Flip Test", new RunMotionProfileOnRio("centerToLeftSwitch", true, false, false, true));
        }
        m_chooser.addObject("Cross Line", new DriveDistanceOnHeading(130, 0));
        m_chooser.addObject("Center auto", new CenterAuto());
        m_chooser.addObject("Right side simple side switch or cross line", new SimpleSideAutoSwitch(false));
        m_chooser.addObject("Left side simple side switch or cross line", new SimpleSideAutoSwitch(true));
        m_chooser.addObject("Right side end of switch no cross", new SideAutoSwitch(false, false, false));
        m_chooser.addObject("Right side front of switch no cross", new SideAutoSwitch(false, true, false));
        m_chooser.addObject("Right side end of switch + cross", new SideAutoSwitch(false, false, true));
        m_chooser.addObject("Right side front of switch + cross", new SideAutoSwitch(false, true, true));
        m_chooser.addObject("Left side end of switch no cross", new SideAutoSwitch(true, false, false));
        m_chooser.addObject("Left side front of switch no cross", new SideAutoSwitch(true, true, false));
        m_chooser.addObject("Left side end of switch + cross", new SideAutoSwitch(true, false, true));
        m_chooser.addObject("Left side front of switch + cross", new SideAutoSwitch(true, true, true));
        m_chooser.addObject("Left side 2 cube or switch same side only", new SideAutoScaleAndSwitch(true, SideEnabledAutos.SAME_SIDE_ONLY));
        m_chooser.addObject("Left side 2 cube both same side or switch + cross", new SideAutoScaleAndSwitch(true, SideEnabledAutos.SAME_SIDE_ONLY_SWITCH_CROSS));
        m_chooser.addObject("Left side 2 cube both together or switch + cross", new SideAutoScaleAndSwitch(true, SideEnabledAutos.NOT_SPLIT));
        m_chooser.addObject("Left side 2 cube always", new SideAutoScaleAndSwitch(true, SideEnabledAutos.ALL));
        m_chooser.addObject("Right side 2 cube or switch same side only", new SideAutoScaleAndSwitch(false, SideEnabledAutos.SAME_SIDE_ONLY));
        m_chooser.addObject("Right side 2 cube both same side or switch + cross", new SideAutoScaleAndSwitch(false, SideEnabledAutos.SAME_SIDE_ONLY_SWITCH_CROSS));
        m_chooser.addObject("Right side 2 cube both together or switch + cross", new SideAutoScaleAndSwitch(false, SideEnabledAutos.NOT_SPLIT));
        m_chooser.addObject("Right side 2 cube always", new SideAutoScaleAndSwitch(false, SideEnabledAutos.ALL));
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
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			DoubleSolenoid solenoid1 = new DoubleSolenoid(1, 4, 5);
			solenoid1.set(Value.kReverse);
			DoubleSolenoid solenoid2 = new DoubleSolenoid(1, 6, 7);
			solenoid2.set(Value.kReverse);
		}
		cameraSubsystem.useFrontCamera();
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
		
		// Wait for game data to arrive, check scale since we don't care about opposite switch
		int retries = 100;
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
