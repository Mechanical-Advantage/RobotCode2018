package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.subsystems.DriveTrain.DriveGear;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * Drives each side of the drive at the specified speeds in the specified gears
 */
public class DriveForTime extends TimedCommand {
	
	private DriveGear gear;
	private double leftSpeed, rightSpeed;

	public DriveForTime(double time, DriveGear gear, double leftSpeed, double rightSpeed) {
		super(time);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveSubsystem);
		this.gear = gear;
		this.leftSpeed = leftSpeed;
		this.rightSpeed = rightSpeed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveSubsystem.switchGear(gear);
		Robot.driveSubsystem.drive(leftSpeed, rightSpeed);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Called once after timeout
	protected void end() {
		Robot.driveSubsystem.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
