package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Retracts the spork
 */
public class RetractSpork extends Command {
	
	private SporkSide side;
	private SporkRetractSpeed speed;

	public RetractSpork(SporkSide side, SporkRetractSpeed speed) {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		switch (side) {
		case LEFT:
			requires(Robot.spork.leftSpork);
			break;
		case RIGHT:
			requires(Robot.spork.rightSpork);
			break;
		}
		this.side = side;
		this.speed = speed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		switch (side) {
		case LEFT:
			switch (speed) {
			case FAST:
				Robot.spork.retractFastLeft();
				break;
			case SLOW:
				Robot.spork.retractSlowLeft();
				break;
			}
			break;
		case RIGHT:
			switch (speed) {
			case FAST:
				Robot.spork.retractFastRight();
				break;
			case SLOW:
				Robot.spork.retractSlowRight();
				break;
			}
			break;
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.spork.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
	
	public enum SporkSide {
		LEFT, RIGHT;
	}
	
	public enum SporkRetractSpeed {
		FAST, SLOW;
	}
}
