package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Retracts the spork
 */
public class ResetSpork extends Command {

	public ResetSpork() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.spork);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.spork.reset();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.spork.isResetComplete();
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.spork.engageLock();
		Robot.spork.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
