package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Resets the scoring arm to the low postion.
 */
public class ResetArm extends Command {

	private static final double moveSpeed = -0.10;

	public ResetArm() {
		super("ResetArm");
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.scoringArm);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.scoringArm.moveArm(moveSpeed);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if(Robot.scoringArm.getBackwardLimit()) {
			return true;
		}
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
