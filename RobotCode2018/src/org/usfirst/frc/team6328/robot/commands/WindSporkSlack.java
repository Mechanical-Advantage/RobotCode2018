package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Winds the slack on the spork, either the initial or the before lift
 */
public class WindSporkSlack extends Command {
	
	private SlackType slackType;
	private boolean leftStopped;
	private boolean rightStopped;

	public WindSporkSlack(SlackType type) {
		super();
		requires(Robot.spork.leftSpork);
		requires(Robot.spork.rightSpork);
		slackType = type;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (isLeftSlackWound()) {
			leftStopped = true;
		} else {
			Robot.spork.windSlackLeft();
			leftStopped = false;
		}
		
		if (isRightSlackWound()) {
			rightStopped = true;
		} else {
			Robot.spork.windSlackRight();
			rightStopped = false;
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (!leftStopped && isLeftSlackWound()) {
			Robot.spork.stopLeft();
			leftStopped = true;
		}
		
		if (!rightStopped && isRightSlackWound()) {
			Robot.spork.stopRight();
			rightStopped = true;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return leftStopped && rightStopped;
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
	
	private boolean isLeftSlackWound() {
		switch (slackType) {
		case AFTER_DEPLOY:
			return Robot.spork.isLeftDeploySlackWound();
		case BEFORE_LIFT:
			return Robot.spork.isLeftLiftSlackWound();
		default:
			return true; // Always return true to prevent getting stuck
		}
	}
	private boolean isRightSlackWound() {
		switch (slackType) {
		case AFTER_DEPLOY:
			return Robot.spork.isRightDeploySlackWound();
		case BEFORE_LIFT:
			return Robot.spork.isRightLiftSlackWound();
		default:
			return true; // Always return true to prevent getting stuck
		}
	}
	
	public enum SlackType {
		AFTER_DEPLOY, BEFORE_LIFT
	}
}
