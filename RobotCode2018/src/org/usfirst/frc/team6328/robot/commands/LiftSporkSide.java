package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.commands.RetractSpork.SporkSide;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Lifts one side of the spork
 */
public class LiftSporkSide extends Command {
	
	private SporkSide side;

	public LiftSporkSide(SporkSide side) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.side = side;
		switch (side) {
		case LEFT:
			requires(Robot.spork.leftSpork);
			break;
		case RIGHT:
			requires(Robot.spork.rightSpork);
			break;
		}
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		switch (side) {
		case LEFT:
			Robot.spork.liftLeft();
			break;
		case RIGHT:
			Robot.spork.liftRight();
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
		switch (side) {
		case LEFT:
			Robot.spork.stopLeft();
			break;
		case RIGHT:
			Robot.spork.stopRight();
			break;
		}
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
