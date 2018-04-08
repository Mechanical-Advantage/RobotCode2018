package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Engage the lock piston on the spork
 */
public class EngageSporkLock extends InstantCommand {

	public EngageSporkLock() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.spork.leftSpork);
		requires(Robot.spork.rightSpork);
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.spork.engageLock();
	}

}
