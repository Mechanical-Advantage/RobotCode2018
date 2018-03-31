package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Releases the spork lock, allowing it to fall down
 */
public class ReleaseSpork extends InstantCommand {

	public ReleaseSpork() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.spork);
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.spork.releaseLock();
	}

}
