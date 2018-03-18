package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Enables all the elevator slaves
 */
public class EnableAllDriveSlaves extends InstantCommand {

	public EnableAllDriveSlaves() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveSubsystem);
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.driveSubsystem.enableAllSlaves();
	}

}
