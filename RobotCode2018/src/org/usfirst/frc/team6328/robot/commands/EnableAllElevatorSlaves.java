package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Enables all the elevator slaves
 */
public class EnableAllElevatorSlaves extends InstantCommand {

	public EnableAllElevatorSlaves() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.elevator);
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.elevator.enableAllSlaves();
	}

}
