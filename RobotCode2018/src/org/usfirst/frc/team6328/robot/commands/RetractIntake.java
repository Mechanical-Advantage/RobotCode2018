package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Retracts the intake.
 */
public class RetractIntake extends InstantCommand {

	public RetractIntake() {
		super("RetractIntake");
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.intake);
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.intake.setRetracted(true);
	}

}
