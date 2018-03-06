package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Opens the intake pistons. Not needed to intake a cube normally.
 */
public class OpenIntake extends InstantCommand {

	public OpenIntake() {
		super("OpenIntake");
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.intake);
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.intake.setOpen(true);
	}

}
