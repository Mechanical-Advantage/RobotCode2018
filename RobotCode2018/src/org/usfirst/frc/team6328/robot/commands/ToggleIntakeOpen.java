package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Toggles the state of the intake opening mechanism.
 */
public class ToggleIntakeOpen extends InstantCommand {

	public ToggleIntakeOpen() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.intake);
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.intake.setOpen(!Robot.intake.getOpen());
	}

}
