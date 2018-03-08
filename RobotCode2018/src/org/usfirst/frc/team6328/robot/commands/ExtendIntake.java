package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.OI.OILED;
import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Extends (un-retracts) the intake;
 */
public class ExtendIntake extends InstantCommand {

	public ExtendIntake() {
		super("ExtendIntake");
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.intake);
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.intake.setRetracted(false);
		Robot.oi.updateLED(OILED.INTAKE_OFF, true);
		Robot.oi.updateLED(OILED.INTAKE_ON, false);
		Robot.oi.updateLED(OILED.INTAKE_RETRACT, false);
	}

}
