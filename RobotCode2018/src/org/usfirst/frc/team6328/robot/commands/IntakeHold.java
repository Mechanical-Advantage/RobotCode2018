package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.OI.OILED;
import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Runs the intake at a low power to hold a cube
 */
public class IntakeHold extends Command {
	
	private static final double power = 0.2;

	public IntakeHold() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.intake);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.intake.intake(power);
		Robot.oi.updateLED(OILED.INTAKE_OFF, true);
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
		Robot.intake.stop();
		Robot.oi.updateLED(OILED.INTAKE_OFF, false);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
