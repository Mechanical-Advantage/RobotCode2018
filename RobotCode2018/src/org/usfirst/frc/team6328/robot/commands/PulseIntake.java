package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * Runs the intake at a low power for a brief period of time
 */
public class PulseIntake extends TimedCommand {
	
	private static final double time = 0.5;
	private static final double power = 0.35;

	public PulseIntake() {
		super(time);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.intake);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.intake.intake(power);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Called once after timeout
	protected void end() {
		Robot.intake.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
