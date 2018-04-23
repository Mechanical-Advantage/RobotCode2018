package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Runs the intake at a low power while the elevator is above a certain height and the intake is lowered
 */
public class IntakeHoldWhileElevatorLifted extends Command {
	
	private static final double power = 0.2;
	private static final double elevatorHeight = 12;
	
	private boolean intakeRunningLast;

	public IntakeHoldWhileElevatorLifted() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.intake);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		intakeRunningLast = false;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (intakeRunningLast && !Robot.intake.getRetracted() && Robot.elevator.getPosition() < elevatorHeight) {
			Robot.intake.stop();
			intakeRunningLast = false;
		} else if (!intakeRunningLast && !Robot.intake.getRetracted() && Robot.elevator.getPosition() >= elevatorHeight) {
			Robot.intake.intake(power);
			intakeRunningLast = true;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.intake.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
