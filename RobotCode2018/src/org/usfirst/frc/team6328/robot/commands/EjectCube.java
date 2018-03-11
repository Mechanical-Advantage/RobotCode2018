package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Eject the cube while the command is running
 */
public class EjectCube extends Command {
	
	private double speed;
	
	public EjectCube() {
		super("EjectCube");
		requires(Robot.intake);
	}

	public EjectCube(double speed) {
		super("EjectCube");
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.intake);
		this.speed = speed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.intake.setRetracted(false);
		if (speed == 0) {
			Robot.intake.eject();
		} else {
			Robot.intake.eject(speed);
		}
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
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
