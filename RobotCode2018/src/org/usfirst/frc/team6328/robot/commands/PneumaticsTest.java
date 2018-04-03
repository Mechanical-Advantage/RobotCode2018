package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PneumaticsTest extends Command {
	
	DoubleSolenoid solenoid;

	public PneumaticsTest() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		solenoid = new DoubleSolenoid(3, 2);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		solenoid.set(Value.kForward);
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
		solenoid.set(Value.kReverse);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
