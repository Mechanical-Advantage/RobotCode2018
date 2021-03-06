package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Command for manual control of the elevator
 */
public class JoystickElevatorControl extends Command {
	
	public static final double deadband = 0.05;

	public JoystickElevatorControl() {
		super("JoystickElevatorControl");
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.elevator);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double joystickValue = Robot.oi.getElevatorJoystick();
		if (Math.abs(joystickValue) <= deadband) {
			Robot.elevator.holdPosition();
		} else {
			Robot.elevator.driveOpenLoop(joystickValue);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
