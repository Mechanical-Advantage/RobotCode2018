package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Toggles the spork between deployed and lock piston engaged.
 */
public class ToggleSporkDeploy extends Command {
	
	private Command deployCommand = new DeploySpork();
	private Command lockEngageCommand = new EngageSporkLock();
	private Command currentCommand;

	public ToggleSporkDeploy() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called once when the command executes
	protected void initialize() {
		if (Robot.spork.isDeployed()) {
			currentCommand = lockEngageCommand;
		} else {
			currentCommand = deployCommand;
		}
		currentCommand.start();
	}

	@Override
	protected boolean isFinished() {
		return !currentCommand.isRunning();
	}
	
	@Override
	protected void end() {
		currentCommand.cancel();
	}

	@Override
	protected void interrupted() {
		end();
	}
}
