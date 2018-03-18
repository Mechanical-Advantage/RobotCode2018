package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Disables the specified slave
 */
public class DisableDriveSlave extends InstantCommand {
	
	private int slave;

	public DisableDriveSlave(int slave) {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveSubsystem);
		this.slave = slave;
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.driveSubsystem.setDisabledSlave(slave);
	}

}
