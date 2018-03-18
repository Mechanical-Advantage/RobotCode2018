package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Disables the specified slave
 */
public class DisableElevatorSlave extends InstantCommand {
	
	private int slave;

	public DisableElevatorSlave(int slave) {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.elevator);
		this.slave = slave;
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.elevator.setDisabledSlave(slave);
	}

}
