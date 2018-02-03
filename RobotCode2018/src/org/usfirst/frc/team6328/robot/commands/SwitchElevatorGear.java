package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.subsystems.Elevator.ElevatorGear;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Switch the elevator gear.
 */
public class SwitchElevatorGear extends InstantCommand {
	
	ElevatorGear gear;

	public SwitchElevatorGear(ElevatorGear gear) {
		super("SwitchElevatorGear");
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.elevator);
		this.gear = gear;
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.elevator.switchGear(gear);
	}

}
