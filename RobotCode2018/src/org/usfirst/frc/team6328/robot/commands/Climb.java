package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.OI.OILED;
import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.subsystems.Elevator.ElevatorGear;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Make the robot climb using the elevator
 */

public class Climb extends Command {

	private static final double climbPosition = 0;

	public Climb() {
		super("Climb");
		requires(Robot.elevator);
	}

	@Override
	protected void initialize() {
		Robot.elevator.switchGear(ElevatorGear.LOW);
		Robot.elevator.setPosition(climbPosition);
		Robot.oi.updateLED(OILED.CLIMB, true);
	}

	@Override
	protected boolean isFinished() {
		Robot.oi.updateLED(OILED.CLIMB, false);
		return Robot.elevator.onTarget();
	}
}
