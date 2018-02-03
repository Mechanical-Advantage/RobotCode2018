package org.usfirst.frc.team6328.robot.commands;

import java.util.HashMap;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.subsystems.Elevator.ElevatorGear;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Set the target elevator position. 
 */
public class SetElevatorPosition extends Command {
	
	private ElevatorPosition targetPosition;
	private static HashMap<ElevatorPosition, Double> positions;

	public SetElevatorPosition(ElevatorPosition position) {
		super("SetElevatorPosition");
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.elevator);
		targetPosition = position;
		
		if (positions == null) {
			positions = new HashMap<ElevatorPosition, Double>();
			positions.put(ElevatorPosition.GROUND, (double) 0);
			positions.put(ElevatorPosition.SWITCH, (double) 0);
			positions.put(ElevatorPosition.SCALE, (double) 0);
			positions.put(ElevatorPosition.CLIMB_GRAB, (double) 0);
		}
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.elevator.switchGear(ElevatorGear.HIGH);
		Robot.elevator.setPosition(positions.get(targetPosition));
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.elevator.onTarget();
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
	
	public enum ElevatorPosition {
		GROUND, SWITCH, SCALE, CLIMB_GRAB
	}
}
