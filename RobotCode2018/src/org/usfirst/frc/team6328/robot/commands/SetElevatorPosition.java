package org.usfirst.frc.team6328.robot.commands;

import java.util.HashMap;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.subsystems.Elevator.ElevatorGear;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Set the target elevator position. 
 */
public class SetElevatorPosition extends Command {
	
	private static final double bangBangSpeed = 0.5;
	
	private ElevatorPosition targetPosition;
	private static HashMap<ElevatorPosition, Double> positions;
	private boolean motionMagic;

	public SetElevatorPosition(ElevatorPosition position, boolean motionMagic) {
		super("SetElevatorPosition");
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.elevator);
		targetPosition = position;
		this.motionMagic = motionMagic;
		
		if (positions == null) {
			positions = new HashMap<ElevatorPosition, Double>();
			positions.put(ElevatorPosition.GROUND, (double) 0);
			positions.put(ElevatorPosition.SWITCH, (double) 20);
			positions.put(ElevatorPosition.SCALE_LOW, (double) 50);
			positions.put(ElevatorPosition.SCALE_MID, (double) 62);
			positions.put(ElevatorPosition.SCALE_HIGH, (double) 74);
			positions.put(ElevatorPosition.CLIMB_GRAB, (double) 86);
		}
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.elevator.switchGear(ElevatorGear.HIGH);
		double targetPositionInches = positions.get(targetPosition);
		if (motionMagic) {
			Robot.elevator.setPosition(targetPositionInches);
		} else {
			if (targetPositionInches > Robot.elevator.getPosition()) {
				Robot.elevator.driveOpenLoop(bangBangSpeed*-1);
			} else if (targetPositionInches < Robot.elevator.getPosition()) {
				Robot.elevator.driveOpenLoop(bangBangSpeed);
			}
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.elevator.onTarget(positions.get(targetPosition));
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
	
	public enum ElevatorPosition {
		GROUND, SWITCH, SCALE_LOW, SCALE_MID, SCALE_HIGH, CLIMB_GRAB
	}
}
